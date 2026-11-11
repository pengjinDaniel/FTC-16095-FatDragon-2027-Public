package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Spinup's three-state EKF, using local Pinpoint increments and delayed field observations. */
public final class PinpointPoseEstimator {
    private static final long HISTORY_NANOS = 1_500_000_000L;
    private static final long MAX_ODOMETRY_GAP_NANOS = 250_000_000L;
    private static final long MAX_VISION_COAST_NANOS = 1_500_000_000L;
    private static final long ANCHOR_CONFIRMATION_NANOS = 300_000_000L;
    private static final long NO_SAMPLE = Long.MIN_VALUE;

    // Starting uncertainty limits for the fixed camera. Refine them with field logs.
    private static final double MAX_POSITION_STD_METERS = 0.35;
    private static final double MAX_HEADING_STD_RADIANS = Math.toRadians(25);
    private static final double VISION_HEADING_STD_RADIANS = Math.toRadians(20);
    private static final double ANCHOR_POSITION_AGREEMENT_METERS = 0.20;
    private static final double ANCHOR_HEADING_AGREEMENT_RADIANS = Math.toRadians(12);

    private static final class State {
        final long timeNanos;
        final Pose2d odometry;
        final Pose2d estimate;
        final double[][] covariance;

        State(long timeNanos, Pose2d odometry, Pose2d estimate, double[][] covariance) {
            this.timeNanos = timeNanos;
            this.odometry = odometry;
            this.estimate = estimate;
            this.covariance = covariance;
        }
    }

    private final NavigableMap<Long, State> history = new TreeMap<>();
    private long lastVisionNanos = NO_SAMPLE;
    private State anchorCandidate;
    private boolean visionPaused;
    private boolean trustedAtPause;
    private String status = "Waiting for MT1 field anchor";

    public void reset() {
        history.clear();
        lastVisionNanos = NO_SAMPLE;
        anchorCandidate = null;
        trustedAtPause = false;
        status = "Waiting for MT1 field anchor";
    }

    public void setVisionPaused(boolean paused, long nowNanos) {
        if (visionPaused == paused) return;
        trustedAtPause = paused && isUsable(nowNanos);
        visionPaused = paused;
    }

    public void updateOdometry(long timeNanos, Pose2D rawPose) {
        State previous = latest();
        if (previous != null && timeNanos <= previous.timeNanos) return;
        if (previous != null && timeNanos - previous.timeNanos > MAX_ODOMETRY_GAP_NANOS) {
            reset();
            previous = null;
        }

        Pose2d odometry = metricPose(rawPose);
        State sample = previous == null
                ? new State(timeNanos, odometry, odometry, diagonal(1, 1, 1))
                : predict(previous, timeNanos, odometry);
        history.put(timeNanos, sample);
        history.headMap(timeNanos - HISTORY_NANOS, false).clear();
    }

    public boolean addVisionMeasurement(Pose2D visionPose, long captureNanos, long nowNanos,
                                        double xyStdMeters, boolean allowReanchor) {
        if (visionPaused) return false;
        double xyVariance = xyStdMeters * xyStdMeters;
        if (!Double.isFinite(xyVariance) || xyStdMeters <= 0) {
            status = "Invalid MT1 measurement noise";
            return false;
        }
        Map.Entry<Long, State> floor = history.floorEntry(captureNanos);
        Map.Entry<Long, State> ceiling = history.ceilingEntry(captureNanos);
        if (floor == null || ceiling == null || captureNanos <= lastVisionNanos) {
            status = "MT1 frame outside OD history or out of order";
            return false;
        }

        State atCapture = floor.getValue();
        if (captureNanos != atCapture.timeNanos) {
            State after = ceiling.getValue();
            double fraction = (double) (captureNanos - atCapture.timeNanos)
                    / (after.timeNanos - atCapture.timeNanos);
            Twist2d delta = atCapture.odometry.log(after.odometry);
            Pose2d interpolated = atCapture.odometry.exp(new Twist2d(
                    fraction * delta.dx, fraction * delta.dy, fraction * delta.dtheta));
            atCapture = predict(atCapture, captureNanos, interpolated);
        }

        Pose2d measurement = metricPose(visionPose);
        double[][] noise = diagonal(xyVariance, xyVariance,
                VISION_HEADING_STD_RADIANS * VISION_HEADING_STD_RADIANS);
        State observation = new State(captureNanos, atCapture.odometry, measurement, noise);
        if (lastVisionNanos == NO_SAMPLE) return confirmAnchor(observation);

        double[] error = difference(measurement, atCapture.estimate);
        double[][] inverse = inverse(add(atCapture.covariance, noise));
        if (inverse == null) {
            status = "Invalid pose covariance";
            return false;
        }
        double normalizedError = 0;
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                normalizedError += error[row] * inverse[row][column] * error[column];
            }
        }
        if (!Double.isFinite(normalizedError) || normalizedError > 11.345
                || Math.hypot(error[0], error[1]) > 0.75
                || Math.abs(error[2]) > Math.toRadians(45)) {
            if (allowReanchor && !isUsable(nowNanos)) return confirmAnchor(observation);
            anchorCandidate = null;
            status = "Rejected MT1 innovation";
            return false;
        }

        double[][] gain = multiply(atCapture.covariance, inverse);
        double[] corrected = {atCapture.estimate.getX(), atCapture.estimate.getY(),
                atCapture.estimate.getRotation().getRadians()};
        double[][] residualGain = diagonal(1, 1, 1);
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                corrected[row] += gain[row][column] * error[column];
                residualGain[row][column] -= gain[row][column];
            }
        }
        // Joseph form keeps the updated covariance symmetric and positive under rounding.
        double[][] covariance = add(propagateCovariance(residualGain, atCapture.covariance),
                propagateCovariance(gain, noise));
        Pose2d estimate = new Pose2d(corrected[0], corrected[1],
                new Rotation2d(wrap(corrected[2])));
        accept(new State(captureNanos, atCapture.odometry, estimate, covariance));
        status = "MT1 fused at capture time";
        return true;
    }

    private boolean confirmAnchor(State observation) {
        if (anchorCandidate != null && observation.timeNanos > anchorCandidate.timeNanos
                && observation.timeNanos - anchorCandidate.timeNanos <= ANCHOR_CONFIRMATION_NANOS) {
            Pose2d expected = anchorCandidate.estimate.plus(
                    observation.odometry.minus(anchorCandidate.odometry));
            double[] error = difference(observation.estimate, expected);
            if (Math.hypot(error[0], error[1]) <= ANCHOR_POSITION_AGREEMENT_METERS
                    && Math.abs(error[2]) <= ANCHOR_HEADING_AGREEMENT_RADIANS) {
                accept(observation);
                status = "Field pose anchored from two MT1 frames";
                return true;
            }
        }
        anchorCandidate = observation;
        status = "Waiting for a second consistent MT1 frame";
        return false;
    }

    private void accept(State corrected) {
        history.put(corrected.timeNanos, corrected);
        State previous = corrected;
        for (Map.Entry<Long, State> entry : history.tailMap(corrected.timeNanos, false).entrySet()) {
            State next = entry.getValue();
            previous = predict(previous, next.timeNanos, next.odometry);
            entry.setValue(previous);
        }
        lastVisionNanos = corrected.timeNanos;
        anchorCandidate = null;
    }

    private static State predict(State previous, long timeNanos, Pose2d odometry) {
        Transform2d delta = odometry.minus(previous.odometry);
        double dx = delta.getTranslation().getX();
        double dy = delta.getTranslation().getY();
        double heading = previous.estimate.getRotation().getRadians();
        double[][] jacobian = diagonal(1, 1, 1);
        jacobian[0][2] = -Math.sin(heading) * dx - Math.cos(heading) * dy;
        jacobian[1][2] = Math.cos(heading) * dx - Math.sin(heading) * dy;

        double dt = (timeNanos - previous.timeNanos) / 1e9;
        double xyVariance = 0.03 * 0.03 * dt + 0.02 * 0.02 * Math.hypot(dx, dy);
        double headingVariance = Math.pow(Math.toRadians(2), 2) * dt
                + Math.pow(Math.toRadians(1), 2) * Math.abs(delta.getRotation().getRadians());
        double[][] covariance = add(propagateCovariance(jacobian, previous.covariance),
                diagonal(xyVariance, xyVariance, headingVariance));
        return new State(timeNanos, odometry, previous.estimate.plus(delta), covariance);
    }

    public boolean isUsable(long nowNanos) {
        State sample = latest();
        return sample != null && lastVisionNanos != NO_SAMPLE
                && nowNanos - sample.timeNanos <= MAX_ODOMETRY_GAP_NANOS
                && (visionPaused ? trustedAtPause : nowNanos - lastVisionNanos <= MAX_VISION_COAST_NANOS)
                && positionStd(sample) <= MAX_POSITION_STD_METERS
                && headingStd(sample) <= MAX_HEADING_STD_RADIANS;
    }

    public Pose2D getEstimatedPose() {
        Pose2d pose = history.lastEntry().getValue().estimate;
        return new Pose2D(DistanceUnit.METER, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getRotation().getRadians());
    }

    public String getStatus(long nowNanos) {
        if (lastVisionNanos == NO_SAMPLE) return status;
        return isUsable(nowNanos) ? status : "Fused pose expired or uncertain: " + status;
    }

    public String getDiagnostics(long nowNanos) {
        if (lastVisionNanos == NO_SAMPLE) return "Field pose pending";
        State sample = latest();
        return String.format(java.util.Locale.US, "MT1 %.2f s | sigma %.1f cm / %.1f deg",
                (nowNanos - lastVisionNanos) / 1e9, positionStd(sample) * 100.0,
                Math.toDegrees(headingStd(sample)));
    }

    private State latest() {
        Map.Entry<Long, State> last = history.lastEntry();
        return last == null ? null : last.getValue();
    }

    private static double positionStd(State state) {
        return Math.sqrt(Math.max(state.covariance[0][0], state.covariance[1][1]));
    }

    private static double headingStd(State state) {
        return Math.sqrt(state.covariance[2][2]);
    }

    private static Pose2d metricPose(Pose2D pose) {
        return new Pose2d(pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
                new Rotation2d(pose.getHeading(AngleUnit.RADIANS)));
    }

    private static double[] difference(Pose2d measurement, Pose2d prediction) {
        return new double[]{measurement.getX() - prediction.getX(),
                measurement.getY() - prediction.getY(),
                wrap(measurement.getRotation().getRadians() - prediction.getRotation().getRadians())};
    }

    private static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double[][] diagonal(double x, double y, double heading) {
        return new double[][]{{x, 0, 0}, {0, y, 0}, {0, 0, heading}};
    }

    private static double[][] add(double[][] left, double[][] right) {
        double[][] sum = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                sum[row][column] = left[row][column] + right[row][column];
            }
        }
        return sum;
    }

    private static double[][] multiply(double[][] left, double[][] right) {
        double[][] product = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                for (int i = 0; i < 3; i++) {
                    product[row][column] += left[row][i] * right[i][column];
                }
            }
        }
        return product;
    }

    private static double[][] propagateCovariance(double[][] transform, double[][] covariance) {
        double[][] transpose = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                transpose[row][column] = transform[column][row];
            }
        }
        return multiply(multiply(transform, covariance), transpose);
    }

    private static double[][] inverse(double[][] matrix) {
        double[][] augmented = new double[3][6];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(matrix[i], 0, augmented[i], 0, 3);
            augmented[i][i + 3] = 1;
        }
        for (int column = 0; column < 3; column++) {
            int pivot = column;
            for (int row = column + 1; row < 3; row++) {
                if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) {
                    pivot = row;
                }
            }
            double[] pivotRow = augmented[pivot];
            double scale = pivotRow[column];
            if (!Double.isFinite(scale) || Math.abs(scale) < 1e-15) return null;
            augmented[pivot] = augmented[column];
            augmented[column] = pivotRow;
            for (int i = 0; i < 6; i++) pivotRow[i] /= scale;
            for (int row = 0; row < 3; row++) {
                if (row == column) continue;
                double factor = augmented[row][column];
                for (int i = 0; i < 6; i++) augmented[row][i] -= factor * pivotRow[i];
            }
        }
        double[][] inverse = new double[3][3];
        for (int i = 0; i < 3; i++) System.arraycopy(augmented[i], 3, inverse[i], 0, 3);
        return inverse;
    }
}
