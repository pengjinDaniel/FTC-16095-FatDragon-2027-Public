package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.List;

/** Size-based POLLEN ranging and camera-relative approach, without a fixed target height. */
public final class PollenChaseController {
    public static final double POLLEN_RADIUS_CM = 3.55;
    public static final double STOP_GAP_CM = 3.0;
    public static final double MAX_FRAME_AGE_MS = 100.0;
    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;
    // Nominal full-frame Limelight 3A optics. Pipeline resolution and zoom must match.
    public static final double FOCAL_X_PIXELS = IMAGE_WIDTH / (2.0 * Math.tan(Math.toRadians(54.5 / 2.0)));
    public static final double FOCAL_Y_PIXELS = IMAGE_HEIGHT / (2.0 * Math.tan(Math.toRadians(42.0 / 2.0)));

    private static final double DRIVE_KP = 0.012;
    private static final double MIN_FORWARD_POWER = 0.07;
    private static final double MAX_FORWARD_POWER = 0.50;
    private static final double TURN_KP = 0.012;
    private static final double MIN_TURN_POWER = 0.07;
    private static final double MAX_TURN_POWER = 0.18;
    private static final double TURN_DEADBAND_DEGREES = 2.0;
    private static final double DRIVE_ALIGNMENT_DEGREES = 8.0;
    private static final double MAX_TARGET_GAP_CM = 300.0;
    private static final int REQUIRED_FRESH_FRAMES = 3;

    public enum State {
        CONFIGURATION_REQUIRED, NO_TARGET, STALE_FRAME, FROZEN_FRAME,
        INVALID_CONTOUR, INVALID_GEOMETRY, ACQUIRING, ALIGNING, APPROACHING, ARRIVED
    }

    private final double cameraForwardCm;
    private final double cameraRightCm;
    private final double cameraHeightCm;
    private final double cameraPitchUpDegrees;
    private final double cameraYawDegrees;
    private final double approachEdgeFromCenterCm;
    private final double rangeScale;
    private final double approachSign;
    private final String configurationError;
    private double lastFrameTimestamp = Double.NaN;
    private long lastFrameChangeNanos;
    private int freshFrames;
    private boolean arrived;
    private State state;
    private double gapCm = Double.NaN;
    private double contourWidthPixels = Double.NaN;
    private double contourHeightPixels = Double.NaN;
    private double cameraRangeCm = Double.NaN;
    private double ballHeightCm = Double.NaN;
    private double bearingDegrees = Double.NaN;
    private double forwardPower;
    private double turnPower;

    public PollenChaseController(double cameraForwardCm, double cameraRightCm, double cameraHeightCm,
                                 double cameraPitchUpDegrees, double cameraYawDegrees,
                                 double approachEdgeFromCenterCm, double rangeScale) {
        this.cameraForwardCm = cameraForwardCm;
        this.cameraRightCm = cameraRightCm;
        this.cameraHeightCm = cameraHeightCm;
        this.cameraPitchUpDegrees = cameraPitchUpDegrees;
        this.cameraYawDegrees = cameraYawDegrees;
        this.approachEdgeFromCenterCm = approachEdgeFromCenterCm;
        this.rangeScale = rangeScale;
        approachSign = Math.cos(Math.toRadians(cameraYawDegrees)) < 0.0 ? -1.0 : 1.0;
        configurationError = validateConfiguration();
        reset();
    }

    private String validateConfiguration() {
        if (!Double.isFinite(cameraForwardCm) || !Double.isFinite(cameraRightCm)
                || !Double.isFinite(cameraHeightCm) || cameraHeightCm < 0.0) {
            return "Measure camera Forward/Right/Height in cm";
        }
        if (!Double.isFinite(cameraPitchUpDegrees) || Math.abs(cameraPitchUpDegrees) >= 85.0) {
            return "Measure CAMERA_PITCH_UP_DEGREES (up is positive)";
        }
        if (!Double.isFinite(cameraYawDegrees)
                || Math.abs(Math.sin(Math.toRadians(cameraYawDegrees))) > 0.001) {
            return "This forward/reverse test requires camera yaw 0 or 180 degrees";
        }
        if (!Double.isFinite(approachEdgeFromCenterCm) || approachEdgeFromCenterCm < 0.0) {
            return "Measure APPROACH_EDGE_FROM_CENTER_CM (rear edge for yaw 180)";
        }
        if (!Double.isFinite(rangeScale) || rangeScale <= 0.0) {
            return "RANGE_SCALE must be finite and positive";
        }
        return null;
    }

    public void reset() {
        lastFrameTimestamp = Double.NaN;
        lastFrameChangeNanos = 0L;
        freshFrames = 0;
        arrived = false;
        gapCm = Double.NaN;
        contourWidthPixels = Double.NaN;
        contourHeightPixels = Double.NaN;
        cameraRangeCm = Double.NaN;
        ballHeightCm = Double.NaN;
        bearingDegrees = Double.NaN;
        forwardPower = 0.0;
        turnPower = 0.0;
        state = configurationError == null ? State.NO_TARGET : State.CONFIGURATION_REQUIRED;
    }

    /**
     * tx/ty are principal-pixel angles (right/up positive); corners are full-frame
     * image pixels. A known sphere diameter supplies range independently of its height.
     */
    public void update(boolean validTarget, double frameTimestamp, double frameAgeMs,
                       double txDegrees, double tyDegrees, List<List<Double>> corners, long nowNanos) {
        forwardPower = 0.0;
        turnPower = 0.0;
        if (arrived) {
            state = State.ARRIVED;
            return;
        }
        gapCm = Double.NaN;
        contourWidthPixels = Double.NaN;
        contourHeightPixels = Double.NaN;
        cameraRangeCm = Double.NaN;
        ballHeightCm = Double.NaN;
        bearingDegrees = Double.NaN;
        if (configurationError != null) {
            reject(State.CONFIGURATION_REQUIRED);
            return;
        }
        if (!validTarget || !Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0
                || !Double.isFinite(txDegrees) || !Double.isFinite(tyDegrees)
                || Math.abs(txDegrees) >= 80.0 || Math.abs(tyDegrees) >= 80.0) {
            reject(State.NO_TARGET);
            return;
        }
        if (!Double.isFinite(frameAgeMs) || frameAgeMs < 0.0 || frameAgeMs > MAX_FRAME_AGE_MS) {
            reject(State.STALE_FRAME);
            return;
        }
        boolean newFrame = frameTimestamp != lastFrameTimestamp;
        if (newFrame) {
            if (Double.isFinite(lastFrameTimestamp) && frameTimestamp < lastFrameTimestamp) {
                freshFrames = 0;
            }
            lastFrameTimestamp = frameTimestamp;
            lastFrameChangeNanos = nowNanos;
        }
        double unchangedMs = (nowNanos - lastFrameChangeNanos) / 1_000_000.0;
        if (unchangedMs < 0.0 || unchangedMs + frameAgeMs > MAX_FRAME_AGE_MS) {
            reject(State.FROZEN_FRAME);
            return;
        }
        if (corners == null || corners.size() < 4) {
            reject(State.INVALID_CONTOUR);
            return;
        }
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (List<Double> point : corners) {
            if (point == null || point.size() != 2 || point.get(0) == null || point.get(1) == null
                    || !Double.isFinite(point.get(0)) || !Double.isFinite(point.get(1))) {
                reject(State.INVALID_CONTOUR);
                return;
            }
            minX = Math.min(minX, point.get(0));
            maxX = Math.max(maxX, point.get(0));
            minY = Math.min(minY, point.get(1));
            maxY = Math.max(maxY, point.get(1));
        }
        double width = maxX - minX, height = maxY - minY;
        if (minX <= 2.0 || minY <= 2.0 || maxX >= IMAGE_WIDTH - 2.0
                || maxY >= IMAGE_HEIGHT - 2.0 || width < 3.0 || height < 3.0) {
            reject(State.INVALID_CONTOUR); // Never range from an image-clipped ball.
            return;
        }

        contourWidthPixels = width;
        contourHeightPixels = height;
        double halfWidth = width / (2.0 * FOCAL_X_PIXELS);
        double halfHeight = height / (2.0 * FOCAL_Y_PIXELS);
        double tanTx = Math.tan(Math.toRadians(txDegrees));
        double tanTy = Math.tan(Math.toRadians(tyDegrees));
        double leftAngle = Math.atan(tanTx - halfWidth);
        double rightAngle = Math.atan(tanTx + halfWidth);
        double bottomAngle = Math.atan(tanTy - halfHeight);
        double topAngle = Math.atan(tanTy + halfHeight);
        double centerXAngle = (leftAngle + rightAngle) / 2.0;
        double centerYAngle = (bottomAngle + topAngle) / 2.0;
        // Tangent rays to a sphere give R / sin(angular radius), without a floor-plane assumption.
        double depthX = POLLEN_RADIUS_CM * Math.cos(centerXAngle)
                / Math.sin((rightAngle - leftAngle) / 2.0);
        double depthY = POLLEN_RADIUS_CM * Math.cos(centerYAngle)
                / Math.sin((topAngle - bottomAngle) / 2.0);
        if (!Double.isFinite(depthX) || !Double.isFinite(depthY) || Math.min(depthX, depthY) <= 0.0
                || Math.max(depthX, depthY) / Math.min(depthX, depthY) > 1.5) {
            reject(State.INVALID_CONTOUR);
            return;
        }
        // Prefer the nearer estimate if a partial color contour shrinks one dimension.
        double opticalForward = Math.min(depthX, depthY) * rangeScale;
        double opticalRight = opticalForward * Math.tan(centerXAngle);
        double opticalUp = opticalForward * Math.tan(centerYAngle);
        cameraRangeCm = Math.sqrt(opticalForward * opticalForward
                + opticalRight * opticalRight + opticalUp * opticalUp);

        double pitch = Math.toRadians(cameraPitchUpDegrees);
        double yaw = Math.toRadians(cameraYawDegrees);
        double cameraHorizontal = Math.cos(pitch) * opticalForward - Math.sin(pitch) * opticalUp;
        double forward = cameraForwardCm + Math.cos(yaw) * cameraHorizontal - Math.sin(yaw) * opticalRight;
        double right = cameraRightCm + Math.sin(yaw) * cameraHorizontal + Math.cos(yaw) * opticalRight;
        ballHeightCm = cameraHeightCm + Math.sin(pitch) * opticalForward + Math.cos(pitch) * opticalUp;
        double alongApproach = approachSign * forward;
        double candidateGap = alongApproach - approachEdgeFromCenterCm - POLLEN_RADIUS_CM;
        if (!Double.isFinite(candidateGap) || !Double.isFinite(right) || !Double.isFinite(ballHeightCm)
                || alongApproach <= 0.0 || candidateGap > MAX_TARGET_GAP_CM) {
            reject(State.INVALID_GEOMETRY);
            return;
        }
        gapCm = candidateGap;
        bearingDegrees = Math.toDegrees(Math.atan2(approachSign * right, alongApproach));
        if (newFrame) freshFrames = Math.min(REQUIRED_FRESH_FRAMES, freshFrames + 1);
        if (freshFrames < REQUIRED_FRESH_FRAMES) {
            state = State.ACQUIRING;
            return;
        }
        if (gapCm <= STOP_GAP_CM + 1e-6) {
            arrived = true;
            state = State.ARRIVED;
            return;
        }

        double absoluteBearing = Math.abs(bearingDegrees);
        if (absoluteBearing > TURN_DEADBAND_DEGREES) {
            turnPower = Math.copySign(Math.min(MAX_TURN_POWER,
                    Math.max(MIN_TURN_POWER, absoluteBearing * TURN_KP)), bearingDegrees);
        }
        if (absoluteBearing <= DRIVE_ALIGNMENT_DEGREES) {
            forwardPower = approachSign * Math.min(MAX_FORWARD_POWER,
                    Math.max(MIN_FORWARD_POWER, (gapCm - STOP_GAP_CM) * DRIVE_KP));
            state = State.APPROACHING;
        } else {
            state = State.ALIGNING;
        }
    }

    private void reject(State reason) {
        freshFrames = 0;
        state = reason;
    }

    public State getState() { return state; }
    public String getConfigurationError() { return configurationError; }
    public double getGapCm() { return gapCm; }
    public double getContourWidthPixels() { return contourWidthPixels; }
    public double getContourHeightPixels() { return contourHeightPixels; }
    public double getCameraRangeCm() { return cameraRangeCm; }
    public double getBallHeightCm() { return ballHeightCm; }
    public double getBearingDegrees() { return bearingDegrees; }
    public double getForwardPower() { return forwardPower; }
    public double getTurnPower() { return turnPower; }
}
