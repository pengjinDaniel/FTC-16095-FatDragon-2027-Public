package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Optional;

final class VisionMath {
    private static final double COINCIDENT_POSE_TOLERANCE = 1e-4;
    // Map rounding is amplified by the floor constraint near X=0.
    private static final double NUMERICAL_TILT_TOLERANCE_DEGREES = 0.01;

    private VisionMath() {
    }

    // The fmap stores a flat HIVE; phi is the rotation from the true robot pose to MT1.
    static HiveSolveResult solveHiveFlatPose(
            Pose3D rawPose, double pivotHeightInches,
            double maxTiltDegrees, double levelToleranceDegrees
    ) {
        HiveSolveResult result = new HiveSolveResult();
        if (rawPose == null || rawPose.getPosition() == null
                || rawPose.getPosition().unit == null || rawPose.getOrientation() == null) {
            result.status = "Missing MT1 3D pose";
            return result;
        }
        if (!Double.isFinite(pivotHeightInches) || pivotHeightInches <= 0.0
                || !Double.isFinite(maxTiltDegrees) || maxTiltDegrees <= 0.0
                || maxTiltDegrees >= 90.0 || !Double.isFinite(levelToleranceDegrees)
                || levelToleranceDegrees < 0.0 || levelToleranceDegrees >= 90.0) {
            result.status = "Invalid HIVE correction parameters";
            return result;
        }
        Position position = rawPose.getPosition().toUnit(DistanceUnit.INCH);
        double yaw = rawPose.getOrientation().getYaw(AngleUnit.RADIANS);
        double pitch = rawPose.getOrientation().getPitch(AngleUnit.RADIANS);
        double roll = rawPose.getOrientation().getRoll(AngleUnit.RADIANS);
        if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                || !Double.isFinite(position.z) || !Double.isFinite(yaw)
                || !Double.isFinite(pitch) || !Double.isFinite(roll)) {
            result.status = "Non-finite MT1 3D pose";
            return result;
        }
        double radius = Math.hypot(position.x, position.z - pivotHeightInches);
        // Permit submicron rounding of the CAD/map coordinates at the degenerate center line.
        if (radius < pivotHeightInches - 1e-5) {
            result.status = "No real HIVE tilt solution";
            return result;
        }
        double alpha = Math.asin(Math.max(-1.0, -pivotHeightInches / radius));
        double base = Math.atan2(position.z - pivotHeightInches, position.x);
        double[] roots = {alpha - base, Math.PI - alpha - base};
        double r00 = Math.cos(yaw) * Math.cos(pitch);
        double r10 = Math.sin(yaw) * Math.cos(pitch);
        double r20 = -Math.sin(pitch);
        double r02 = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll)
                + Math.sin(yaw) * Math.sin(roll);
        double r22 = Math.cos(pitch) * Math.cos(roll);
        int candidatesWithinTilt = 0;
        for (int i = 0; i < roots.length; i++) {
            double phi = Math.atan2(Math.sin(roots[i]), Math.cos(roots[i]));
            result.candidatePhiDegrees[i] = Math.toDegrees(phi);
            double cos = Math.cos(phi);
            double sin = Math.sin(phi);
            // The full robot-up vector checks both pitch and roll, including +/-90 yaw.
            double upZ = sin * r02 + cos * r22;
            double levelError = Math.toDegrees(Math.acos(Math.max(-1.0, Math.min(1.0, upZ))));
            result.candidateLevelErrorsDegrees[i] = levelError;
            if (Math.abs(Math.toDegrees(phi)) > maxTiltDegrees + NUMERICAL_TILT_TOLERANCE_DEGREES) {
                continue;
            }
            candidatesWithinTilt++;
            if (levelError > levelToleranceDegrees) {
                continue;
            }
            double x = position.x * cos - position.z * sin + pivotHeightInches * sin;
            if (Math.abs(x) <= COINCIDENT_POSE_TOLERANCE) x = 0.0;
            double heading = Math.atan2(r10, cos * r00 - sin * r20);
            if (!Double.isFinite(x) || !Double.isFinite(heading)) {
                continue;
            }
            HiveCorrection correction = new HiveCorrection(
                    new Pose2D(DistanceUnit.INCH, x, position.y, AngleUnit.RADIANS, heading),
                    -Math.toDegrees(phi), levelError
            );
            HiveCorrection first = result.candidates[0];
            if (first != null
                    && Math.abs(first.robotPose.getX(DistanceUnit.INCH) - x)
                    <= COINCIDENT_POSE_TOLERANCE
                    && Math.abs(Math.IEEEremainder(
                    first.robotPose.getHeading(AngleUnit.DEGREES) - Math.toDegrees(heading), 360.0))
                    <= COINCIDENT_POSE_TOLERANCE) {
                // Coincident floor roots meet at X=0; preserve deterministic CELL selection there.
                double firstHeading = first.robotPose.getHeading(AngleUnit.DEGREES);
                double mergedHeading = firstHeading + 0.5 * Math.IEEEremainder(
                        Math.toDegrees(heading) - firstHeading, 360.0);
                result.candidates[0] = new HiveCorrection(
                        new Pose2D(DistanceUnit.INCH, 0.0, position.y, AngleUnit.DEGREES, mergedHeading),
                        (first.hiveAngleDegrees + correction.hiveAngleDegrees) / 2.0,
                        Math.min(first.levelErrorDegrees, levelError));
            } else {
                result.candidates[result.usableCandidateCount++] = correction;
            }
        }
        result.status = result.usableCandidateCount > 0 ? "Ambiguous HIVE pose; waiting for Pinpoint"
                : candidatesWithinTilt == 0 ? "HIVE tilt exceeds limit" : "Robot level check failed";
        selectHiveCorrection(result, null);
        return result;
    }

    static HiveCorrection selectHiveCorrection(HiveSolveResult result, Pose2D referencePose) {
        if (result.usableCandidateCount == 0) return null;
        result.correction = null;
        if (result.usableCandidateCount == 1) {
            result.correction = result.candidates[0];
        } else if (referencePose != null
                && Double.isFinite(referencePose.getX(DistanceUnit.INCH))
                && Double.isFinite(referencePose.getY(DistanceUnit.INCH))
                && Double.isFinite(referencePose.getHeading(AngleUnit.DEGREES))) {
            double[] distances = new double[2];
            for (int i = 0; i < distances.length; i++) {
                Pose2D candidate = result.candidates[i].robotPose;
                distances[i] = Math.hypot(
                        candidate.getX(DistanceUnit.INCH) - referencePose.getX(DistanceUnit.INCH),
                        candidate.getY(DistanceUnit.INCH) - referencePose.getY(DistanceUnit.INCH));
            }
            if (Math.abs(distances[0] - distances[1]) > COINCIDENT_POSE_TOLERANCE) {
                result.correction = result.candidates[distances[0] < distances[1] ? 0 : 1];
            }
        }
        result.status = result.correction == null ? "Ambiguous HIVE pose; waiting for Pinpoint"
                : "Corrected MT1 field pose ready";
        return result.correction;
    }

    static double[] cellTargetForHiveTilt(
            double[] referenceXY, double referenceHeightInches, double pivotHeightInches,
            double referenceTiltDegrees, double hiveAngleDegrees
    ) {
        if (referenceXY == null || referenceXY.length < 2
                || !Double.isFinite(referenceXY[0]) || !Double.isFinite(referenceXY[1])
                || !Double.isFinite(referenceHeightInches) || !Double.isFinite(pivotHeightInches)
                || !Double.isFinite(referenceTiltDegrees) || referenceTiltDegrees <= 0.0
                || referenceTiltDegrees >= 90.0
                || (!Double.isNaN(hiveAngleDegrees) && !Double.isFinite(hiveAngleDegrees))) {
            return null;
        }
        double referenceAngle = -Math.copySign(referenceTiltDegrees, referenceXY[0]);
        double currentAngle = Double.isNaN(hiveAngleDegrees) ? referenceAngle : hiveAngleDegrees;
        double delta = Math.toRadians(currentAngle - referenceAngle);
        return new double[]{
                referenceXY[0] * Math.cos(delta)
                        + (referenceHeightInches - pivotHeightInches) * Math.sin(delta),
                referenceXY[1]
        };
    }

    static final class HiveSolveResult {
        final double[] candidatePhiDegrees = {Double.NaN, Double.NaN};
        final double[] candidateLevelErrorsDegrees = {Double.NaN, Double.NaN};
        int usableCandidateCount;
        final HiveCorrection[] candidates = new HiveCorrection[2];
        HiveCorrection correction;
        String status;
    }

    static double coordinateYawErrorDegrees(
            Pose2D robotPose, double targetXInches, double targetYInches,
            double shooterYawDegrees
    ) {
        if (robotPose == null || !Double.isFinite(targetXInches)
                || !Double.isFinite(targetYInches) || !Double.isFinite(shooterYawDegrees)) {
            return Double.NaN;
        }
        double dx = targetXInches - robotPose.getX(DistanceUnit.INCH);
        double dy = targetYInches - robotPose.getY(DistanceUnit.INCH);
        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        if (!Double.isFinite(dx) || !Double.isFinite(dy)
                || !Double.isFinite(heading) || Math.hypot(dx, dy) <= 1e-6) {
            return Double.NaN;
        }
        double error = Math.atan2(dy, dx) - Math.toRadians(shooterYawDegrees) - heading;
        return Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error)));
    }

    static final class HiveCorrection {
        final Pose2D robotPose;
        final double hiveAngleDegrees;
        final double levelErrorDegrees;

        HiveCorrection(Pose2D robotPose, double hiveAngleDegrees, double levelErrorDegrees) {
            this.robotPose = robotPose;
            this.hiveAngleDegrees = hiveAngleDegrees;
            this.levelErrorDegrees = levelErrorDegrees;
        }
    }

    static Optional<Position> getRobotSpacePositionInches(Pose3D pose) {
        if (pose == null || pose.getPosition() == null || pose.getPosition().unit == null) {
            return Optional.empty();
        }

        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
        if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                || !Double.isFinite(position.z)) {
            return Optional.empty();
        }

        return Optional.of(position);
    }
}
