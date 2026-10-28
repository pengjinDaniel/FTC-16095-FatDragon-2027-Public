package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Optional;

final class VisionMath {
    private static final double COINCIDENT_POSE_TOLERANCE = 1e-4;
    // Allow small floating-point error at the configured tilt boundary.
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
        double r00 = Math.cos(yaw) * Math.cos(pitch);
        double r10 = Math.sin(yaw) * Math.cos(pitch);
        double r20 = -Math.sin(pitch);
        double r02 = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll)
                + Math.sin(yaw) * Math.sin(roll);
        double r22 = Math.cos(pitch) * Math.cos(roll);
        // A level robot has R_raw * up = Ry(phi) * up. This gives one tilt from
        // the full visual orientation, without the two floor roots or a Pinpoint reference.
        double phi = Math.atan2(r02, r22);
        double levelError = Math.toDegrees(Math.acos(
                Math.max(-1.0, Math.min(1.0, Math.hypot(r02, r22)))));
        result.candidatePhiDegrees[0] = Math.toDegrees(phi);
        result.candidateLevelErrorsDegrees[0] = levelError;
        if (Math.abs(Math.toDegrees(phi)) > maxTiltDegrees + NUMERICAL_TILT_TOLERANCE_DEGREES) {
            result.status = "HIVE tilt exceeds limit";
            return result;
        }
        if (levelError > levelToleranceDegrees) {
            result.status = "Robot level check failed";
            return result;
        }
        double cos = Math.cos(phi);
        double sin = Math.sin(phi);
        double x = position.x * cos - position.z * sin + pivotHeightInches * sin;
        if (Math.abs(x) <= COINCIDENT_POSE_TOLERANCE) x = 0.0;
        double heading = Math.atan2(r10, cos * r00 - sin * r20);
        if (!Double.isFinite(x) || !Double.isFinite(heading)) {
            result.status = "Invalid corrected Limelight pose";
            return result;
        }
        result.correction = new HiveCorrection(
                new Pose2D(DistanceUnit.INCH, x, position.y, AngleUnit.RADIANS, heading),
                -Math.toDegrees(phi), levelError);
        result.candidates[0] = result.correction;
        result.usableCandidateCount = 1;
        result.status = "Corrected Limelight field pose ready";
        return result;
    }

    static double[] cellTargetForHiveTilt(
            double[] referenceXY, double referenceHeightInches, double pivotHeightInches,
            double referenceTiltDegrees, double hiveAngleDegrees
    ) {
        if (referenceXY == null || referenceXY.length < 2
                || !Double.isFinite(referenceXY[0]) || !Double.isFinite(referenceXY[1])
                || !Double.isFinite(referenceHeightInches) || !Double.isFinite(pivotHeightInches)
                || !Double.isFinite(referenceTiltDegrees) || referenceTiltDegrees < 0.0
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

    static double[] tagClusterCenterForHiveTilt(CellCluster cluster, double hiveAngleDegrees) {
        double[][] centers = VisionConstants.tagClusterCenterFieldXYInches;
        if (cluster == null || centers == null || cluster.getTuningIndex() >= centers.length) {
            return null;
        }
        double[] center = centers[cluster.getTuningIndex()];
        if (center == null || center.length < 2) return null;
        double angle = Double.isNaN(hiveAngleDegrees)
                ? -Math.copySign(VisionConstants.cellTargetReferenceTiltDegrees, center[0])
                : hiveAngleDegrees;
        return cellTargetForHiveTilt(center, VisionConstants.tagClusterCenterReferenceHeightInches,
                VisionConstants.hivePivotHeightInches, 0.0, angle);
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
