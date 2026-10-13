package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Optional;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

final class VisionMath {
    private VisionMath() {
    }

    // Filtering the detection list cannot remove a tag from an already fused MT1 pose.
    // Reject unexpected contributors instead of treating that pose as alliance-only.
    static String localizationTagRejection(List<Integer> ids, Alliance alliance, int fusedCount) {
        if (ids == null || ids.isEmpty()) return "No AprilTag detections";
        Set<Integer> unique = new HashSet<>();
        for (Integer id : ids) {
            if (id == null) return "Malformed AprilTag detection";
            if (!unique.add(id)) return "Duplicate AprilTag ID " + id;
            Optional<CellCluster> cluster = CellCluster.forTagId(id);
            if (!cluster.isPresent()) return "Unexpected localization Tag " + id;
            if (!cluster.get().isForAlliance(alliance)) return "Mixed-alliance HIVE tags";
        }
        if (fusedCount < 0 || fusedCount > unique.size()) {
            return "MT1 tag count inconsistent with detections";
        }
        return null;
    }

    // Diagnostics only: individual planar-tag solutions may be ambiguous. Never use
    // their disagreement to choose a new heading or reject an otherwise valid MT1.
    static double[] fieldPoseDifference(Pose3D reference, Pose3D individual) {
        if (reference == null || individual == null || reference.getOrientation() == null
                || individual.getOrientation() == null) return null;
        Optional<Position> a = getRobotSpacePositionInches(reference);
        Optional<Position> b = getRobotSpacePositionInches(individual);
        if (!a.isPresent() || !b.isPresent()) return null;
        double distance = Math.hypot(b.get().x - a.get().x, b.get().y - a.get().y);
        double yaw = Math.IEEEremainder(individual.getOrientation().getYaw(AngleUnit.DEGREES)
                - reference.getOrientation().getYaw(AngleUnit.DEGREES), 360.0);
        return Double.isFinite(distance) && Double.isFinite(yaw)
                ? new double[]{distance, yaw} : null;
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
        if (radius < pivotHeightInches - 1e-9) {
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
            if (Math.abs(Math.toDegrees(phi)) > maxTiltDegrees + 1e-9) {
                continue;
            }
            candidatesWithinTilt++;
            if (levelError > levelToleranceDegrees) {
                continue;
            }
            double x = position.x * cos - position.z * sin + pivotHeightInches * sin;
            double heading = Math.atan2(r10, cos * r00 - sin * r20);
            if (!Double.isFinite(x) || !Double.isFinite(heading)) {
                continue;
            }
            result.usableCandidateCount++;
            HiveCorrection correction = new HiveCorrection(
                    new Pose2D(DistanceUnit.INCH, x, position.y, AngleUnit.RADIANS, heading),
                    -Math.toDegrees(phi), levelError
            );
            if (result.correction == null || levelError < result.correction.levelErrorDegrees) {
                result.correction = correction;
            }
        }
        result.status = result.correction != null ? "Corrected MT1 field pose ready"
                : candidatesWithinTilt == 0 ? "HIVE tilt exceeds limit" : "Robot level check failed";
        return result;
    }

    static final class HiveSolveResult {
        final double[] candidatePhiDegrees = {Double.NaN, Double.NaN};
        final double[] candidateLevelErrorsDegrees = {Double.NaN, Double.NaN};
        int usableCandidateCount;
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
