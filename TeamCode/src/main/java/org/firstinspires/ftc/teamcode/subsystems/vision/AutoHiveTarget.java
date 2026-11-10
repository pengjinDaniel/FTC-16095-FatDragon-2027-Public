package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Observes one CELL without writing vision corrections into Pedro's Pinpoint localizer. */
public class AutoHiveTarget {
    private final Vision vision;
    private final CellCluster cluster;
    private HiveState state = HiveState.UNKNOWN;
    private double distanceCm = Double.NaN;
    private double yawErrorDegrees = Double.NaN;
    private String status = "Waiting for a HIVE observation";

    public AutoHiveTarget(Vision vision, CellCluster cluster) {
        if (cluster == null || !cluster.isForAlliance(vision.getAlliance())) {
            throw new IllegalArgumentException("The CELL must belong to the vision alliance");
        }
        this.vision = vision;
        this.cluster = cluster;
    }

    public void update(Pose pedroPose) {
        state = HiveState.UNKNOWN;
        distanceCm = Double.NaN;
        yawErrorDegrees = Double.NaN;
        status = vision.getPoseStatus();
        if (pedroPose == null || !Double.isFinite(pedroPose.x())
                || !Double.isFinite(pedroPose.y()) || !Double.isFinite(pedroPose.heading())) {
            status = "Invalid Pedro pose";
            return;
        }
        Pose2D fieldPose = toHiveFieldPose(pedroPose);
        double[] target = fieldTarget();
        int requiredTagCount = Math.max(1, VisionConstants.minimumBotposeTagCount);
        if (vision.getRawRobotFieldPose().isPresent()
                && vision.getBotposeTagCount() >= requiredTagCount
                && vision.getUsableTagCount() >= requiredTagCount) {
            VisionMath.HiveSolveResult solve = VisionMath.solveHiveFlatPose(
                    vision.getRawRobotFieldPose().get(), VisionConstants.hivePivotHeightInches,
                    VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees);
            VisionMath.HiveCorrection correction = solve.correction;
            status = solve.status;
            if (correction != null && target != null) {
                // Read HIVE tilt independently of Pinpoint translation drift.
                double headingDifference = Math.abs(Math.IEEEremainder(
                        correction.robotPose.getHeading(AngleUnit.DEGREES)
                                - fieldPose.getHeading(AngleUnit.DEGREES), 360.0));
                if (headingDifference < DriveConstants.maximumVisionHeadingDifferenceDegrees) {
                    double hiveAngleDegrees = correction.hiveAngleDegrees;
                    state = Math.abs(hiveAngleDegrees) <= VisionConstants.highHiveTiltThresholdDegrees
                            ? HiveState.MIDDLE
                            : -Math.sin(Math.toRadians(hiveAngleDegrees)) * target[0] > 0.0
                            ? HiveState.HIGH : HiveState.LOW;
                } else {
                    status = "HIVE heading disagrees with Pedro pose";
                }
            }
        }
        // Aim and shot calibration share the upstream CELL-opening center.
        // Only a current observation can report HIGH; odometry supplies the yaw error.
        if (target != null) {
            yawErrorDegrees = VisionMath.coordinateYawErrorDegrees(fieldPose,
                    target[0], target[1], VisionConstants.shooterYawDegrees);
            distanceCm = DistanceUnit.INCH.toCm(Math.hypot(
                    target[0] - fieldPose.getX(DistanceUnit.INCH),
                    target[1] - fieldPose.getY(DistanceUnit.INCH)));
        }
    }

    private double[] fieldTarget() {
        double[][] targets = VisionConstants.cellTargetFieldXYInches;
        if (targets == null || cluster.getTuningIndex() >= targets.length) return null;
        double[] target = targets[cluster.getTuningIndex()];
        return target != null && target.length >= 2
                && Double.isFinite(target[0]) && Double.isFinite(target[1]) ? target : null;
    }

    // The custom HIVE fmap shares this route's axes, with its origin at field center.
    public static Pose2D toHiveFieldPose(Pose pose) {
        return new Pose2D(DistanceUnit.INCH, pose.x() - 72.0, pose.y() - 72.0,
                AngleUnit.RADIANS, pose.heading());
    }

    public double getYawErrorDegrees() { return yawErrorDegrees; }

    public HiveState getState() { return state; }
    public double getDistanceCm() { return distanceCm; }
    public String getStatus() { return status; }
    public CellCluster getCluster() { return cluster; }
}
