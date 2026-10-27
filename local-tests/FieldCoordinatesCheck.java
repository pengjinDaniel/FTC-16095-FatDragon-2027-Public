package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Checks the centered HIVE map and the same live CELL selection used by Blue/Red. */
public class FieldCoordinatesCheck {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        for (Alliance alliance : Alliance.values()) {
            for (double x : new double[]{-60, -20, 0, 20, 60}) {
                for (double y : new double[]{-60, -20, 0, 20, 60}) {
                    CellCluster selected = CellCluster.forFieldX(alliance, x).get();
                    double[] target = VisionConstants.cellTargetFieldXYInches[selected.getTuningIndex()];
                    check(target[0] == (x >= 0 ? 16.52 : -16.52), "audience side follows signed X");
                    check(target[1] == (alliance == Alliance.RED ? -12.75 : 12.75), "alliance target Y");
                    for (int heading = -180; heading < 180; heading += 15) {
                        Pose2D inches = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
                        Pose2D centimeters = new Pose2D(DistanceUnit.CM, x * 2.54, y * 2.54,
                                AngleUnit.RADIANS, Math.toRadians(heading));
                        double expected = Math.IEEEremainder(
                                Math.toDegrees(Math.atan2(target[1] - y, target[0] - x))
                                        - VisionConstants.shooterYawDegrees - heading, 360);
                        double actual = VisionMath.coordinateYawErrorDegrees(
                                inches, target[0], target[1], VisionConstants.shooterYawDegrees);
                        check(Math.abs(Math.IEEEremainder(actual - expected, 360)) < 1e-8,
                                "coordinate yaw must use the centered map");
                        check(Math.abs(Math.IEEEremainder(actual - VisionMath.coordinateYawErrorDegrees(
                                centimeters, target[0], target[1], VisionConstants.shooterYawDegrees),
                                360)) < 1e-8, "pose units must not change aiming");
                    }
                }
            }
            CellCluster left = CellCluster.forFieldX(alliance, -.01).get();
            CellCluster right = CellCluster.forFieldX(alliance, .01).get();
            check(left != right, "crossing X=0 selects the current half without an RT latch");
            check(CellCluster.forFieldX(alliance, 0).get() == right, "center line belongs to audience half");
            check(!CellCluster.forFieldX(alliance, Double.NaN).isPresent(), "invalid X");
        }
        for (int id = 30; id <= 45; id++) {
            CellCluster cell = CellCluster.forTagId(id).get();
            check(cell.contains(id), "known tag belongs to its CELL");
            check(cell.isForAlliance(id < 38 ? Alliance.RED : Alliance.BLUE), "tag alliance");
        }
        check(!CellCluster.forTagId(29).isPresent() && !CellCluster.forTagId(46).isPresent(),
                "unknown tags have no CELL");
        check(Double.isNaN(VisionMath.coordinateYawErrorDegrees(null, 0, 0, 180)),
                "missing pose cannot aim");
        System.out.println("PASS FieldCoordinatesCheck: " + checks + " centered-map, units and live-selection assertions");
    }
}
