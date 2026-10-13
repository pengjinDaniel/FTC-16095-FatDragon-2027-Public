package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.*;

public class CellSelectionLatchCheck {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static Pose2D fieldPose(double mapX) {
        return FieldCoordinates.fromCenteredHiveMap(
                new Pose2D(DistanceUnit.INCH,mapX,0,AngleUnit.DEGREES,0));
    }
    private static double error(double x, CellCluster cell) {
        double[] target = VisionConstants.cellTargetFieldXYInches[cell.getTuningIndex()];
        return VisionMath.coordinateYawErrorDegrees(
                fieldPose(x),
                target[0], target[1], VisionConstants.shooterYawDegrees);
    }
    public static void main(String[] args) {
        for (Alliance alliance : Alliance.values()) {
            CellCluster left = CellCluster.forFieldPose(alliance, fieldPose(-.01)).get();
            CellCluster right = CellCluster.forFieldPose(alliance, fieldPose(.01)).get();
            CellSelectionLatch latch = new CellSelectionLatch();
            latch.setHeld(true);
            // A frame with invalid geometry must not capture a target.
            latch.select(left);
            check(latch.getLatched() == null, "selection before valid geometry must not latch");
            latch.confirm(null);
            check(latch.getLatched() == null, "missing target must not latch");
            latch.confirm(left);
            double initialError = error(-.01, left);
            for (int i = 0; i < 200; i++) {
                latch.setHeld(true);
                double x = (i % 2 == 0) ? .01 : -.01;
                CellCluster selected = latch.select(CellCluster.forFieldPose(alliance, fieldPose(x)).get());
                check(selected == left, "centerline jitter must not switch CELL during RT");
                check(Math.abs(error(x, selected) - initialError) < .1,
                        "0.5mm pose jitter must not cause the original 108 degree jump");
                latch.confirm(selected);
            }
            check(latch.select(null) == left, "missing candidate does not erase the held CELL");
            check(Math.abs(error(-40, left) - initialError) > 10,
                    "bearing must still update as the robot translates");
            latch.setHeld(false); // RT release or B cancellation
            check(latch.getLatched() == null && latch.select(right) == right, "release clears target");
            latch.confirm(right);
            check(latch.getLatched() == null, "INIT/manual must not lock target");
            latch.setHeld(true);
            latch.confirm(right);
            check(latch.select(left) == right, "new RT press must capture the new side");
        }
        System.out.println("PASS: both alliances; 200 centerline crossings; fresh valid capture; live bearing; release/B and reacquisition");
    }
}
