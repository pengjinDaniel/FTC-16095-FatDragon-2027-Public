package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.Arrays;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/** Host regression checks using the actual FTC SDK pose classes. */
public class VisionTagFrameCheck {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static Pose3D pose(DistanceUnit unit, double x, double y, double yaw) {
        return new Pose3D(new Position(unit, x, y, 0, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, yaw, 0, 0, 0));
    }
    public static void main(String[] args) {
        for (int first : new int[]{30, 34, 38, 42}) {
            Alliance alliance = first < 38 ? Alliance.RED : Alliance.BLUE;
            check(VisionMath.localizationTagRejection(
                    Arrays.asList(first, first + 1, first + 2, first + 3), alliance, 4) == null,
                    "four valid tags must remain supported");
            check(VisionMath.localizationTagRejection(
                    Arrays.asList(first, first + 1, first + 2, first + 3), alliance, 2) == null,
                    "MT1 may use a subset of detections");
            check(VisionMath.localizationTagRejection(
                    Arrays.asList(first, first + 1, 999), alliance, 3) != null,
                    "discarding an unknown ID must not validate an already fused pose");
            check(VisionMath.localizationTagRejection(
                    Arrays.asList(first, first), alliance, 2) != null,
                    "duplicate detections must not satisfy a two-tag requirement");
        }
        check(VisionMath.localizationTagRejection(Arrays.asList(38, 39, 42, 43), Alliance.BLUE, 4) == null,
                "same-alliance multiple CELL groups remain supported");
        check(VisionMath.localizationTagRejection(Arrays.asList(38, 39, 30), Alliance.BLUE, 3) != null,
                "mixed-alliance frame must be rejected");
        check(VisionMath.localizationTagRejection(Arrays.asList(38, 39), Alliance.BLUE, 4) != null,
                "fused count cannot exceed detection count");
        check(VisionMath.localizationTagRejection(Arrays.asList(38, null), Alliance.BLUE, 2) != null,
                "malformed detection");
        check(VisionMath.localizationTagRejection(Collections.emptyList(), Alliance.BLUE, 0) != null,
                "empty frame");
        check(VisionMath.localizationTagRejection(Arrays.asList(38, 39), Alliance.BLUE, -1) != null,
                "negative count");
        // Single-tag mode is not enabled: the existing minimum-count check still applies in Vision.
        Pose3D reference = pose(DistanceUnit.INCH, 0, 0, 179);
        double[] difference = VisionMath.fieldPoseDifference(
                reference, pose(DistanceUnit.MM, 76.2, 101.6, -179));
        check(difference != null && Math.abs(difference[0] - 5) < 1e-6,
                "position comparison must convert units");
        check(Math.abs(difference[1] - 2) < 1e-6, "179 to -179 is +2, not -358");
        check(VisionMath.fieldPoseDifference(null, reference) == null, "missing pose unavailable");
        check(VisionMath.fieldPoseDifference(reference, pose(DistanceUnit.INCH, Double.NaN, 0, 0)) == null,
                "non-finite diagnostic pose unavailable");
        System.out.println("PASS: all four CELL groups, MT1 subsets, unexpected/duplicate/mixed IDs, count mismatch, missing data, pose units and yaw wrap");
    }
}
