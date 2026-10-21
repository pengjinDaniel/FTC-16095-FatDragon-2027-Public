package org.firstinspires.ftc.teamcode.subsystems.vision;

public class VisionConstants {
    public static String limelightName = "limelight";
    public static int bluePipelineIndex = 0;
    public static int redPipelineIndex = 1;
    public static int pollRateHz = 50;
    public static long maxResultAgeMs = 200;
    public static double calibrationIntervalSeconds = 1.5;
    // Minimum individual AprilTag area as a percentage of the image (0-100).
    public static double minimumTargetArea = 0.01;
    public static int minimumBotposeTagCount = 2;

    // 19725 flat-HIVE map: field-center origin, inches, +X toward the audience.
    // Target order follows CellCluster.getTuningIndex(). User selected these references.
    public static double[][] cellTargetFieldXYInches = {
            {-17.7, -12.75}, {17.7, -12.75},
            {17.7, 12.75}, {-17.7, 12.75}
    };
    public static double hivePivotHeightInches = 43.95;
    public static double maximumHiveTiltDegrees = 35.0;
    public static double robotLevelToleranceDegrees = 10.0;
    public static double highHiveTiltThresholdDegrees = 23.0;

    public static double shooterYawDegrees = 180.0;

    private VisionConstants() {
    }
}
