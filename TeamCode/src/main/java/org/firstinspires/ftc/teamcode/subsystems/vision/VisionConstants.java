package org.firstinspires.ftc.teamcode.subsystems.vision;

public class VisionConstants {
    public static String limelightName = "limelight";
    public static int bluePipelineIndex = 0;
    public static int redPipelineIndex = 1;
    public static int pollRateHz = 50;
    public static long maxResultAgeMs = 200;
    // Initial outlier thresholds; compare vision correction offsets, not normal robot motion.
    public static double calibrationSampleSpreadInches = 6.0;
    public static double calibrationSampleSpreadDegrees = 10.0;
    // Minimum individual AprilTag area as a percentage of the image (0-100).
    public static double minimumTargetArea = 0.01;
    public static int minimumBotposeTagCount = 2;

    // Unified field frame: inches, center (72,72), +X toward blue, +Y away from audience.
    // Target order follows CellCluster.getTuningIndex(). User selected these references.
    public static double[][] cellTargetFieldXYInches = {
            {59.25, 89.7}, {59.25, 54.3},
            {84.75, 54.3}, {84.75, 89.7}
    };
    public static double hivePivotHeightInches = 43.95;
    public static double maximumHiveTiltDegrees = 35.0;
    public static double robotLevelToleranceDegrees = 10.0;
    public static double highHiveTiltThresholdDegrees = 23.0;

    public static double shooterYawDegrees = 180.0;

    private VisionConstants() {
    }
}
