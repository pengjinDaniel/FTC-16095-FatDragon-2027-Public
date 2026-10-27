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
    public static int minimumBotposeTagCount = 1;

    // Official CAD/SDK opening targets at the nominal 30-degree HIGH position.
    // Field-center origin, inches, +X toward the audience; ordered by CellCluster tuning index.
    public static double[][] cellTargetFieldXYInches = {
            {-16.52, -12.75}, {16.52, -12.75},
            {16.52, 12.75}, {-16.52, 12.75}
    };
    public static double cellTargetReferenceHeightInches = 58.25;
    public static double cellTargetReferenceTiltDegrees = 30.0;
    // Official CAD pivot height; the manual rounds this to 43.95 inches.
    public static double hivePivotHeightInches = 43.9497007874;
    public static double maximumHiveTiltDegrees = 35.0;
    public static double robotLevelToleranceDegrees = 10.0;
    public static double highHiveTiltThresholdDegrees = 23.0;

    public static double shooterYawDegrees = 180.0;

    private VisionConstants() {
    }
}
