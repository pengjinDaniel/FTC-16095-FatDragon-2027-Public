package org.firstinspires.ftc.teamcode.subsystems.vision;

public class VisionConstants {
    public static String limelightName = "limelight";
    public static int bluePipelineIndex = 0;
    public static int redPipelineIndex = 1;
    public static int pollRateHz = 50;
    public static long maxResultAgeMs = 200;
    public static double calibrationIntervalSeconds = 1.5;
    public static double rtStablePositionToleranceCm = 30.0;
    public static double rtStableHeadingToleranceDegrees = 30.0;
    public static double rtStableHiveAngleToleranceDegrees = 10.0;
    public static double rtCalibrationTimeoutSeconds = 2.0;
    // Minimum individual AprilTag area as a percentage of the image (0-100).
    public static double minimumTargetArea = 0.01;
    public static int minimumBotposeTagCount = 0;

    // Official CAD/SDK opening targets at the nominal 30-degree HIGH position.
    // Field-center origin, inches, +X toward the audience; ordered by CellCluster tuning index.
    public static double[][] cellTargetFieldXYInches = {
            {-16.52, -12.75}, {16.52, -12.75},
            {16.52, 12.75}, {-16.52, 12.75}
    };
    public static double cellTargetReferenceHeightInches = 58.25;
    public static double cellTargetReferenceTiltDegrees = 30.0;
    // Four-tag centroids from the BIOBUZZ flat fmaps, in the same CellCluster order.
    // Fmap translations are meters; these reference points are at zero HIVE tilt.
    public static double[][] tagClusterCenterFieldXYInches = {
            {-0.36243411 / 0.0254, -12.75}, {0.36243411 / 0.0254, -12.75},
            {0.36243411 / 0.0254, 12.75}, {-0.36243411 / 0.0254, 12.75}
    };
    public static double tagClusterCenterReferenceHeightInches = 1.07826892 / 0.0254;
    // Official CAD pivot height; the manual rounds this to 43.95 inches.
    public static double hivePivotHeightInches = 43.9497007874;
    public static double maximumHiveTiltDegrees = 35.0;
    public static double robotLevelToleranceDegrees = 60.0;
    public static double highHiveTiltThresholdDegrees = 23.0;

    public static double shooterYawDegrees = 180.0;

    private VisionConstants() {
    }
}
