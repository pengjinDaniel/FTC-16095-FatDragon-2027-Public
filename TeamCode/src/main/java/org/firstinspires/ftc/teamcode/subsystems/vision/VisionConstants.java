package org.firstinspires.ftc.teamcode.subsystems.vision;

public class VisionConstants {
    public static String limelightName = "limelight";
    public static int pipelineIndex = 0;
    public static int pollRateHz = 50;
    public static double minimumTargetArea = 0.01;

    // Index 0 corresponds to tag 30 and index 15 corresponds to tag 45.
    // Populate with measured robot-space tag-center Z values for the HIGH configuration.
    // An empty array or non-finite tolerance intentionally classifies every CELL as UNKNOWN.
    public static double[] highTagHeightsInches = new double[0];
    public static double tagHeightToleranceInches = Double.NaN;
    public static int minimumPoseObservationsForHiveState = 1;

    // No tag-to-CELL transform is assumed. Populate this map only from measured or official
    // geometry; an empty map intentionally disables CELL-center targeting and firing.
    public static TagToCellTransformMap tagToCellTransforms = TagToCellTransformMap.empty();

    // Order: red opposite audience, red audience, blue audience, blue opposite audience.
    public static double[] clusterYawOffsetsDegrees = {0.0, 0.0, 0.0, 0.0};
    public static double[] clusterRange3DOffsetsInches = {0.0, 0.0, 0.0, 0.0};
    public static double[] clusterHorizontalDistanceOffsetsInches = {0.0, 0.0, 0.0, 0.0};

    private VisionConstants() {
    }
}
