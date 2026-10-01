package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public abstract class CoordinateAimTestBase extends OpMode {
    private static final double[][] HIGH_CELL_CENTER_XY = {
            {59.25, 88.52},
            {59.25, 55.48},
            {84.75, 55.48},
            {84.75, 88.52}
    };

    private static final double[][] HIGH_TAG_FIELD_XYZ = {
            {65.75, 85.11, 49.79}, // 30
            {62.00, 85.11, 49.79}, // 31
            {56.50, 85.11, 49.79}, // 32
            {52.75, 85.11, 49.79}, // 33
            {52.75, 58.89, 49.79}, // 34
            {56.50, 58.89, 49.79}, // 35
            {62.00, 58.89, 49.79}, // 36
            {65.75, 58.89, 49.79}, // 37
            {78.25, 58.89, 49.79}, // 38
            {82.00, 58.89, 49.79}, // 39
            {87.50, 58.89, 49.79}, // 40
            {91.25, 58.89, 49.79}, // 41
            {91.25, 85.11, 49.79}, // 42
            {87.50, 85.11, 49.79}, // 43
            {82.00, 85.11, 49.79}, // 44
            {78.25, 85.11, 49.79}  // 45
    };

    private static final boolean CAMERA_POSE_VERIFIED = true;
    private static final boolean[] HIGH_CELL_REPEATABILITY_VERIFIED = {
            true, true, true, true
    };

    private static final int MIN_TAG_ID = 30;
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double HIGH_Z_TOLERANCE_INCHES = 1;
    // Adjacent BIOBUZZ cluster tags are 3.75 in apart nominally.
    private static final double MIN_TAG_BASELINE_INCHES = 3.0;
    private static final double MAX_PAIR_BASELINE_ERROR_INCHES = 2.5;
    private static final double MAX_TAG_RESIDUAL_INCHES = 2.5;
    private static final double MAX_TRANSLATION_SPEED_INCHES_PER_SECOND = 1.0;
    private static final double MAX_HEADING_SPEED_DEGREES_PER_SECOND = 5.0;
    private static final double MAX_CORRECTION_INCHES = 5.0;
    private static final double MAX_CORRECTION_HEADING_ERROR_DEGREES = 10.0;
    private static final double MIN_AIM_DISTANCE_INCHES = 3.0;
    private static final double AIM_KP = 0.012;
    private static final double MAX_AIM_TURN_POWER = 0.7;
    private static final double AIM_DEADBAND_DEGREES = 2.0;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final int REQUIRED_STABLE_FRAMES = 5;

    private Drive drive;
    private Limelight3A limelight;
    private CellCluster targetCluster;
    private CellCluster observedHighCluster;
    private CellCluster previousHighCluster;
    private VisionPose visionPose;
    private VisionPose previousStablePose;
    private long lastFrameTimestampMs = -1;
    private long observedHighFrameTimestampMs = -1;
    private long lastHighFrameTimestampMs = -1;
    private int stableFrames;
    private int stableHighFrames;
    private boolean localized;
    private boolean highPositionValid;
    private boolean correctionApplied;
    private boolean aimWasRequested;
    private boolean stopLatched;
    private long aimStartFrameTimestampMs = -1;
    private double headingErrorDegrees = Double.NaN;
    private List<Integer> visibleTagIds = Collections.emptyList();
    private Map<Integer, Double> visibleTagRobotYInches = Collections.emptyMap();
    private Map<Integer, Double> visibleTagHeightsInches = Collections.emptyMap();
    private Map<Integer, Double> visibleTagRollDegrees = Collections.emptyMap();
    private String status = "Waiting for AprilTags";

    protected abstract Alliance getAlliance();

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.pipelineSwitch(VisionConstants.pipelineIndex);
        limelight.start();
        drive.stop();
    }

    @Override
    public void init_loop() {
        updateLocalization();
        showTelemetry();
    }

    @Override
    public void loop() {
        updateLocalization();
        boolean aimRequested = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        if (!aimRequested) {
            correctionApplied = false;
            targetCluster = null;
        } else if (!aimWasRequested) {
            correctionApplied = false;
            targetCluster = highPositionValid ? observedHighCluster : null;
            aimStartFrameTimestampMs = observedHighFrameTimestampMs;
        }

        if (gamepad1.b) {
            stopLatched = true;
        } else if (!aimRequested) {
            stopLatched = false;
        }

        if (!localized || stopLatched) {
            drive.stop();
            headingErrorDegrees = Double.NaN;
        } else if (aimRequested) {
            if (!highPositionValid || targetCluster == null
                    || observedHighCluster != targetCluster) {
                drive.stop();
                headingErrorDegrees = Double.NaN;
                status = "Current HIGH CELL not confirmed; release RT to retry";
            } else if (!correctionApplied && !correctFromCurrentHighTags()) {
                drive.stop();
                headingErrorDegrees = Double.NaN;
            } else {
                aimFromOdometry();
            }
        } else {
            headingErrorDegrees = Double.NaN;
            drive.moveRobot(
                    -applyDeadband(gamepad1.left_stick_y),
                    applyDeadband(gamepad1.left_stick_x),
                    applyDeadband(gamepad1.right_stick_x)
            );
        }

        aimWasRequested = aimRequested;
        showTelemetry();
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (limelight != null) {
            limelight.stop();
        }
    }

    private void updateLocalization() {
        drive.periodic();
        visionPose = readVisionPose();
        if (observedHighCluster == null) {
            previousHighCluster = null;
            stableHighFrames = 0;
        } else if (observedHighFrameTimestampMs != lastHighFrameTimestampMs) {
            stableHighFrames = observedHighCluster == previousHighCluster
                    ? stableHighFrames + 1 : 1;
            previousHighCluster = observedHighCluster;
            lastHighFrameTimestampMs = observedHighFrameTimestampMs;
        }
        highPositionValid = observedHighCluster != null
                && stableHighFrames >= REQUIRED_STABLE_FRAMES
                && CAMERA_POSE_VERIFIED
                && drive.getPinpointStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        if (visionPose == null) {
            previousStablePose = null;
            stableFrames = 0;
            if (!localized) {
                status = "Need calibrated HIGH state and two tags for field pose";
            }
            return;
        }
        if (visionPose.frameTimestampMs == lastFrameTimestampMs) {
            return;
        }
        lastFrameTimestampMs = visionPose.frameTimestampMs;
        if (previousStablePose != null
                && previousStablePose.cluster == visionPose.cluster
                && Math.hypot(
                        previousStablePose.xInches - visionPose.xInches,
                        previousStablePose.yInches - visionPose.yInches
                ) <= 1.5
                && Math.abs(angleDifferenceDegrees(
                        previousStablePose.headingRadians,
                        visionPose.headingRadians
                )) <= 3.0) {
            stableFrames++;
        } else {
            stableFrames = 1;
        }
        previousStablePose = visionPose;

        if (!CAMERA_POSE_VERIFIED) {
            status = "Verify Limelight camera pose and axes, then enable test constant";
            return;
        }
        if (localized) {
            return;
        }
        if (drive.getPinpointStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || drive.getTranslationSpeedInchesPerSecond()
                        > MAX_TRANSLATION_SPEED_INCHES_PER_SECOND
                || drive.getHeadingSpeedDegreesPerSecond()
                        > MAX_HEADING_SPEED_DEGREES_PER_SECOND) {
            status = "Waiting for stationary Pinpoint READY";
            return;
        }

        if (!isHighCellLandmarkVerified(visionPose.cluster)) {
            status = "HIGH CELL repeatability not verified; pose initialization blocked";
            return;
        }
        if (stableFrames >= REQUIRED_STABLE_FRAMES) {
            drive.setPose(new Pose2D(
                    DistanceUnit.INCH,
                    visionPose.xInches,
                    visionPose.yInches,
                    AngleUnit.RADIANS,
                    visionPose.headingRadians
            ));
            localized = true;
            status = "Initialized Pinpoint X/Y/heading from HIGH tags";
        }
    }

    private boolean correctFromCurrentHighTags() {
        if (!isHighCellLandmarkVerified(targetCluster)) {
            status = "HIGH CELL repeatability not verified; correction blocked";
            return false;
        }
        if (visionPose == null || visionPose.cluster != targetCluster) {
            status = "Waiting for two HIGH tags to correct Pinpoint";
            return false;
        }
        if (visionPose.frameTimestampMs <= aimStartFrameTimestampMs
                || stableFrames < REQUIRED_STABLE_FRAMES) {
            status = "Waiting for a new stable HIGH tag frame";
            return false;
        }
        if (drive.getTranslationSpeedInchesPerSecond()
                > MAX_TRANSLATION_SPEED_INCHES_PER_SECOND
                || drive.getHeadingSpeedDegreesPerSecond()
                > MAX_HEADING_SPEED_DEGREES_PER_SECOND) {
            status = "Stop moving before vision correction";
            return false;
        }

        Pose2D odPose = drive.getPose();
        double heading = odPose.getHeading(AngleUnit.RADIANS);
        if (Math.abs(angleDifferenceDegrees(heading, visionPose.headingRadians))
                > MAX_CORRECTION_HEADING_ERROR_DEGREES) {
            status = "Vision/Pinpoint heading mismatch; correction rejected";
            return false;
        }
        double[] correctedXY = xyFromTagsAtHeading(visionPose.tags, heading);
        if (!isWithinField(correctedXY[0], correctedXY[1])) {
            status = "Vision XY outside FIELD; correction rejected";
            return false;
        }
        double correction = Math.hypot(
                correctedXY[0] - odPose.getX(DistanceUnit.INCH),
                correctedXY[1] - odPose.getY(DistanceUnit.INCH)
        );
        if (correction > MAX_CORRECTION_INCHES) {
            status = "Vision/Pinpoint XY mismatch; correction rejected";
            return false;
        }
        drive.setXYInches(correctedXY[0], correctedXY[1]);
        correctionApplied = true;
        status = "Corrected Pinpoint X/Y once for this RT press";
        return true;
    }

    private void aimFromOdometry() {
        Pose2D pose = drive.getPose();
        double[] center = HIGH_CELL_CENTER_XY[targetCluster.getTuningIndex()];
        double dx = center[0] - pose.getX(DistanceUnit.INCH);
        double dy = center[1] - pose.getY(DistanceUnit.INCH);
        if (Math.hypot(dx, dy) < MIN_AIM_DISTANCE_INCHES) {
            drive.stop();
            status = "Too close to CELL center for a stable heading";
            return;
        }

        double desiredHeading = Math.atan2(dy, dx);
        headingErrorDegrees = angleDifferenceDegrees(
                desiredHeading,
                pose.getHeading(AngleUnit.RADIANS)
        );
        // Drive.moveRobot positive turn is clockwise; Pinpoint heading is CCW.
        double turn = Math.abs(headingErrorDegrees) <= AIM_DEADBAND_DEGREES
                ? 0.0
                : Range.clip(
                        -AIM_KP * headingErrorDegrees,
                        -MAX_AIM_TURN_POWER,
                        MAX_AIM_TURN_POWER
                );
        drive.moveRobot(0.0, 0.0, turn);
    }

    private VisionPose readVisionPose() {
        observedHighCluster = null;
        observedHighFrameTimestampMs = -1;
        visibleTagIds = Collections.emptyList();
        visibleTagRobotYInches = Collections.emptyMap();
        visibleTagHeightsInches = Collections.emptyMap();
        visibleTagRollDegrees = Collections.emptyMap();
        if (!limelight.isConnected()) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || result.getStaleness() < 0
                || result.getStaleness() > VisionConstants.maxResultAgeMs) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return null;
        }

        List<Integer> ids = new ArrayList<>();
        Map<Integer, Double> robotYs = new LinkedHashMap<>();
        Map<Integer, Double> heights = new LinkedHashMap<>();
        Map<Integer, Double> rolls = new LinkedHashMap<>();
        Map<CellCluster, Boolean> highGroups = new EnumMap<>(CellCluster.class);
        Map<CellCluster, List<TagPoint>> groups = new EnumMap<>(CellCluster.class);
        Map<CellCluster, Boolean> rejectedGroups = new EnumMap<>(CellCluster.class);
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            Optional<CellCluster> cluster = CellCluster.forTagId(tagId);
            if (!cluster.isPresent() || !cluster.get().isForAlliance(getAlliance())) {
                continue;
            }
            ids.add(tagId);
            Pose3D robotTagPose = fiducial.getTargetPoseRobotSpace();
            if (robotTagPose == null || robotTagPose.getPosition() == null) {
                continue;
            }
            Position robotTag = robotTagPose.getPosition().toUnit(DistanceUnit.INCH);
            if (!Double.isFinite(robotTag.x) || !Double.isFinite(robotTag.y)
                    || !Double.isFinite(robotTag.z)
                    || Math.hypot(robotTag.x, robotTag.y) < 1.0) {
                continue;
            }
            robotYs.put(tagId, robotTag.y);
            heights.put(tagId, robotTag.z);
            double tagRoll = robotTagPose.getOrientation() == null
                    ? Double.NaN
                    : robotTagPose.getOrientation().getRoll(AngleUnit.DEGREES);
            rolls.put(tagId, tagRoll);
            double[] fieldTag = HIGH_TAG_FIELD_XYZ[tagId - MIN_TAG_ID];
            if (!Double.isFinite(fieldTag[2])
                    || !Double.isFinite(fiducial.getTargetArea())
                    || fiducial.getTargetArea() < VisionConstants.minimumTargetArea) {
                continue;
            }
            if (!isHighTagPose(tagId, robotTag.z, fieldTag[2], tagRoll)) {
                rejectedGroups.put(cluster.get(), true);
                continue;
            }
            highGroups.put(cluster.get(), true);
            if (!Double.isFinite(fieldTag[0]) || !Double.isFinite(fieldTag[1])) {
                continue;
            }

            // Convert robot-space Y to left-positive for pose fitting.
            // The minus sign assumes this Limelight reports right-positive Y.
            groups.computeIfAbsent(cluster.get(), ignored -> new ArrayList<>())
                    .add(new TagPoint(
                            robotTag.x,
                            -robotTag.y,
                            fieldTag[0],
                            fieldTag[1],
                            fiducial.getTargetArea()
                    ));
        }
        visibleTagIds = Collections.unmodifiableList(ids);
        visibleTagRobotYInches = Collections.unmodifiableMap(robotYs);
        visibleTagHeightsInches = Collections.unmodifiableMap(heights);
        visibleTagRollDegrees = Collections.unmodifiableMap(rolls);

        CellCluster onlyHigh = null;
        for (CellCluster cluster : highGroups.keySet()) {
            if (Boolean.TRUE.equals(rejectedGroups.get(cluster))) {
                continue;
            }
            if (onlyHigh != null) {
                status = "Two HIGH CELLs detected; localization rejected";
                return null;
            }
            onlyHigh = cluster;
        }
        observedHighCluster = onlyHigh;
        if (onlyHigh == null) {
            return null;
        }
        observedHighFrameTimestampMs = result.getControlHubTimeStamp();
        return fitCluster(onlyHigh,
                groups.containsKey(onlyHigh) ? groups.get(onlyHigh) : Collections.emptyList(),
                observedHighFrameTimestampMs);
    }

    private static boolean isHighTagPose(
            int tagId, double observedZ, double expectedZ, double observedRoll
    ) {
        int index = tagId - MIN_TAG_ID;
        double[] highRolls = VisionConstants.highTagRollDegrees;
        double rollTolerance = VisionConstants.tagRollToleranceDegrees;
        return highRolls != null
                && index >= 0
                && index < highRolls.length
                && Double.isFinite(highRolls[index])
                && Double.isFinite(observedRoll)
                && Double.isFinite(rollTolerance)
                && rollTolerance >= 0.0
                && rollTolerance < 180.0
                && Math.abs(Math.IEEEremainder(
                        observedRoll - highRolls[index], 360.0
                )) <= rollTolerance
                && Math.abs(observedZ - expectedZ) <= HIGH_Z_TOLERANCE_INCHES;
    }

    private static VisionPose fitCluster(
            CellCluster cluster, List<TagPoint> points, long timestampMs
    ) {
        if (points.size() < 2) {
            return null;
        }

        double bestHeading = Double.NaN;
        double bestX = Double.NaN;
        double bestY = Double.NaN;
        int bestCount = 0;
        double bestError = Double.POSITIVE_INFINITY;
        for (int i = 0; i < points.size(); i++) {
            for (int j = i + 1; j < points.size(); j++) {
                TagPoint a = points.get(i);
                TagPoint b = points.get(j);
                double robotDX = b.robotX - a.robotX;
                double robotDY = b.robotY - a.robotY;
                double fieldDX = b.fieldX - a.fieldX;
                double fieldDY = b.fieldY - a.fieldY;
                double robotBaseline = Math.hypot(robotDX, robotDY);
                double fieldBaseline = Math.hypot(fieldDX, fieldDY);
                if (robotBaseline < MIN_TAG_BASELINE_INCHES
                        || fieldBaseline < MIN_TAG_BASELINE_INCHES
                        || Math.abs(robotBaseline - fieldBaseline)
                                > MAX_PAIR_BASELINE_ERROR_INCHES) {
                    continue;
                }
                double heading = Math.atan2(
                        robotDX * fieldDY - robotDY * fieldDX,
                        robotDX * fieldDX + robotDY * fieldDY
                );
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);
                double x = a.fieldX - (cos * a.robotX - sin * a.robotY);
                double y = a.fieldY - (sin * a.robotX + cos * a.robotY);
                int inliers = 0;
                double error = 0.0;
                for (TagPoint point : points) {
                    double residual = residual(point, x, y, cos, sin);
                    if (residual <= MAX_TAG_RESIDUAL_INCHES) {
                        inliers++;
                        error += residual * residual;
                    }
                }
                if (inliers > bestCount || (inliers == bestCount && error < bestError)) {
                    bestCount = inliers;
                    bestError = error;
                    bestHeading = heading;
                    bestX = x;
                    bestY = y;
                }
            }
        }
        if (bestCount < 2 || bestCount <= points.size() / 2) {
            return null;
        }

        List<TagPoint> inliers = new ArrayList<>();
        for (TagPoint point : points) {
            if (residual(point, bestX, bestY,
                    Math.cos(bestHeading), Math.sin(bestHeading))
                    <= MAX_TAG_RESIDUAL_INCHES) {
                inliers.add(point);
            }
        }

        double totalWeight = 0.0;
        double robotXMean = 0.0;
        double robotYMean = 0.0;
        double fieldXMean = 0.0;
        double fieldYMean = 0.0;
        for (TagPoint point : inliers) {
            totalWeight += point.weight;
            robotXMean += point.robotX * point.weight;
            robotYMean += point.robotY * point.weight;
            fieldXMean += point.fieldX * point.weight;
            fieldYMean += point.fieldY * point.weight;
        }
        robotXMean /= totalWeight;
        robotYMean /= totalWeight;
        fieldXMean /= totalWeight;
        fieldYMean /= totalWeight;

        double dot = 0.0;
        double cross = 0.0;
        for (TagPoint point : inliers) {
            double rx = point.robotX - robotXMean;
            double ry = point.robotY - robotYMean;
            double fx = point.fieldX - fieldXMean;
            double fy = point.fieldY - fieldYMean;
            dot += point.weight * (rx * fx + ry * fy);
            cross += point.weight * (rx * fy - ry * fx);
        }
        double heading = Math.atan2(cross, dot);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = fieldXMean - (cos * robotXMean - sin * robotYMean);
        double y = fieldYMean - (sin * robotXMean + cos * robotYMean);
        double maxResidual = 0.0;
        for (TagPoint point : inliers) {
            maxResidual = Math.max(maxResidual, residual(point, x, y, cos, sin));
        }
        if (!isWithinField(x, y) || !Double.isFinite(heading)
                || maxResidual > MAX_TAG_RESIDUAL_INCHES) {
            return null;
        }
        return new VisionPose(cluster, x, y, heading, maxResidual, inliers, timestampMs);
    }

    private static double residual(TagPoint point, double x, double y,
                                   double cos, double sin) {
        return Math.hypot(
                x + cos * point.robotX - sin * point.robotY - point.fieldX,
                y + sin * point.robotX + cos * point.robotY - point.fieldY
        );
    }

    private static double[] xyFromTagsAtHeading(List<TagPoint> tags, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = 0.0;
        double y = 0.0;
        double weight = 0.0;
        for (TagPoint point : tags) {
            x += point.weight * (point.fieldX - cos * point.robotX + sin * point.robotY);
            y += point.weight * (point.fieldY - sin * point.robotX - cos * point.robotY);
            weight += point.weight;
        }
        return new double[]{x / weight, y / weight};
    }

    private static boolean isWithinField(double x, double y) {
        return Double.isFinite(x) && Double.isFinite(y)
                && x >= 0.0 && x <= FIELD_SIZE_INCHES
                && y >= 0.0 && y <= FIELD_SIZE_INCHES;
    }

    private static boolean isHighCellLandmarkVerified(CellCluster cluster) {
        return cluster != null
                && HIGH_CELL_REPEATABILITY_VERIFIED[cluster.getTuningIndex()];
    }

    private static double angleDifferenceDegrees(double firstRadians, double secondRadians) {
        double difference = firstRadians - secondRadians;
        return Math.toDegrees(Math.atan2(Math.sin(difference), Math.cos(difference)));
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) > DriveConstants.joystickDeadband ? value : 0.0;
    }

    private void showTelemetry() {
        Pose2D pose = drive.getPose();
        telemetry.addLine("RT: coordinate aim | sticks: manual drive | B: stop until RT release");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", status);
        telemetry.addData("Limelight robot pose verified", CAMERA_POSE_VERIFIED);
        telemetry.addData("Pinpoint", drive.getPinpointStatus());
        telemetry.addData("Visible alliance tag IDs", visibleTagIds);
        telemetry.addData("Limelight robot Y raw (in)", visibleTagRobotYInches);
        telemetry.addData("Visible tag Z (in)", visibleTagHeightsInches);
        telemetry.addData("Visible Limelight tag roll (deg)", visibleTagRollDegrees);
        telemetry.addData("HIGH candidate CELL", observedHighCluster == null
                ? "NONE" : observedHighCluster.name());
        telemetry.addData("HIGH stable frames", stableHighFrames);
        telemetry.addData("HIGH tag fit count", visionPose == null ? 0 : visionPose.tags.size());
        telemetry.addData("Fit max residual (in)", visionPose == null
                ? Double.NaN : visionPose.maxResidualInches);
        telemetry.addData("Stable frames", stableFrames);
        CellCluster displayedCluster = targetCluster != null
                ? targetCluster : observedHighCluster;
        telemetry.addData("Selected CELL", displayedCluster == null
                ? "NONE" : displayedCluster.name());
        telemetry.addData("HIGH CELL center XY (in)", displayedCluster == null
                ? "NONE" : String.format("%.1f, %.1f",
                        HIGH_CELL_CENTER_XY[displayedCluster.getTuningIndex()][0],
                        HIGH_CELL_CENTER_XY[displayedCluster.getTuningIndex()][1]));
        telemetry.addData("Localized", localized);
        telemetry.addData("HIGH valid", highPositionValid);
        telemetry.addData("HIGH landmark verified", isHighCellLandmarkVerified(displayedCluster));
        telemetry.addData("Vision corrected this RT press", correctionApplied);
        telemetry.addData("Vision pose X/Y/H", visionPose == null
                ? "NONE"
                : String.format("%.1f, %.1f, %.1f",
                        visionPose.xInches, visionPose.yInches,
                        Math.toDegrees(visionPose.headingRadians)));
        telemetry.addData("Pinpoint X/Y/H", "%.1f, %.1f, %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Aim error (deg)", "%.1f", headingErrorDegrees);
        telemetry.update();
    }

    private static final class TagPoint {
        final double robotX;
        final double robotY;
        final double fieldX;
        final double fieldY;
        final double weight;

        TagPoint(double robotX, double robotY, double fieldX, double fieldY, double weight) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.weight = weight;
        }
    }

    private static final class VisionPose {
        final CellCluster cluster;
        final double xInches;
        final double yInches;
        final double headingRadians;
        final double maxResidualInches;
        final List<TagPoint> tags;
        final long frameTimestampMs;

        VisionPose(CellCluster cluster, double xInches, double yInches,
                   double headingRadians, double maxResidualInches,
                   List<TagPoint> tags, long frameTimestampMs) {
            this.cluster = cluster;
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = headingRadians;
            this.maxResidualInches = maxResidualInches;
            this.tags = tags;
            this.frameTimestampMs = frameTimestampMs;
        }
    }
}
