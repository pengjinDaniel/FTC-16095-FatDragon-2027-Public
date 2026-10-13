package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

public class Vision extends SubsystemBase {
    public enum LocalizationMode { MULTI_TAG, SINGLE_TAG }
    private final Limelight3A limelight;
    private final Alliance alliance;
    private final int pipelineIndex;
    private final LocalizationMode localizationMode;
    private CellCluster preferredSingleTagCell;
    private int expectedCalibrationTagId = -1;
    private int calibrationTagId = -1;
    private int localizationTagCount;
    private String poseStatus = "Waiting for AprilTags";
    private String targetStatus = "Field pose not initialized";
    private Pose3D rawRobotFieldPose;
    private Pose2D correctedRobotFieldPose;
    private double frameTimestamp = Double.NaN;
    private int botposeTagCount;
    private int usableTagCount;
    private int actualPipelineIndex = -1;
    private long resultAgeMs = -1;
    private double lastPhysicalFrameTimestamp = Double.NaN;
    private long lastPhysicalFrameSeenNanos;
    private long physicalFrameAgeMs = -1;
    private boolean rawResultValid;
    private boolean hiveSolveSuccessful;
    private boolean correctedPoseAccepted;
    private VisionMath.HiveSolveResult hiveSolveResult;
    private double averageTagDistanceMeters = Double.NaN;
    private double tagSpanMeters = Double.NaN;
    private double averageTagArea = Double.NaN;
    private double[] mt1StandardDeviations;
    private double hiveAngleDegrees = Double.NaN;
    private double robotLevelErrorDegrees = Double.NaN;
    private boolean correctionApplied;
    private boolean validAimingTarget;
    private boolean validShootingTarget;
    private double targetHorizontalDistanceInches = Double.NaN;
    private double yawErrorDegrees = Double.NaN;
    private double targetXInches = Double.NaN;
    private double targetYInches = Double.NaN;
    private double desiredHeadingDegrees = Double.NaN;
    private Pose3D representativeTagPoseRobotSpace;
    private CellCluster selectedCluster;
    private final CellSelectionLatch aimCellSelection = new CellSelectionLatch();
    private HiveState targetHiveState = HiveState.UNKNOWN;
    private int selectedTagId = -1;
    private List<Integer> visibleTagIds = Collections.emptyList();
    private List<Integer> detectedTagIds = Collections.emptyList();
    private Map<Integer, String> tagPoseAgreement = Collections.emptyMap();
    private Map<Integer, Double> visibleTagHeightsInches = Collections.emptyMap();
    private Map<Integer, Pose3D> visibleTagPosesRobotSpace = Collections.emptyMap();
    private Map<CellCluster, HiveState> observedClusterStates = Collections.emptyMap();

    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this(hardwareMap, alliance, LocalizationMode.MULTI_TAG);
    }

    public Vision(HardwareMap hardwareMap, Alliance alliance, LocalizationMode localizationMode) {
        this.alliance = alliance;
        this.localizationMode = localizationMode;
        pipelineIndex = alliance == Alliance.BLUE
                ? VisionConstants.bluePipelineIndex : VisionConstants.redPipelineIndex;
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        clearCurrentFrame();
        if (!limelight.isConnected()) {
            poseStatus = "Limelight disconnected";
            return;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            poseStatus = "No Limelight result";
            return;
        }
        actualPipelineIndex = result.getPipelineIndex();
        resultAgeMs = result.getStaleness();
        rawResultValid = result.isValid();
        if (!rawResultValid) {
            poseStatus = "Limelight result invalid";
            return;
        }
        if (actualPipelineIndex != pipelineIndex) {
            poseStatus = "Waiting for pipeline " + pipelineIndex + "; current " + actualPipelineIndex;
            return;
        }
        if (resultAgeMs < 0 || resultAgeMs > VisionConstants.maxResultAgeMs) {
            poseStatus = "Limelight result stale: " + resultAgeMs + " ms";
            return;
        }
        frameTimestamp = result.getTimestamp();
        if (!Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0) {
            poseStatus = "Invalid Limelight frame timestamp";
            return;
        }
        long nowNanos = System.nanoTime();
        if (frameTimestamp != lastPhysicalFrameTimestamp) {
            lastPhysicalFrameTimestamp = frameTimestamp;
            lastPhysicalFrameSeenNanos = nowNanos;
        }
        // SDK staleness measures HTTP reception age, not whether the camera frame advanced.
        physicalFrameAgeMs = (nowNanos - lastPhysicalFrameSeenNanos) / 1_000_000L;
        if (physicalFrameAgeMs > VisionConstants.maxResultAgeMs) {
            poseStatus = "Limelight physical frame frozen: " + physicalFrameAgeMs + " ms";
            return;
        }
        averageTagDistanceMeters = result.getBotposeAvgDist();
        tagSpanMeters = result.getBotposeSpan();
        averageTagArea = result.getBotposeAvgArea();
        double[] stddev = result.getStddevMt1();
        mt1StandardDeviations = stddev == null ? null : stddev.clone();
        botposeTagCount = result.getBotposeTagCount();
        Pose3D aggregatePose = result.getBotpose();
        rawRobotFieldPose = localizationMode == LocalizationMode.MULTI_TAG ? aggregatePose : null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            poseStatus = "No AprilTag detections";
            return;
        }
        List<Integer> ids = new ArrayList<>();
        List<Integer> allIds = new ArrayList<>();
        Map<Integer, String> agreement = new LinkedHashMap<>();
        Map<Integer, Double> heights = new LinkedHashMap<>();
        Map<Integer, Pose3D> poses = new LinkedHashMap<>();
        double representativeArea = -1.0;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial == null) {
                allIds.add(null);
                continue;
            }
            int id = fiducial.getFiducialId();
            allIds.add(id);
            double[] difference = VisionMath.fieldPoseDifference(
                    aggregatePose, fiducial.getRobotPoseFieldSpace());
            agreement.put(id, difference == null ? "UNAVAILABLE"
                    : String.format(Locale.US, "%.2f in / %+.2f deg", difference[0], difference[1]));
            Optional<CellCluster> cluster = CellCluster.forTagId(id);
            if (!cluster.isPresent()) {
                continue;
            }
            if (!cluster.get().isForAlliance(alliance)) {
                continue;
            }
            if (ids.contains(id)) continue;
            ids.add(id);
            Pose3D pose = fiducial.getTargetPoseRobotSpace();
            Optional<Position> position =
                    VisionMath.getRobotSpacePositionInches(pose);
            if (position.isPresent()) {
                heights.put(id, position.get().z);
                poses.put(id, pose);
            }
            // JSON fiducial ta is a fraction; compare areas in percentage units.
            double area = fiducial.getTargetArea() * 100.0;
            if (Double.isFinite(area) && area >= VisionConstants.minimumTargetArea) {
                usableTagCount++;
                if (area > representativeArea) {
                    representativeArea = area;
                    selectedTagId = id;
                    representativeTagPoseRobotSpace = pose;
                }
            }
        }
        visibleTagIds = Collections.unmodifiableList(ids);
        detectedTagIds = Collections.unmodifiableList(allIds);
        tagPoseAgreement = Collections.unmodifiableMap(agreement);
        visibleTagHeightsInches = Collections.unmodifiableMap(heights);
        visibleTagPosesRobotSpace = Collections.unmodifiableMap(poses);
        if (localizationMode == LocalizationMode.SINGLE_TAG) {
            CellCluster requestedCell = aimCellSelection.getLatched() != null
                    ? aimCellSelection.getLatched() : preferredSingleTagCell;
            SingleTagLocalization.Selection source = SingleTagLocalization.select(
                    result, alliance, requestedCell, VisionConstants.minimumTargetArea);
            expectedCalibrationTagId = source.expectedId;
            // Even if other tags are visible, only this fiducial's t6r_fs is used.
            rawRobotFieldPose = source.pose();
            if (source.tag == null) {
                poseStatus = source.status;
                return;
            }
            calibrationTagId = source.tag.getFiducialId();
            selectedTagId = calibrationTagId;
            representativeTagPoseRobotSpace = source.tag.getTargetPoseRobotSpace();
            localizationTagCount = 1;
        } else {
            String tagRejection = VisionMath.localizationTagRejection(allIds, alliance, botposeTagCount);
            if (tagRejection != null) {
                poseStatus = "Rejected: " + tagRejection;
                return;
            }
            if (botposeTagCount < VisionConstants.minimumBotposeTagCount) {
                poseStatus = "Need at least " + VisionConstants.minimumBotposeTagCount
                        + " MT1 localization tags; got " + botposeTagCount;
                return;
            }
            if (usableTagCount < VisionConstants.minimumBotposeTagCount) {
                poseStatus = "Need at least " + VisionConstants.minimumBotposeTagCount
                        + " alliance tags with area >= " + VisionConstants.minimumTargetArea
                        + "%; got " + usableTagCount;
                return;
            }
            localizationTagCount = botposeTagCount;
        }
        hiveSolveResult = VisionMath.solveHiveFlatPose(
                rawRobotFieldPose, VisionConstants.hivePivotHeightInches,
                VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees
        );
        poseStatus = hiveSolveResult.status;
        hiveSolveSuccessful = hiveSolveResult.correction != null;
        if (!hiveSolveSuccessful) {
            return;
        }
        correctedRobotFieldPose = hiveSolveResult.correction.robotPose;
        hiveAngleDegrees = hiveSolveResult.correction.hiveAngleDegrees;
        robotLevelErrorDegrees = hiveSolveResult.correction.levelErrorDegrees;
        Map<CellCluster, HiveState> states = new EnumMap<>(CellCluster.class);
        for (CellCluster cluster : CellCluster.values()) {
            double[] target = getFieldTarget(cluster);
            if (cluster.isForAlliance(alliance) && target != null) {
                HiveState state = Math.abs(hiveAngleDegrees)
                        <= VisionConstants.highHiveTiltThresholdDegrees ? HiveState.MIDDLE
                        : -Math.sin(Math.toRadians(hiveAngleDegrees)) * target[0] > 0.0
                        ? HiveState.HIGH : HiveState.LOW;
                states.put(cluster, state);
            }
        }
        observedClusterStates = Collections.unmodifiableMap(states);
        poseStatus = localizationMode == LocalizationMode.SINGLE_TAG
                ? "Corrected single Tag " + calibrationTagId + " pose ready"
                : "Corrected MT1 field pose ready";
    }

    public void updateFieldTarget(Drive drive, boolean correctFromVision) {
        if (correctFromVision) {
            if (correctedRobotFieldPose != null) {
                correctionApplied |= drive.calibrateFieldPose(correctedRobotFieldPose, frameTimestamp);
            } else {
                drive.resetVisionRejectionSequence();
            }
        }
        correctedPoseAccepted = correctedRobotFieldPose != null
                && drive.hasAcceptedVisionFrame(frameTimestamp);
        validAimingTarget = false;
        validShootingTarget = false;
        selectedCluster = null;
        targetHiveState = HiveState.UNKNOWN;
        targetXInches = Double.NaN;
        targetYInches = Double.NaN;
        desiredHeadingDegrees = Double.NaN;
        targetHorizontalDistanceInches = Double.NaN;
        yawErrorDegrees = Double.NaN;
        if (!DriveConstants.pinpointConfigurationConfirmed) {
            targetStatus = "Pinpoint wheel type/offsets need confirmation";
            return;
        }
        if (!drive.isFieldPoseInitialized()) {
            preferredSingleTagCell = null;
            targetStatus = "Field pose not initialized; wait for corrected vision pose";
            return;
        }
        if (drive.getPinpointStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            targetStatus = "Pinpoint not ready";
            return;
        }
        Pose2D robotPose = drive.getPose();
        selectedCluster = aimCellSelection.select(
                CellCluster.forFieldX(alliance, robotPose.getX(DistanceUnit.INCH)).orElse(null));
        preferredSingleTagCell = selectedCluster;
        double[] target = getFieldTarget(selectedCluster);
        if (target == null) {
            targetStatus = "CELL field coordinates missing";
            return;
        }
        targetXInches = target[0];
        targetYInches = target[1];
        double dx = targetXInches - robotPose.getX(DistanceUnit.INCH);
        double dy = targetYInches - robotPose.getY(DistanceUnit.INCH);
        targetHorizontalDistanceInches = Math.hypot(dx, dy);
        desiredHeadingDegrees = Math.IEEEremainder(
                Math.toDegrees(Math.atan2(dy, dx)) - VisionConstants.shooterYawDegrees, 360.0);
        yawErrorDegrees = VisionMath.coordinateYawErrorDegrees(
                robotPose, targetXInches, targetYInches, VisionConstants.shooterYawDegrees);
        validAimingTarget = Double.isFinite(yawErrorDegrees)
                && Double.isFinite(targetHorizontalDistanceInches);
        if (validAimingTarget) aimCellSelection.confirm(selectedCluster);
        targetHiveState = observedClusterStates.getOrDefault(selectedCluster, HiveState.UNKNOWN);
        validShootingTarget = validAimingTarget && correctedPoseAccepted
                && targetHiveState == HiveState.HIGH
                && (localizationMode != LocalizationMode.SINGLE_TAG
                    || calibrationTagId == SingleTagLocalization.tagIdFor(selectedCluster));
        targetStatus = validAimingTarget ? "Coordinate aim ready from Pinpoint" : "Invalid aim geometry";
    }

    private static double[] getFieldTarget(CellCluster cluster) {
        double[][] targets = VisionConstants.cellTargetFieldXYInches;
        if (cluster == null || targets == null || cluster.getTuningIndex() >= targets.length) {
            return null;
        }
        double[] target = targets[cluster.getTuningIndex()];
        return target != null && target.length >= 2
                && Double.isFinite(target[0]) && Double.isFinite(target[1]) ? target : null;
    }

    public String getTargetStatus() { return targetStatus; }
    public String getPoseStatus() { return poseStatus; }
    public LocalizationMode getLocalizationMode() { return localizationMode; }
    public int getExpectedCalibrationTagId() { return expectedCalibrationTagId; }
    public int getCalibrationTagId() { return calibrationTagId; }
    public int getLocalizationTagCount() { return localizationTagCount; }
    public int getExpectedPipelineIndex() { return pipelineIndex; }
    public int getActualPipelineIndex() { return actualPipelineIndex; }
    public int getBotposeTagCount() { return botposeTagCount; }
    public int getUsableTagCount() { return usableTagCount; }
    public long getResultAgeMs() { return resultAgeMs; }
    public long getPhysicalFrameAgeMs() { return physicalFrameAgeMs; }
    public boolean isRawResultValid() { return rawResultValid; }
    public boolean isHiveSolveSuccessful() { return hiveSolveSuccessful; }
    public boolean isCorrectedPoseAccepted() { return correctedPoseAccepted; }
    public double getAverageTagDistanceMeters() { return averageTagDistanceMeters; }
    public double getTagSpanMeters() { return tagSpanMeters; }
    public double getAverageTagArea() { return averageTagArea; }
    public int getUsableHiveCandidateCount() {
        return hiveSolveResult == null ? 0 : hiveSolveResult.usableCandidateCount;
    }
    public String getHiveCandidateDiagnostics() {
        return hiveSolveResult == null ? "NONE"
                : Arrays.toString(hiveSolveResult.candidatePhiDegrees) + " deg / level "
                + Arrays.toString(hiveSolveResult.candidateLevelErrorsDegrees) + " deg";
    }
    public String getMt1StandardDeviationDiagnostics() {
        return mt1StandardDeviations == null ? "UNAVAILABLE" : Arrays.toString(mt1StandardDeviations);
    }
    public double getHiveAngleDegrees() { return hiveAngleDegrees; }
    public double getRobotLevelErrorDegrees() { return robotLevelErrorDegrees; }
    public double getTargetXInches() { return targetXInches; }
    public double getTargetYInches() { return targetYInches; }
    public double getDesiredHeadingDegrees() { return desiredHeadingDegrees; }
    public boolean wasCorrectionApplied() { return correctionApplied; }
    public boolean hasVisibleAllianceTag() { return !visibleTagIds.isEmpty(); }
    public boolean hasValidAimingTarget() { return validAimingTarget; }
    public boolean hasValidShootingTarget() { return validShootingTarget; }
    public double getTargetHorizontalDistanceInches() { return targetHorizontalDistanceInches; }
    public double getYawErrorDegrees() { return yawErrorDegrees; }
    public Alliance getAlliance() { return alliance; }
    public int getSelectedTagId() { return selectedTagId; }
    public HiveState getTargetHiveState() { return targetHiveState; }
    public List<Integer> getVisibleTagIds() { return visibleTagIds; }
    public List<Integer> getDetectedTagIds() { return detectedTagIds; }
    public Map<Integer, String> getTagPoseAgreement() { return tagPoseAgreement; }
    public double getRawToCorrectedHeadingDeltaDegrees() {
        return correctedRobotFieldPose == null || rawRobotFieldPose == null
                || rawRobotFieldPose.getOrientation() == null ? Double.NaN
                : Math.IEEEremainder(correctedRobotFieldPose.getHeading(AngleUnit.DEGREES)
                    - rawRobotFieldPose.getOrientation().getYaw(AngleUnit.DEGREES), 360.0);
    }
    public Map<Integer, Double> getVisibleTagHeightsInches() { return visibleTagHeightsInches; }
    public Map<Integer, Pose3D> getVisibleTagPosesRobotSpace() { return visibleTagPosesRobotSpace; }
    public Map<CellCluster, HiveState> getObservedClusterStates() { return observedClusterStates; }
    public Optional<CellCluster> getSelectedCluster() { return Optional.ofNullable(selectedCluster); }
    public Optional<Pose3D> getRawRobotFieldPose() { return Optional.ofNullable(rawRobotFieldPose); }
    public Optional<Pose2D> getCorrectedRobotFieldPose() {
        return Optional.ofNullable(correctedRobotFieldPose);
    }
    public Optional<Pose3D> getRepresentativeTagPoseRobotSpace() {
        return Optional.ofNullable(representativeTagPoseRobotSpace);
    }

    public void setAimSelectionHeld(boolean held) { aimCellSelection.setHeld(held); }
    public Optional<CellCluster> getLatchedAimCluster() {
        return Optional.ofNullable(aimCellSelection.getLatched());
    }

    public void stop() {
        preferredSingleTagCell = null;
        aimCellSelection.setHeld(false);
        limelight.stop();
        clearCurrentFrame();
        validAimingTarget = false;
        validShootingTarget = false;
    }

    private void clearCurrentFrame() {
        poseStatus = "Waiting for AprilTags";
        correctionApplied = false;
        rawRobotFieldPose = null;
        correctedRobotFieldPose = null;
        frameTimestamp = Double.NaN;
        botposeTagCount = 0;
        localizationTagCount = 0;
        calibrationTagId = -1;
        expectedCalibrationTagId = localizationMode == LocalizationMode.SINGLE_TAG
                ? SingleTagLocalization.tagIdFor(aimCellSelection.getLatched() != null
                    ? aimCellSelection.getLatched() : preferredSingleTagCell) : -1;
        usableTagCount = 0;
        actualPipelineIndex = -1;
        resultAgeMs = -1;
        physicalFrameAgeMs = -1;
        rawResultValid = false;
        hiveSolveSuccessful = false;
        correctedPoseAccepted = false;
        hiveSolveResult = null;
        averageTagDistanceMeters = Double.NaN;
        tagSpanMeters = Double.NaN;
        averageTagArea = Double.NaN;
        mt1StandardDeviations = null;
        hiveAngleDegrees = Double.NaN;
        robotLevelErrorDegrees = Double.NaN;
        representativeTagPoseRobotSpace = null;
        selectedTagId = -1;
        visibleTagIds = Collections.emptyList();
        detectedTagIds = Collections.emptyList();
        tagPoseAgreement = Collections.emptyMap();
        visibleTagHeightsInches = Collections.emptyMap();
        visibleTagPosesRobotSpace = Collections.emptyMap();
        observedClusterStates = Collections.emptyMap();
        validShootingTarget = false;
    }
}
