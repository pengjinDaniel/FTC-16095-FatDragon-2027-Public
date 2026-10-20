package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;
    private final Alliance alliance;
    private final int pipelineIndex;
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
    private CellCluster selectedCluster;
    private HiveState targetHiveState = HiveState.UNKNOWN;
    private List<Integer> visibleTagIds = Collections.emptyList();
    private Map<Integer, Pose3D> visibleTagPosesRobotSpace = Collections.emptyMap();
    private Map<CellCluster, HiveState> observedClusterStates = Collections.emptyMap();

    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
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
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            poseStatus = "No AprilTag detections";
            return;
        }
        List<Integer> ids = new ArrayList<>();
        Map<Integer, Pose3D> poses = new LinkedHashMap<>();
        boolean foreignAllianceTag = false;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            Optional<CellCluster> cluster = CellCluster.forTagId(id);
            if (!cluster.isPresent()) {
                continue;
            }
            if (!cluster.get().isForAlliance(alliance)) {
                foreignAllianceTag = true;
                continue;
            }
            ids.add(id);
            Pose3D pose = fiducial.getTargetPoseRobotSpace();
            if (VisionMath.getRobotSpacePositionInches(pose).isPresent()) {
                poses.put(id, pose);
            }
            // JSON fiducial ta is a fraction; compare areas in percentage units.
            double area = fiducial.getTargetArea() * 100.0;
            if (Double.isFinite(area) && area >= VisionConstants.minimumTargetArea) {
                usableTagCount++;
            }
        }
        visibleTagIds = Collections.unmodifiableList(ids);
        visibleTagPosesRobotSpace = Collections.unmodifiableMap(poses);
        if (foreignAllianceTag) {
            poseStatus = "Rejected mixed-alliance HIVE tags";
            return;
        }
        botposeTagCount = result.getBotposeTagCount();
        rawRobotFieldPose = result.getBotpose();
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
        poseStatus = "Corrected MT1 field pose ready";
    }

    // October 9 policy: INIT always calibrates; after START, A or RT requests calibration.
    public void updateAimTarget(
            Drive drive, boolean started, boolean stopped, boolean calibrationRequested
    ) {
        updateFieldTarget(drive, !started || (!stopped && calibrationRequested));
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
            targetStatus = "Field pose not initialized; wait for corrected MT1 pose";
            return;
        }
        if (drive.getPinpointStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            targetStatus = "Pinpoint not ready";
            return;
        }
        Pose2D robotPose = drive.getPose();
        selectedCluster = CellCluster.forFieldX(alliance, robotPose.getX(DistanceUnit.INCH))
                .orElse(null);
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
        targetHiveState = observedClusterStates.getOrDefault(selectedCluster, HiveState.UNKNOWN);
        validShootingTarget = validAimingTarget && correctedPoseAccepted
                && targetHiveState == HiveState.HIGH;
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
    public boolean hasValidAimingTarget() { return validAimingTarget; }
    public boolean hasValidShootingTarget() { return validShootingTarget; }
    public double getTargetHorizontalDistanceInches() { return targetHorizontalDistanceInches; }
    public double getYawErrorDegrees() { return yawErrorDegrees; }
    public Alliance getAlliance() { return alliance; }
    public HiveState getTargetHiveState() { return targetHiveState; }
    public List<Integer> getVisibleTagIds() { return visibleTagIds; }
    public Map<Integer, Pose3D> getVisibleTagPosesRobotSpace() { return visibleTagPosesRobotSpace; }
    public Map<CellCluster, HiveState> getObservedClusterStates() { return observedClusterStates; }
    public Optional<CellCluster> getSelectedCluster() { return Optional.ofNullable(selectedCluster); }
    public Optional<Pose3D> getRawRobotFieldPose() { return Optional.ofNullable(rawRobotFieldPose); }
    public Optional<Pose2D> getCorrectedRobotFieldPose() {
        return Optional.ofNullable(correctedRobotFieldPose);
    }

    public void stop() {
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
        visibleTagIds = Collections.emptyList();
        visibleTagPosesRobotSpace = Collections.emptyMap();
        observedClusterStates = Collections.emptyMap();
        validShootingTarget = false;
    }
}
