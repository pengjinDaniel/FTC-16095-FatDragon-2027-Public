package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

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
    private boolean hasKnownHiveAngle;
    private double lastKnownHiveAngleDegrees;
    private double robotLevelErrorDegrees = Double.NaN;
    private boolean correctionApplied;
    private String calibrationStatus = "Waiting for corrected vision";
    private boolean calibrationSucceeded;
    private long lastSuccessfulCalibrationNanos;
    private double calibrationCooldownRemainingSeconds;
    private enum RtCalibrationState { IDLE, PAUSED, FAILED }
    private static final int REQUIRED_STABLE_FRAMES = 3;
    private RtCalibrationState rtCalibrationState = RtCalibrationState.IDLE;
    private boolean rtAimRequested;
    private double lastCalibrationFrameTimestamp = Double.NaN;
    private int stableFrameCount;
    private Pose2D anchorVisionPose;
    private Pose2D anchorOdometryPose;
    private double anchorHiveAngleDegrees;
    private long lastCalibrationSampleNanos;
    private CellCluster rtSelectedCluster;
    private boolean validAimingTarget;
    private boolean validShootingTarget;
    private double targetHorizontalDistanceInches = Double.NaN;
    private double tagCenterHorizontalDistanceInches = Double.NaN;
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
        int requiredTagCount = Math.max(1, VisionConstants.minimumBotposeTagCount);
        if (botposeTagCount < requiredTagCount) {
            poseStatus = "Need at least " + requiredTagCount
                    + " MT1 localization tags; got " + botposeTagCount;
            return;
        }
        if (usableTagCount < requiredTagCount) {
            poseStatus = "Need at least " + requiredTagCount
                    + " alliance tags with area >= " + VisionConstants.minimumTargetArea
                    + "%; got " + usableTagCount;
            return;
        }
        hiveSolveResult = VisionMath.solveHiveFlatPose(
                rawRobotFieldPose, VisionConstants.hivePivotHeightInches,
                VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees
        );
        poseStatus = hiveSolveResult.status;
    }

    private void updateHiveCorrection() {
        if (hiveSolveResult == null) return;
        VisionMath.HiveCorrection correction = hiveSolveResult.correction;
        poseStatus = hiveSolveResult.status;
        hiveSolveSuccessful = correction != null;
        correctedRobotFieldPose = correction == null ? null : correction.robotPose;
        if (!hiveSolveSuccessful) return;
        hiveAngleDegrees = correction.hiveAngleDegrees;
        robotLevelErrorDegrees = correction.levelErrorDegrees;
        if (!rtAimRequested) {
            hasKnownHiveAngle = true;
            lastKnownHiveAngleDegrees = hiveAngleDegrees;
        }
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
    }

    // Distance-only callers keep normal calibration without starting an RT aim session.
    public void updateAimTarget(Drive drive, boolean stopped) {
        updateAimTarget(drive, stopped, false);
    }

    public void updateAimTarget(Drive drive, boolean stopped, boolean aimRequested) {
        updateAimTarget(drive, stopped, aimRequested, System.nanoTime());
    }

    void updateAimTarget(Drive drive, boolean stopped, boolean aimRequested, long nowNanos) {
        if (!aimRequested) {
            rtCalibrationState = RtCalibrationState.IDLE;
            rtSelectedCluster = null;
            if (rtAimRequested) resetCalibrationSamples();
        } else if (!rtAimRequested) {
            rtCalibrationState = RtCalibrationState.PAUSED;
            resetCalibrationSamples();
            rtSelectedCluster = drive.isFieldPoseInitialized()
                    ? CellCluster.forFieldX(alliance, drive.getPose().getX(DistanceUnit.INCH)).orElse(null)
                    : null;
        }
        rtAimRequested = aimRequested;
        if (stopped && aimRequested) rtCalibrationState = RtCalibrationState.FAILED;
        updateFieldTarget(drive, !stopped, nowNanos);
    }

    public void updateFieldTarget(Drive drive, boolean correctFromVision) {
        updateFieldTarget(drive, correctFromVision, System.nanoTime());
    }

    void updateFieldTarget(Drive drive, boolean correctFromVision, long nowNanos) {
        updateHiveCorrection();
        updateLocalization(drive, correctFromVision, nowNanos);
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
        tagCenterHorizontalDistanceInches = Double.NaN;
        yawErrorDegrees = Double.NaN;
        if (rtAimRequested && rtCalibrationState == RtCalibrationState.FAILED) {
            targetStatus = calibrationStatus;
            return;
        }
        if (!drive.isFieldPoseInitialized()) {
            targetStatus = "Waiting for visual field initialization or Pinpoint readiness";
            return;
        }
        Pose2D robotPose = drive.getPose();
        selectedCluster = rtAimRequested && rtCalibrationState == RtCalibrationState.PAUSED
                ? rtSelectedCluster : CellCluster.forFieldX(alliance, robotPose.getX(DistanceUnit.INCH))
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
        double[] tagCenter = VisionMath.tagClusterCenterForHiveTilt(selectedCluster,
                hasKnownHiveAngle ? lastKnownHiveAngleDegrees : Double.NaN);
        if (tagCenter != null) {
            tagCenterHorizontalDistanceInches = Math.hypot(
                    tagCenter[0] - robotPose.getX(DistanceUnit.INCH),
                    tagCenter[1] - robotPose.getY(DistanceUnit.INCH));
        }
        desiredHeadingDegrees = Math.IEEEremainder(
                Math.toDegrees(Math.atan2(dy, dx)) - VisionConstants.shooterYawDegrees, 360.0);
        yawErrorDegrees = VisionMath.coordinateYawErrorDegrees(
                robotPose, targetXInches, targetYInches, VisionConstants.shooterYawDegrees);
        validAimingTarget = Double.isFinite(yawErrorDegrees)
                && Double.isFinite(targetHorizontalDistanceInches);
        // Retain the last observed HIVE geometry while odometry continues through tag loss.
        targetHiveState = !hasKnownHiveAngle ? HiveState.UNKNOWN
                : Math.abs(lastKnownHiveAngleDegrees) <= VisionConstants.highHiveTiltThresholdDegrees
                ? HiveState.MIDDLE : -Math.sin(Math.toRadians(lastKnownHiveAngleDegrees))
                * targetXInches > 0.0 ? HiveState.HIGH : HiveState.LOW;
        validShootingTarget = validAimingTarget && targetHiveState == HiveState.HIGH;
        targetStatus = validAimingTarget ? "Coordinate aim ready from Pinpoint" : "Invalid aim geometry";
    }

    private void updateLocalization(Drive drive, boolean correctFromVision, long nowNanos) {
        double interval = VisionConstants.calibrationIntervalSeconds;
        boolean validInterval = Double.isFinite(interval) && interval >= 0.0;
        calibrationCooldownRemainingSeconds = calibrationSucceeded && validInterval
                ? Math.max(0.0, interval
                - (nowNanos - lastSuccessfulCalibrationNanos) / 1_000_000_000.0) : 0.0;
        if (!correctFromVision || rtAimRequested || !validInterval
                || calibrationCooldownRemainingSeconds > 0.0) {
            resetCalibrationSamples();
            // Frames observed while paused cannot become part of the next stable window.
            lastCalibrationFrameTimestamp = Double.isFinite(frameTimestamp)
                    ? frameTimestamp : lastPhysicalFrameTimestamp;
            if (!correctFromVision) {
                calibrationStatus = "Paused: B stop";
            } else if (rtAimRequested) {
                updateRtPause(drive);
            } else {
                calibrationStatus = validInterval ? "Waiting after successful calibration"
                        : "Invalid calibration interval";
            }
            return;
        }
        updateStableCalibration(drive, nowNanos);
    }

    private boolean applyVisionCalibration(Drive drive, long nowNanos) {
        boolean applied = drive.updateVisionPose(correctedRobotFieldPose, frameTimestamp, nowNanos);
        correctionApplied |= applied;
        if (applied) {
            calibrationSucceeded = true;
            lastSuccessfulCalibrationNanos = nowNanos;
            calibrationCooldownRemainingSeconds = VisionConstants.calibrationIntervalSeconds;
        }
        calibrationStatus = drive.getLocalizationStatus();
        return applied;
    }

    private void updateRtPause(Drive drive) {
        if (rtCalibrationState == RtCalibrationState.FAILED) return;
        if (!drive.isFieldPoseInitialized()) {
            rtCalibrationState = RtCalibrationState.FAILED;
            calibrationStatus = "RT field pose unavailable; release RT to calibrate";
        } else {
            calibrationStatus = "Paused: RT; using Pinpoint and cached HIVE geometry";
        }
    }

    private void updateStableCalibration(Drive drive, long nowNanos) {
        double positionTolerance = VisionConstants.rtStablePositionToleranceCm;
        double headingTolerance = VisionConstants.rtStableHeadingToleranceDegrees;
        double hiveTolerance = VisionConstants.rtStableHiveAngleToleranceDegrees;
        if (!Double.isFinite(positionTolerance) || positionTolerance < 0.0
                || !Double.isFinite(headingTolerance) || headingTolerance < 0.0
                || !Double.isFinite(hiveTolerance) || hiveTolerance < 0.0) {
            resetCalibrationSamples();
            calibrationStatus = "Invalid calibration stability settings";
            return;
        }
        Pose2D odometryPose = drive.getOdometryPose();
        if (!isFinitePose(correctedRobotFieldPose) || !Double.isFinite(hiveAngleDegrees)
                || !Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0
                || !isFinitePose(odometryPose)) {
            resetCalibrationSamples();
            if (Double.isFinite(frameTimestamp)) lastCalibrationFrameTimestamp = frameTimestamp;
            calibrationStatus = "Collecting 0/3: " + (!isFinitePose(odometryPose)
                    ? "Pinpoint " + drive.getOdometryStatus() + "; " + drive.getLocalizationStatus()
                    : poseStatus);
            return;
        }
        if (frameTimestamp == lastCalibrationFrameTimestamp) {
            calibrationStatus = "Collecting " + stableFrameCount + "/3; waiting for a new frame";
            return;
        }
        boolean restarted = Double.isFinite(lastCalibrationFrameTimestamp) && frameTimestamp < lastCalibrationFrameTimestamp;
        lastCalibrationFrameTimestamp = frameTimestamp;
        if (restarted || nowNanos - lastCalibrationSampleNanos > VisionConstants.maxResultAgeMs * 1_000_000L) {
            resetCalibrationSamples();
        }
        if (stableFrameCount > 0) {
            // Compare visual observations after transporting the first pose by relative odometry.
            // A large constant field-frame disagreement is allowed; it is never a rejection gate.
            double headingOffset = anchorVisionPose.getHeading(AngleUnit.RADIANS)
                    - anchorOdometryPose.getHeading(AngleUnit.RADIANS);
            double dx = odometryPose.getX(DistanceUnit.CM)
                    - anchorOdometryPose.getX(DistanceUnit.CM);
            double dy = odometryPose.getY(DistanceUnit.CM)
                    - anchorOdometryPose.getY(DistanceUnit.CM);
            double expectedX = anchorVisionPose.getX(DistanceUnit.CM)
                    + Math.cos(headingOffset) * dx - Math.sin(headingOffset) * dy;
            double expectedY = anchorVisionPose.getY(DistanceUnit.CM)
                    + Math.sin(headingOffset) * dx + Math.cos(headingOffset) * dy;
            double expectedHeading = anchorVisionPose.getHeading(AngleUnit.DEGREES)
                    + odometryPose.getHeading(AngleUnit.DEGREES)
                    - anchorOdometryPose.getHeading(AngleUnit.DEGREES);
            double positionDifference = Math.hypot(
                    correctedRobotFieldPose.getX(DistanceUnit.CM) - expectedX,
                    correctedRobotFieldPose.getY(DistanceUnit.CM) - expectedY);
            double headingDifference = Math.abs(Math.IEEEremainder(
                    correctedRobotFieldPose.getHeading(AngleUnit.DEGREES) - expectedHeading, 360.0));
            if (positionDifference > positionTolerance || headingDifference > headingTolerance
                    || Math.abs(hiveAngleDegrees - anchorHiveAngleDegrees) > hiveTolerance) {
                resetCalibrationSamples();
            }
        }
        if (stableFrameCount == 0) {
            anchorVisionPose = correctedRobotFieldPose;
            anchorOdometryPose = odometryPose;
            anchorHiveAngleDegrees = hiveAngleDegrees;
        }
        lastCalibrationSampleNanos = nowNanos;
        stableFrameCount++;
        calibrationStatus = "Collecting " + stableFrameCount + "/3";
        if (stableFrameCount < REQUIRED_STABLE_FRAMES) return;
        if (applyVisionCalibration(drive, nowNanos)) {
            hasKnownHiveAngle = true;
            lastKnownHiveAngleDegrees = hiveAngleDegrees;
            calibrationStatus = "Vision corrected 3/3; waiting after successful calibration";
        } else {
            calibrationStatus = "Waiting for a successful vision overwrite: " + drive.getLocalizationStatus();
        }
        resetCalibrationSamples();
    }

    private void resetCalibrationSamples() {
        stableFrameCount = 0;
        anchorVisionPose = null;
        anchorOdometryPose = null;
    }

    private static boolean isFinitePose(Pose2D pose) {
        return pose != null && Double.isFinite(pose.getX(DistanceUnit.CM))
                && Double.isFinite(pose.getY(DistanceUnit.CM))
                && Double.isFinite(pose.getHeading(AngleUnit.DEGREES));
    }

    private double[] getFieldTarget(CellCluster cluster) {
        double[][] targets = VisionConstants.cellTargetFieldXYInches;
        if (cluster == null || targets == null || cluster.getTuningIndex() >= targets.length) {
            return null;
        }
        double[] target = targets[cluster.getTuningIndex()];
        return VisionMath.cellTargetForHiveTilt(
                target, VisionConstants.cellTargetReferenceHeightInches,
                VisionConstants.hivePivotHeightInches, VisionConstants.cellTargetReferenceTiltDegrees,
                hasKnownHiveAngle ? lastKnownHiveAngleDegrees : Double.NaN);
    }

    public boolean isAimCalibrationReady() {
        return !rtAimRequested || rtCalibrationState == RtCalibrationState.PAUSED;
    }

    public String getRtCalibrationStatus() {
        if (!rtAimRequested) return "未启动";
        if (rtCalibrationState == RtCalibrationState.PAUSED) return "RT期间暂停校准；使用Pinpoint";
        if (rtCalibrationState == RtCalibrationState.FAILED) return "已停止；松开RT恢复校准";
        return "RT期间暂停校准";
    }

    public String getCalibrationStatus() { return calibrationStatus; }
    public double getCalibrationCooldownRemainingSeconds() { return calibrationCooldownRemainingSeconds; }
    public double getSecondsSinceLastCalibration() {
        return calibrationSucceeded
                ? Math.max(0.0, (System.nanoTime() - lastSuccessfulCalibrationNanos) / 1_000_000_000.0)
                : Double.NaN;
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
    public double getTagCenterHorizontalDistanceInches() { return tagCenterHorizontalDistanceInches; }
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
        tagCenterHorizontalDistanceInches = Double.NaN;
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
