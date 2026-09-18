package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;
    private final Alliance alliance;
    private final CellTargetEstimator targetEstimator = new CellTargetEstimator();

    private boolean visibleAllianceTag;
    private boolean validShootingTarget;
    private double targetRange3DInches = Double.NaN;
    private double targetHorizontalDistanceInches = Double.NaN;
    private double yawErrorDegrees = Double.NaN;
    private Pose3D representativeTagPoseRobotSpace;
    private Pose3D targetCellPoseRobotSpace;
    private CellCluster selectedCluster;
    private HiveState targetHiveState = HiveState.UNKNOWN;
    private int selectedTagId = -1;
    private List<Integer> visibleTagIds = Collections.emptyList();
    private Map<Integer, Double> visibleTagHeightsInches = Collections.emptyMap();
    private Map<CellCluster, HiveState> observedClusterStates = Collections.emptyMap();

    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.pipelineSwitch(VisionConstants.pipelineIndex);
        limelight.start();
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        clearCurrentFrame();

        if (!limelight.isConnected()) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return;
        }

        List<Integer> currentVisibleIds = new ArrayList<>();
        Map<Integer, Double> currentTagHeights = new LinkedHashMap<>();
        Map<CellCluster, List<CellTargetEstimator.TagObservation>> groupedObservations =
                new EnumMap<>(CellCluster.class);

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            Optional<CellCluster> possibleCluster = CellCluster.forTagId(tagId);
            if (!possibleCluster.isPresent()
                    || !possibleCluster.get().isForAlliance(alliance)) {
                continue;
            }

            CellCluster cluster = possibleCluster.get();
            currentVisibleIds.add(tagId);

            Pose3D tagPoseRobotSpace = fiducial.getTargetPoseRobotSpace();
            Optional<VisionMath.RobotSpacePosition> position =
                    VisionMath.getRobotSpacePositionInches(tagPoseRobotSpace);
            if (position.isPresent()) {
                currentTagHeights.put(tagId, position.get().upInches);
            }

            double targetArea = fiducial.getTargetArea();
            if (!position.isPresent()
                    || !Double.isFinite(targetArea)
                    || targetArea < VisionConstants.minimumTargetArea
                    || position.get().range3DInches <= 0.0) {
                continue;
            }

            List<CellTargetEstimator.TagObservation> clusterObservations =
                    groupedObservations.get(cluster);
            if (clusterObservations == null) {
                clusterObservations = new ArrayList<>();
                groupedObservations.put(cluster, clusterObservations);
            }
            clusterObservations.add(new CellTargetEstimator.TagObservation(
                    tagId,
                    tagPoseRobotSpace,
                    targetArea,
                    position.get().upInches
            ));
        }

        visibleTagIds = Collections.unmodifiableList(currentVisibleIds);
        visibleTagHeightsInches = Collections.unmodifiableMap(currentTagHeights);
        visibleAllianceTag = !currentVisibleIds.isEmpty();

        TagToCellTransformMap transformMap = VisionConstants.tagToCellTransforms;
        if (transformMap == null) {
            transformMap = TagToCellTransformMap.empty();
        }
        applyEstimate(targetEstimator.estimate(groupedObservations, transformMap));
    }

    public boolean hasVisibleAllianceTag() {
        return visibleAllianceTag;
    }

    public boolean hasValidShootingTarget() {
        return validShootingTarget;
    }

    public double getTargetRange3DInches() {
        return targetRange3DInches;
    }

    public double getTargetHorizontalDistanceInches() {
        return targetHorizontalDistanceInches;
    }

    public double getYawErrorDegrees() {
        return yawErrorDegrees;
    }

    public Optional<Pose3D> getRepresentativeTagPoseRobotSpace() {
        return Optional.ofNullable(representativeTagPoseRobotSpace);
    }

    public Optional<Pose3D> getTargetCellPoseRobotSpace() {
        return Optional.ofNullable(targetCellPoseRobotSpace);
    }

    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    public Map<Integer, Double> getVisibleTagHeightsInches() {
        return visibleTagHeightsInches;
    }

    public Map<CellCluster, HiveState> getObservedClusterStates() {
        return observedClusterStates;
    }

    public Set<Integer> getCalibratedTransformTagIds() {
        TagToCellTransformMap transformMap = VisionConstants.tagToCellTransforms;
        if (transformMap == null) {
            return Collections.emptySet();
        }
        return transformMap.getCalibratedTagIds();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public Optional<CellCluster> getSelectedCluster() {
        return Optional.ofNullable(selectedCluster);
    }

    public int getSelectedTagId() {
        return selectedTagId;
    }

    public HiveState getTargetHiveState() {
        return targetHiveState;
    }

    public void stop() {
        limelight.stop();
        clearCurrentFrame();
    }

    private void applyEstimate(CellTargetEstimator.Result estimate) {
        selectedCluster = estimate.selectedCluster;
        selectedTagId = estimate.representativeTagId;
        representativeTagPoseRobotSpace = estimate.representativeTagPoseRobotSpace;
        targetCellPoseRobotSpace = estimate.cellCenterPoseRobotSpace;
        targetHiveState = estimate.hiveState;
        yawErrorDegrees = estimate.yawErrorDegrees;
        targetRange3DInches = estimate.range3DInches;
        targetHorizontalDistanceInches = estimate.horizontalDistanceInches;
        validShootingTarget = estimate.validShootingTarget;
        observedClusterStates = estimate.clusterStates;
    }

    private void clearCurrentFrame() {
        visibleAllianceTag = false;
        validShootingTarget = false;
        targetRange3DInches = Double.NaN;
        targetHorizontalDistanceInches = Double.NaN;
        yawErrorDegrees = Double.NaN;
        representativeTagPoseRobotSpace = null;
        targetCellPoseRobotSpace = null;
        selectedCluster = null;
        targetHiveState = HiveState.UNKNOWN;
        selectedTagId = -1;
        visibleTagIds = Collections.emptyList();
        visibleTagHeightsInches = Collections.emptyMap();
        observedClusterStates = Collections.emptyMap();
    }
}
