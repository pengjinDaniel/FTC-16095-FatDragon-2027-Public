package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

final class CellTargetEstimator {
    private static final int FIRST_BIOBUZZ_TAG_ID = 30;

    Result estimate(
            Map<CellCluster, List<TagObservation>> groupedObservations,
            TagToCellTransformMap transformMap
    ) {
        Map<CellCluster, HiveState> clusterStates = new EnumMap<>(CellCluster.class);
        List<ClusterEvaluation> evaluations = new ArrayList<>();
        List<ClusterEvaluation> highEvaluations = new ArrayList<>();

        for (Map.Entry<CellCluster, List<TagObservation>> entry
                : groupedObservations.entrySet()) {
            ClusterEvaluation evaluation = evaluateCluster(
                    entry.getKey(),
                    entry.getValue(),
                    transformMap
            );
            evaluations.add(evaluation);
            clusterStates.put(evaluation.cluster, evaluation.hiveState);
            if (evaluation.hiveState == HiveState.HIGH) {
                highEvaluations.add(evaluation);
            }
        }

        ClusterEvaluation selectedEvaluation = null;
        if (highEvaluations.size() == 1) {
            selectedEvaluation = highEvaluations.get(0);
        } else if (highEvaluations.isEmpty() && evaluations.size() == 1) {
            // Keep one unambiguous UNKNOWN observation visible for calibration telemetry.
            selectedEvaluation = evaluations.get(0);
        }

        // Two independently classified HIGH CELLs are contradictory, so no target is selected.
        if (selectedEvaluation == null) {
            return Result.empty(clusterStates);
        }

        return buildResult(selectedEvaluation, clusterStates);
    }

    private ClusterEvaluation evaluateCluster(
            CellCluster cluster,
            List<TagObservation> observations,
            TagToCellTransformMap transformMap
    ) {
        HiveState hiveState = classifyHiveState(observations);
        Optional<Pose3D> cellCenterPose = estimateCellCenterPose(observations, transformMap);

        TagObservation representative = null;
        for (TagObservation observation : observations) {
            if (representative == null || observation.targetArea > representative.targetArea) {
                representative = observation;
            }
        }

        return new ClusterEvaluation(cluster, hiveState, cellCenterPose, representative);
    }

    private HiveState classifyHiveState(List<TagObservation> observations) {
        int minimumObservations = Math.max(
                1,
                VisionConstants.minimumPoseObservationsForHiveState
        );
        double tolerance = VisionConstants.tagHeightToleranceInches;
        if (observations.size() < minimumObservations
                || !Double.isFinite(tolerance)
                || tolerance < 0.0) {
            return HiveState.UNKNOWN;
        }

        for (TagObservation observation : observations) {
            double highHeight = getTagHeightReference(
                    VisionConstants.highTagHeightsInches,
                    observation.tagId
            );
            if (!Double.isFinite(highHeight)
                    || Math.abs(observation.tagHeightInches - highHeight) > tolerance) {
                return HiveState.UNKNOWN;
            }
        }

        return HiveState.HIGH;
    }

    private Optional<Pose3D> estimateCellCenterPose(
            List<TagObservation> observations,
            TagToCellTransformMap transformMap
    ) {
        List<Pose3D> cellCenterEstimates = new ArrayList<>();
        List<Double> confidenceWeights = new ArrayList<>();

        for (TagObservation observation : observations) {
            Optional<TagToCellTransformMap.TagToCellTransform> transform =
                    transformMap.getTagToCellCenterTransform(observation.tagId);
            if (!transform.isPresent()) {
                continue;
            }

            Optional<Pose3D> estimate = transform.get()
                    .estimateCellCenterPoseRobotSpace(observation.tagPoseRobotSpace);
            if (!estimate.isPresent()) {
                continue;
            }

            cellCenterEstimates.add(estimate.get());
            confidenceWeights.add(observation.targetArea);
        }

        // Translation is confidence-weighted in one robot-relative frame. Rotation uses a
        // sign-aligned quaternion average, never a component-wise Euler-angle average.
        return VisionMath.weightedAveragePoses(cellCenterEstimates, confidenceWeights);
    }

    private Result buildResult(
            ClusterEvaluation evaluation,
            Map<CellCluster, HiveState> clusterStates
    ) {
        Pose3D cellCenterPose = evaluation.cellCenterPose.orElse(null);
        double yawErrorDegrees = Double.NaN;
        double range3DInches = Double.NaN;
        double horizontalDistanceInches = Double.NaN;

        if (cellCenterPose != null) {
            Optional<VisionMath.RobotSpacePosition> position =
                    VisionMath.getRobotSpacePositionInches(cellCenterPose);
            if (position.isPresent()) {
                VisionMath.RobotSpacePosition robotSpacePosition = position.get();
                int tuningIndex = evaluation.cluster.getTuningIndex();
                yawErrorDegrees = Math.toDegrees(Math.atan2(
                        robotSpacePosition.lateralInches,
                        robotSpacePosition.forwardInches
                )) + getTuningValue(
                        VisionConstants.clusterYawOffsetsDegrees,
                        tuningIndex
                );
                range3DInches = robotSpacePosition.range3DInches + getTuningValue(
                        VisionConstants.clusterRange3DOffsetsInches,
                        tuningIndex
                );
                horizontalDistanceInches = robotSpacePosition.horizontalDistanceInches
                        + getTuningValue(
                        VisionConstants.clusterHorizontalDistanceOffsetsInches,
                        tuningIndex
                );
            }
        }

        boolean validShootingTarget = evaluation.hiveState == HiveState.HIGH
                && Double.isFinite(yawErrorDegrees)
                && Double.isFinite(range3DInches)
                && range3DInches > 0.0
                && Double.isFinite(horizontalDistanceInches)
                && horizontalDistanceInches > 0.0;

        return new Result(
                evaluation.cluster,
                evaluation.hiveState,
                evaluation.representativeTag == null
                        ? -1
                        : evaluation.representativeTag.tagId,
                evaluation.representativeTag == null
                        ? null
                        : evaluation.representativeTag.tagPoseRobotSpace,
                cellCenterPose,
                yawErrorDegrees,
                range3DInches,
                horizontalDistanceInches,
                validShootingTarget,
                clusterStates
        );
    }

    private static double getTagHeightReference(double[] references, int tagId) {
        int index = tagId - FIRST_BIOBUZZ_TAG_ID;
        if (references == null || index < 0 || index >= references.length) {
            return Double.NaN;
        }
        return references[index];
    }

    private static double getTuningValue(double[] values, int index) {
        if (values == null || index < 0 || index >= values.length) {
            return 0.0;
        }
        return values[index];
    }

    static final class TagObservation {
        final int tagId;
        final Pose3D tagPoseRobotSpace;
        final double targetArea;
        final double tagHeightInches;

        TagObservation(
                int tagId,
                Pose3D tagPoseRobotSpace,
                double targetArea,
                double tagHeightInches
        ) {
            this.tagId = tagId;
            this.tagPoseRobotSpace = tagPoseRobotSpace;
            this.targetArea = targetArea;
            this.tagHeightInches = tagHeightInches;
        }
    }

    static final class Result {
        final CellCluster selectedCluster;
        final HiveState hiveState;
        final int representativeTagId;
        final Pose3D representativeTagPoseRobotSpace;
        final Pose3D cellCenterPoseRobotSpace;
        final double yawErrorDegrees;
        final double range3DInches;
        final double horizontalDistanceInches;
        final boolean validShootingTarget;
        final Map<CellCluster, HiveState> clusterStates;

        Result(
                CellCluster selectedCluster,
                HiveState hiveState,
                int representativeTagId,
                Pose3D representativeTagPoseRobotSpace,
                Pose3D cellCenterPoseRobotSpace,
                double yawErrorDegrees,
                double range3DInches,
                double horizontalDistanceInches,
                boolean validShootingTarget,
                Map<CellCluster, HiveState> clusterStates
        ) {
            this.selectedCluster = selectedCluster;
            this.hiveState = hiveState;
            this.representativeTagId = representativeTagId;
            this.representativeTagPoseRobotSpace = representativeTagPoseRobotSpace;
            this.cellCenterPoseRobotSpace = cellCenterPoseRobotSpace;
            this.yawErrorDegrees = yawErrorDegrees;
            this.range3DInches = range3DInches;
            this.horizontalDistanceInches = horizontalDistanceInches;
            this.validShootingTarget = validShootingTarget;
            this.clusterStates = Collections.unmodifiableMap(
                    new EnumMap<>(clusterStates)
            );
        }

        static Result empty(Map<CellCluster, HiveState> clusterStates) {
            return new Result(
                    null,
                    HiveState.UNKNOWN,
                    -1,
                    null,
                    null,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    false,
                    clusterStates
            );
        }
    }

    private static final class ClusterEvaluation {
        final CellCluster cluster;
        final HiveState hiveState;
        final Optional<Pose3D> cellCenterPose;
        final TagObservation representativeTag;

        ClusterEvaluation(
                CellCluster cluster,
                HiveState hiveState,
                Optional<Pose3D> cellCenterPose,
                TagObservation representativeTag
        ) {
            this.cluster = cluster;
            this.hiveState = hiveState;
            this.cellCenterPose = cellCenterPose;
            this.representativeTag = representativeTag;
        }
    }
}
