package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public final class TagToCellTransformMap {
    private final Map<Integer, TagToCellTransform> transforms;

    public TagToCellTransformMap(Map<Integer, TagToCellTransform> transforms) {
        Map<Integer, TagToCellTransform> validatedTransforms = new LinkedHashMap<>();
        if (transforms != null) {
            for (Map.Entry<Integer, TagToCellTransform> entry : transforms.entrySet()) {
                if (!CellCluster.forTagId(entry.getKey()).isPresent()) {
                    throw new IllegalArgumentException(
                            "Unsupported BIOBUZZ AprilTag ID: " + entry.getKey()
                    );
                }
                if (entry.getValue() == null) {
                    throw new IllegalArgumentException(
                            "Tag-to-CELL transform cannot be null for tag " + entry.getKey()
                    );
                }
                validatedTransforms.put(entry.getKey(), entry.getValue());
            }
        }
        this.transforms = Collections.unmodifiableMap(validatedTransforms);
    }

    public static TagToCellTransformMap empty() {
        return new TagToCellTransformMap(Collections.emptyMap());
    }

    public Optional<TagToCellTransform> getTagToCellCenterTransform(int tagId) {
        return Optional.ofNullable(transforms.get(tagId));
    }

    public Set<Integer> getCalibratedTagIds() {
        return transforms.keySet();
    }

    public static final class TagToCellTransform {
        private final Pose3D cellCenterPoseInTagSpace;

        /**
         * @param cellCenterPoseInTagSpace measured T_tag_cell transform
         */
        public TagToCellTransform(Pose3D cellCenterPoseInTagSpace) {
            if (!VisionMath.isValidPose(cellCenterPoseInTagSpace)) {
                throw new IllegalArgumentException("Tag-to-CELL transform must be a finite pose");
            }
            this.cellCenterPoseInTagSpace = cellCenterPoseInTagSpace;
        }

        public Optional<Pose3D> estimateCellCenterPoseRobotSpace(Pose3D tagPoseRobotSpace) {
            // T_robot_cell = T_robot_tag * T_tag_cell.
            return VisionMath.compose(tagPoseRobotSpace, cellCenterPoseInTagSpace);
        }
    }
}
