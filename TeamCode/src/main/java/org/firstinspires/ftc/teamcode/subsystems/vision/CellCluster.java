package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public enum CellCluster {
    RED_OPPOSITE_AUDIENCE(Alliance.RED, 30, 33, 0),
    RED_AUDIENCE(Alliance.RED, 34, 37, 1),
    BLUE_AUDIENCE(Alliance.BLUE, 38, 41, 2),
    BLUE_OPPOSITE_AUDIENCE(Alliance.BLUE, 42, 45, 3);

    private final Alliance alliance;
    private final int firstTagId;
    private final int lastTagId;
    private final int tuningIndex;

    CellCluster(Alliance alliance, int firstTagId, int lastTagId, int tuningIndex) {
        this.alliance = alliance;
        this.firstTagId = firstTagId;
        this.lastTagId = lastTagId;
        this.tuningIndex = tuningIndex;
    }

    public boolean isForAlliance(Alliance requestedAlliance) {
        return alliance == requestedAlliance;
    }

    public boolean contains(int tagId) {
        return tagId >= firstTagId && tagId <= lastTagId;
    }

    public int getTuningIndex() {
        return tuningIndex;
    }

    // Only accepts the same unified field pose used by aiming and telemetry.
    // Centerline belongs to the audience half, matching the previous boundary policy.
    public static Optional<CellCluster> forFieldPose(Alliance alliance, Pose2D pose) {
        if (alliance == null || pose == null
                || !Double.isFinite(pose.getX(DistanceUnit.INCH))
                || !Double.isFinite(pose.getY(DistanceUnit.INCH))
                || !Double.isFinite(pose.getHeading(AngleUnit.RADIANS))) {
            return Optional.empty();
        }
        boolean audience = FieldCoordinates.isAudienceSide(pose);
        if (alliance == Alliance.RED) {
            return Optional.of(audience ? RED_AUDIENCE : RED_OPPOSITE_AUDIENCE);
        }
        return Optional.of(audience ? BLUE_AUDIENCE : BLUE_OPPOSITE_AUDIENCE);
    }

    public static Optional<CellCluster> forTagId(int tagId) {
        for (CellCluster cluster : values()) {
            if (cluster.contains(tagId)) {
                return Optional.of(cluster);
            }
        }
        return Optional.empty();
    }
}
