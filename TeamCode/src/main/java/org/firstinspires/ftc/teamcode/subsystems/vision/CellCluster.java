package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.Optional;

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

    // In the 19725 fmap, audience-side CELLS have positive field X.
    // The center line belongs to the positive-X half for deterministic selection.
    public static Optional<CellCluster> forFieldX(Alliance alliance, double fieldXInches) {
        if (alliance == null || !Double.isFinite(fieldXInches)) {
            return Optional.empty();
        }
        if (alliance == Alliance.RED) {
            return Optional.of(fieldXInches >= 0.0 ? RED_AUDIENCE : RED_OPPOSITE_AUDIENCE);
        }
        return Optional.of(fieldXInches >= 0.0 ? BLUE_AUDIENCE : BLUE_OPPOSITE_AUDIENCE);
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
