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

    public static Optional<CellCluster> forTagId(int tagId) {
        for (CellCluster cluster : values()) {
            if (cluster.contains(tagId)) {
                return Optional.of(cluster);
            }
        }
        return Optional.empty();
    }
}
