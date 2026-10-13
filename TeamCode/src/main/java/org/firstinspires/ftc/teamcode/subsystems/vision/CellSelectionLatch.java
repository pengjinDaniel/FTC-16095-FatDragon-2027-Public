package org.firstinspires.ftc.teamcode.subsystems.vision;

/** Keeps one CELL per held aim request; bearing still follows the current robot pose. */
public final class CellSelectionLatch {
    private boolean held;
    private CellCluster latched;

    public void setHeld(boolean held) {
        this.held = held;
        if (!held) latched = null;
    }

    public CellCluster select(CellCluster candidate) {
        return latched != null ? latched : candidate;
    }

    /** Called only after the selected target's geometry passes validation. */
    public void confirm(CellCluster candidate) {
        if (held && latched == null && candidate != null) latched = candidate;
    }

    public CellCluster getLatched() { return latched; }
}
