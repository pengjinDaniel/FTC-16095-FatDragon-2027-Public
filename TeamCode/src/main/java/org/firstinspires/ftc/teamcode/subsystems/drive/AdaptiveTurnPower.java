package org.firstinspires.ftc.teamcode.subsystems.drive;

/** Discovery-only friction compensation. Keep the PID gains and power ceilings explicit. */
public final class AdaptiveTurnPower {
    private double stationarySeconds;
    public void reset() { stationarySeconds = 0; }
    public boolean update(double errorDegrees, double angularSpeed, double dt, ScheduledTurnPid.Config c) {
        if (c == null || !c.valid() || !Double.isFinite(errorDegrees)
                || !Double.isFinite(angularSpeed) || !Double.isFinite(dt) || dt <= 0 || dt > .25
                || Math.abs(errorDegrees) <= c.stopToleranceDegrees || Math.abs(angularSpeed) > 2) {
            reset(); return false;
        }
        stationarySeconds += dt;
        if (stationarySeconds < .3) return false;
        stationarySeconds = 0;
        double limit = Math.max(0, Math.min(.25, c.nearMax - .02));
        double old = errorDegrees > 0 ? c.positiveStaticPower : c.negativeStaticPower;
        double next = Math.min(limit, old + .015);
        if (next <= old) return false;
        if (errorDegrees > 0) c.positiveStaticPower = next;
        else c.negativeStaticPower = next;
        return true;
    }
}
