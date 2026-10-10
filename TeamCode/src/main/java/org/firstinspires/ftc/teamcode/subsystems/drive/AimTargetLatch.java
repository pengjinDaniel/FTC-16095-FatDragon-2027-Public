package org.firstinspires.ftc.teamcode.subsystems.drive;

/** Keep one physical CELL target for each trigger hold, even near the field-half boundary. */
public final class AimTargetLatch {
    private boolean latched;
    private double x, y;
    public void clear() { latched = false; }
    public boolean update(boolean requested, boolean refresh, boolean valid, double targetX, double targetY) {
        if (!requested || refresh) clear();
        if (requested && !latched && valid && Double.isFinite(targetX) && Double.isFinite(targetY)) {
            x = targetX; y = targetY; latched = true;
        }
        return latched;
    }
    public boolean isLatched() { return latched; }
    public double x() { return latched ? x : Double.NaN; }
    public double y() { return latched ? y : Double.NaN; }
    public double errorDegrees(double robotX, double robotY, double headingDegrees, double shooterYawDegrees) {
        if (!latched || !Double.isFinite(headingDegrees)) return Double.NaN;
        double desired = AimGeometry.desiredRobotHeadingRadians(x - robotX, y - robotY, shooterYawDegrees);
        return ScheduledTurnPid.wrapDegrees(Math.toDegrees(desired) - headingDegrees);
    }
}
