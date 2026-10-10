package org.firstinspires.ftc.teamcode.subsystems.drive;

/** Robot X forward/Y right; field headings CCW. No camera mounting transform here. */
public final class AimGeometry {
    private AimGeometry() {}
    public static double clockwiseError(double forward, double right, double shooterCcwDegrees) {
        if (!Double.isFinite(forward) || !Double.isFinite(right)
                || !Double.isFinite(shooterCcwDegrees) || Math.hypot(forward, right) < 1e-9)
            return Double.NaN;
        return ScheduledTurnPid.wrapDegrees(Math.toDegrees(Math.atan2(right, forward))
                + shooterCcwDegrees);
    }
    public static double desiredRobotHeadingRadians(double fieldDx, double fieldDy,
            double shooterCcwDegrees) {
        if (!Double.isFinite(fieldDx) || !Double.isFinite(fieldDy)
                || !Double.isFinite(shooterCcwDegrees) || Math.hypot(fieldDx, fieldDy) < 1e-9)
            return Double.NaN;
        return Math.IEEEremainder(Math.atan2(fieldDy, fieldDx)
                - Math.toRadians(shooterCcwDegrees), 2 * Math.PI);
    }
}
