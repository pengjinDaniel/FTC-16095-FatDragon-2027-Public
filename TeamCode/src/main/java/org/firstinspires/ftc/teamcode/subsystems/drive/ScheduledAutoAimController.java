package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

public class ScheduledAutoAimController {
    private final PIDFController controller = new PIDFController(
            DriveConstants.autoAimKp,
            DriveConstants.autoAimKi,
            DriveConstants.autoAimKd,
            DriveConstants.autoAimKf
    );

    private final ScheduledTurnPid scheduled = new ScheduledTurnPid();
    private long previousNanos = -1;
    private boolean previousScheduledMode = AimTuneConstants.autoAimScheduledPidEnabled;
    private boolean locked;
    private long withinToleranceSinceNanos = -1;
    private double rawTurnPower;
    private boolean holding;

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        return calculateInternal(hasValidTarget, yawErrorDegrees, false, 0);
    }
    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees, double ccwGyroDegreesPerSecond) {
        return calculateInternal(hasValidTarget, yawErrorDegrees, true, ccwGyroDegreesPerSecond);
    }
    private double calculateInternal(boolean hasValidTarget, double yawErrorDegrees, boolean useGyro, double ccwGyroRate) {
        double maxTurnPower = DriveConstants.autoAimMaxTurnPower;
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(maxTurnPower) || maxTurnPower < 0.0
                || !validLockConfig() || (useGyro && !Double.isFinite(ccwGyroRate))
                || (!AimTuneConstants.autoAimScheduledPidEnabled && !validLegacyPid())) {
            reset();
            return 0.0;
        }

        if (previousScheduledMode != AimTuneConstants.autoAimScheduledPidEnabled) {
            reset(); previousScheduledMode = AimTuneConstants.autoAimScheduledPidEnabled;
        }
        controller.setPIDF(
                DriveConstants.autoAimKp,
                DriveConstants.autoAimKi,
                DriveConstants.autoAimKd,
                DriveConstants.autoAimKf
        );
        updateLockState(Math.abs(yawErrorDegrees));

        long now = System.nanoTime();
        double dt = previousNanos < 0 ? .02 : (now - previousNanos) / 1e9;
        previousNanos = now;
        ScheduledTurnPid.Config c = new ScheduledTurnPid.Config();
        c.nearP = AimTuneConstants.autoAimNearKp; c.nearI = AimTuneConstants.autoAimNearKi;
        c.nearD = AimTuneConstants.autoAimNearKd; c.farP = AimTuneConstants.autoAimFarKp;
        c.farI = AimTuneConstants.autoAimFarKi; c.farD = AimTuneConstants.autoAimFarKd;
        c.nearDegrees = AimTuneConstants.autoAimNearErrorDegrees;
        c.farDegrees = AimTuneConstants.autoAimFarErrorDegrees;
        c.nearMax = AimTuneConstants.autoAimNearMaxPower;
        c.farMax = Math.min(Math.min(maxTurnPower, 1.0), AimTuneConstants.autoAimFarMaxPower);
        c.slewPerSecond = AimTuneConstants.autoAimSlewPerSecond;
        // This controller accepts clockwise-positive error; scan export names physical directions.
        c.positiveStaticPower = AimTuneConstants.autoAimCwStaticPower;
        c.negativeStaticPower = AimTuneConstants.autoAimCcwStaticPower;
        c.stopToleranceDegrees = AimTuneConstants.autoAimStopToleranceDegrees;
        if (!Double.isFinite(dt) || dt <= 0 || dt > .25
                || (AimTuneConstants.autoAimScheduledPidEnabled && !c.valid())
                || !Double.isFinite(DriveConstants.autoAimYawToTurnSign)
                || Math.abs(DriveConstants.autoAimYawToTurnSign) != 1.0) {
            reset(); return 0.0;
        }
        if (AimTuneConstants.autoAimScheduledPidEnabled) {
            double error = Math.abs(yawErrorDegrees);
            if (error <= c.stopToleranceDegrees) holding = true;
            else if (error > Math.max(c.stopToleranceDegrees, DriveConstants.aimUnlockToleranceDegrees)) holding = false;
            if (holding) {
                // A noisy boundary crossing must not reapply the full friction kick.
                scheduled.reset(); rawTurnPower = 0; return 0;
            }
        }
        // Latest Vision emits desired-heading minus robot-heading (CCW degrees).
        // Drive.moveRobot accepts clockwise power; convert exactly once here.
        double output = (AimTuneConstants.autoAimScheduledPidEnabled
                ? (useGyro ? scheduled.calculateWithDerivative(-yawErrorDegrees, dt, c, ccwGyroRate)
                           : scheduled.calculate(-yawErrorDegrees, dt, c))
                : controller.calculate(-Math.toRadians(yawErrorDegrees), 0.0))
                * DriveConstants.autoAimYawToTurnSign;
        if (!Double.isFinite(output)) {
            reset();
            return 0.0;
        }
        rawTurnPower = output;
        return Range.clip(
                output,
                -Math.min(maxTurnPower, 1.0),
                Math.min(maxTurnPower, 1.0)
        );
    }

    public double getRawTurnPower() {
        return rawTurnPower;
    }

    private static boolean validLockConfig() {
        double enter = DriveConstants.aimLockToleranceDegrees;
        double exit = DriveConstants.aimUnlockToleranceDegrees;
        double seconds = DriveConstants.aimLockStableTimeSeconds;
        return Double.isFinite(enter) && Double.isFinite(exit) && Double.isFinite(seconds)
                && enter >= 0 && exit >= enter && seconds >= 0;
    }

    private static boolean validLegacyPid() {
        return Double.isFinite(DriveConstants.autoAimKp) && Double.isFinite(DriveConstants.autoAimKi)
                && Double.isFinite(DriveConstants.autoAimKd) && Double.isFinite(DriveConstants.autoAimKf);
    }

    public boolean isLocked() {
        return locked;
    }
    public boolean isHolding() { return holding; }

    public void reset() {
        locked = false;
        holding = false;
        withinToleranceSinceNanos = -1;
        rawTurnPower = 0.0;
        controller.reset();
        scheduled.reset(); previousNanos = -1;
    }

    private void updateLockState(double absoluteYawErrorDegrees) {
        if (locked) {
            if (absoluteYawErrorDegrees > DriveConstants.aimUnlockToleranceDegrees) {
                reset();
            }
            return;
        }

        if (absoluteYawErrorDegrees > DriveConstants.aimLockToleranceDegrees) {
            withinToleranceSinceNanos = -1;
            return;
        }

        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0) {
            withinToleranceSinceNanos = now;
            return;
        }

        double stableSeconds = (now - withinToleranceSinceNanos) / 1_000_000_000.0;
        locked = stableSeconds >= DriveConstants.aimLockStableTimeSeconds;
    }
}
