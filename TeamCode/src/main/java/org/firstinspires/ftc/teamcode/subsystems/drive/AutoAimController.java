package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

public class AutoAimController {
    private final PIDFController controller = new PIDFController(
            DriveConstants.autoAimKp,
            DriveConstants.autoAimKi,
            DriveConstants.autoAimKd,
            DriveConstants.autoAimKf
    );

    private boolean locked;
    private long withinToleranceSinceNanos = -1;
    private double rawTurnPower;

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        double maxTurnPower = DriveConstants.autoAimMaxTurnPower;
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(maxTurnPower) || maxTurnPower < 0.0) {
            reset();
            return 0.0;
        }

        controller.setPIDF(
                DriveConstants.autoAimKp,
                DriveConstants.autoAimKi,
                DriveConstants.autoAimKd,
                DriveConstants.autoAimKf
        );
        updateLockState(Math.abs(yawErrorDegrees));

        // A fixed setpoint lets FTCLib differentiate changes in the measurement.
        double output = controller.calculate(-Math.toRadians(yawErrorDegrees), 0.0)
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

    public boolean isLocked() {
        return locked;
    }

    public void reset() {
        locked = false;
        withinToleranceSinceNanos = -1;
        rawTurnPower = 0.0;
        controller.reset();
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
