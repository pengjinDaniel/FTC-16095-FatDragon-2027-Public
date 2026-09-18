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

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)) {
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

        double output = controller.calculate(0.0, yawErrorDegrees)
                * DriveConstants.autoAimYawToTurnSign;
        return Range.clip(
                output,
                -DriveConstants.autoAimMaxTurnPower,
                DriveConstants.autoAimMaxTurnPower
        );
    }

    public boolean isLocked() {
        return locked;
    }

    public void reset() {
        locked = false;
        withinToleranceSinceNanos = -1;
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
