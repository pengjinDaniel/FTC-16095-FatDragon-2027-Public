package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

public class AutoAimController {
    private final PIDFController controller = new PIDFController(
            DriveConstants.autoAimKp,
            DriveConstants.autoAimKi,
            DriveConstants.autoAimKd,
            0.0
    );

    private boolean locked;
    private long withinToleranceSinceNanos = -1;
    private double rawTurnPower;
    private double acquisitionTurnDirection;

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees,
                                     boolean calibrationReady) {
        if (calibrationReady) {
            if (acquisitionTurnDirection != 0.0) reset();
            return calculateTurnPower(hasValidTarget, yawErrorDegrees);
        }
        double limit = getEffectiveMaxTurnPower();
        double power = DriveConstants.autoAimAcquisitionTurnPower;
        double steeringGain = DriveConstants.autoAimKp + DriveConstants.autoAimKf;
        double turnSign = DriveConstants.autoAimYawToTurnSign;
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(power) || power <= 0.0 || power > 1.0
                || !Double.isFinite(steeringGain) || steeringGain == 0.0
                || !Double.isFinite(turnSign) || turnSign == 0.0 || limit <= 0.0) {
            reset();
            return 0.0;
        }
        locked = false;
        withinToleranceSinceNanos = -1;
        if (acquisitionTurnDirection == 0.0) {
            // The old pose supplies only a direction; its error never controls search speed.
            acquisitionTurnDirection = Math.signum(steeringGain)
                    * Math.signum(yawErrorDegrees) * Math.signum(turnSign);
        }
        rawTurnPower = acquisitionTurnDirection * power;
        return Range.clip(rawTurnPower, -limit, limit);
    }

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        double maxTurnPower = getEffectiveMaxTurnPower();
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(maxTurnPower) || maxTurnPower <= 0.0) {
            reset();
            return 0.0;
        }

        controller.setPIDF(
                DriveConstants.autoAimKp,
                DriveConstants.autoAimKi,
                DriveConstants.autoAimKd,
                0.0
        );
        updateLockState(Math.abs(yawErrorDegrees));

        double yawErrorRadians = Math.toRadians(yawErrorDegrees);
        // A fixed setpoint lets FTCLib differentiate the changing error measurement.
        // Preserve the original error-scaled feedforward outside the zero-setpoint PID.
        double output = (controller.calculate(-yawErrorRadians, 0.0)
                + DriveConstants.autoAimKf * yawErrorRadians)
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

    public static double getEffectiveMaxTurnPower() {
        double controllerLimit = DriveConstants.autoAimMaxTurnPower;
        double testedLimit = DriveConstants.autoAimTestMaxTurnPower;
        if (!Double.isFinite(controllerLimit) || !Double.isFinite(testedLimit)
                || controllerLimit < 0.0 || testedLimit < 0.0) {
            return 0.0;
        }
        return Math.min(1.0, Math.min(controllerLimit, testedLimit));
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
        acquisitionTurnDirection = 0.0;
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
