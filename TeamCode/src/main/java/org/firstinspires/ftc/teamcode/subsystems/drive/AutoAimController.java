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
    private double derivativeTurnPower;

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        return calculateTurnPower(hasValidTarget, yawErrorDegrees, 0.0, false);
    }

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees,
                                    double headingVelocityDegreesPerSecond) {
        return calculateTurnPower(hasValidTarget, yawErrorDegrees,
                headingVelocityDegreesPerSecond, true);
    }

    private double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees,
                                      double headingVelocityDegreesPerSecond,
                                      boolean useHeadingVelocity) {
        double maxTurnPower = DriveConstants.autoAimMaxTurnPower;
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(maxTurnPower) || maxTurnPower < 0.0
                || (useHeadingVelocity && !Double.isFinite(headingVelocityDegreesPerSecond))) {
            reset();
            return 0.0;
        }

        controller.setPIDF(
                DriveConstants.autoAimKp,
                DriveConstants.autoAimKi,
                useHeadingVelocity ? 0.0 : DriveConstants.autoAimKd,
                DriveConstants.autoAimKf
        );
        updateLockState(Math.abs(yawErrorDegrees));

        double positionErrorDegrees = yawErrorDegrees;
        double acceptedToleranceDegrees = locked
                ? DriveConstants.aimUnlockToleranceDegrees : DriveConstants.aimLockToleranceDegrees;
        if (useHeadingVelocity && Math.abs(yawErrorDegrees) <= acceptedToleranceDegrees) {
            // Stop chasing accepted-angle noise; measured-rate braking remains active.
            positionErrorDegrees = 0.0;
            controller.clearTotalError();
        }

        double pidOutput = controller.calculate(-Math.toRadians(positionErrorDegrees), 0.0);
        // Pose corrections and angle wrapping must not be mistaken for physical rotation.
        double derivativeOutput = useHeadingVelocity
                ? -DriveConstants.autoAimKd * Math.toRadians(headingVelocityDegreesPerSecond)
                : DriveConstants.autoAimKd * controller.getVelocityError();
        double output = (pidOutput + (useHeadingVelocity ? derivativeOutput : 0.0))
                * DriveConstants.autoAimYawToTurnSign;
        if (!Double.isFinite(output)) {
            reset();
            return 0.0;
        }
        rawTurnPower = output;
        derivativeTurnPower = derivativeOutput * DriveConstants.autoAimYawToTurnSign;
        return Range.clip(
                output,
                -Math.min(maxTurnPower, 1.0),
                Math.min(maxTurnPower, 1.0)
        );
    }

    public double getRawTurnPower() {
        return rawTurnPower;
    }

    public double getDerivativeTurnPower() {
        return derivativeTurnPower;
    }

    public boolean isLocked() {
        return locked;
    }

    public void reset() {
        locked = false;
        withinToleranceSinceNanos = -1;
        rawTurnPower = 0.0;
        derivativeTurnPower = 0.0;
        controller.reset();
    }

    private void updateLockState(double absoluteYawErrorDegrees) {
        if (locked) {
            if (absoluteYawErrorDegrees > DriveConstants.aimUnlockToleranceDegrees) {
                // Keep PID history while unlocking so derivative braking stays active.
                locked = false;
                withinToleranceSinceNanos = -1;
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
