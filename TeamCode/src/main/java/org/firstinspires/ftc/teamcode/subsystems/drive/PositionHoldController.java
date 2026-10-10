package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PositionHoldController {
    private final PIDFController xController = new PIDFController(
            DriveConstants.positionHoldKp,
            DriveConstants.positionHoldKi,
            DriveConstants.positionHoldKd,
            0.0
    );
    private final PIDFController yController = new PIDFController(
            DriveConstants.positionHoldKp,
            DriveConstants.positionHoldKi,
            DriveConstants.positionHoldKd,
            0.0
    );

    private boolean anchored;
    private boolean locked;
    private double anchorX;
    private double anchorY;
    private double forwardPower;
    private double strafePower;
    private double positionErrorInches = Double.NaN;
    private long withinToleranceSinceNanos = -1;

    public void update(Pose2D pose, GoBildaPinpointDriver.DeviceStatus status,
                       double speedInchesPerSecond) {
        forwardPower = 0.0;
        strafePower = 0.0;
        if (!hasValidConfiguration() || pose == null
                || status != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(speedInchesPerSecond)) {
            reset();
            return;
        }

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            reset();
            return;
        }

        if (!anchored) {
            anchorX = x;
            anchorY = y;
            anchored = true;
            positionErrorInches = 0.0;
            xController.reset();
            yController.reset();
            return;
        }

        positionErrorInches = Math.hypot(anchorX - x, anchorY - y);
        updateLockState(positionErrorInches, speedInchesPerSecond);

        if (positionErrorInches <= DriveConstants.positionLockEnterToleranceInches) {
            xController.reset();
            yController.reset();
            return;
        }

        xController.setPIDF(
                DriveConstants.positionHoldKp,
                DriveConstants.positionHoldKi,
                DriveConstants.positionHoldKd,
                0.0
        );
        yController.setPIDF(
                DriveConstants.positionHoldKp,
                DriveConstants.positionHoldKi,
                DriveConstants.positionHoldKd,
                0.0
        );

        double maxPower = Math.max(0.0, Math.min(1.0, DriveConstants.positionHoldMaxPower));
        double fieldXPower = Range.clip(xController.calculate(x, anchorX), -maxPower, maxPower);
        double fieldYPower = Range.clip(yController.calculate(y, anchorY), -maxPower, maxPower);
        double magnitude = Math.hypot(fieldXPower, fieldYPower);
        if (magnitude > maxPower && magnitude > 0.0) {
            fieldXPower *= maxPower / magnitude;
            fieldYPower *= maxPower / magnitude;
        }

        forwardPower = fieldXPower * Math.cos(heading) + fieldYPower * Math.sin(heading);
        strafePower = fieldXPower * Math.sin(heading) - fieldYPower * Math.cos(heading);
    }

    public void reset() {
        anchored = false;
        locked = false;
        forwardPower = 0.0;
        strafePower = 0.0;
        positionErrorInches = Double.NaN;
        withinToleranceSinceNanos = -1;
        xController.reset();
        yController.reset();
    }

    public boolean isLocked() {
        return locked;
    }

    public double getForwardPower() {
        return forwardPower;
    }

    public double getStrafePower() {
        return strafePower;
    }

    public double getPositionErrorInches() {
        return positionErrorInches;
    }

    private static boolean hasValidConfiguration() {
        return DriveConstants.positionHoldCalibrated
                && Double.isFinite(DriveConstants.positionHoldKp)
                && Double.isFinite(DriveConstants.positionHoldKi)
                && Double.isFinite(DriveConstants.positionHoldKd)
                && Double.isFinite(DriveConstants.positionHoldMaxPower)
                && DriveConstants.positionHoldMaxPower > 0.0
                && DriveConstants.positionHoldMaxPower <= 1.0
                && Double.isFinite(DriveConstants.positionLockEnterToleranceInches)
                && DriveConstants.positionLockEnterToleranceInches >= 0.0
                && Double.isFinite(DriveConstants.positionLockExitToleranceInches)
                && DriveConstants.positionLockExitToleranceInches
                >= DriveConstants.positionLockEnterToleranceInches
                && Double.isFinite(DriveConstants.positionLockMaxSpeedInchesPerSecond)
                && DriveConstants.positionLockMaxSpeedInchesPerSecond >= 0.0
                && Double.isFinite(DriveConstants.positionLockStableTimeSeconds)
                && DriveConstants.positionLockStableTimeSeconds >= 0.0;
    }

    private void updateLockState(double errorInches, double speedInchesPerSecond) {
        if (locked) {
            if (errorInches > DriveConstants.positionLockExitToleranceInches
                    || speedInchesPerSecond > DriveConstants.positionLockMaxSpeedInchesPerSecond) {
                locked = false;
                withinToleranceSinceNanos = -1;
            }
            return;
        }

        if (errorInches > DriveConstants.positionLockEnterToleranceInches
                || speedInchesPerSecond > DriveConstants.positionLockMaxSpeedInchesPerSecond) {
            withinToleranceSinceNanos = -1;
            return;
        }

        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0) {
            withinToleranceSinceNanos = now;
            return;
        }

        locked = (now - withinToleranceSinceNanos) / 1_000_000_000.0
                >= DriveConstants.positionLockStableTimeSeconds;
    }
}
