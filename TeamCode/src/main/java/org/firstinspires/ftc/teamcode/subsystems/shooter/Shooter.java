package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Shooter extends SubsystemBase {
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final PIDFController leftController;
    private final PIDFController rightController;

    private double targetVelocity = Double.NaN;
    private boolean velocityControlActive;
    private boolean feeding;
    private boolean ready;
    private long withinToleranceSinceNanos = -1;

    public Shooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);

        leftShooter.setDirection(ShooterConstants.leftShooterDirection);
        rightShooter.setDirection(ShooterConstants.rightShooterDirection);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftController = new PIDFController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );
        rightController = new PIDFController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );

        idle();
    }

    public boolean setTargetFromDistance(double distanceInches) {
        double mappedVelocity = ShooterVelocityModel.getVelocityForDistance(
                distanceInches,
                ShooterConstants.distancePointsInches,
                ShooterConstants.velocityPointsTicksPerSecond
        );
        if (!Double.isFinite(mappedVelocity) || mappedVelocity <= 0.0) {
            return false;
        }

        setTargetVelocity(mappedVelocity);
        return hasTargetVelocity();
    }

    public void setTargetVelocity(double velocityTicksPerSecond) {
        if (!Double.isFinite(velocityTicksPerSecond) || velocityTicksPerSecond <= 0.0) {
            return;
        }

        double clippedVelocity = Range.clip(
                velocityTicksPerSecond,
                ShooterConstants.minVelocity,
                ShooterConstants.maxVelocity
        );

        if (!velocityControlActive
                || !Double.isFinite(targetVelocity)
                || Math.abs(clippedVelocity - targetVelocity)
                > ShooterConstants.targetChangeResetThreshold) {
            resetReadyState();
            leftController.reset();
            rightController.reset();
        }

        targetVelocity = clippedVelocity;
        velocityControlActive = true;
    }

    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
    }

    public void idle() {
        if (velocityControlActive || feeding || ready || Double.isFinite(targetVelocity)) {
            leftController.reset();
            rightController.reset();
        }

        velocityControlActive = false;
        feeding = false;
        targetVelocity = Double.NaN;
        resetReadyState();
        setPower(ShooterConstants.idlePower, ShooterConstants.idlePower);
    }

    public void stop() {
        velocityControlActive = false;
        feeding = false;
        targetVelocity = Double.NaN;
        resetReadyState();
        setPower(0.0, 0.0);
    }

    public boolean hasTargetVelocity() {
        return velocityControlActive && Double.isFinite(targetVelocity);
    }

    public boolean isReady() {
        return ready;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getLeftVelocity() {
        return Math.abs(leftShooter.getVelocity());
    }

    public double getRightVelocity() {
        return Math.abs(rightShooter.getVelocity());
    }

    @Override
    public void periodic() {
        if (!velocityControlActive || !Double.isFinite(targetVelocity)) {
            setPower(ShooterConstants.idlePower, ShooterConstants.idlePower);
            return;
        }

        leftController.setPIDF(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );
        rightController.setPIDF(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );

        double leftVelocity = getLeftVelocity();
        double rightVelocity = getRightVelocity();
        double loadBoost = feeding ? ShooterConstants.feedLoadBoost : 0.0;
        double leftOutput = Range.clip(
                leftController.calculate(leftVelocity, targetVelocity) + loadBoost,
                0.0,
                ShooterConstants.maxPower
        );
        double rightOutput = Range.clip(
                rightController.calculate(rightVelocity, targetVelocity) + loadBoost,
                0.0,
                ShooterConstants.maxPower
        );

        setPower(leftOutput, rightOutput);
        updateReadyState(leftVelocity, rightVelocity);
    }

    private void updateReadyState(double leftVelocity, double rightVelocity) {
        double maxError = Math.max(
                Math.abs(targetVelocity - leftVelocity),
                Math.abs(targetVelocity - rightVelocity)
        );

        if (ready) {
            if (maxError > ShooterConstants.readyExitTolerance) {
                resetReadyState();
            }
            return;
        }

        if (maxError > ShooterConstants.readyEnterTolerance) {
            withinToleranceSinceNanos = -1;
            return;
        }

        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0) {
            withinToleranceSinceNanos = now;
            return;
        }

        double stableSeconds = (now - withinToleranceSinceNanos) / 1_000_000_000.0;
        ready = stableSeconds >= ShooterConstants.readyStableTimeSeconds;
    }

    private void resetReadyState() {
        ready = false;
        withinToleranceSinceNanos = -1;
    }

    private void setPower(double leftPower, double rightPower) {
        leftShooter.setPower(leftPower);
        rightShooter.setPower(rightPower);
    }
}
