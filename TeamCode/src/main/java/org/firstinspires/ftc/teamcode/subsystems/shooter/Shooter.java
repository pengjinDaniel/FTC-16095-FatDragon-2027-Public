package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Shooter extends SubsystemBase {
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final Servo pitchServo;
    private final PIDFController leftController;
    private final PIDFController rightController;

    private double targetVelocity = Double.NaN;
    private boolean velocityControlActive;
    private boolean fixedPowerActive;
    private double stableLeftVelocity = Double.NaN;
    private double stableRightVelocity = Double.NaN;
    private boolean ready;
    private long withinToleranceSinceNanos = -1;
    private double pitchPosition = ShooterConstants.pitchStartPosition;
    private double pitchSettleReferencePosition = ShooterConstants.pitchStartPosition;
    private long pitchChangedAtNanos;

    public Shooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);

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

        pitchServo.setPosition(pitchPosition);
        pitchChangedAtNanos = System.nanoTime();
        idle();
    }

    public boolean startFixedPower() {
        if (!areFixedPowerValuesValid()) {
            return false;
        }
        if (!fixedPowerActive) {
            leftController.reset();
            rightController.reset();
            resetReadyState();
        }
        fixedPowerActive = true;
        velocityControlActive = false;
        targetVelocity = Double.NaN;
        setPitchPosition(ShooterConstants.pitchStartPosition);
        double power = Math.min(ShooterConstants.fixedPower, ShooterConstants.maxPower);
        setPower(power, power);
        return true;
    }

    public boolean setTargetFromDistance(double distanceInches) {
        if (!areVelocityControlValuesCalibrated()) {
            return false;
        }
        double mappedVelocity = ShooterVelocityModel.getVelocityForDistance(
                distanceInches,
                ShooterConstants.distancePointsInches,
                ShooterConstants.velocityPointsTicksPerSecond
        );
        double mappedPitch = ShooterVelocityModel.getPitchForDistance(
                distanceInches,
                ShooterConstants.distancePointsInches,
                ShooterConstants.pitchPoints
        );
        if (!Double.isFinite(mappedVelocity) || mappedVelocity <= 0.0
                || !Double.isFinite(mappedPitch)) {
            return false;
        }

        setTargetVelocity(mappedVelocity);
        setPitchPosition(mappedPitch);
        return hasTargetVelocity();
    }

    public double getPitchPosition() {
        return pitchPosition;
    }

    public boolean isPitchReady() {
        return (fixedPowerActive || hasTargetVelocity())
                && Double.isFinite(ShooterConstants.pitchSettleTimeSeconds)
                && ShooterConstants.pitchSettleTimeSeconds >= 0.0
                && (System.nanoTime() - pitchChangedAtNanos) / 1_000_000_000.0
                >= ShooterConstants.pitchSettleTimeSeconds;
    }

    public void setTargetVelocity(double velocityTicksPerSecond) {
        if (!areVelocityControlValuesCalibrated()
                || !Double.isFinite(velocityTicksPerSecond)
                || velocityTicksPerSecond <= 0.0) {
            return;
        }

        if (!velocityControlActive
                || !Double.isFinite(targetVelocity)
                || Math.abs(velocityTicksPerSecond - targetVelocity)
                > ShooterConstants.targetChangeResetThreshold) {
            resetReadyState();
            leftController.reset();
            rightController.reset();
        }

        fixedPowerActive = false;
        targetVelocity = velocityTicksPerSecond;
        velocityControlActive = true;
    }

    public void idle() {
        if (fixedPowerActive || velocityControlActive || ready || Double.isFinite(targetVelocity)) {
            leftController.reset();
            rightController.reset();
        }

        fixedPowerActive = false;
        velocityControlActive = false;
        targetVelocity = Double.NaN;
        resetReadyState();
        setPower(ShooterConstants.idlePower, ShooterConstants.idlePower);
    }

    public void stop() {
        fixedPowerActive = false;
        velocityControlActive = false;
        targetVelocity = Double.NaN;
        resetReadyState();
        leftController.reset();
        rightController.reset();
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

    public double getLeftPower() {
        return leftShooter.getPower();
    }

    public double getRightPower() {
        return rightShooter.getPower();
    }

    public String getControlMode() {
        return fixedPowerActive ? "FIXED POWER" : velocityControlActive ? "PID" : "IDLE";
    }

    @Override
    public void periodic() {
        if (fixedPowerActive) {
            if (!areFixedPowerValuesValid()) {
                stop();
                return;
            }
            setPitchPosition(ShooterConstants.pitchStartPosition);
            double power = Math.min(ShooterConstants.fixedPower, ShooterConstants.maxPower);
            setPower(power, power);
            updateFixedPowerReadyState(getLeftVelocity(), getRightVelocity());
            return;
        }
        if (!areVelocityControlValuesCalibrated()
                || !velocityControlActive || !Double.isFinite(targetVelocity)) {
            idle();
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
        double leftOutput = Range.clip(
                leftController.calculate(leftVelocity, targetVelocity),
                0.0,
                ShooterConstants.maxPower
        );
        double rightOutput = Range.clip(
                rightController.calculate(rightVelocity, targetVelocity),
                0.0,
                ShooterConstants.maxPower
        );

        setPower(leftOutput, rightOutput);
        updateReadyState(leftVelocity, rightVelocity);
    }

    private void updateFixedPowerReadyState(double leftVelocity, double rightVelocity) {
        if (!areReadyValuesValid() || !Double.isFinite(leftVelocity)
                || !Double.isFinite(rightVelocity) || leftVelocity <= 0.0 || rightVelocity <= 0.0) {
            resetReadyState();
            return;
        }

        double tolerance = ready ? ShooterConstants.readyExitTolerance
                : ShooterConstants.readyEnterTolerance;
        double maxRelativeChange = Math.max(
                Math.abs(leftVelocity - stableLeftVelocity) / stableLeftVelocity,
                Math.abs(rightVelocity - stableRightVelocity) / stableRightVelocity);
        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0 || !Double.isFinite(maxRelativeChange)
                || maxRelativeChange > tolerance) {
            stableLeftVelocity = leftVelocity;
            stableRightVelocity = rightVelocity;
            withinToleranceSinceNanos = now;
            ready = false;
            return;
        }
        ready = (now - withinToleranceSinceNanos) / 1_000_000_000.0
                >= ShooterConstants.readyStableTimeSeconds;
    }

    private void updateReadyState(double leftVelocity, double rightVelocity) {
        double maxError = Math.max(
                Math.abs(targetVelocity - leftVelocity),
                Math.abs(targetVelocity - rightVelocity)
        ) / targetVelocity;

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
        stableLeftVelocity = Double.NaN;
        stableRightVelocity = Double.NaN;
    }

    private boolean areFixedPowerValuesValid() {
        return Double.isFinite(ShooterConstants.fixedPower)
                && ShooterConstants.fixedPower > 0.0 && ShooterConstants.fixedPower <= 1.0
                && Double.isFinite(ShooterConstants.maxPower)
                && ShooterConstants.maxPower > 0.0 && ShooterConstants.maxPower <= 1.0
                && Double.isFinite(ShooterConstants.pitchStartPosition);
    }

    private boolean areVelocityControlValuesCalibrated() {
        return ShooterConstants.velocityControlCalibrated
                && Double.isFinite(ShooterConstants.kP)
                && Double.isFinite(ShooterConstants.kI)
                && Double.isFinite(ShooterConstants.kD)
                && Double.isFinite(ShooterConstants.kF)
                && (ShooterConstants.kP != 0.0 || ShooterConstants.kI != 0.0
                || ShooterConstants.kD != 0.0 || ShooterConstants.kF != 0.0)
                && areReadyValuesValid();
    }

    private boolean areReadyValuesValid() {
        return Double.isFinite(ShooterConstants.readyEnterTolerance)
                && ShooterConstants.readyEnterTolerance > 0.0
                && ShooterConstants.readyEnterTolerance <= 1.0
                && Double.isFinite(ShooterConstants.readyExitTolerance)
                && ShooterConstants.readyExitTolerance >= ShooterConstants.readyEnterTolerance
                && ShooterConstants.readyExitTolerance <= 1.0
                && Double.isFinite(ShooterConstants.readyStableTimeSeconds)
                && ShooterConstants.readyStableTimeSeconds > 0.0
                && Double.isFinite(ShooterConstants.targetChangeResetThreshold)
                && ShooterConstants.targetChangeResetThreshold >= 0.0;
    }

    private void setPower(double leftPower, double rightPower) {
        leftShooter.setPower(leftPower);
        rightShooter.setPower(rightPower);
    }

    private void setPitchPosition(double position) {
        double clippedPosition = Range.clip(position,
                ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition);
        if (Math.abs(clippedPosition - pitchSettleReferencePosition)
                > ShooterConstants.pitchChangeResetThreshold) {
            pitchChangedAtNanos = System.nanoTime();
            pitchSettleReferencePosition = clippedPosition;
        }
        pitchPosition = clippedPosition;
        pitchServo.setPosition(pitchPosition);
    }
}
