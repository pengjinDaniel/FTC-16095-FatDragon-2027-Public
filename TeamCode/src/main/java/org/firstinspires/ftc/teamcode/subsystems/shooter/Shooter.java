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
    private double requestedFixedPower;
    private double leftPidOutput;
    private double rightPidOutput;
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
        stop();
    }

    public boolean setFixedPower(double power) {
        if (!Double.isFinite(power) || power < 0.0 || power > 1.0
                || !isPowerLimitValid() || !Double.isFinite(pitchPosition)) {
            stop();
            return false;
        }
        if (power == 0.0) {
            stop();
            return true;
        }
        if (!fixedPowerActive || requestedFixedPower != power) {
            leftController.reset();
            rightController.reset();
            resetReadyState();
        }
        requestedFixedPower = power;
        fixedPowerActive = true;
        velocityControlActive = false;
        targetVelocity = Double.NaN;
        leftPidOutput = 0.0;
        rightPidOutput = 0.0;
        double limitedPower = Math.min(power, ShooterConstants.maxPower);
        setPower(limitedPower, limitedPower);
        return true;
    }

    public boolean setDistanceTarget(double distanceCm) {
        if (!areDistanceShotValuesValid() || !Double.isFinite(distanceCm) || distanceCm < 0.0) {
            stop();
            return false;
        }
        // Clamp to the measured endpoints; do not extrapolate beyond either shot.
        double fraction = Range.clip(
                (distanceCm - ShooterConstants.centerDistanceCm)
                        / (ShooterConstants.wallDistanceCm - ShooterConstants.centerDistanceCm),
                0.0, 1.0);
        double velocity = (1.0 - fraction) * ShooterConstants.centerVelocityTicksPerSecond
                + fraction * ShooterConstants.wallVelocityTicksPerSecond;
        double pitch = (1.0 - fraction) * ShooterConstants.centerPitchPosition
                + fraction * ShooterConstants.wallPitchPosition;
        setPitchPosition(pitch);
        setTargetVelocity(velocity);
        return hasTargetVelocity();
    }

    public static boolean areDistanceShotValuesValid() {
        return areVelocityControlValuesCalibrated()
                && Double.isFinite(ShooterConstants.pitchMinPosition)
                && Double.isFinite(ShooterConstants.pitchMaxPosition)
                && ShooterConstants.pitchMinPosition >= 0.0
                && ShooterConstants.pitchMaxPosition <= 1.0
                && ShooterConstants.pitchMinPosition <= ShooterConstants.pitchMaxPosition
                && isDistancePointValid(ShooterConstants.centerDistanceCm,
                        ShooterConstants.centerVelocityTicksPerSecond, ShooterConstants.centerPitchPosition)
                && isDistancePointValid(ShooterConstants.wallDistanceCm,
                        ShooterConstants.wallVelocityTicksPerSecond, ShooterConstants.wallPitchPosition)
                && ShooterConstants.centerDistanceCm != ShooterConstants.wallDistanceCm;
    }

    private static boolean isDistancePointValid(double distanceCm, double velocity, double pitch) {
        return Double.isFinite(distanceCm) && distanceCm > 0.0
                && Double.isFinite(velocity) && velocity > 0.0
                && Double.isFinite(pitch) && pitch >= ShooterConstants.pitchMinPosition
                && pitch <= ShooterConstants.pitchMaxPosition;
    }

    public double getPitchPosition() {
        return pitchPosition;
    }

    public boolean isPitchReady() {
        return Double.isFinite(pitchPosition)
                && (fixedPowerActive || hasTargetVelocity())
                && Double.isFinite(ShooterConstants.pitchSettleTimeSeconds)
                && ShooterConstants.pitchSettleTimeSeconds >= 0.0
                && (System.nanoTime() - pitchChangedAtNanos) / 1_000_000_000.0
                >= ShooterConstants.pitchSettleTimeSeconds;
    }

    public void setTargetVelocity(double velocityTicksPerSecond) {
        if (!areVelocityControlValuesCalibrated()
                || !Double.isFinite(velocityTicksPerSecond)
                || velocityTicksPerSecond <= 0.0) {
            stop();
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
        setFixedPower(ShooterConstants.idlePower);
    }

    public void stop() {
        fixedPowerActive = false;
        velocityControlActive = false;
        targetVelocity = Double.NaN;
        leftPidOutput = 0.0;
        rightPidOutput = 0.0;
        resetReadyState();
        leftController.reset();
        rightController.reset();
        setPower(0.0, 0.0);
    }

    public boolean hasTargetVelocity() {
        return velocityControlActive && Double.isFinite(targetVelocity);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isReady() {
        return ready;
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

    public double getLeftPidOutput() {
        return leftPidOutput;
    }

    public double getRightPidOutput() {
        return rightPidOutput;
    }

    public String getControlMode() {
        return fixedPowerActive ? "FIXED POWER" : velocityControlActive ? "PID" : "IDLE";
    }

    @Override
    public void periodic() {
        if (fixedPowerActive) {
            if (!isPowerLimitValid() || !Double.isFinite(requestedFixedPower)
                    || !Double.isFinite(pitchPosition)) {
                stop();
                return;
            }
            double power = Math.min(requestedFixedPower, ShooterConstants.maxPower);
            setPower(power, power);
            updateFixedPowerReadyState(getLeftVelocity(), getRightVelocity());
            return;
        }
        if (!areVelocityControlValuesCalibrated()
                || !velocityControlActive || !Double.isFinite(targetVelocity)
                || !Double.isFinite(pitchPosition)) {
            stop();
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
        if (!Double.isFinite(leftVelocity) || !Double.isFinite(rightVelocity)) {
            stop();
            return;
        }
        leftPidOutput = leftController.calculate(leftVelocity, targetVelocity);
        rightPidOutput = rightController.calculate(rightVelocity, targetVelocity);
        if (!Double.isFinite(leftPidOutput) || !Double.isFinite(rightPidOutput)) {
            stop();
            return;
        }
        double leftOutput = Range.clip(leftPidOutput, 0.0, ShooterConstants.maxPower);
        double rightOutput = Range.clip(rightPidOutput, 0.0, ShooterConstants.maxPower);

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

    private static boolean isPowerLimitValid() {
        return Double.isFinite(ShooterConstants.maxPower)
                && ShooterConstants.maxPower > 0.0 && ShooterConstants.maxPower <= 1.0;
    }

    private static boolean areVelocityControlValuesCalibrated() {
        return ShooterConstants.velocityControlCalibrated
                && isPowerLimitValid()
                && Double.isFinite(ShooterConstants.kP)
                && Double.isFinite(ShooterConstants.kI)
                && Double.isFinite(ShooterConstants.kD)
                && Double.isFinite(ShooterConstants.kF)
                && (ShooterConstants.kP != 0.0 || ShooterConstants.kI != 0.0
                || ShooterConstants.kD != 0.0 || ShooterConstants.kF != 0.0)
                && areReadyValuesValid();
    }

    private static boolean areReadyValuesValid() {
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

    public void setPitchPosition(double position) {
        if (!Double.isFinite(position)
                || !Double.isFinite(ShooterConstants.pitchMinPosition)
                || !Double.isFinite(ShooterConstants.pitchMaxPosition)
                || ShooterConstants.pitchMinPosition < 0.0
                || ShooterConstants.pitchMaxPosition > 1.0
                || ShooterConstants.pitchMinPosition > ShooterConstants.pitchMaxPosition) {
            pitchPosition = Double.NaN;
            stop();
            return;
        }
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
