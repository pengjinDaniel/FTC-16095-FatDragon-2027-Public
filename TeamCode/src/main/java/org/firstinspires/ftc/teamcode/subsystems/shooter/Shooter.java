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
    // Both motors drive one flywheel; only the right encoder supplies speed feedback.
    private final PIDFController rightController;

    private double targetVelocity = Double.NaN;
    private double velocitySettleReferenceTarget = Double.NaN;
    private boolean velocityControlActive;
    private boolean fixedPowerActive;
    private double requestedFixedPower;
    private double leftPidOutput;
    private double rightPidOutput;
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
        return setDistanceTarget(distanceCm, true);
    }

    public boolean setDistanceTarget(double distanceCm, boolean updatePitch) {
        double[][] points = getDistanceShotPoints();
        if (!areDistanceShotValuesValid(points) || !Double.isFinite(distanceCm) || distanceCm < 0.0) {
            stop();
            return false;
        }
        double[] lower = null;
        double[] upper = null;
        for (double[] point : points) {
            if (point[0] <= distanceCm && (lower == null || point[0] > lower[0])) lower = point;
            if (point[0] >= distanceCm && (upper == null || point[0] < upper[0])) upper = point;
        }
        // Clamp to the configured endpoints; interpolate only between adjacent points.
        if (lower == null) lower = upper;
        if (upper == null) upper = lower;
        double fraction = lower == upper ? 0.0
                : (distanceCm - lower[0]) / (upper[0] - lower[0]);
        double velocity = (1.0 - fraction) * lower[1] + fraction * upper[1];
        if (updatePitch) {
            double pitch = (1.0 - fraction) * lower[2] + fraction * upper[2];
            setPitchPosition(pitch);
        }
        setTargetVelocity(velocity);
        return hasTargetVelocity();
    }

    public static boolean areDistanceShotValuesValid() {
        return areDistanceShotValuesValid(getDistanceShotPoints());
    }

    private static double[][] getDistanceShotPoints() {
        double[][] additional = ShooterConstants.additionalShotPoints;
        if (additional == null) return null;
        double[][] points = new double[additional.length + 2][];
        points[0] = new double[]{ShooterConstants.centerDistanceCm,
                ShooterConstants.centerVelocityTicksPerSecond, ShooterConstants.centerPitchPosition};
        points[1] = new double[]{ShooterConstants.wallDistanceCm,
                ShooterConstants.wallVelocityTicksPerSecond, ShooterConstants.wallPitchPosition};
        System.arraycopy(additional, 0, points, 2, additional.length);
        return points;
    }

    private static boolean areDistanceShotValuesValid(double[][] points) {
        if (!areVelocityControlValuesCalibrated() || points == null
                || !Double.isFinite(ShooterConstants.pitchMinPosition)
                || !Double.isFinite(ShooterConstants.pitchMaxPosition)
                || ShooterConstants.pitchMinPosition < 0.0
                || ShooterConstants.pitchMaxPosition > 1.0
                || ShooterConstants.pitchMinPosition > ShooterConstants.pitchMaxPosition) {
            return false;
        }
        for (int i = 0; i < points.length; i++) {
            double[] point = points[i];
            if (point == null || point.length != 3
                    || !isDistancePointValid(point[0], point[1], point[2])) return false;
            for (int j = 0; j < i; j++) {
                if (point[0] == points[j][0]) return false;
            }
        }
        return true;
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
                || !Double.isFinite(velocitySettleReferenceTarget)
                || Math.abs(velocityTicksPerSecond - velocitySettleReferenceTarget)
                > ShooterConstants.targetChangeResetThreshold) {
            velocitySettleReferenceTarget = velocityTicksPerSecond;
            resetReadyState();
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
        velocitySettleReferenceTarget = Double.NaN;
        leftPidOutput = 0.0;
        rightPidOutput = 0.0;
        resetReadyState();
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
            updateFixedPowerReadyState(getRightVelocity());
            return;
        }
        if (!areVelocityControlValuesCalibrated()
                || !velocityControlActive || !Double.isFinite(targetVelocity)
                || !Double.isFinite(pitchPosition)) {
            stop();
            return;
        }

        rightController.setPIDF(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );

        double rightVelocity = getRightVelocity();
        if (!Double.isFinite(rightVelocity)) {
            stop();
            return;
        }
        rightPidOutput = rightController.calculate(rightVelocity, targetVelocity);
        leftPidOutput = rightPidOutput;
        if (!Double.isFinite(rightPidOutput)) {
            stop();
            return;
        }
        double output = Range.clip(rightPidOutput, 0.0, ShooterConstants.maxPower);

        setPower(output, output);
        updateReadyState(rightVelocity);
    }

    private void updateFixedPowerReadyState(double rightVelocity) {
        if (!areReadyValuesValid() || !Double.isFinite(rightVelocity) || rightVelocity <= 0.0) {
            resetReadyState();
            return;
        }

        double tolerance = ready ? ShooterConstants.readyExitTolerance
                : ShooterConstants.readyEnterTolerance;
        double relativeChange = Math.abs(rightVelocity - stableRightVelocity) / stableRightVelocity;
        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0 || !Double.isFinite(relativeChange)
                || relativeChange > tolerance) {
            stableRightVelocity = rightVelocity;
            withinToleranceSinceNanos = now;
            ready = false;
            return;
        }
        ready = (now - withinToleranceSinceNanos) / 1_000_000_000.0
                >= ShooterConstants.readyStableTimeSeconds;
    }

    private void updateReadyState(double rightVelocity) {
        double relativeError = Math.abs(targetVelocity - rightVelocity) / targetVelocity;

        if (ready) {
            if (relativeError > ShooterConstants.readyExitTolerance) {
                resetReadyState();
            }
            return;
        }

        if (relativeError > ShooterConstants.readyEnterTolerance) {
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
