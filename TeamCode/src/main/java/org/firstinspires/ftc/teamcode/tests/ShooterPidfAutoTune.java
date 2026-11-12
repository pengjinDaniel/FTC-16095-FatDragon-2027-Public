package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

import java.util.Arrays;

@Config
public final class ShooterPidfAutoTune extends LinearOpMode {
    // Bench targets can be configured before measuring the shot tables.
    public static double lowTestVelocityTicksPerSecond = 1400.0;
    public static double highTestVelocityTicksPerSecond = 1650.0;

    private static final long SAMPLE_MS = 25;
    private static final double FIXED_POWER_WARMUP_SECONDS = 1.10;
    private static final double FIXED_POWER_TIMEOUT_SECONDS = 4.0;
    private static final double STEADY_WINDOW_SECONDS = 0.40;
    private static final double STEP_SECONDS = 3.0;
    private static final double[] POWER_FRACTIONS = {0.25, 0.40, 0.55, 0.70, 0.85, 1.0};
    private static final int[] P_FACTORS = {1, 2, 3, 4, 5};

    private Shooter shooter;
    private double powerLimit;
    private double speedLimit;
    private double enterTolerance;
    private double exitTolerance;
    private double stableSeconds;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        try {
            telemetry.addLine("Set test targets in the ShooterPidfAutoTune Dashboard config.");
            telemetry.addLine("Empty the shooter. After START, hold R2 to tune.");
            telemetry.addLine("Release R2 or press B to stop.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive() && !gamepad1.b && triggerHeld()) {
                sleep(SAMPLE_MS);
            }
            while (opModeIsActive() && !gamepad1.b && !triggerHeld()) {
                sleep(SAMPLE_MS);
            }
            tune();
        } catch (TuningStopped stopped) {
            telemetry.addData("Tuning stopped", stopped.getMessage());
        } finally {
            try {
                shooter.stop();
            } finally {
                CommandScheduler.getInstance().unregisterSubsystem(shooter);
            }
        }
        telemetry.setMsTransmissionInterval(0);
        telemetry.update();
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private void tune() {
        double low = lowTestVelocityTicksPerSecond;
        double high = highTestVelocityTicksPerSecond;
        powerLimit = ShooterConstants.maxPower;
        enterTolerance = ShooterConstants.readyEnterTolerance;
        exitTolerance = ShooterConstants.readyExitTolerance;
        stableSeconds = ShooterConstants.readyStableTimeSeconds;
        requireActive();
        if (!Double.isFinite(low) || !Double.isFinite(high) || low <= 0.0 || high <= low
                || !Double.isFinite(powerLimit) || powerLimit <= 0.0 || powerLimit > 1.0
                || !Double.isFinite(enterTolerance) || enterTolerance <= 0.0
                || !Double.isFinite(exitTolerance) || exitTolerance < enterTolerance || exitTolerance > 1.0
                || !Double.isFinite(stableSeconds) || stableSeconds <= 0.0 || stableSeconds >= STEP_SECONDS) {
            throw new TuningStopped("Check the test targets, power limit and readiness settings.");
        }
        telemetry.addData("Test targets", "%.0f and %.0f ticks/s", low, high);
        speedLimit = high * 1.25;
        double kF = measureFeedforward(low, high);
        double[] candidates = new double[P_FACTORS.length + 1];
        for (int i = 0; i < P_FACTORS.length; i++) {
            candidates[i] = P_FACTORS[i] * kF;
        }
        candidates[P_FACTORS.length] = ShooterConstants.kP;
        Arrays.sort(candidates);
        double bestP = Double.NaN;
        double bestTime = Double.POSITIVE_INFINITY;
        for (double kP : candidates) {
            if (!Double.isFinite(kP) || kP <= 0.0) continue;
            PIDFController candidate = new PIDFController(kP, 0.0, 0.0, kF);
            if (!Double.isFinite(testStep(candidate, kP, low))) {
                continue;
            }
            double upTime = testStep(candidate, kP, high);
            if (!Double.isFinite(upTime)) continue;
            double downTime = testStep(candidate, kP, low);
            double worstTime = Math.max(upTime, downTime);
            // Keep the lower gain when the measured difference is within one sample.
            if (worstTime < bestTime - SAMPLE_MS / 1000.0) {
                bestP = kP;
                bestTime = worstTime;
            }
        }
        requireActive();
        telemetry.addData("Tested targets", "%.0f and %.0f ticks/s", low, high);
        telemetry.addData("Measured kF", "%.8f", kF);
        if (!Double.isFinite(bestP)) {
            throw new TuningStopped("No P candidate settled at both speeds.");
        }

        telemetry.addLine("Bench recommendation. Copy into ShooterConstants to apply.");
        telemetry.addData("Worst step settling time", "%.0f ms", bestTime * 1000.0);
        telemetry.addData("kP", "%.8f", bestP);
        telemetry.addData("kI", "0.0");
        telemetry.addData("kD", "0.0");
    }

    private double measureFeedforward(double low, double high) {
        double[] powers = new double[POWER_FRACTIONS.length];
        double[] speeds = new double[POWER_FRACTIONS.length];
        int count = 0;
        for (double fraction : POWER_FRACTIONS) {
            double power = fraction * powerLimit;
            double speed = measureFixedPower(power);
            if (count > 0 && speed <= speeds[count - 1]) {
                throw new TuningStopped("Speed did not increase with fixed power.");
            }
            powers[count] = power;
            speeds[count] = speed;
            count++;
            if (speed >= high) {
                break;
            }
        }
        double lowPower = powerAtVelocity(low, powers, speeds, count);
        double highPower = powerAtVelocity(high, powers, speeds, count);
        if (!Double.isFinite(lowPower) || !Double.isFinite(highPower)) {
            throw new TuningStopped("Fixed-power readings did not bracket both target speeds.");
        }
        // Fit the two target duties to the existing controller's power = kF * target.
        double kF = (low * lowPower + high * highPower) / (low * low + high * high);
        if (highPower >= powerLimit || kF * high >= powerLimit) {
            throw new TuningStopped("The high target leaves no power for speed recovery.");
        }
        return kF;
    }

    private double measureFixedPower(double power) {
        telemetry.addData("Stage", "Fixed power %.2f", power);
        telemetry.update();
        applyPower(power);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < FIXED_POWER_WARMUP_SECONDS) {
            requireActive();
            readVelocity();
            sleep(SAMPLE_MS);
        }

        double average;
        double min;
        double max;
        int samples;
        do {
            double windowEnd = Math.min(timer.seconds() + STEADY_WINDOW_SECONDS, FIXED_POWER_TIMEOUT_SECONDS);
            double sum = 0.0;
            min = Double.POSITIVE_INFINITY;
            max = Double.NEGATIVE_INFINITY;
            samples = 0;
            while (timer.seconds() < windowEnd) {
                requireActive();
                double speed = readVelocity();
                sum += speed;
                min = Math.min(min, speed);
                max = Math.max(max, speed);
                samples++;
                sleep(SAMPLE_MS);
            }
            requireActive();
            average = samples == 0 ? Double.NaN : sum / samples;
            if (samples >= 6) {
                if (average <= 0.0) break;
                if (max - min <= Math.max(30.0, average * 0.06)) return average;
            }
        } while (timer.seconds() + STEADY_WINDOW_SECONDS <= FIXED_POWER_TIMEOUT_SECONDS);

        telemetry.addData("Fixed-power window", "power %.2f / avg %.0f / min %.0f / max %.0f ticks/s / %d samples",
                power, average, min, max, samples);
        if (samples < 6) {
            throw new TuningStopped("Too few encoder samples in the fixed-power window.");
        }
        if (average <= 0.0) {
            throw new TuningStopped("Right encoder reported zero throughout the fixed-power window.");
        }
        throw new TuningStopped("Right encoder speed did not settle at fixed power.");
    }

    private double testStep(PIDFController controller, double kP, double target) {
        telemetry.addData("Stage", "Testing P %.6f at %.0f ticks/s", kP, target);
        telemetry.update();
        controller.reset();
        ElapsedTime timer = new ElapsedTime();
        double inBandSince = Double.NaN;
        double readyAt = Double.NaN;
        double lastSample = 0.0;
        double tailMin = Double.POSITIVE_INFINITY;
        double tailMax = Double.NEGATIVE_INFINITY;
        double output = 0.0;
        while (timer.seconds() < STEP_SECONDS) {
            requireActive();
            double speed = readVelocity();
            double elapsed = timer.seconds();
            if (speed == 0.0 && elapsed >= STEADY_WINDOW_SECONDS) {
                throw new TuningStopped("No right encoder speed during closed-loop testing.");
            }
            // Match the shooter's entry and exit bands while observing the entire step.
            double tolerance = Double.isFinite(readyAt) ? exitTolerance : enterTolerance;
            if (Math.abs(speed - target) > tolerance * target || elapsed - lastSample > 0.10) {
                inBandSince = Double.NaN;
                readyAt = Double.NaN;
            } else if (!Double.isFinite(inBandSince)) {
                inBandSince = elapsed;
            } else if (!Double.isFinite(readyAt) && elapsed - inBandSince >= stableSeconds) {
                readyAt = elapsed;
            }
            lastSample = elapsed;
            if (elapsed >= STEP_SECONDS - STEADY_WINDOW_SECONDS) {
                tailMin = Math.min(tailMin, speed);
                tailMax = Math.max(tailMax, speed);
            }
            // Use the same FTCLib controller and output limits as Shooter.periodic().
            output = Range.clip(controller.calculate(speed, target), 0.0, powerLimit);
            applyPower(output);
            sleep(SAMPLE_MS);
        }
        requireActive();
        telemetry.addData("Last closed-loop step", "P %.6f / target %.0f / right %.0f to %.0f ticks/s / power %.2f",
                kP, target, tailMin, tailMax, output);
        return Double.isFinite(readyAt) && timer.seconds() - lastSample <= 0.10
                ? readyAt : Double.POSITIVE_INFINITY;
    }

    static double powerAtVelocity(double target, double[] powers, double[] speeds, int count) {
        if (count < 2 || target < speeds[0] || target > speeds[count - 1]) {
            return Double.NaN;
        }
        for (int i = 1; i < count; i++) {
            if (target <= speeds[i]) {
                double fraction = (target - speeds[i - 1]) / (speeds[i] - speeds[i - 1]);
                return powers[i - 1] + fraction * (powers[i] - powers[i - 1]);
            }
        }
        return Double.NaN;
    }

    private boolean triggerHeld() {
        return gamepad1.right_trigger > 0.5;
    }

    private void requireActive() {
        if (!opModeIsActive() || Thread.currentThread().isInterrupted()
                || gamepad1.b || !triggerHeld()) {
            throw new TuningStopped("Operator canceled the run.");
        }
        if (Double.compare(powerLimit, ShooterConstants.maxPower) != 0) {
            throw new TuningStopped("Power limit changed during tuning.");
        }
    }

    private void applyPower(double power) {
        requireActive();
        if (!shooter.setFixedPower(power)) {
            throw new TuningStopped("Shooter rejected the trial power.");
        }
    }

    private double readVelocity() {
        double speed = shooter.getRightVelocity();
        if (!Double.isFinite(speed) || speed > speedLimit) {
            throw new TuningStopped("Right encoder speed is invalid or above the test limit.");
        }
        return speed;
    }

    private static final class TuningStopped extends RuntimeException {
        TuningStopped(String message) {
            super(message);
        }
    }
}
