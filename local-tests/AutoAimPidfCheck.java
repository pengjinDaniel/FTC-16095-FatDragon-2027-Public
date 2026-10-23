package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import java.lang.reflect.Field;

/** Exercises the production auto-aim controller with the real FTCLib implementation. */
public class AutoAimPidfCheck {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void close(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual + " != " + expected);
    }

    private static void gains(double p, double i, double d, double f) {
        DriveConstants.autoAimKp = p;
        DriveConstants.autoAimKi = i;
        DriveConstants.autoAimKd = d;
        DriveConstants.autoAimKf = f;
        DriveConstants.autoAimYawToTurnSign = 1.0;
        DriveConstants.autoAimMaxTurnPower = 1.0;
        DriveConstants.autoAimTestMaxTurnPower = 1.0;
        DriveConstants.aimLockToleranceDegrees = 0.0;
        DriveConstants.aimUnlockToleranceDegrees = 0.0;
        DriveConstants.aimLockStableTimeSeconds = 0.05;
    }

    private static PIDFController pid(AutoAimController aim) throws Exception {
        Field field = AutoAimController.class.getDeclaredField("controller");
        field.setAccessible(true);
        return (PIDFController) field.get(aim);
    }

    private static double next(AutoAimController aim, double degrees, double seconds)
            throws Exception {
        Field field = PIDFController.class.getDeclaredField("lastTimeStamp");
        field.setAccessible(true);
        field.setDouble(pid(aim), System.nanoTime() / 1e9 - seconds);
        return aim.calculateTurnPower(true, degrees);
    }

    public static void main(String[] args) throws Exception {
        double ten = Math.toRadians(10.0);
        gains(0.0, 0.0, 0.02, 0.0);
        AutoAimController aim = new AutoAimController();
        close(aim.calculateTurnPower(true, 20.0), 0.0, "first D sample has no history");
        double output = next(aim, 10.0, 0.02);
        double expectedD = 0.02 * (ten - Math.toRadians(20.0)) / pid(aim).getPeriod();
        check(Math.abs(expectedD) > 0.01, "changing error excites derivative");
        close(output, expectedD, "D responds to decreasing error");
        close(next(aim, 10.0, 0.02), 0.0, "D is zero for a constant error");
        check(next(aim, 20.0, 0.02) > 0.0, "D reverses with increasing error");

        gains(-1.0, 0.0, 0.0, 0.0);
        aim = new AutoAimController();
        close(aim.calculateTurnPower(true, 10.0), -ten, "P preserves positive-error turn sign");
        close(aim.calculateTurnPower(true, -10.0), ten, "P preserves negative-error turn sign");
        DriveConstants.autoAimKp = -0.5;
        close(aim.calculateTurnPower(true, 10.0), -0.5 * ten, "P updates live");

        gains(0.0, 1.0, 0.0, 0.0);
        aim = new AutoAimController();
        close(aim.calculateTurnPower(true, 10.0), 0.0, "first I sample has zero period");
        double firstIntegral = next(aim, 10.0, 0.05);
        close(firstIntegral, ten * pid(aim).getPeriod(), "I integrates error over time");
        check(next(aim, 10.0, 0.05) > firstIntegral, "I accumulates a persistent error");
        close(next(aim, 90.0, 100.0), 1.0, "FTCLib integral bound is retained");
        aim.reset();
        close(aim.getRawTurnPower(), 0.0, "reset clears reported output");
        close(aim.calculateTurnPower(true, 10.0), 0.0, "reset clears integral and timing history");

        gains(0.0, 0.0, 0.0, 0.3);
        aim = new AutoAimController();
        close(aim.calculateTurnPower(true, 10.0), 0.3 * ten, "F preserves error-scaled feedforward");
        close(aim.calculateTurnPower(true, -10.0), -0.3 * ten, "F follows error direction");
        close(aim.calculateTurnPower(true, 0.0), 0.0, "F is zero at zero error");
        DriveConstants.autoAimKf = -0.4;
        close(aim.calculateTurnPower(true, 10.0), -0.4 * ten, "F updates live");

        gains(-0.7, 0.1, -0.02, -0.03);
        aim = new AutoAimController();
        aim.calculateTurnPower(true, 20.0);
        output = next(aim, 10.0, 0.02);
        double period = pid(aim).getPeriod();
        double expected = -0.7 * ten + 0.1 * ten * period
                - 0.02 * (ten - Math.toRadians(20.0)) / period - 0.03 * ten;
        close(aim.getRawTurnPower(), expected, "P, I, D and F contribute together");
        close(output, expected, "combined unsaturated output reaches the drive");

        gains(-2.0, 0.0, 0.0, 0.0);
        DriveConstants.autoAimMaxTurnPower = 0.2;
        DriveConstants.autoAimTestMaxTurnPower = 0.1;
        aim = new AutoAimController();
        close(aim.calculateTurnPower(true, 90.0), -0.1, "positive error respects smaller cap");
        close(aim.getRawTurnPower(), -Math.PI, "telemetry retains unclipped output");
        close(aim.calculateTurnPower(true, -90.0), 0.1, "negative error respects smaller cap");
        DriveConstants.autoAimYawToTurnSign = -1.0;
        close(aim.calculateTurnPower(true, 90.0), 0.1, "configured turn sign applies to all terms");
        close(aim.calculateTurnPower(false, 90.0), 0.0, "missing target stops output");
        close(aim.getRawTurnPower(), 0.0, "missing target clears reported output");
        close(aim.calculateTurnPower(true, Double.NaN), 0.0, "NaN error stops output");
        close(aim.calculateTurnPower(true, Double.POSITIVE_INFINITY), 0.0,
                "infinite error stops output");
        DriveConstants.autoAimMaxTurnPower = Double.NaN;
        close(aim.calculateTurnPower(true, 10.0), 0.0, "invalid cap stops output");
        gains(0.0, 0.0, 0.0, Double.NaN);
        close(aim.calculateTurnPower(true, 10.0), 0.0, "nonfinite gain stops output");

        gains(-1.0, 0.0, 0.0, 0.0);
        DriveConstants.aimLockToleranceDegrees = 5.0;
        DriveConstants.aimUnlockToleranceDegrees = 8.0;
        DriveConstants.aimLockStableTimeSeconds = 0.0;
        aim = new AutoAimController();
        aim.calculateTurnPower(true, 4.0);
        output = aim.calculateTurnPower(true, 4.0);
        check(aim.isLocked() && output != 0.0, "lock keeps corrective output active");
        aim.calculateTurnPower(true, 7.0);
        check(aim.isLocked(), "unlock hysteresis is retained");
        aim.calculateTurnPower(true, 9.0);
        check(!aim.isLocked(), "exceeding unlock tolerance clears lock");

        System.out.println("PASS AutoAimPidfCheck: " + checks
                + " actual-FTCLib PIDF, live tuning, limits, reset and lock assertions");
    }
}
