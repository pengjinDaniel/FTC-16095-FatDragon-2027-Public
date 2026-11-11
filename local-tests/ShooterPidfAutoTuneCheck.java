package org.firstinspires.ftc.teamcode.tests;

public class ShooterPidfAutoTuneCheck {
    private static void near(double actual, double expected, String message) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > 1e-8) {
            throw new AssertionError(message + ": expected " + expected + ", got " + actual);
        }
    }

    private static void missing(double actual, String message) {
        if (!Double.isNaN(actual)) {
            throw new AssertionError(message + ": got " + actual);
        }
    }

    public static void main(String[] args) {
        double[] powers = {0.50, 0.60, 0.70};
        double[] speeds = {1300, 1560, 1820};
        near(ShooterPidfAutoTune.powerAtVelocity(1495, powers, speeds, 3), 0.575,
                "interpolates the near target between measured points");
        near(ShooterPidfAutoTune.powerAtVelocity(1660, powers, speeds, 3), 0.6384615385,
                "interpolates the far target between measured points");
        near(ShooterPidfAutoTune.powerAtVelocity(1820, powers, speeds, 3), 0.70,
                "accepts a measured endpoint");
        missing(ShooterPidfAutoTune.powerAtVelocity(1660, powers, speeds, 2),
                "rejects a far target outside the samples already measured");
        missing(ShooterPidfAutoTune.powerAtVelocity(1250, powers, speeds, 3),
                "rejects a target below the measured range");
        missing(ShooterPidfAutoTune.powerAtVelocity(1300, powers, speeds, 1),
                "requires two measured power levels");
        System.out.println("PASS ShooterPidfAutoTuneCheck: interpolation and range checks");
    }
}
