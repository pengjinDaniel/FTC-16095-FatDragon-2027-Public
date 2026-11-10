package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.LinkedHashMap;
import java.util.Map;
import sun.misc.Unsafe;

/** Checks production shot outputs without connecting motors or servos. */
public class ShooterDistanceCheck {
    private static int checks;
    private static final double[] velocities = {1000.0, 1000.0};

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual + " != " + expected);
    }

    private static void set(Shooter shooter, String name, Object value) throws Exception {
        Field field = Shooter.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(shooter, value);
    }

    private static Shooter shooter() throws Exception {
        Field allocator = Unsafe.class.getDeclaredField("theUnsafe");
        allocator.setAccessible(true);
        Shooter shooter = (Shooter) ((Unsafe) allocator.get(null)).allocateInstance(Shooter.class);
        set(shooter, "distanceVelocityScale", (java.util.function.DoubleSupplier) () -> 1.0);
        for (String name : new String[]{"leftShooter", "rightShooter"}) {
            int index = name.equals("leftShooter") ? 0 : 1;
            double[] power = {0.0};
            DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                    (proxy, method, args) -> {
                        switch (method.getName()) {
                            case "setPower": power[0] = (double) args[0]; return null;
                            case "getPower": return power[0];
                            case "getVelocity": return velocities[index];
                            default: throw new AssertionError("Unexpected motor call: " + method);
                        }
                    });
            set(shooter, name, motor);
        }
        Servo servo = (Servo) Proxy.newProxyInstance(
                Servo.class.getClassLoader(), new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (!method.getName().equals("setPosition")) {
                        throw new AssertionError("Unexpected servo call: " + method);
                    }
                    return null;
                });
        set(shooter, "pitchServo", servo);
        set(shooter, "rightController", new PIDFController(0, 0, 0, 0));
        set(shooter, "pitchPosition", 0.5);
        set(shooter, "pitchSettleReferencePosition", 0.5);
        set(shooter, "pitchChangedAtNanos", System.nanoTime());
        set(shooter, "withinToleranceSinceNanos", -1L);
        return shooter;
    }

    private static void configure() {
        // Synthetic test inputs, not calibration values for the robot.
        ShooterConstants.maxPower = 1.0;
        ShooterConstants.pitchMinPosition = 0.0;
        ShooterConstants.pitchMaxPosition = 0.55;
        ShooterConstants.centerDistanceCm = 100.0;
        ShooterConstants.centerVelocityTicksPerSecond = 1000.0;
        ShooterConstants.centerPitchPosition = 0.4;
        ShooterConstants.wallDistanceCm = 300.0;
        ShooterConstants.wallVelocityTicksPerSecond = 2000.0;
        ShooterConstants.wallPitchPosition = 0.2;
        ShooterConstants.additionalShotPoints = new double[0][];
        ShooterConstants.velocityControlCalibrated = true;
        ShooterConstants.kP = 0.001;
        ShooterConstants.kI = 0.0;
        ShooterConstants.kD = 0.0;
        ShooterConstants.kF = 0.0002;
        ShooterConstants.readyEnterTolerance = 0.06;
        ShooterConstants.readyExitTolerance = 0.06;
        ShooterConstants.readyStableTimeSeconds = 0.05;
        ShooterConstants.targetChangeResetThreshold = 30.0;
        ShooterConstants.pitchSettleTimeSeconds = 0.2;
        ShooterConstants.pitchChangeResetThreshold = 0.005;
        velocities[0] = 0.0;
        velocities[1] = 1000.0;
    }

    private static void shot(Shooter shooter, double distance, double velocity, double pitch) {
        check(shooter.setDistanceTarget(distance), "valid shot accepted");
        near(shooter.getTargetVelocity(), velocity, "target velocity in ticks/s");
        near(shooter.getPitchPosition(), pitch, "pitch");
        check(shooter.hasTargetVelocity(), "distance shots use velocity control");
        check(shooter.getControlMode().equals("PID"), "distance shots activate PID");
    }

    private static void stopped(Shooter shooter, double distance) {
        check(!shooter.setDistanceTarget(distance), "invalid shot rejected");
        near(shooter.getLeftPower(), 0.0, "left stops on invalid input");
        near(shooter.getRightPower(), 0.0, "right stops on invalid input");
        check(!shooter.hasTargetVelocity(), "invalid shot clears velocity target");
        check(!shooter.isReady(), "invalid shot clears readiness");
    }

    private static void multiPointShots() throws Exception {
        Shooter shooter = shooter();
        configure();
        ShooterConstants.additionalShotPoints = new double[][]{
                {250.0, 1950.0, 0.22}, {150.0, 1100.0, 0.36}, {200.0, 1700.0, 0.3}
        };
        check(Shooter.areDistanceShotValuesValid(), "unsorted calibration points accepted");
        shot(shooter, 100.0, 1000.0, 0.4);
        shot(shooter, 125.0, 1050.0, 0.38);
        shot(shooter, 150.0, 1100.0, 0.36);
        shot(shooter, 175.0, 1400.0, 0.33);
        shot(shooter, 200.0, 1700.0, 0.3);
        shot(shooter, 225.0, 1825.0, 0.26);
        shot(shooter, 250.0, 1950.0, 0.22);
        shot(shooter, 275.0, 1975.0, 0.21);
        shot(shooter, 300.0, 2000.0, 0.2);
        shot(shooter, 0.0, 1000.0, 0.4);
        shot(shooter, 350.0, 2000.0, 0.2);
        ShooterConstants.additionalShotPoints[1][1] = 1300.0;
        shot(shooter, 150.0, 1300.0, 0.36);
        shot(shooter, 175.0, 1500.0, 0.33);

        ShooterConstants.additionalShotPoints = new double[][]{
                {350.0, 2100.0, 0.15}, {50.0, 800.0, 0.45}
        };
        shot(shooter, 0.0, 800.0, 0.45);
        shot(shooter, 75.0, 900.0, 0.425);
        shot(shooter, 325.0, 2050.0, 0.175);
        shot(shooter, 400.0, 2100.0, 0.15);

        for (double[][] invalid : new double[][][]{
                null, {null}, {{150.0, 1250.0}}, {{150.0, 1250.0, 0.35, 0.0}},
                {{100.0, 1250.0, 0.35}}, {{300.0, 1250.0, 0.35}},
                {{150.0, 1250.0, 0.35}, {150.0, 1400.0, 0.3}},
                {{0.0, 1250.0, 0.35}}, {{150.0, 0.0, 0.35}}, {{150.0, 1250.0, 0.6}}
        }) {
            configure();
            shot(shooter, 200.0, 1500.0, 0.3);
            ShooterConstants.additionalShotPoints = invalid;
            check(!Shooter.areDistanceShotValuesValid(), "malformed or duplicate point rejected");
            stopped(shooter, 200.0);
        }
        for (int column = 0; column < 3; column++) {
            for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1.0}) {
                configure();
                shot(shooter, 200.0, 1500.0, 0.3);
                ShooterConstants.additionalShotPoints = new double[][]{{150.0, 1250.0, 0.35}};
                ShooterConstants.additionalShotPoints[0][column] = invalid;
                check(!Shooter.areDistanceShotValuesValid(), "invalid point value rejected");
                stopped(shooter, 200.0);
            }
        }
    }

    private static void overspeedMustReducePower() throws Exception {
        Shooter shooter = shooter();
        shooter.setTargetVelocity(1740.0);
        velocities[0] = 0.0; // The shared flywheel has only the right encoder connected.
        velocities[1] = 1740.0;
        shooter.periodic();
        double atTargetLeftPower = shooter.getLeftPower();
        double atTargetRightPower = shooter.getRightPower();
        velocities[1] = 1880.0;
        shooter.periodic();
        System.out.printf(java.util.Locale.US,
                "Overspeed replay: target 1740, actual 1880, raw output %.3f / %.3f, power %.3f / %.3f%n",
                shooter.getLeftPidOutput(), shooter.getRightPidOutput(),
                shooter.getLeftPower(), shooter.getRightPower());
        check(shooter.getLeftPower() < atTargetLeftPower,
                "left power must decrease for the reported 140 ticks/s overspeed");
        check(shooter.getRightPower() < atTargetRightPower,
                "right power must decrease for the reported 140 ticks/s overspeed");
        check(!shooter.isReady(), "reported overspeed cannot satisfy shooting readiness");

        for (double target : new double[]{ShooterConstants.centerVelocityTicksPerSecond,
                ShooterConstants.targetVelocity, ShooterConstants.wallVelocityTicksPerSecond}) {
            shooter.setTargetVelocity(target);
            velocities[1] = target;
            shooter.periodic();
            check(shooter.getLeftPower() < ShooterConstants.maxPower
                            && shooter.getRightPower() < ShooterConstants.maxPower,
                    "configured shot speeds must leave feedback headroom at target: " + target);
            double atTargetPower = shooter.getLeftPower();
            velocities[1] = target * (1.0 + ShooterConstants.readyExitTolerance) + 1.0;
            shooter.periodic();
            check(shooter.getLeftPower() < atTargetPower && shooter.getRightPower() < atTargetPower,
                    "overspeed outside the ready band must reduce both wheel powers: " + target);
            check(!shooter.isReady(), "overspeed must keep the feed gate blocked: " + target);
        }
    }

    public static void main(String[] args) throws Exception {
        Map<Field, Double> original = new LinkedHashMap<>();
        for (Field field : ShooterConstants.class.getFields()) {
            if (field.getType() == double.class) original.put(field, field.getDouble(null));
        }
        boolean originalCalibrated = ShooterConstants.velocityControlCalibrated;
        double[][] originalAdditionalShotPoints = ShooterConstants.additionalShotPoints;
        try {
            overspeedMustReducePower();
            multiPointShots();
            Shooter shooter = shooter();
            configure();
            ShooterConstants.centerVelocityTicksPerSecond = Double.NaN;
            check(!Shooter.areDistanceShotValuesValid(), "unfilled calibration stays disabled");
            stopped(shooter, 200.0);
            configure();
            check(Shooter.areDistanceShotValuesValid(), "complete calibration accepted");
            shot(shooter, 100.0, 1000.0, 0.4);
            shot(shooter, 150.0, 1250.0, 0.35);
            shot(shooter, 200.0, 1500.0, 0.3);
            shot(shooter, 300.0, 2000.0, 0.2);
            shot(shooter, 50.0, 1000.0, 0.4);
            shot(shooter, 350.0, 2000.0, 0.2);
            ShooterConstants.centerDistanceCm = 67.0;
            ShooterConstants.centerVelocityTicksPerSecond = 1590.0;
            ShooterConstants.centerPitchPosition = 0.52;
            ShooterConstants.wallDistanceCm = 112.0;
            ShooterConstants.wallVelocityTicksPerSecond = 1780.0;
            ShooterConstants.wallPitchPosition = 0.37;
            shot(shooter, 67.0, 1590.0, 0.52);
            shot(shooter, 89.5, 1685.0, 0.445);
            shot(shooter, 112.0, 1780.0, 0.37);
            shot(shooter, 120.0, 1780.0, 0.37);
            configure();
            ShooterConstants.centerDistanceCm = 300.0;
            ShooterConstants.wallDistanceCm = 100.0;
            shot(shooter, 200.0, 1500.0, 0.3);
            shot(shooter, 50.0, 2000.0, 0.2);
            shot(shooter, 350.0, 1000.0, 0.4);

            for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1.0}) {
                configure();
                shot(shooter, 200.0, 1500.0, 0.3);
                stopped(shooter, invalid);
            }
            for (String name : new String[]{"centerDistanceCm", "centerVelocityTicksPerSecond", "centerPitchPosition",
                    "wallDistanceCm", "wallVelocityTicksPerSecond", "wallPitchPosition", "targetChangeResetThreshold"}) {
                for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1.0}) {
                    configure();
                    shot(shooter, 200.0, 1500.0, 0.3);
                    ShooterConstants.class.getField(name).setDouble(null, invalid);
                    check(!Shooter.areDistanceShotValuesValid(), "invalid configuration: " + name);
                    stopped(shooter, 200.0);
                }
            }
            configure();
            ShooterConstants.wallDistanceCm = ShooterConstants.centerDistanceCm;
            stopped(shooter, 200.0);
            configure();
            ShooterConstants.centerVelocityTicksPerSecond = 0.0;
            stopped(shooter, 200.0);
            configure();
            ShooterConstants.maxPower = 0.0;
            stopped(shooter, 200.0);
            configure();
            ShooterConstants.wallPitchPosition = 0.6;
            stopped(shooter, 200.0);

            configure();
            ShooterConstants.velocityControlCalibrated = false;
            check(!Shooter.areDistanceShotValuesValid(), "distance shots require calibrated PID");
            stopped(shooter, 200.0);
            configure();
            ShooterConstants.kP = 0.0;
            ShooterConstants.kF = 0.0;
            stopped(shooter, 200.0);
            configure();
            ShooterConstants.kP = Double.NaN;
            stopped(shooter, 200.0);

            configure();
            velocities[1] = 1200.0;
            shot(shooter, 200.0, 1500.0, 0.3);
            shooter.periodic();
            near(shooter.getLeftPower(), 0.6, "left motor follows the shared right encoder PID");
            near(shooter.getRightPower(), 0.6, "right PID reacts to shared flywheel speed");
            velocities[0] = 0.0;
            velocities[1] = 1400.0;
            shooter.periodic();
            near(shooter.getLeftPower(), 0.4, "left follower reduces power as shared speed rises");
            near(shooter.getRightPower(), 0.4, "right PID reduces power as speed rises");

            configure();
            ShooterConstants.maxPower = 0.4;
            shot(shooter, 200.0, 1500.0, 0.3);
            shooter.periodic();
            near(shooter.getLeftPower(), 0.4, "PID output obeys power cap without clipping ticks/s");
            near(shooter.getRightPower(), 0.4, "both PID outputs obey power cap");
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            shooter.periodic();
            check(!shooter.isReady(), "stable speed below target does not become ready");

            configure();
            velocities[0] = 0.0;
            shot(shooter, 200.0, 1500.0, 0.3);
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            shooter.periodic();
            check(!shooter.isReady(), "the right encoder must reach the shared flywheel target");
            velocities[1] = 1500.0;
            shooter.periodic();
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            set(shooter, "pitchChangedAtNanos", System.nanoTime() - 300_000_000L);
            shooter.periodic();
            check(shooter.isReady() && shooter.isPitchReady(), "steady shot becomes ready without a left encoder");
            near(shooter.getLeftVelocity(), 0.0, "raw disconnected left encoder remains zero");
            near(shooter.getLeftPower(), shooter.getRightPower(), "both motors drive the shared flywheel equally");
            shot(shooter, 200.0, 1500.0, 0.3);
            check(shooter.isReady(), "unchanged target preserves readiness");
            shot(shooter, 201.0, 1505.0, 0.299);
            check(shooter.isReady(), "small target change preserves readiness");
            for (int i = 0; i < 100; i++) {
                shot(shooter, 200.0 + (i % 2 == 0 ? 0.1 : -0.1),
                        1500.0 + (i % 2 == 0 ? 0.5 : -0.5),
                        0.3 + (i % 2 == 0 ? -0.0001 : 0.0001));
                check(shooter.isReady(), "repeated distance jitter preserves readiness");
            }
            shot(shooter, 206.0, 1530.0, 0.294);
            check(shooter.isReady(), "exact threshold preserves readiness");
            shot(shooter, 206.2, 1531.0, 0.2938);
            check(!shooter.isReady(), "cumulative target change restarts readiness");
            shooter.periodic();
            long started = System.nanoTime() - 20_000_000L;
            set(shooter, "withinToleranceSinceNanos", started);
            shot(shooter, 206.3, 1531.5, 0.2937);
            Field timer = Shooter.class.getDeclaredField("withinToleranceSinceNanos");
            timer.setAccessible(true);
            check(timer.getLong(shooter) == started, "small target change preserves stability timer");

            shot(shooter, 200.0, 1500.0, 0.3);
            velocities[1] = 1420.0;
            shooter.periodic();
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            shooter.periodic();
            check(shooter.isReady(), "speed error between five and six percent is accepted");
            velocities[1] = 1409.0;
            shooter.periodic();
            check(!shooter.isReady(), "right encoder outside six percent clears readiness");
            shot(shooter, 250.0, 1750.0, 0.25);
            check(!shooter.isPitchReady(), "significant pitch change restarts settling");

            shooter.setFixedPower(0.5);
            shooter.periodic();
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            shooter.periodic();
            check(shooter.isReady(), "manual power becomes ready without a left encoder");
            shooter.setFixedPower(0.501);
            check(!shooter.isReady(), "manual power retains exact-change readiness reset");
            velocities[1] = 0.0;
            shooter.periodic();
            check(!shooter.isReady(), "missing right encoder blocks fixed-power readiness");
            velocities[1] = Double.NaN;
            shooter.periodic();
            check(!shooter.isReady(), "invalid right encoder blocks fixed-power readiness");
            for (double invalidVelocity : new double[]{Double.NaN, Double.POSITIVE_INFINITY}) {
                configure();
                shooter.setTargetVelocity(1500.0);
                velocities[1] = invalidVelocity;
                shooter.periodic();
                near(shooter.getLeftPower(), 0.0, "invalid right feedback stops the left follower");
                near(shooter.getRightPower(), 0.0, "invalid right feedback stops the right motor");
                check(!shooter.isReady(), "invalid right feedback cannot become ready");
            }
            configure();
            shooter.setTargetVelocity(1500.0);
            velocities[1] = 0.0;
            shooter.periodic();
            check(!shooter.isReady(), "zero right feedback cannot become ready");
            shooter.stop();
            near(shooter.getLeftPower(), 0.0, "explicit stop left");
            near(shooter.getRightPower(), 0.0, "explicit stop right");
            check(!shooter.isReady(), "explicit stop clears readiness");
            System.out.println("PASS ShooterDistanceCheck: " + checks + " assertions");
        } finally {
            ShooterConstants.velocityControlCalibrated = originalCalibrated;
            ShooterConstants.additionalShotPoints = originalAdditionalShotPoints;
            for (Map.Entry<Field, Double> entry : original.entrySet()) {
                entry.getKey().setDouble(null, entry.getValue());
            }
        }
    }
}
