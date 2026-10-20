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
        set(shooter, "leftController", new PIDFController(0, 0, 0, 0));
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
        ShooterConstants.velocityControlCalibrated = true;
        ShooterConstants.kP = 0.001;
        ShooterConstants.kI = 0.0;
        ShooterConstants.kD = 0.0;
        ShooterConstants.kF = 0.0002;
        ShooterConstants.readyEnterTolerance = 0.05;
        ShooterConstants.readyExitTolerance = 0.05;
        ShooterConstants.readyStableTimeSeconds = 0.05;
        ShooterConstants.targetChangeResetThreshold = 0.0;
        ShooterConstants.pitchSettleTimeSeconds = 0.2;
        ShooterConstants.pitchChangeResetThreshold = 0.005;
        velocities[0] = 1000.0;
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

    public static void main(String[] args) throws Exception {
        Map<Field, Double> original = new LinkedHashMap<>();
        for (Field field : ShooterConstants.class.getFields()) {
            if (field.getType() == double.class) original.put(field, field.getDouble(null));
        }
        boolean originalCalibrated = ShooterConstants.velocityControlCalibrated;
        try {
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
            near(shooter.getLeftPower(), 0.8, "left PID reacts to measured speed");
            near(shooter.getRightPower(), 0.6, "right PID independently reacts to measured speed");
            velocities[0] = 1200.0;
            velocities[1] = 1400.0;
            shooter.periodic();
            near(shooter.getLeftPower(), 0.6, "left PID reduces power as speed rises");
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
            velocities[0] = 1500.0;
            shot(shooter, 200.0, 1500.0, 0.3);
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            shooter.periodic();
            check(!shooter.isReady(), "both flywheels must reach the target");
            velocities[1] = 1500.0;
            shooter.periodic();
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            set(shooter, "pitchChangedAtNanos", System.nanoTime() - 300_000_000L);
            shooter.periodic();
            check(shooter.isReady() && shooter.isPitchReady(), "steady shot becomes ready");
            shot(shooter, 200.0, 1500.0, 0.3);
            check(shooter.isReady(), "unchanged target preserves readiness");
            shot(shooter, 201.0, 1505.0, 0.299);
            check(!shooter.isReady(), "velocity change uses the shared PID reset threshold");
            shot(shooter, 250.0, 1750.0, 0.25);
            check(!shooter.isPitchReady(), "significant pitch change restarts settling");

            shooter.setFixedPower(0.5);
            shooter.periodic();
            set(shooter, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
            shooter.periodic();
            check(shooter.isReady(), "manual power becomes ready");
            shooter.setFixedPower(0.501);
            check(!shooter.isReady(), "manual power retains exact-change readiness reset");
            shooter.stop();
            near(shooter.getLeftPower(), 0.0, "explicit stop left");
            near(shooter.getRightPower(), 0.0, "explicit stop right");
            check(!shooter.isReady(), "explicit stop clears readiness");
            System.out.println("PASS ShooterDistanceCheck: " + checks + " assertions");
        } finally {
            ShooterConstants.velocityControlCalibrated = originalCalibrated;
            for (Map.Entry<Field, Double> entry : original.entrySet()) {
                entry.getKey().setDouble(null, entry.getValue());
            }
        }
    }
}
