package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Exercises actual SDK registration, Pedro configurations/paths, guards and shutdown. */
public class PedroTuningCheck {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-6, message + ": " + actual);
    }

    private static void set(OpMode opMode, String name, boolean value) {
        for (Class<?> owner = opMode.getClass(); owner != null; owner = owner.getSuperclass()) {
            try {
                Field field = owner.getDeclaredField(name);
                field.setAccessible(true);
                field.setBoolean(opMode, value);
                return;
            } catch (NoSuchFieldException ignored) {
            } catch (IllegalAccessException error) { throw new AssertionError(error); }
        }
        throw new AssertionError("Missing SDK lifecycle field: " + name);
    }

    private static void configuration() throws Exception {
        for (Object config : new Object[]{Constants.drivetrainConfig, Constants.localizerConfig}) {
            for (Field field : config.getClass().getFields()) {
                if (ConfigVar.class.isAssignableFrom(field.getType())) {
                    check(((ConfigVar<?>) field.get(config)).get() != null, "unset field: " + field.getName());
                }
            }
        }
        check(Constants.drivetrainConfig.frontLeftName.get().equals(DriveConstants.leftFrontMotorName)
                && Constants.drivetrainConfig.backLeftName.get().equals(DriveConstants.leftBackMotorName)
                && Constants.drivetrainConfig.frontRightName.get().equals(DriveConstants.rightFrontMotorName)
                && Constants.drivetrainConfig.backRightName.get().equals(DriveConstants.rightBackMotorName),
                "all four hardware names match competition drive");
        check(Constants.drivetrainConfig.frontLeftDirection.get() == DriveConstants.leftFrontMotorDirection
                && Constants.drivetrainConfig.backLeftDirection.get() == DriveConstants.leftBackMotorDirection
                && Constants.drivetrainConfig.frontRightDirection.get() == DriveConstants.rightFrontMotorDirection
                && Constants.drivetrainConfig.backRightDirection.get() == DriveConstants.rightBackMotorDirection,
                "all four motor directions match competition drive");
        check(Constants.localizerConfig.name.get().equals(DriveConstants.pinpointName), "Pinpoint name");
        check(Constants.localizerConfig.xPodDirection.get() == DriveConstants.pinpointForwardPodDirection
                && Constants.localizerConfig.yPodDirection.get() == DriveConstants.pinpointStrafePodDirection,
                "pod directions match competition drive");
        check(Constants.localizerConfig.offsetUnits.get() == DistanceUnit.MM, "offsets use millimeters");
        check(Constants.localizerConfig.globalDistanceUnit.get() == DistanceUnit.INCH, "poses use inches");
        near(Constants.localizerConfig.xPodOffset.get(), DriveConstants.pinpointXOffsetMm, "forward pod offset");
        near(Constants.localizerConfig.yPodOffset.get(), DriveConstants.pinpointYOffsetMm, "strafe pod offset");
    }

    private static Telemetry telemetry() {
        return (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> method.getReturnType() == boolean.class ? false : null);
    }

    private static Map<String, OpMode> register() throws Exception {
        Method registrar = Tests.class.getMethod("registerOpModes", OpModeManager.class);
        check(registrar.isAnnotationPresent(OpModeRegistrar.class)
                && Modifier.isStatic(registrar.getModifiers()), "SDK scanner can discover the registrar");
        Map<String, OpMode> opModes = new LinkedHashMap<>();
        OpModeManager manager = (OpModeManager) Proxy.newProxyInstance(OpModeManager.class.getClassLoader(),
                new Class<?>[]{OpModeManager.class}, (proxy, method, args) -> {
                    OpModeMeta meta = (OpModeMeta) args[0];
                    check(meta.group.equals("Pedro Pathing"), "test group");
                    check(meta.flavor == OpModeMeta.Flavor.TELEOP, "test flavor");
                    check(opModes.put(meta.name, (OpMode) args[1]) == null, "duplicate registration");
                    return null;
                });
        Tests.registerOpModes(manager);
        check(opModes.keySet().equals(new HashSet<>(Arrays.asList(
                "Pedro Localization Test", "Pedro Pose Test", "Pedro Driving Test", "Pedro Odometry Test",
                "Pedro Hold Test", "Pedro Line Test", "Pedro Curve Test", "Pedro Interpolation Test",
                "Pedro Triangle Test", "Pedro Circle Test"))), "all ten Driver Station entries");
        return opModes;
    }

    private static void unmeasuredPathsDoNotTouchHardware(Map<String, OpMode> opModes) throws Exception {
        ForesightConfig original = Constants.foresightConfig;
        try {
            Constants.foresightConfig = null;
            int guarded = 0;
            for (Map.Entry<String, OpMode> entry : opModes.entrySet()) {
                OpMode opMode = entry.getValue();
                if (opMode instanceof TestsLocalization || opMode instanceof TestsPose
                        || opMode instanceof TestsDriving || opMode instanceof TestsOdometry) continue;
                opMode.telemetry = telemetry();
                set(opMode, "isStarted", true);
                check(opMode.hardwareMap == null, "host check has no hardware");
                Method run = opMode.getClass().getMethod("runTuningOpMode");
                check(Boolean.FALSE.equals(run.invoke(opMode)), "missing calibration exits: " + entry.getKey());
                guarded++;
            }
            check(guarded == 6, "all six hold/path entries require measured calibration");
        } finally { Constants.foresightConfig = original; }
    }

    private static void paths() {
        Path triangle = TestsShape.triangle(24);
        Path circle = TestsShape.circle(10);
        check(triangle.getSegments().size() == 3, "triangle has three line segments");
        check(circle.getSegments().size() == 4, "reference circle has four curves");
        for (Path path : new Path[]{triangle, circle}) {
            near(path.get(0).x(), 0, "closed path starts at local X zero");
            near(path.get(0).y(), 0, "closed path starts at local Y zero");
            near(path.get(1).x(), 0, "closed path returns to X zero");
            near(path.get(1).y(), 0, "closed path returns to Y zero");
            for (int i = 0; i <= 100; i++) {
                Pose pose = path.get(i / 100.0);
                check(Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading()),
                        "path samples remain finite");
            }
        }
        for (int i = 0; i <= 100; i++) {
            Pose pose = circle.get(i / 100.0);
            double expected = Math.atan2(10 - pose.y(), -pose.x());
            near(Math.IEEEremainder(pose.heading() - expected, 2 * Math.PI), 0, "circle faces its center");
        }
        for (double invalid : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            for (boolean isCircle : new boolean[]{false, true}) {
                boolean rejected = false;
                try { if (isCircle) TestsShape.circle(invalid); else TestsShape.triangle(invalid); }
                catch (IllegalArgumentException expected) { rejected = true; }
                check(rejected, "invalid path size is rejected");
            }
        }
    }

    private static class Motors implements Drivetrain {
        OpMode opMode;
        int ending;
        int stops;
        DrivePowers last = DrivePowers.zero();
        DrivePowers command;
        @Override public void drive(DrivePowers powers, boolean manual) {
            command = last = powers;
            if (ending == 1) throw new IllegalStateException("Synthetic motor failure");
            if (ending == 2) Thread.currentThread().interrupt();
            else set(opMode, "stopRequested", true);
        }
        @Override public void stop() { stops++; last = DrivePowers.zero(); }
        @Override public void stop(boolean brake) { stop(); }
        @Override public double maxScaling(DrivePowers current, DrivePowers delta) { return 1; }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
        @Override public double interpolateVelocity(double x, double y, double theta) { return 0; }
    }

    private static void drivetrainStopsOnEveryExit() throws Exception {
        for (int ending = 0; ending < 3; ending++) {
            Motors motors = new Motors();
            motors.ending = ending;
            TestsDriving opMode = new TestsDriving(hardwareMap -> motors);
            motors.opMode = opMode;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad1.left_stick_y = -.4f;
            opMode.gamepad1.left_stick_x = -.2f;
            opMode.gamepad1.right_stick_x = -.3f;
            set(opMode, "isStarted", true);
            boolean failed = false;
            try { opMode.runTuningOpMode(); }
            catch (IllegalStateException expected) { failed = expected.getMessage().contains("Synthetic motor failure"); }
            finally { Thread.interrupted(); }
            check(failed == (ending == 1), "motor exceptions propagate");
            near(motors.command.forward(), .4, "forward joystick sign");
            near(motors.command.strafe(), .2, "left joystick sign");
            near(motors.command.turn(), .3, "CCW joystick sign");
            check(motors.stops == 2, "stop before START and on every exit");
            near(motors.last.forward(), 0, "stopped forward power");
            near(motors.last.strafe(), 0, "stopped strafe power");
            near(motors.last.turn(), 0, "stopped turn power");
        }
    }

    public static void main(String[] args) throws Exception {
        configuration();
        Map<String, OpMode> opModes = register();
        int factories = 0;
        for (Method method : Tuning.class.getDeclaredMethods()) {
            if (!method.isAnnotationPresent(Tuner.class)) continue;
            check(Modifier.isStatic(method.getModifiers()) && method.getParameterCount() == 0
                            && method.getReturnType() == Procedure.class,
                    "AutoTune factory signature");
            check(method.invoke(null) instanceof Procedure, "AutoTune factory does not initialize hardware");
            factories++;
        }
        check(factories == 4, "four AutoTune procedures remain registered");
        unmeasuredPathsDoNotTouchHardware(opModes);
        paths();
        drivetrainStopsOnEveryExit();
        System.out.println("PASS PedroTuningCheck: " + checks
                + " real configuration, registration, paths, guards and lifecycle checks");
    }
}
