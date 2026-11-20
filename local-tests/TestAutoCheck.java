package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;
import sun.misc.Unsafe;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Collections;
import java.util.Map;

public class TestAutoCheck {
    private static int checks;
    private static final Unsafe ALLOCATOR = allocator();

    private static Unsafe allocator() {
        try {
            Field f = Unsafe.class.getDeclaredField("theUnsafe");
            f.setAccessible(true);
            return (Unsafe) f.get(null);
        } catch (Exception e) { throw new AssertionError(e); }
    }

    private static void set(Object object, Class<?> owner, String name, Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(object, value);
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void close(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 0.002,
                message + ": expected " + expected + ", got " + actual);
    }

    private static Vision vision() throws Exception {
        Vision result = (Vision) ALLOCATOR.allocateInstance(Vision.class);
        set(result, Vision.class, "alliance", Alliance.RED);
        set(result, Vision.class, "poseStatus", "Synthetic observation");
        return result;
    }

    private static void frame(Vision vision, Pose robot, double theta) throws Exception {
        double phi = Math.toRadians(-theta);
        double x = robot.x() - 72, y = robot.y() - 72, yaw = robot.heading();
        double pivot = 43.9497007874;
        Pose3D raw = new Pose3D(new Position(DistanceUnit.INCH,
                Math.cos(phi) * x - pivot * Math.sin(phi), y,
                pivot - Math.sin(phi) * x - pivot * Math.cos(phi), 0),
                new YawPitchRollAngles(AngleUnit.RADIANS,
                        Math.atan2(Math.sin(yaw), Math.cos(phi) * Math.cos(yaw)),
                        Math.asin(Math.sin(phi) * Math.cos(yaw)),
                        Math.atan2(Math.sin(phi) * Math.sin(yaw), Math.cos(phi)), 0));
        set(vision, Vision.class, "rawRobotFieldPose", raw);
        set(vision, Vision.class, "botposeTagCount", 2);
        set(vision, Vision.class, "usableTagCount", 2);
    }

    private static void checkHive() throws Exception {
        Vision vision = vision();
        AutoHiveTarget target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
        Pose robot = new Pose(57.7667, 23.3801, Math.toRadians(270));
        close(AutoHiveTarget.toHiveFieldPose(robot).getX(DistanceUnit.INCH), -14.2333, "field X");
        close(AutoHiveTarget.toHiveFieldPose(robot).getY(DistanceUnit.INCH), -48.6199, "field Y");
        for (double theta : new double[]{30, -30, 0, 22, -22, 24, -24}) {
            frame(vision, robot, theta);
            target.update(robot);
            HiveState expected = Math.abs(theta) <= 23 ? HiveState.MIDDLE
                    : theta > 0 ? HiveState.HIGH : HiveState.LOW;
            check(target.getState() == expected, "state at " + theta + " degrees");
            close(target.getDistanceCm(), DistanceUnit.INCH.toCm(Math.hypot(
                    -17.7 - (robot.x() - 72.0), -12.75 - (robot.y() - 72.0))),
                    "CELL-opening distance stays aligned with aiming at HIVE angle " + theta);
        }
        frame(vision, robot, 30);
        target.update(robot);
        double distance = target.getDistanceCm();
        double expectedYaw = Math.toDegrees(Math.atan2(-12.75 - (robot.y() - 72.0),
                -17.7 - (robot.x() - 72.0))) - VisionConstants.shooterYawDegrees
                - Math.toDegrees(robot.heading());
        close(target.getYawErrorDegrees(), Math.IEEEremainder(expectedYaw, 360.0),
                "autonomous yaw uses the same CELL opening and shooter offset as TeleOp");
        double yaw = target.getYawErrorDegrees();
        close(distance, DistanceUnit.INCH.toCm(Math.hypot(
                -17.7 - (robot.x() - 72.0), -12.75 - (robot.y() - 72.0))),
                "autonomous uses the same CELL-opening center as aiming");
        Method clear = Vision.class.getDeclaredMethod("clearCurrentFrame");
        clear.setAccessible(true);
        clear.invoke(vision);
        target.update(robot);
        check(target.getState() == HiveState.UNKNOWN, "tag loss cannot reuse an old HIGH");
        close(target.getDistanceCm(), distance, "tag loss preserves distance geometry");
        close(target.getYawErrorDegrees(), yaw, "tag loss preserves coordinate aiming");
        Pose visionPose = new Pose(robot.x() + 2, robot.y(), robot.heading());
        frame(vision, visionPose, 30);
        target.update(robot);
        check(target.getState() == HiveState.HIGH, "Pinpoint translation drift does not mask HIGH");
        close(target.getDistanceCm(), distance, "shooting distance still uses Pinpoint");
        close(target.getYawErrorDegrees(), yaw, "shooting yaw still uses Pinpoint");
        frame(vision, visionPose, -30);
        target.update(robot);
        check(target.getState() == HiveState.LOW, "translation drift does not turn LOW into HIGH");
        frame(vision, new Pose(robot.x(), robot.y(), robot.heading() + Math.toRadians(20)), 30);
        target.update(robot);
        check(target.getState() == HiveState.UNKNOWN, "heading disagreement blocks HIGH");
        frame(vision, robot, 30);
        set(vision, Vision.class, "usableTagCount", 0);
        target.update(robot);
        check(target.getState() == HiveState.UNKNOWN, "undersized tags cannot report HIGH");
        target.update(new Pose(Double.NaN, 0, 0));
        check(target.getState() == HiveState.UNKNOWN && Double.isNaN(target.getDistanceCm()),
                "invalid odometry blocks shots");
        check(Double.isNaN(target.getYawErrorDegrees()), "invalid odometry clears aiming error");
        boolean rejected = false;
        try { new AutoHiveTarget(vision, CellCluster.BLUE_AUDIENCE); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "wrong-alliance CELL rejected");
    }

    private static class Motors implements Drivetrain {
        boolean stopped;
        int writes;
        public void drive(DrivePowers powers, boolean manual) { writes++; }
        public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
        public void stop() { stopped = true; }
        public void stop(boolean brake) { stop(); }
        public Map<String, Object> debug() { return Collections.emptyMap(); }
        public double interpolateVelocity(double a, double b, double theta) { return 0; }
    }

    private static class ShotFollower extends Follower {
        Pose current = new Pose(58.7467, 8.8149, Math.toRadians(90));
        ShotFollower() { super(null, new Motors(), null); }
        @Override public Pose pose() { return current; }
        @Override public Velocity velocity() { return Velocity.zero(); }
    }

    private static class Collector extends Intake {
        boolean feeding;
        int clears;
        Collector() { super(null); }
        @Override public void updateShotFeed(boolean enabled) { feeding = enabled; }
        @Override public void clearBallCount() { clears++; }
        @Override public void stop() { feeding = false; }
    }

    private static class Flywheels extends Shooter {
        boolean ready;
        boolean targetSet;
        boolean pitchBlocked;
        boolean rejectDistance;
        double requestedDistanceCm;
        Flywheels() { super(null); }
        @Override public boolean setDistanceTarget(double distance) {
            requestedDistanceCm = distance;
            targetSet = Double.isFinite(distance) && !rejectDistance;
            return targetSet;
        }
        @Override public boolean hasTargetVelocity() { return targetSet; }
        @Override public boolean isReady() { return ready; }
        @Override public boolean isPitchReady() { return !pitchBlocked; }
        @Override public void stop() { targetSet = false; }
        @Override public void periodic() { }
    }

    private static Transit transit() throws Exception {
        Transit transit = (Transit) ALLOCATOR.allocateInstance(Transit.class);
        DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (p, method, args) -> {
                    if (method.getName().equals("setPower")) return null;
                    throw new AssertionError("Unexpected motor call: " + method);
                });
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (p, method, args) -> {
                    if (method.getName().equals("setPosition")) return null;
                    throw new AssertionError("Unexpected servo call: " + method);
                });
        set(transit, Transit.class, "transitMotor", motor);
        set(transit, Transit.class, "limitServo", servo);
        set(transit, Transit.class, "feedReverseTimer", new ElapsedTime());
        set(transit, Transit.class, "transitState", Transit.TransitState.CLOSE);
        set(transit, Transit.class, "firingState", Transit.FiringState.IDLE);
        return transit;
    }

    public static void main(String[] args) throws Exception {
        checkHive();
        Blue3Tip auto = new Blue3Tip();
        checkBlue3TipBadReadFallback(auto);
        check(Constants.createAlgorithm() != null, "FTC1 calibration creates a Foresight algorithm");
        ForesightConfig calibratedConfig = Constants.foresightConfig;
        boolean missingConfig = false;
        try {
            Constants.foresightConfig = null;
            Constants.createAlgorithm();
        } catch (IllegalStateException expected) {
            missingConfig = expected.getMessage().contains("Foresight Tuner");
        } finally {
            Constants.foresightConfig = calibratedConfig;
        }
        check(missingConfig, "unmeasured Foresight configuration has an actionable error");

        Intake realIntake = (Intake) ALLOCATOR.allocateInstance(Intake.class);
        DcMotor realMotor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (p, method, values) -> null);
        set(realIntake, Intake.class, "intakeMotor", realMotor);
        set(realIntake, Intake.class, "ballCount", 4);
        realIntake.updateShotFeed(false);
        close(realIntake.getRequestedPower(), 0, "full bin held before feeding");
        realIntake.updateShotFeed(true);
        close(realIntake.getRequestedPower(), 0.5, "real intake assists feeding");
        realIntake.stop();
        close(realIntake.getRequestedPower(), 0, "real intake stops on cancellation");
        checkFollowerOwnership(auto.startToShoot1());
        checkAutoShoot();
        System.out.println("PASS TestAutoCheck: " + checks + " checks");
    }

    private static void checkAutoShoot() throws Exception {
        ShotFollower paths = new ShotFollower();
        Collector intake = (Collector) ALLOCATOR.allocateInstance(Collector.class);
        Flywheels shooter = (Flywheels) ALLOCATOR.allocateInstance(Flywheels.class);
        Transit transit = transit();
        Vision vision = vision();
        AutoHiveTarget target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
        boolean[] located = {true};
        AutoShootCommand shot = new AutoShootCommand(paths, shooter, intake, transit, target,
                () -> located[0], 900, 5000);
        double reverseTime = TransitConstants.feedReverseTime;
        try {
            TransitConstants.feedReverseTime = 0;
            shooter.targetSet = true;
            shot.initialize();
            check(shooter.targetSet, "shot initialization preserves pre-spun shooter");
            check(shot.getRequirements().contains(shooter) && shot.getRequirements().contains(intake)
                    && shot.getRequirements().contains(transit), "shot owns all three mechanisms");
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE, "shooter speed blocks feeding");
            Motors motors = (Motors) paths.drivetrain;
            check(paths.idle() && motors.stopped, "the drivetrain is stopped before the shot");
            shooter.ready = true;
            shooter.pitchBlocked = true;
            TransitConstants.feedReverseTime = -1;
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE
                    && shot.getStatus().equals("Invalid transit gate or reverse settings"),
                    "ready shot reports invalid transit settings");
            TransitConstants.feedReverseTime = 0;
            shot.execute();
            check(transit.getFiringState() == Transit.FiringState.FEEDING && intake.feeding,
                    "speed-ready shot feeds with the distance target applied");
            double initialDistance = shooter.requestedDistanceCm;
            paths.current = new Pose(paths.current.x() + 3, paths.current.y() + 2,
                    paths.current.heading() + .2);
            shot.execute();
            check(transit.getFiringState() == Transit.FiringState.FEEDING && intake.feeding,
                    "pose changes during a shot keep feeding active");
            check(shooter.requestedDistanceCm < initialDistance,
                    "approaching the CELL updates the shooter distance");
            close(shooter.requestedDistanceCm, target.getDistanceCm(), "shooter uses the updated distance");
            check(paths.idle() && motors.writes == 0, "pose changes leave the drivetrain idle");
            set(shot, AutoShootCommand.class, "startedNanos", System.nanoTime() - 5_100_000_000L);
            check(!shot.isFinished(), "the preparation deadline cannot cut short active feeding");
            located[0] = false;
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE && !intake.feeding
                    && !shooter.targetSet && motors.stopped && paths.idle(),
                    "localization loss closes the gate and stops every output");
            located[0] = true;
            shooter.rejectDistance = true;
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE && motors.stopped,
                    "invalid distance calibration blocks feeding and motion");
            shooter.rejectDistance = false;
            paths.current = new Pose(Double.NaN, 0, 0);
            shot.execute();
            check(!shooter.targetSet && motors.stopped, "non-finite pose cannot keep a shot running");
            set(shot, AutoShootCommand.class, "startedNanos", System.nanoTime() - 5_100_000_000L);
            check(shot.isFinished() && shot.hasTimedOut(), "total timeout includes readiness waiting");
            shot.end(false);
            check(shot.getStatus().equals("Shot timed out (Waiting for valid localization)"),
                    "timeout preserves the last shot blocker");
            paths.current = new Pose(58.5392, 17.7643, 0);
            shooter.ready = true;
            shot.initialize();
            check(!shot.hasTimedOut() && !shot.isFinished(), "rescheduling resets both shot timers");
            shot.execute();
            check(intake.feeding && paths.idle(), "rescheduled shot feeds with the follower idle");
            shot.end(true);
            check(motors.stopped && paths.idle() && !shooter.targetSet && !intake.feeding
                    && transit.getState() == Transit.TransitState.CLOSE,
                    "cancellation stops follower, flywheels, intake and transit");
            check(intake.clears == 2, "localization loss and cancellation clear the count while feeding");
            boolean invalidTime = false;
            try { new AutoShootCommand(paths, shooter, intake, transit, target, () -> true, 900, 800); }
            catch (IllegalArgumentException expected) { invalidTime = true; }
            check(invalidTime, "timeout shorter than feed duration is rejected");
        } finally {
            shot.end(true);
            TransitConstants.feedReverseTime = reverseTime;
        }
    }

    private static void checkBlue3TipBadReadFallback(Blue3Tip auto) throws Exception {
        Blue3Tip.CachedPinpointLocalizer localizer = new Blue3Tip.CachedPinpointLocalizer(null, null);
        set(auto, Blue3Tip.class, "localizer", localizer);
        Method ready = Blue3Tip.class.getDeclaredMethod("localizationReady");
        ready.setAccessible(true);
        MotionState good = MotionState.ofVelocity(new Pose(4, 5, 0.2), new Velocity(8, 9, 0.1));
        MotionState partialRead = MotionState.ofVelocity(new Pose(70, 50, 1.2), new Velocity(0, 40, 1.0));
        MotionState recovered = MotionState.ofVelocity(new Pose(5, 6, 0.3), new Velocity(7, 8, 0.2));

        localizer.acceptSample(DeviceStatus.FAULT_BAD_READ, partialRead, 0);
        check(!(boolean) ready.invoke(auto), "a bad read cannot start Auto without a valid sample");
        localizer.acceptSample(DeviceStatus.READY, good, 10_000_000L);
        check((boolean) ready.invoke(auto) && localizer.isFresh(), "a finite READY sample is accepted");

        localizer.acceptSample(DeviceStatus.FAULT_BAD_READ, partialRead, 30_000_000L);
        check((boolean) ready.invoke(auto) && localizer.usingCachedSample() && !localizer.isFresh(),
                "one bad frame preserves the running Auto");
        Motors motors = new Motors();
        Follower follower = new Follower(localizer, motors, Constants.createAlgorithm());
        follower.hold(new Pose(6, 7, 0.2));
        follower.update(0.02);
        check(motors.writes == 1 && follower.localizer.state() == good,
                "Pedro can keep controlling with the prior complete sample, including velocity");

        localizer.acceptSample(DeviceStatus.READY, recovered, 50_000_000L);
        check((boolean) ready.invoke(auto) && !localizer.usingCachedSample() && localizer.state() == recovered,
                "the next good frame restores live localization");
        localizer.acceptSample(DeviceStatus.FAULT_BAD_READ, partialRead, 70_000_000L);
        check((boolean) ready.invoke(auto), "a later isolated fault can use the new good sample");
        localizer.acceptSample(DeviceStatus.FAULT_BAD_READ, partialRead, 90_000_000L);
        check(!(boolean) ready.invoke(auto), "the second consecutive bad read stops Auto");

        localizer.acceptSample(DeviceStatus.READY, good, 110_000_000L);
        localizer.acceptSample(DeviceStatus.FAULT_BAD_READ, partialRead, 211_000_000L);
        check(!(boolean) ready.invoke(auto), "a sample older than 100 ms cannot keep Auto running");
        localizer.acceptSample(DeviceStatus.READY, good, 230_000_000L);
        localizer.acceptSample(DeviceStatus.NOT_READY, partialRead, 240_000_000L);
        check(!(boolean) ready.invoke(auto), "NOT_READY never uses the cache");
        localizer.acceptSample(DeviceStatus.READY,
                MotionState.ofVelocity(new Pose(4, 5, 0.2), new Velocity(Double.NaN, 0, 0)), 250_000_000L);
        check(!(boolean) ready.invoke(auto), "a nonfinite READY sample also stops Auto");
    }

    private static void checkFollowerOwnership(Path path) {
        Motors motors = new Motors();
        int[] algorithmCalls = {0};
        com.pedropathing.algorithm.Algorithm algorithm =
                (com.pedropathing.algorithm.Algorithm) Proxy.newProxyInstance(
                com.pedropathing.algorithm.Algorithm.class.getClassLoader(),
                new Class<?>[]{com.pedropathing.algorithm.Algorithm.class}, (p, m, a) -> {
                    if (m.getName().startsWith("calculate")) {
                        algorithmCalls[0]++; return DrivePowers.zero();
                    }
                    if (m.getName().equals("isBusy")) return false;
                    return null;
                });
        com.pedropathing.localization.Localizer localizer =
                (com.pedropathing.localization.Localizer) Proxy.newProxyInstance(
                com.pedropathing.localization.Localizer.class.getClassLoader(),
                new Class<?>[]{com.pedropathing.localization.Localizer.class}, (p, m, a) -> null);
        Follower follower = new Follower(localizer, motors, algorithm);
        follower.hold(new Pose(0, 0, 0));
        follower.stop();
        follower.update(0.02);
        check(motors.stopped && motors.writes == 0 && algorithmCalls[0] == 0 && follower.idle(),
                "stopping endpoint hold leaves the drivetrain idle");
        follower.follow(path);
        follower.update(0.02);
        check(motors.writes == 1 && algorithmCalls[0] == 1 && follower.following(),
                "next path drives the follower again");
        follower.stop(); follower.update(0.02);
        check(motors.stopped, "idle follower stops drivetrain");
    }
}
