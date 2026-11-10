package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/** Runs the real auto command groups, transit sequence, paths, and HIVE math without robot hardware. */
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

    private static class Paths extends Follower {
        final List<Path> seen = new ArrayList<>();
        Pose current = new Pose(58.7467, 8.8149, Math.toRadians(90));
        boolean busy;
        Paths() { super(null, new Motors(), null); }
        @Override public void follow(Path path) { seen.add(path); busy = true; }
        @Override public boolean isBusy() { return busy; }
        @Override public Pose pose() { return current; }
        @Override public Velocity velocity() { return Velocity.zero(); }
        @Override public void stop() { super.stop(); busy = false; }
        void finish() { current = seen.get(seen.size() - 1).endPose(); busy = false; }
    }

    private static class Collector extends Intake {
        boolean requested;
        int clears;
        boolean feeding;
        @Override public void updateShotFeed(boolean enabled) { requested = false; feeding = enabled; }
        Collector() { super(null); }
        @Override public void updateInputs(boolean intake, boolean jam) { requested = intake; }
        @Override public void clearBallCount() { clears++; }
        @Override public void stop() { requested = false; feeding = false; }
        @Override public void periodic() { }
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

    private static class Odometry extends GoBildaPinpointDriver {
        Odometry() { super(null, false); }
        @Override public DeviceStatus getDeviceStatus() { return DeviceStatus.READY; }
    }

    private static class ShotTarget extends AutoHiveTarget {
        ShotTarget(Vision vision) { super(vision, CellCluster.RED_OPPOSITE_AUDIENCE); }
        // The scheduler test supplies observations; checkHive exercises the real geometry.
        @Override public void update(Pose pose) { }
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

    private static RedTeammates3 auto(Paths paths, Collector intake, Flywheels shooter,
                             Transit transit, AutoHiveTarget target) throws Exception {
        RedTeammates3 auto = new RedTeammates3();
        paths.current = auto.path1().get(0);
        Servo flowerServo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (p, method, values) -> null);
        set(intake, Intake.class, "leftFlowerServo", flowerServo);
        set(intake, Intake.class, "rightFlowerServo", flowerServo);
        intake.raiseFlower();
        set(auto, RedTeammates3.class, "follower", paths);
        set(auto, RedTeammates3.class, "intake", intake);
        set(auto, RedTeammates3.class, "shooter", shooter);
        set(auto, RedTeammates3.class, "transit", transit);
        set(auto, RedTeammates3.class, "target", target);
        set(auto, RedTeammates3.class, "pinpoint", ALLOCATOR.allocateInstance(Odometry.class));
        return auto;
    }

    public static void main(String[] args) throws Exception {
        checkHive();
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

        Paths paths = new Paths();
        Collector intake = (Collector) ALLOCATOR.allocateInstance(Collector.class);
        Flywheels shooter = (Flywheels) ALLOCATOR.allocateInstance(Flywheels.class);
        Transit transit = transit();
        shooter.targetSet = true;
        AutoHiveTarget target = new ShotTarget(vision());
        set(target, AutoHiveTarget.class, "distanceCm", 100.0);
        set(target, AutoHiveTarget.class, "yawErrorDegrees", 20.0);
        shooter.pitchBlocked = true;
        RedTeammates3 auto = auto(paths, intake, shooter, transit, target);
        close(auto.path1().get(0).x(), 53.8888, "first path start X");
        close(auto.path1().get(0).heading(), Math.toRadians(270), "first path heading");
        close(auto.path1().endPose().x(), 58.5282, "first path end X");
        close(auto.path1().endPose().y(), 16.7053, "first path end Y");
        Path secondPath = auto.path2();
        close(secondPath.get(0.5).x(), 42.851575, "second path midpoint X");
        close(secondPath.get(0.5).y(), 36.439675, "second path midpoint Y");
        close(secondPath.endPose().x(), 12.0367, "second path end X");
        close(secondPath.endPose().y(), 46.7768, "second path end Y");
        double headingProgress = secondPath.curve.pathCompletion(0.35) / 0.701;
        close(secondPath.get(0.35).heading(), Math.toRadians(270 - 90 * headingProgress),
                "second path turns through the first 70.1 percent");
        close(secondPath.get(0.9).heading(), Math.PI, "second path holds 180 degrees near the end");
        close(Math.IEEEremainder(auto.path3().endPose().heading(), 2 * Math.PI),
                Math.toRadians(-177.0643), "third path end heading");
        close(auto.path4().endPose().x(), 58.3904, "fourth path end X");
        close(auto.path4().endPose().y(), 22.1377, "fourth path end Y");
        close(auto.path5().get(0.5).x(), 44.361375, "fifth path midpoint X");
        close(auto.path5().get(0.5).y(), 12.0945, "fifth path midpoint Y");
        close(auto.path5().endPose().x(), 10.2145, "fifth path end X");
        close(auto.path5().endPose().y(), 6.5435, "fifth path end Y");
        close(auto.path5().endPose().heading(), Math.PI, "fifth path end heading");
        close(auto.path6().get(0.5).y(), 14.411625, "sixth path follows the supplied curve");
        close(auto.path7().endPose().x(), 8.0604, "park end X");
        close(auto.path7().endPose().y(), 99.6839, "park end Y");

        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(transit);
        Command routine = auto.autoRoutine();
        scheduler.schedule(routine);
        long started = System.nanoTime(), path2Ended = 0, path4Ended = 0, feedStarted = 0;
        int[] shootingPaths = {1, 4, 6};
        int shots = 0;
        boolean previousFeeding = false, checkedIntakeWait = false;
        boolean checkedWait = false, checkedShotDelay = false;
        Field stageField = RedTeammates3.class.getDeclaredField("stage");
        stageField.setAccessible(true);
        while (scheduler.isScheduled(routine)) {
            long now = System.nanoTime();
            check(now - started < 16_000_000_000L, "routine must finish: paths=" + paths.seen.size()
                    + ", stage=" + stageField.get(auto) + ", feed=" + transit.getFiringState()
                    + ", ready=" + shooter.ready + ", target=" + shooter.targetSet);
            int path = paths.seen.size();
            boolean finishingPath6 = path == 6 && paths.busy;
            if (path == 2 && path2Ended != 0 && now - path2Ended < 1_000_000_000L) {
                check(intake.requested && transit.getRequestedPower() > 0,
                        "intake stays on through the path 2 wait");
                check(intake.getFlowerState() == Intake.FlowerState.DEPLOYED,
                        "flower stays deployed through the path 2 wait");
                checkedIntakeWait = true;
            }
            if (paths.busy) {
                if (path == 2) {
                    check(intake.requested && transit.getRequestedPower() > 0,
                            "path 2 applies both LT outputs");
                    check(intake.getFlowerState() == Intake.FlowerState.DEPLOYED,
                            "path 2 deploys the flower");
                    path2Ended = now;
                }
                if (path == 3) {
                    check(now - path2Ended >= 1_000_000_000L, "path 2 waits 1000 ms");
                    check(!intake.requested, "LT released before path 3");
                    check(intake.getFlowerState() == Intake.FlowerState.RAISED,
                            "path 3 raises the flower");
                    checkedWait = true;
                }
                if (path == 4) path4Ended = now;
                if (path == 5 || path == 6) {
                    check(intake.requested && transit.getRequestedPower() > 0,
                            "paths 5 and 6 collect with intake and transit");
                }
                if (path == 7) check(!intake.requested, "collection ends before path 7");
                paths.finish();
            }
            shooter.ready = now - started >= 150_000_000L;
            if (path == 4 && path4Ended != 0) {
                boolean stillWaiting = now - path4Ended < 7_000_000_000L;
                set(target, AutoHiveTarget.class, "state", stillWaiting ? HiveState.HIGH : HiveState.UNKNOWN);
                if (stillWaiting) {
                    check(transit.getState() == Transit.TransitState.CLOSE,
                            "path 4 keeps the gate closed for seven seconds even at HIGH");
                    check(paths.idle(), "path 4 waits for its shot with the follower idle");
                    checkedShotDelay = true;
                }
            } else if (path == 6) {
                set(target, AutoHiveTarget.class, "state", HiveState.UNKNOWN);
            }
            scheduler.run();
            if (finishingPath6) {
                check(!intake.requested && transit.getRequestedPower() == 0,
                        "path 6 releases collection before shooting");
            }
            boolean feeding = transit.getFiringState() == Transit.FiringState.FEEDING;
            check(intake.feeding == feeding, "intake assists only during forward feed");
            if (feeding && !previousFeeding) {
                check(shots < shootingPaths.length && path == shootingPaths[shots],
                        "feeds after paths 1, 4, and 6 in order");
                if (path == 4) {
                    long delay = now - path4Ended;
                    check(delay >= 7_000_000_000L && delay < 7_600_000_000L,
                            "path 4 feeds after the fixed delay even without HIGH");
                }
                feedStarted = System.nanoTime();
            }
            if (!feeding && previousFeeding) {
                long duration = System.nanoTime() - feedStarted;
                check(duration >= 880_000_000L && duration < 1_300_000_000L,
                        "actual forward feed lasts 900 ms; got " + duration / 1e6);
                shots++;
            }
            previousFeeding = feeding;
            Thread.sleep(2);
        }
        check(paths.seen.size() == 7 && shots == 3, "seven paths and exactly three shots");
        check(checkedIntakeWait && checkedWait && checkedShotDelay,
                "intake wait and path 4 fixed delay exercised");
        check(transit.getState() == Transit.TransitState.CLOSE, "routine closes the gate");
        check(intake.clears == 3, "each completed feed clears the ball count");
        scheduler.cancelAll();
        checkPath2Timeout(auto, paths, intake, transit, false);
        checkPath2Timeout(auto, paths, intake, transit, true);

        Command speedShot = (Command) privateShot(auto, false);
        speedShot.initialize();
        shooter.ready = false;
        speedShot.execute();
        check(paths.idle() && transit.getState() == Transit.TransitState.CLOSE && !intake.feeding,
                "the shot waits for flywheel speed with the follower idle");
        shooter.ready = true;
        speedShot.execute();
        check(transit.getState() == Transit.TransitState.OPEN,
                "flywheel speed starts the shot with yaw error and pitch still settling");
        set(target, AutoHiveTarget.class, "yawErrorDegrees", Double.NaN);
        Thread.sleep(115);
        speedShot.execute();
        check(intake.feeding && paths.idle(),
                "feeding at speed uses distance with the follower idle and no yaw estimate");
        set(speedShot, AutoShootCommand.class, "startedNanos", System.nanoTime() - 5_100_000_000L);
        check(!speedShot.isFinished(), "the preparation deadline cannot cut short active feeding");
        speedShot.end(true);
        check(transit.getFiringState() == Transit.FiringState.IDLE && !intake.feeding
                        && ((Motors) paths.drivetrain).stopped,
                "interruption stops feeding and drivetrain");

        AutoShootCommand expiredShot = (AutoShootCommand) privateShot(auto, false);
        expiredShot.initialize();
        set(expiredShot, AutoShootCommand.class, "startedNanos", System.nanoTime() - 5_100_000_000L);
        check(expiredShot.isFinished() && expiredShot.hasTimedOut(), "waiting counts toward shot timeout");
        expiredShot.end(false);
        check(expiredShot.getStatus().equals("Shot timed out (Preparing shot)") && !intake.feeding
                && transit.getState() == Transit.TransitState.CLOSE && !shooter.targetSet,
                "timeout stops the shooting mechanisms and reports the reason");

        // Hold a full bin during shot preparation, then assist feeding.
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
        checkFollowerOwnership(auto.path1());
        checkAutoShoot();
        scheduler.reset();
        System.out.println("PASS TestAutoCheck: " + checks + " checks; real scheduler and transit timers");
    }

    private static void checkPath2Timeout(RedTeammates3 auto, Paths paths, Collector intake, Transit transit,
                                          boolean finishesLate) throws InterruptedException {
        paths.seen.clear();
        Motors motors = (Motors) paths.drivetrain;
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Command routine = auto.autoRoutine();
        long started = System.nanoTime(), path2Started = 0, path2Arrived = 0;
        scheduler.schedule(routine);
        try {
            while (paths.seen.size() < 3 && scheduler.isScheduled(routine)) {
                long now = System.nanoTime();
                check(now - started < 8_000_000_000L, "path 2 timeout must start path 3");
                if (paths.seen.size() == 1 && paths.busy) paths.finish();
                if (paths.seen.size() == 2) {
                    if (path2Started == 0) {
                        path2Started = now;
                        motors.stopped = false;
                    }
                    if (finishesLate && paths.busy && now - path2Started >= 4_750_000_000L) {
                        paths.finish();
                        path2Arrived = now;
                    }
                }
                scheduler.run();
                Thread.sleep(2);
            }
            check(paths.seen.size() == 3, "path 3 starts after the path 2 timeout");
            long now = System.nanoTime();
            long path2Elapsed = now - path2Started;
            check(path2Elapsed >= 4_950_000_000L && path2Elapsed < 5_600_000_000L,
                    "path 2 uses a five-second total deadline");
            if (finishesLate) {
                check(path2Arrived != 0 && now - path2Arrived < 1_000_000_000L,
                        "the deadline cuts short the intake wait after a late arrival");
            }
            check(motors.stopped && !intake.requested && transit.getRequestedPower() == 0,
                    "timeout releases the path 2 drivetrain and intake outputs");
            check(intake.getFlowerState() == Intake.FlowerState.RAISED,
                    "path 3 raises the flower after timeout");
        } finally {
            scheduler.cancel(routine);
        }
    }

    private static void checkAutoShoot() throws Exception {
        Paths paths = new Paths();
        Collector intake = (Collector) ALLOCATOR.allocateInstance(Collector.class);
        Flywheels shooter = (Flywheels) ALLOCATOR.allocateInstance(Flywheels.class);
        Transit transit = transit();
        Vision vision = vision();
        AutoHiveTarget target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
        boolean[] located = {true};
        AutoShootCommand shot = new AutoShootCommand(paths, shooter, intake, transit, target,
                () -> located[0], 900, 5000, false);
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
            boolean invalidTime = false;
            try { new AutoShootCommand(paths, shooter, intake, transit, target, () -> true, 900, 800, false); }
            catch (IllegalArgumentException expected) { invalidTime = true; }
            check(invalidTime, "timeout shorter than feed duration is rejected");
        } finally {
            shot.end(true);
            TransitConstants.feedReverseTime = reverseTime;
        }
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

    private static Object privateShot(RedTeammates3 auto, boolean delayBeforeShooting) throws Exception {
        Method method = RedTeammates3.class.getDeclaredMethod("shootFor", long.class, boolean.class);
        method.setAccessible(true);
        return method.invoke(auto, 900L, delayBeforeShooting);
    }
}
