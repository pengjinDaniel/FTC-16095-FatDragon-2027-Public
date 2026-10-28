package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
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
        set(vision, Vision.class, "botposeTagCount", 1);
        set(vision, Vision.class, "usableTagCount", 1);
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
            check(Double.isFinite(target.getDistanceCm()), "valid shot distance");
        }
        frame(vision, robot, 30);
        target.update(robot);
        double distance = target.getDistanceCm();
        close(distance, DistanceUnit.INCH.toCm(Math.hypot(
                -13.1064522228 - (robot.x() - 72.0), -12.75 - (robot.y() - 72.0))),
                "autonomous uses the same four-tag-center shot distance");
        Method clear = Vision.class.getDeclaredMethod("clearCurrentFrame");
        clear.setAccessible(true);
        clear.invoke(vision);
        target.update(robot);
        check(target.getState() == HiveState.UNKNOWN, "tag loss cannot reuse an old HIGH");
        close(target.getDistanceCm(), distance, "tag loss preserves distance geometry");
        frame(vision, new Pose(robot.x() + 100, robot.y(), robot.heading()), 30);
        target.update(robot);
        check(target.getState() == HiveState.UNKNOWN, "out-of-gate observation cannot report HIGH");
        frame(vision, robot, 30);
        set(vision, Vision.class, "usableTagCount", 0);
        target.update(robot);
        check(target.getState() == HiveState.UNKNOWN, "undersized tags cannot report HIGH");
        target.update(new Pose(Double.NaN, 0, 0));
        check(target.getState() == HiveState.UNKNOWN && Double.isNaN(target.getDistanceCm()),
                "invalid odometry blocks shots");
        boolean rejected = false;
        try { new AutoHiveTarget(vision, CellCluster.BLUE_AUDIENCE); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "wrong-alliance CELL rejected");
    }

    private static class Motors implements Drivetrain {
        boolean stopped;
        public void drive(DrivePowers powers, boolean manual) { }
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
        @Override public void stop() { busy = false; }
        void finish() { current = seen.get(seen.size() - 1).endPose(); busy = false; }
    }

    private static class Collector extends Intake {
        boolean requested;
        int clears;
        Collector() { super(null); }
        @Override public void updateInputs(boolean intake, boolean jam) { requested = intake; }
        @Override public void clearBallCount() { clears++; }
        @Override public void stop() { requested = false; }
        @Override public void periodic() { }
    }

    private static class Flywheels extends Shooter {
        boolean ready;
        boolean targetSet;
        Flywheels() { super(null); }
        @Override public boolean setDistanceTarget(double distance) {
            targetSet = Double.isFinite(distance);
            return targetSet;
        }
        @Override public boolean hasTargetVelocity() { return targetSet; }
        @Override public boolean isReady() { return ready; }
        @Override public boolean isPitchReady() { return true; }
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

    private static Test auto(Paths paths, Collector intake, Flywheels shooter,
                             Transit transit, AutoHiveTarget target) throws Exception {
        Test auto = (Test) ALLOCATOR.allocateInstance(Test.class);
        String[] names = {"start", "path1Start", "path1", "point2", "point2Control1", "point3",
                "point4Start", "point4", "point5", "point5Control1", "point6Start", "point6",
                "point7", "point7Control1"};
        double[][] poses = {{58.7467,8.8149,90},{58.7467,8.8149,270},{58.5392,17.7643,270},
                {14.0701,47.376,180},{46.5237,43.2977,0},{20.7471,47.2235,178.6914},
                {20.7471,47.2235,180},{57.7667,23.3801,270},{8.2064,7.9837,179},
                {47.2051,8.5841,0},{8.2064,7.9837,180},{58.3834,17.221,270},
                {8.8312,94.3907,270},{18.8328,61.2259,0}};
        PoseFactory factory = PoseFactory.degrees();
        for (int i = 0; i < names.length; i++) {
            set(auto, Test.class, names[i], factory.of(poses[i][0], poses[i][1], poses[i][2]));
        }
        set(auto, Test.class, "follower", paths);
        set(auto, Test.class, "intake", intake);
        set(auto, Test.class, "shooter", shooter);
        set(auto, Test.class, "transit", transit);
        set(auto, Test.class, "target", target);
        return auto;
    }

    public static void main(String[] args) throws Exception {
        checkHive();
        boolean missingConfig = false;
        try { Constants.createAlgorithm(); }
        catch (IllegalStateException expected) {
            missingConfig = expected.getMessage().contains("Foresight Tuner");
        }
        check(missingConfig, "unmeasured Foresight configuration has an actionable error");

        Paths paths = new Paths();
        Collector intake = (Collector) ALLOCATOR.allocateInstance(Collector.class);
        Flywheels shooter = (Flywheels) ALLOCATOR.allocateInstance(Flywheels.class);
        Transit transit = transit();
        shooter.targetSet = true;
        AutoHiveTarget target = new AutoHiveTarget(vision(), CellCluster.RED_OPPOSITE_AUDIENCE);
        set(target, AutoHiveTarget.class, "distanceCm", 100.0);
        Test auto = auto(paths, intake, shooter, transit, target);
        close(auto.path1().get(0).heading(), Math.toRadians(270), "first path heading");
        close(auto.path2().endPose().x(), 14.0701, "second path end X");
        close(auto.path4().endPose().y(), 23.3801, "fourth path end Y");
        close(auto.path7().endPose().y(), 94.3907, "park end Y");

        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(transit);
        Command routine = auto.autoRoutine();
        scheduler.schedule(routine);
        long started = System.nanoTime(), path2Ended = 0, path4Ended = 0, feedStarted = 0;
        int shots = 0;
        boolean previousFeeding = false, checkedWait = false, checkedHighWait = false;
        Field stageField = Test.class.getDeclaredField("stage");
        stageField.setAccessible(true);
        while (scheduler.isScheduled(routine)) {
            long now = System.nanoTime();
            check(now - started < 8_000_000_000L, "routine must finish: paths=" + paths.seen.size()
                    + ", stage=" + stageField.get(auto) + ", feed=" + transit.getFiringState()
                    + ", ready=" + shooter.ready + ", target=" + shooter.targetSet);
            int path = paths.seen.size();
            if (paths.busy) {
                if (path == 2) {
                    check(intake.requested && transit.getRequestedPower() > 0,
                            "path 2 applies both LT outputs");
                    path2Ended = now;
                }
                if (path == 3) {
                    check(now - path2Ended >= 490_000_000L, "path 2 waits 500 ms");
                    check(!intake.requested, "LT released before path 3");
                    checkedWait = true;
                }
                if (path == 4) path4Ended = now;
                if (path == 5) check(intake.requested, "path 5 applies LT");
                if (path == 6 || path == 7) check(!intake.requested, "LT released after path 5");
                paths.finish();
            }
            shooter.ready = now - started >= 150_000_000L;
            if (path == 4 && path4Ended != 0 && now - path4Ended < 200_000_000L) {
                set(target, AutoHiveTarget.class, "state",
                        now - path4Ended < 100_000_000L ? HiveState.LOW : HiveState.UNKNOWN);
                check(transit.getFiringState() != Transit.FiringState.FEEDING,
                        "LOW and UNKNOWN keep path 4 waiting");
                checkedHighWait = true;
            } else if (path == 4) {
                set(target, AutoHiveTarget.class, "state", HiveState.HIGH);
            }
            scheduler.run();
            boolean feeding = transit.getFiringState() == Transit.FiringState.FEEDING;
            if (feeding && !previousFeeding) feedStarted = System.nanoTime();
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
        check(checkedWait && checkedHighWait, "both waiting behaviors exercised");
        check(transit.getState() == Transit.TransitState.CLOSE, "routine closes the gate");
        check(intake.clears == 3, "each completed feed clears the ball count");
        scheduler.cancelAll();

        Command waitingShot = (Command) privateShot(auto, true);
        waitingShot.initialize();
        set(target, AutoHiveTarget.class, "state", HiveState.HIGH);
        waitingShot.execute();
        check(transit.getState() == Transit.TransitState.OPEN, "HIGH starts reverse/feed preparation");
        set(target, AutoHiveTarget.class, "state", HiveState.UNKNOWN);
        waitingShot.execute();
        check(transit.getState() == Transit.TransitState.CLOSE, "tag loss during reverse closes the gate");
        waitingShot.end(true);
        check(transit.getFiringState() == Transit.FiringState.IDLE && !intake.requested,
                "cancellation stops feeding and intake");
        scheduler.reset();
        System.out.println("PASS TestAutoCheck: " + checks + " checks; real scheduler and transit timers");
    }

    private static Object privateShot(Test auto, boolean waitForHigh) throws Exception {
        Method method = Test.class.getDeclaredMethod("shootFor", long.class, boolean.class);
        method.setAccessible(true);
        return method.invoke(auto, 900L, waitForHigh);
    }
}
