package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
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
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
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
        check(Double.isNaN(target.getYawErrorDegrees()), "invalid odometry clears aiming error");
        boolean rejected = false;
        try { new AutoHiveTarget(vision, CellCluster.BLUE_AUDIENCE); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "wrong-alliance CELL rejected");
    }

    private static class Motors implements Drivetrain {
        boolean stopped;
        int writes;
        DrivePowers last;
        public void drive(DrivePowers powers, boolean manual) { writes++; last = powers; }
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
        DrivePowers manualRequest;
        Paths() { super(null, new Motors(), null); }
        @Override public void follow(Path path) { seen.add(path); busy = true; }
        @Override public boolean isBusy() { return busy; }
        @Override public Pose pose() { return current; }
        @Override public Velocity velocity() { return Velocity.zero(); }
        @Override public void manual(DrivePowers powers) {
            super.manual(powers);
            manualRequest = powers;
        }
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
        Flywheels() { super(null); }
        @Override public boolean setDistanceTarget(double distance) {
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

    private static void aimAtCell(Paths paths) {
        Pose pose = paths.current;
        double heading = Math.atan2(-12.75 - (pose.y() - 72.0), -17.7 - (pose.x() - 72.0))
                - Math.toRadians(VisionConstants.shooterYawDegrees);
        paths.current = new Pose(pose.x(), pose.y(), heading);
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
        set(auto, Test.class, "pinpoint", ALLOCATOR.allocateInstance(Odometry.class));
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
        AutoHiveTarget target = new ShotTarget(vision());
        set(target, AutoHiveTarget.class, "distanceCm", 100.0);
        set(target, AutoHiveTarget.class, "yawErrorDegrees", 0.0);
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
            check(intake.feeding == feeding, "intake assists only during forward feed");
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
        set(target, AutoHiveTarget.class, "yawErrorDegrees", 20.0);
        waitingShot.execute();
        check(paths.manual() && transit.getState() == Transit.TransitState.CLOSE && !intake.feeding,
                "aim owns Follower MANUAL; misalignment blocks feeding");
        check(((Motors) paths.drivetrain).writes == 0,
                "the shot command requests power without directly driving the motors");
        Field manualOutput = Follower.class.getDeclaredField("manualPowers");
        manualOutput.setAccessible(true);
        check(((DrivePowers) manualOutput.get(paths)).turn() > 0.0,
                "TeleOp clockwise output is converted to Pedro CCW convention");
        set(target, AutoHiveTarget.class, "yawErrorDegrees", 0.0);
        waitingShot.execute();
        Thread.sleep(170);
        waitingShot.execute();
        check(transit.getState() == Transit.TransitState.OPEN, "HIGH starts reverse/feed preparation");
        set(target, AutoHiveTarget.class, "state", HiveState.UNKNOWN);
        waitingShot.execute();
        check(transit.getState() == Transit.TransitState.CLOSE, "tag loss during reverse closes the gate");
        waitingShot.end(true);
        check(transit.getFiringState() == Transit.FiringState.IDLE && !intake.requested,
                "cancellation stops feeding and intake");
        Command relockShot = (Command) privateShot(auto, false);
        relockShot.initialize(); relockShot.execute();
        Thread.sleep(170); relockShot.execute();
        Thread.sleep(115); relockShot.execute();
        check(intake.feeding, "aligned and ready enables auxiliary intake feed");
        set(target, AutoHiveTarget.class, "yawErrorDegrees", 10.0);
        relockShot.execute();
        check(transit.getState() == Transit.TransitState.CLOSE && !intake.feeding
                && !relockShot.isFinished(), "lost alignment cancels feed and resets shot timer");
        set(target, AutoHiveTarget.class, "yawErrorDegrees", Double.NaN);
        relockShot.execute();
        check(paths.idle() && ((Motors) paths.drivetrain).stopped, "invalid aim stops turning");
        relockShot.end(true);
        check(!intake.feeding && ((Motors) paths.drivetrain).stopped,
                "interruption stops feed and drivetrain");

        AutoShootCommand expiredShot = (AutoShootCommand) privateShot(auto, false);
        expiredShot.initialize();
        set(expiredShot, AutoShootCommand.class, "startedNanos", System.nanoTime() - 5_100_000_000L);
        check(expiredShot.isFinished() && expiredShot.hasTimedOut(), "waiting counts toward shot timeout");
        expiredShot.end(false);
        check(expiredShot.getStatus().equals("Shot timed out") && !intake.feeding
                && transit.getState() == Transit.TransitState.CLOSE && !shooter.targetSet,
                "timeout stops the shooting mechanisms and reports the reason");

        // Real Intake: waiting for aim must override full-bin reverse, then assist feeding.
        Intake realIntake = (Intake) ALLOCATOR.allocateInstance(Intake.class);
        DcMotor realMotor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (p, method, values) -> null);
        set(realIntake, Intake.class, "intakeMotor", realMotor);
        set(realIntake, Intake.class, "ballCount", 4);
        realIntake.updateShotFeed(false);
        close(realIntake.getRequestedPower(), 0, "full bin held while aiming");
        realIntake.updateShotFeed(true);
        close(realIntake.getRequestedPower(), 0.5, "real intake assists feeding");
        realIntake.stop();
        close(realIntake.getRequestedPower(), 0, "real intake stops on cancellation");
        checkFollowerOwnership(auto.path1());
        checkAutoShoot();
        scheduler.reset();
        System.out.println("PASS TestAutoCheck: " + checks + " checks; real scheduler and transit timers");
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
        double stableTime = DriveConstants.aimLockStableTimeSeconds;
        double reverseTime = TransitConstants.feedReverseTime;
        boolean holdCalibrated = DriveConstants.positionHoldCalibrated;
        try {
            DriveConstants.aimLockStableTimeSeconds = 0;
            TransitConstants.feedReverseTime = 0;
            DriveConstants.positionHoldCalibrated = false;
            shooter.targetSet = true;
            shot.initialize();
            check(shooter.targetSet, "shot initialization preserves pre-spun shooter");
            check(shot.getRequirements().contains(shooter) && shot.getRequirements().contains(intake)
                    && shot.getRequirements().contains(transit), "shot owns all three mechanisms");
            aimAtCell(paths);
            paths.current = new Pose(paths.current.x(), paths.current.y(),
                    paths.current.heading() - Math.toRadians(10));
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE, "unlocked aim blocks feeding");
            Motors motors = (Motors) paths.drivetrain;
            check(paths.manualRequest.turn() > 0, "positive yaw error gives Pedro CCW power");
            paths.current = new Pose(paths.current.x(), paths.current.y(),
                    paths.current.heading() + Math.toRadians(20));
            shot.execute();
            check(paths.manualRequest.turn() < 0, "negative yaw error gives Pedro CW power");
            aimAtCell(paths);
            shot.execute();
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE, "shooter speed blocks feeding");
            shooter.ready = true;
            shooter.pitchBlocked = true;
            shot.execute();
            check(transit.getState() == Transit.TransitState.CLOSE, "pitch settling blocks feeding");
            shooter.pitchBlocked = false;
            shot.execute();
            check(transit.getFiringState() == Transit.FiringState.FEEDING && intake.feeding,
                    "ready shot uses both TeleOp feed outputs");
            shooter.ready = false;
            paths.current = new Pose(paths.current.x(), paths.current.y(), paths.current.heading() + .2);
            shot.execute();
            check(transit.getFiringState() == Transit.FiringState.IDLE && !intake.feeding,
                    "lost aim pauses automatic feeding");
            check(motors.writes == 0, "shot submits turn requests without driving the motors directly");
            shooter.ready = true;
            aimAtCell(paths);
            shot.execute();
            shot.execute();
            check(intake.feeding, "feeding resumes after realigning");
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
            check(shot.getStatus().equals("Shot timed out"), "timeout is distinguishable from completion");
            paths.current = new Pose(58.5392, 17.7643, 0);
            aimAtCell(paths);
            shooter.ready = true;
            DriveConstants.positionHoldCalibrated = true;
            shot.initialize();
            check(!shot.hasTimedOut() && !shot.isFinished(), "rescheduling resets both shot timers");
            shot.execute();
            paths.current = new Pose(paths.current.x() + 3, paths.current.y() + 2,
                    paths.current.heading());
            shot.execute();
            double fieldX = paths.manualRequest.forward() * Math.cos(paths.current.heading())
                    - paths.manualRequest.strafe() * Math.sin(paths.current.heading());
            double fieldY = paths.manualRequest.forward() * Math.sin(paths.current.heading())
                    + paths.manualRequest.strafe() * Math.cos(paths.current.heading());
            check(fieldX < 0 && fieldY < 0, "Pedro position hold pushes toward the original anchor");
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
            DriveConstants.aimLockStableTimeSeconds = stableTime;
            TransitConstants.feedReverseTime = reverseTime;
            DriveConstants.positionHoldCalibrated = holdCalibrated;
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
        follower.manual(0, 0, 0.25);
        follower.update(0.02);
        check(motors.writes == 1 && algorithmCalls[0] == 0 && follower.manual(),
                "aim MANUAL replaces endpoint HOLD; one drivetrain write, no hold controller");
        close(motors.last.turn(), 0.25, "manual aim output reaches Pedro drivetrain");
        follower.follow(path);
        follower.update(0.02);
        check(motors.writes == 2 && algorithmCalls[0] == 1 && follower.following(),
                "next path regains exclusive Follower output");
        follower.stop(); follower.update(0.02);
        check(motors.stopped, "idle follower stops drivetrain");
    }

    private static Object privateShot(Test auto, boolean waitForHigh) throws Exception {
        Method method = Test.class.getDeclaredMethod("shootFor", long.class, boolean.class);
        method.setAccessible(true);
        return method.invoke(auto, 900L, waitForHigh);
    }
}
