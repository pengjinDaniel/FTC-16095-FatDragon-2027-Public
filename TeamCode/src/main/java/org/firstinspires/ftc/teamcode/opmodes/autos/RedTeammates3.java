package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "RedTeammates3", group = "Auto")
public class RedTeammates3 extends AllianceAuto {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Vision vision;
    private AutoHiveTarget target;
    private GoBildaPinpointDriver pinpoint;
    private AutoShootCommand activeShot;
    private static final long SHOT_TIMEOUT_MS = 5000;
    private String stage = "INIT";
    private String path4ShotResult = "Pending";

    private final Pose start = poseFactory.of(53.8888, 8.5301, 270);
    private final Pose path1 = poseFactory.of(58.5282, 16.7053, 270);
    private final Pose point2 = poseFactory.of(12.0367, 46.7768, 180);
    private final Pose point2Control1 = poseFactory.of(50.4207, 41.1383, 0);
    private final Pose point3 = poseFactory.of(20.7471, 47.2235, -177.0643);
    private final Pose point4Start = poseFactory.of(20.7471, 47.2235, 180);
    private final Pose point4 = poseFactory.of(58.3904, 22.1377, 270);
    private final Pose point5 = poseFactory.of(10.2145, 6.5435, 180);
    private final Pose point5Control1 = poseFactory.of(54.4203, 9.8484, 0);
    private final Pose point6 = poseFactory.of(58.3834, 17.221, 270);
    private final Pose point6Control1 = poseFactory.of(40.87, 16.941, 0);
    private final Pose point7 = poseFactory.of(8.0604, 99.6839, 270);
    private final Pose point7Control1 = poseFactory.of(18.8328, 61.2259, 0);

    public RedTeammates3() {
        this(Alliance.RED);
    }

    RedTeammates3(Alliance alliance) {
        super(alliance);
    }

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                followPath(1, path1()),
                shootFor(900, false),
                intakeDuringPath(2, path2(), 1000)
                        .withTimeout(5000)
                        .beforeStarting(() -> {
                            if (intake.getFlowerState() == Intake.FlowerState.RAISED) {
                                intake.toggleFlower();
                            }
                        }),
                followPath(3, path3()).beforeStarting(intake::raiseFlower),
                followPath(4, path4()),
                shootFor(900, true),
                intakeDuringPath(5, path5(), 0),
                intakeDuringPath(6, path6(), 0),
                shootFor(900, false),
                followPath(7, path7()));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        try {
            if (Constants.foresightConfig == null) {
                while (opModeInInit() && !isStopRequested()) {
                    telemetry.addLine("Run AutoTune > Foresight Tuner first.");
                    telemetry.addLine("Paste its Java export into pedro/Constants.java and rebuild.");
                    telemetry.update();
                    idle();
                }
                return;
            }

            follower = Constants.create(hardwareMap);
            intake = new Intake(hardwareMap);
            shooter = new Shooter(hardwareMap);
            transit = new Transit(hardwareMap);
            vision = new Vision(hardwareMap, alliance);
            target = new AutoHiveTarget(vision, shotCell);
            pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            follower.setPose(start);
            follower.update();
            scheduler.addButton(() -> {
                if (activeShot != null) return;
                target.update(follower.pose());
                if (isStarted() && !isStopRequested()) {
                    shooter.setDistanceTarget(target.getDistanceCm());
                }
            });

            while (opModeInInit() && !isStopRequested()) {
                follower.update();
                scheduler.run();
                showTelemetry();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                telemetry.addLine("Pinpoint is not READY; autonomous stopped.");
                telemetry.update();
                return;
            }

            follower.setPose(start);
            Command routine = autoRoutine();
            scheduler.schedule(routine);
            while (opModeIsActive() && scheduler.isScheduled(routine)) {
                follower.update();
                if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                    stage = "Stopped: Pinpoint is not READY";
                    showTelemetry();
                    break;
                }
                scheduler.run();
                showTelemetry();
                idle();
            }
        } finally {
            scheduler.cancelAll();
            if (transit != null) transit.cancelFeeding();
            if (intake != null) {
                intake.stop();
                if (intake.getFlowerState() == Intake.FlowerState.DEPLOYED) intake.raiseFlower();
            }
            if (shooter != null) shooter.stop();
            if (vision != null) vision.stop();
            if (follower != null) {
                follower.stop();
                follower.drivetrain.stop();
            }
            scheduler.reset();
        }
    }

    private Command followPath(int number, Path path) {
        return followPath(number, path, 0);
    }

    private Command followPath(int number, Path path, long waitMs) {
        return new CommandBase() {
            private ElapsedTime waitTimer;

            @Override
            public void initialize() {
                waitTimer = null;
                stage = "Path " + number;
                follower.follow(path);
            }

            @Override
            public void execute() {
                if (waitMs > 0 && waitTimer == null && !follower.isBusy()) {
                    waitTimer = new ElapsedTime();
                    stage = "Intake wait: " + waitMs + " ms";
                }
            }

            @Override
            public boolean isFinished() {
                return !follower.isBusy()
                        && (waitMs == 0 || (waitTimer != null && waitTimer.milliseconds() >= waitMs));
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) {
                    follower.stop();
                    follower.drivetrain.stop();
                }
            }
        };
    }

    private Command intakeDuringPath(int number, Path path, long waitMs) {
        return new ParallelDeadlineGroup(followPath(number, path, waitMs), intakeRequest());
    }

    private Command intakeRequest() {
        return new FunctionalCommand(
                () -> updateIntake(true),
                () -> updateIntake(true),
                interrupted -> updateIntake(false),
                () -> false, intake, transit);
    }

    private void updateIntake(boolean requested) {
        intake.updateInputs(requested, false);
        transit.updateIntakeRequest(requested, false);
    }

    private Command shootFor(long milliseconds, boolean delayBeforeShooting) {
        return new AutoShootCommand(follower, shooter, intake, transit, target,
                () -> pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                milliseconds, SHOT_TIMEOUT_MS, delayBeforeShooting) {
            @Override
            public void initialize() {
                activeShot = this;
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                stage = getStatus();
                if (delayBeforeShooting) path4ShotResult = stage;
                activeShot = null;
            }
        };
    }

    private void showTelemetry() {
        telemetry.addData("Stage", activeShot == null ? stage : activeShot.getStatus());
        telemetry.addData("X / Y / heading", "%.2f / %.2f / %.1f",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("CELL / HIVE", "%s / %s", target.getCluster(), target.getState());
        telemetry.addData("HIVE observation", target.getStatus());
        telemetry.addData("Shot distance (cm)", target.getDistanceCm());
        telemetry.addData("Shot yaw error (deg)", target.getYawErrorDegrees());
        telemetry.addData("Shooter at speed", shooter.isReady());
        telemetry.addData("Transit / gate", "%s / %s", transit.getFiringState(), transit.getState());
        telemetry.addData("Path 4 shot", path4ShotResult);
        telemetry.addData("Balls", intake.getBallCount());
        telemetry.update();
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).heading(Interpolator.piecewise()
                .until(0.701, Interpolator.linear(path1, point2))
                .until(1.0, Interpolator.constant(point2)));
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).linear(point5, point6);
    }

    public Path path7() {
        return curve(point6, point7Control1, point7).linear(point6, point7);
    }
}
