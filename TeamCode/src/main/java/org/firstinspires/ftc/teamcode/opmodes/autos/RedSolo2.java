package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "RedSolo2", group = "Auto")
public class RedSolo2 extends LinearOpMode {
    private static final long SHOT_DURATION_MS = 900;
    private static final long SHOT_TIMEOUT_MS = 5000;
    private static final long PATH_5_WAIT_MS = 1000;
    private static final long PATH_5_TIMEOUT_MS = 4000;

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Vision vision;
    private AutoHiveTarget target;
    private GoBildaPinpointDriver pinpoint;
    private AutoShootCommand activeShot;
    private int pathNumber;
    private boolean path5TimedOut;
    private String lastShotResult = "Pending";

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(53.9758, 8.557, 270);
    private final Pose path1 = poseFactory.of(57.2779, 24.3089, 270);
    private final Pose point2 = poseFactory.of(7.56, 8.0921, -90);
    private final Pose point2Control1 = poseFactory.of(6.2499, 30.0306, 0);
    private final Pose point2Segment1Start = poseFactory.of(7.56, 8.0921, 270);
    private final Pose point2Segment1End = poseFactory.of(7.56, 8.0921, 270);
    private final Pose point2Segment2Heading = poseFactory.of(7.56, 8.0921, -90);
    private final Pose point3Start = poseFactory.of(7.56, 8.0921, 270);
    private final Pose point3 = poseFactory.of(58.1225, 122.0799, 90);
    private final Pose point3Control1 = poseFactory.of(27.545, 112.1358, 0);
    private final Pose point4 = poseFactory.of(46.5666, 127.544, 90);
    private final Pose point4Segment1Start = poseFactory.of(46.5666, 127.544, 90);
    private final Pose point4Segment1End = poseFactory.of(46.5666, 127.544, 90);
    private final Pose point4Segment2Heading = poseFactory.of(46.5666, 127.544, 90);
    private final Pose point5 = poseFactory.of(46.6072, 132.0961, 90);
    private final Pose point6 = poseFactory.of(46.6966, 127.8097, 90);
    private final Pose point7 = poseFactory.of(58.0777, 121.299, 90);
    private final Pose point8Start = poseFactory.of(58.0777, 121.299, 90);
    private final Pose point8 = poseFactory.of(11.6138, 93.617, 0);

    public Command autoRoutine() {
        Command deployFlower = new WaitUntilCommand(
                () -> follower.parametricCompletion() >= 0.5 || !follower.isBusy())
                .andThen(new InstantCommand(() -> {
                    if (intake.getFlowerState() == Intake.FlowerState.RAISED) {
                        intake.toggleFlower();
                    }
                }, intake));

        return new SequentialCommandGroup(
                followPath(1, path1()),
                shoot(),
                followPath(2, path2()).beforeStarting(() -> {
                    intake.updateInputs(true, false);
                    transit.updateIntakeRequest(true, false);
                }),
                followPath(3, path3()),
                shoot(),
                new ParallelDeadlineGroup(followPath(4, path4()), deployFlower),
                followPath(5, path5(), PATH_5_WAIT_MS, PATH_5_TIMEOUT_MS),
                followPath(6, path6()),
                followPath(7, path7()),
                shoot(),
                followPath(8, path8()));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        try {
            if (Constants.foresightConfig == null) {
                telemetry.addLine("Run AutoTune > Foresight Tuner and export its config before running.");
                telemetry.update();
                waitForStart();
                return;
            }

            follower = Constants.create(hardwareMap);
            intake = new Intake(hardwareMap);
            shooter = new Shooter(hardwareMap);
            transit = new Transit(hardwareMap);
            vision = new Vision(hardwareMap, Alliance.RED);
            target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
            pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            follower.setPose(start);

            scheduler.addButton(() -> {
                if (activeShot != null || pathNumber == 8) return;
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
            if (!localizationReady()) {
                telemetry.addLine("Pinpoint is not READY; autonomous stopped.");
                telemetry.update();
                return;
            }

            follower.setPose(start);
            Command routine = autoRoutine();
            scheduler.schedule(routine);
            while (opModeIsActive() && scheduler.isScheduled(routine)) {
                follower.update();
                if (!localizationReady()) {
                    telemetry.addLine("Pinpoint lost READY status; autonomous stopped.");
                    telemetry.update();
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
            if (follower != null) stopDrive();
            scheduler.reset();
        }
    }

    private boolean localizationReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private Command followPath(int number, Path path) {
        return followPath(number, path, 0, 0);
    }

    private Command followPath(int number, Path path, long waitMs, long timeoutMs) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private double waitStartedMs;
            private boolean timedOut;

            @Override
            public void initialize() {
                pathNumber = number;
                waitStartedMs = -1;
                timedOut = false;
                timer.reset();
                follower.follow(path);
            }

            @Override
            public void execute() {
                if (waitMs == 0 && timeoutMs == 0) return;
                double elapsedMs = timer.milliseconds();
                timedOut = timeoutMs > 0 && elapsedMs >= timeoutMs;
                if (!timedOut && waitMs > 0 && waitStartedMs < 0 && !follower.isBusy()) {
                    waitStartedMs = elapsedMs;
                }
            }

            @Override
            public boolean isFinished() {
                return timedOut || (waitMs == 0 ? !follower.isBusy()
                        : waitStartedMs >= 0 && timer.milliseconds() - waitStartedMs >= waitMs);
            }

            @Override
            public void end(boolean interrupted) {
                if (timedOut) path5TimedOut = true;
                if (interrupted || timedOut) stopDrive();
            }
        };
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private Command shoot() {
        return new AutoShootCommand(follower, shooter, intake, transit, target,
                this::localizationReady, SHOT_DURATION_MS, SHOT_TIMEOUT_MS, false) {
            @Override
            public void initialize() {
                activeShot = this;
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                lastShotResult = getStatus();
                activeShot = null;
            }
        };
    }

    private void showTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Path", pathNumber);
        telemetry.addData("X / Y / heading", "%.2f / %.2f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Shot", activeShot == null ? lastShotResult : activeShot.getStatus());
        telemetry.addData("Shot distance (cm)", target.getDistanceCm());
        telemetry.addData("Shooter at speed", shooter.isReady());
        telemetry.addData("Flower", intake.getFlowerState());
        telemetry.addData("Path 5 timed out", path5TimedOut);
        telemetry.update();
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).heading(Interpolator.piecewise()
                .until(0.5997, Interpolator.linear(point2Segment1Start, point2Segment1End))
                .until(1, Interpolator.constant(point2Segment2Heading)));
    }

    public Path path3() {
        return curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return line(point3, point4).heading(Interpolator.piecewise()
                .until(0.6315, Interpolator.linear(point4Segment1Start, point4Segment1End))
                .until(1, Interpolator.constant(point4Segment2Heading)));
    }

    public Path path5() {
        return line(point4, point5).constant(point5);
    }

    public Path path6() {
        return line(point5, point6).constant(point6);
    }

    public Path path7() {
        return line(point6, point7).constant(point7);
    }

    public Path path8() {
        return line(point8Start, point8).linear(point8Start, point8);
    }
}
