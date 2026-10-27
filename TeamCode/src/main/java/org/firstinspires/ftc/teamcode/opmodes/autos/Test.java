package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "test", group = "Auto")
public class Test extends LinearOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Vision vision;
    private AutoHiveTarget target;
    private String stage = "INIT";

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(58.7467, 8.8149, 90);
    private final Pose path1Start = poseFactory.of(58.7467, 8.8149, 270);
    private final Pose path1 = poseFactory.of(58.5392, 17.7643, 270);
    private final Pose point2 = poseFactory.of(14.0701, 47.376, 180);
    private final Pose point2Control1 = poseFactory.of(46.5237, 43.2977, 0);
    private final Pose point3 = poseFactory.of(20.7471, 47.2235, 178.6914);
    private final Pose point4Start = poseFactory.of(20.7471, 47.2235, 180);
    private final Pose point4 = poseFactory.of(57.7667, 23.3801, 270);
    private final Pose point5 = poseFactory.of(8.2064, 7.9837, 179);
    private final Pose point5Control1 = poseFactory.of(47.2051, 8.5841, 0);
    private final Pose point6Start = poseFactory.of(8.2064, 7.9837, 180);
    private final Pose point6 = poseFactory.of(58.3834, 17.221, 270);
    private final Pose point7 = poseFactory.of(8.8312, 94.3907, 270);
    private final Pose point7Control1 = poseFactory.of(18.8328, 61.2259, 0);

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                followPath(1, path1()),
                shootFor(900, false),
                intakeDuringPath(2, path2(), 500),
                followPath(3, path3()),
                followPath(4, path4()),
                shootFor(900, true),
                intakeDuringPath(5, path5(), 0),
                followPath(6, path6()),
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
            vision = new Vision(hardwareMap, Alliance.RED);
            target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
            GoBildaPinpointDriver pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            follower.setPose(start);
            follower.update();
            scheduler.addButton(() -> {
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
            if (intake != null) intake.stop();
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
        return new FunctionalCommand(
                () -> {
                    stage = "Path " + number;
                    follower.follow(path);
                },
                () -> { },
                interrupted -> {
                    if (interrupted) {
                        follower.stop();
                        follower.drivetrain.stop();
                    }
                },
                () -> !follower.isBusy());
    }

    private Command intakeDuringPath(int number, Path path, long waitMs) {
        Command collectPath = new ParallelDeadlineGroup(followPath(number, path), intakeRequest());
        if (waitMs <= 0) return collectPath;
        // FTCLib 2.1.1 resets a sequential group's completion in end(); keep sequences
        // outside ParallelDeadlineGroup so its deadline can still report completion.
        return new SequentialCommandGroup(collectPath,
                new InstantCommand(() -> stage = "Intake wait: " + waitMs + " ms"),
                new ParallelDeadlineGroup(new WaitCommand(waitMs), intakeRequest()));
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

    private Command shootFor(long milliseconds, boolean waitForHigh) {
        return new CommandBase() {
            private long feedStartedNanos;

            {
                addRequirements(intake, transit);
            }

            @Override
            public void initialize() {
                feedStartedNanos = -1;
                transit.cancelFeeding();
                intake.updateInputs(false, false);
            }

            @Override
            public void execute() {
                intake.updateInputs(false, false);
                if (!Double.isFinite(target.getDistanceCm()) || !shooter.hasTargetVelocity()) {
                    transit.cancelFeeding();
                    feedStartedNanos = -1;
                    stage = "Waiting for a valid shot distance";
                    return;
                }
                if (waitForHigh && feedStartedNanos < 0 && target.getState() != HiveState.HIGH) {
                    transit.cancelFeeding();
                    stage = "Waiting for HIGH: " + target.getState();
                    return;
                }

                transit.requestFeed(shooter.isReady() && shooter.isPitchReady());
                if (transit.getFiringState() == Transit.FiringState.FEEDING) {
                    // Readiness waiting and the transit reverse pulse do not consume shot time.
                    if (feedStartedNanos < 0) feedStartedNanos = System.nanoTime();
                    stage = "Shooting: " + milliseconds + " ms";
                } else {
                    stage = "Preparing shot: " + transit.getFiringState();
                }
            }

            @Override
            public boolean isFinished() {
                return feedStartedNanos >= 0
                        && (System.nanoTime() - feedStartedNanos) / 1_000_000L >= milliseconds;
            }

            @Override
            public void end(boolean interrupted) {
                if (transit.getState() == Transit.TransitState.OPEN) intake.clearBallCount();
                transit.cancelFeeding();
                intake.stop();
            }
        };
    }

    private void showTelemetry() {
        telemetry.addData("Stage", stage);
        telemetry.addData("X / Y / heading", "%.2f / %.2f / %.1f",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("CELL / HIVE", "%s / %s", target.getCluster(), target.getState());
        telemetry.addData("HIVE observation", target.getStatus());
        telemetry.addData("Shot distance (cm)", target.getDistanceCm());
        telemetry.addData("Shooter ready", shooter.isReady() && shooter.isPitchReady());
        telemetry.addData("Transit", transit.getFiringState());
        telemetry.addData("Balls", intake.getBallCount());
        telemetry.update();
    }

    public Path path1() {
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).linear(path1, point2);
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
        return line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return curve(point6, point7Control1, point7).linear(point6, point7);
    }
}
