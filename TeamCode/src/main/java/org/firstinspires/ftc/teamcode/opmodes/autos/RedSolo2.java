package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

@Config
@Autonomous(name = "RedSolo2", group = "Auto")
public class RedSolo2 extends LinearOpMode {
    // Fractions of the distance-calibrated target velocity; tune independently in Dashboard.
    public static double firstShotVelocityScale = 0.98;
    public static double secondShotVelocityScale = 0.93;
    public static double thirdShotVelocityScale = 0.93;
    private static final long SHOT_DURATION_MS = 900;
    private static final long SHOT_TIMEOUT_MS = 7000;
    private static final long PATH_4_WAIT_MS = 1000;
    // Extra path budget includes the arrival wait; shooting approaches use Pedro completion.
    private static final long PATH_TIMEOUT_MS = 2500;
    private static final long COLLECT_PATH_TIMEOUT_MS = 3000;

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Vision vision;
    private AutoHiveTarget target;
    private GoBildaPinpointDriver pinpoint;
    private AutoShootCommand activeShot;
    private int pathNumber;
    private int lastTimedOutPath;
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
    private final Pose point3Control1 = poseFactory.of(51.1409, 111.8053, 0);
    private final Pose point3Control2 = poseFactory.of(8.1528, 131.0802, 0);
    private final Pose point3Control3 = poseFactory.of(32.2808, 107.5054, 0);
    private final Pose point4 = poseFactory.of(46.1507, 134.6827, 90);
    private final Pose point4Control1 = poseFactory.of(57.5688, 117.2271, 0);
    private final Pose point4Control2 = poseFactory.of(61.1741, 119.1627, 0);
    private final Pose point4Control3 = poseFactory.of(50.5966, 111.4921, 0);
    private final Pose point4Control4 = poseFactory.of(44.7229, 110.8736, 0);
    private final Pose point5 = poseFactory.of(46.6966, 127.8097, 90);
    private final Pose point6 = poseFactory.of(58.0777, 121.299, 90);
    private final Pose point7Start = poseFactory.of(58.0777, 121.299, 90);
    private final Pose point7 = poseFactory.of(11.6138, 93.617, 0);

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                followPath(1, path1()),
                shoot(false),
                followPath(2, path2()).beforeStarting(() -> {
                    intake.updateInputs(true, false);
                    transit.updateIntakeRequest(true, false);
                }),
                followPath(3, path3()),
                shoot(true),
                new ParallelDeadlineGroup(
                        followPath(4, path4(), PATH_4_WAIT_MS),
                        collectUntilPathEnds()),
                followPath(5, path5()),
                followPath(6, path6()),
                shoot(false),
                followPath(7, path7()));
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
            shooter = new Shooter(hardwareMap, this::getShotVelocityScale);
            transit = new Transit(hardwareMap);
            vision = new Vision(hardwareMap, Alliance.RED);
            target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
            pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            follower.setPose(start);

            scheduler.addButton(() -> {
                if (activeShot != null || pathNumber == 7) return;
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
            }
            if (shooter != null) shooter.stop();
            if (vision != null) vision.stop();
            if (follower != null) stopDrive();
            scheduler.reset();
        }
    }

    /** Collect throughout path 4 and its arrival wait, then release intake before path 5. */
    private Command collectUntilPathEnds() {
        return new FunctionalCommand(
                () -> {
                    intake.updateInputs(true, false);
                    transit.updateIntakeRequest(true, false);
                },
                () -> {
                    intake.updateInputs(true, false);
                    transit.updateIntakeRequest(true, false);
                },
                interrupted -> {
                    intake.stop();
                    transit.cancelFeeding();
                },
                () -> false, intake, transit);
    }

    private boolean localizationReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private long pathTimeoutMs(int number) {
        switch (number) {
            case 1: case 3: case 6:
                return 0;
            case 4:
                return COLLECT_PATH_TIMEOUT_MS;
            default:
                return PATH_TIMEOUT_MS;
        }
    }

    private Command followPath(int number, Path path) {
        return followPath(number, path, 0);
    }

    private Command followPath(int number, Path path, long waitMs) {
        // 0 disables our extra deadline; Pedro still decides when the path ends.
        final long timeoutMs = pathTimeoutMs(number);
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
                if (timedOut) lastTimedOutPath = number;
                if (interrupted || timedOut) {
                    stopDrive();
                    intake.stop();
                    transit.cancelFeeding();
                }
            }
        };
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private double getShotVelocityScale() {
        if (pathNumber <= 1) return firstShotVelocityScale;
        if (pathNumber <= 3) return secondShotVelocityScale;
        return thirdShotVelocityScale;
    }

    private Command shoot(boolean deployFlower) {
        return new AutoShootCommand(follower, shooter, intake, transit, target,
                this::localizationReady, SHOT_DURATION_MS, SHOT_TIMEOUT_MS, false) {
            @Override
            public void initialize() {
                activeShot = this;
                super.initialize();
                if (deployFlower && intake.getFlowerState() == Intake.FlowerState.RAISED) {
                    intake.toggleFlower();
                }
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
        telemetry.addData("Shot CELL", target.getCluster());
        telemetry.addData("Shot distance (cm)", target.getDistanceCm());
        telemetry.addData("Shot velocity scale", getShotVelocityScale());
        if (activeShot != null) {
            telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f",
                    shooter.getTargetVelocity(), shooter.getRightVelocity());
            telemetry.addData("Shooter raw / applied power", "%.3f / %.3f",
                    shooter.getRightPidOutput(), shooter.getRightPower());
        }
        telemetry.addData("Shooter at speed", shooter.isReady());
        telemetry.addData("Flower", intake.getFlowerState());
        telemetry.addData("Last timed-out path (0 = none)", lastTimedOutPath);
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
        return curve(point3Start, point3Control1, point3Control2, point3Control3, point3)
                .linear(point3Start, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4Control3, point4Control4, point4)
                .constant(point4);
    }

    public Path path5() {
        return line(point4, point5).constant(point5);
    }

    public Path path6() {
        return line(point5, point6).constant(point6);
    }

    public Path path7() {
        return line(point7Start, point7).linear(point7Start, point7);
    }
}
