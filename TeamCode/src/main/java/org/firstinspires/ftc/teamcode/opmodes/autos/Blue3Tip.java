package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "blue3tip", group = "Auto")
public class Blue3Tip extends LinearOpMode {
    private static final double VISUALIZER_FIELD_CENTER_INCHES = 70.75;
    private static final double FLOWER_DEPLOY_PROGRESS = 0.2308;
    private static final long SHOT_DURATION_MS = 900;
    private static final long SHOT_TIMEOUT_MS = 4000;
    private static final long FLOWER_WAIT_MS = 1000;
    private static final long FLOWER_1_TIMEOUT_MS = 4500;
    private static final long FLOWER_2_TIMEOUT_MS = 3000;
    private static final long PINPOINT_BAD_READ_HOLD_NANOS = 100_000_000L;

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private GoBildaPinpointDriver pinpoint;
    private CachedPinpointLocalizer localizer;
    private String activePath = "INIT";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86.9861, 132.7126, 90);
    private final Pose startToShoot1 = poseFactory.of(83.2809, 119.8001, 90);
    private final Pose shoot1ToFlower1 = poseFactory.of(94.6415, 5.9784, -90);
    private final Pose shoot1ToFlower1Control1 = poseFactory.of(69.91, 46.5995, 0);
    private final Pose shoot1ToFlower1Control2 = poseFactory.of(94.8075, 27.3745, 0);
    private final Pose shoot1ToFlower1Segment1Start = poseFactory.of(94.6415, 5.9784, 90);
    private final Pose shoot1ToFlower1Segment1End = poseFactory.of(94.6415, 5.9784, 270);
    private final Pose shoot1ToFlower1Segment2Heading = poseFactory.of(94.6415, 5.9784, -90);
    private final Pose flower1ToShoot2Start = poseFactory.of(94.6415, 5.9784, 270);
    private final Pose flower1ToShoot2 = poseFactory.of(94.7688, 23.938, 298);
    private final Pose shoot2ToPreload = poseFactory.of(133.4227, 39.942, 90);
    private final Pose shoot2ToPreloadControl1 = poseFactory.of(136.844, 10.0469, 0);
    private final Pose shoot2ToPreloadSegment1Start = poseFactory.of(133.4227, 39.942, 298);
    private final Pose shoot2ToPreloadSegment1End = poseFactory.of(133.4227, 39.942, 90);
    private final Pose shoot2ToPreloadSegment2Heading = poseFactory.of(133.4227, 39.942, 90);
    private final Pose preloadToShoot3Start = poseFactory.of(133.4227, 39.942, 90);
    private final Pose preloadToShoot3 = poseFactory.of(119.0077, 20.9762, 320);
    private final Pose preloadToShoot3Control1 = poseFactory.of(122.6139, 30.4268, 0);
    private final Pose shoot3ToGarden = poseFactory.of(135.3047, 133.0247, 90);
    private final Pose shoot3ToGardenControl1 = poseFactory.of(91.1865, 44.5569, 0);
    private final Pose shoot3ToGardenControl2 = poseFactory.of(114.6315, 112.7725, 0);
    private final Pose shoot3ToGardenSegment1Start = poseFactory.of(135.3047, 133.0247, 320);
    private final Pose shoot3ToGardenSegment1End = poseFactory.of(135.3047, 133.0247, 90);
    private final Pose shoot3ToGardenSegment2Heading = poseFactory.of(135.3047, 133.0247, 90);
    private final Pose gardenToShoot4Start = poseFactory.of(135.3047, 133.0247, 90);
    private final Pose gardenToShoot4 = poseFactory.of(106.4229, 117.4638, 50);
    private final Pose shoot4ToFlower2 = poseFactory.of(131.0573, 94.5523, 0);
    private final Pose shoot4ToFlower2Control1 = poseFactory.of(97.0248, 92.0752, 0);
    private final Pose shoot4ToFlower2Segment1Start = poseFactory.of(131.0573, 94.5523, 50);
    private final Pose shoot4ToFlower2Segment1End = poseFactory.of(131.0573, 94.5523, 360);
    private final Pose shoot4ToFlower2Segment2Heading = poseFactory.of(131.0573, 94.5523, 0);
    private final Pose flower2ToShoot5Start = poseFactory.of(131.0573, 94.5523, 360);
    private final Pose flower2ToShoot5 = poseFactory.of(106.2286, 116.8084, 60);
    private final Pose flower2ToShoot5Control1 = poseFactory.of(108.4182, 93.0569, 0);
    private final Pose shoot5ToPark = poseFactory.of(132.9232, 43.2893, 180);
    private final Pose shoot5ToParkControl1 = poseFactory.of(129.7047, 87.2703, 0);
    private final Pose shoot5ToParkSegment1Start = poseFactory.of(132.9232, 43.2893, 60);
    private final Pose shoot5ToParkSegment1End = poseFactory.of(132.9232, 43.2893, 180);
    private final Pose shoot5ToParkSegment2Heading = poseFactory.of(132.9232, 43.2893, 180);

    private Command autoRoutine(AutoHiveTarget audience, AutoHiveTarget opposite) {
        return new SequentialCommandGroup(
                approachShot("startToShoot1", startToShoot1(), startToShoot1, opposite),
                shoot(opposite),
                // Keep collecting from Flower 1 through the approach to shot 2.
                followPath("shoot1ToFlower1", shoot1ToFlower1(), FLOWER_WAIT_MS, true)
                        .withTimeout(FLOWER_1_TIMEOUT_MS)
                        .beforeStarting(this::requestCollection),
                approachShot("flower1ToShoot2", flower1ToShoot2(), flower1ToShoot2, audience)
                        .beforeStarting(intake::raiseFlower),
                shoot(audience),
                // The collection request stays active through the next approach until the shot takes over.
                followPath("shoot2ToPreload", shoot2ToPreload()).beforeStarting(this::requestCollection),
                approachShot("preloadToShoot3", preloadToShoot3(), preloadToShoot3, audience),
                shoot(audience),
                followPath("shoot3ToGarden", shoot3ToGarden()).beforeStarting(this::requestCollection),
                approachShot("gardenToShoot4", gardenToShoot4(), gardenToShoot4, opposite)
                        .beforeStarting(this::deployFlower),
                shoot(opposite),
                collectDuring(followPath("shoot4ToFlower2", shoot4ToFlower2(), FLOWER_WAIT_MS, false))
                        .withTimeout(FLOWER_2_TIMEOUT_MS),
                approachShot("flower2ToShoot5", flower2ToShoot5(), flower2ToShoot5, opposite),
                shoot(opposite),
                followPath("shoot5ToPark", shoot5ToPark()).beforeStarting(intake::raiseFlower));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Vision vision = null;
        try {
            if (Constants.foresightConfig == null) {
                telemetry.addLine("Run AutoTune > Foresight Tuner and export its config before running.");
                telemetry.update();
                waitForStart();
                return;
            }

            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            localizer = new CachedPinpointLocalizer(new PinpointLocalizer(hardwareMap, Constants.localizerConfig), pinpoint);
            follower = new Follower(localizer, new Mecanum(hardwareMap, Constants.drivetrainConfig),
                    Constants.createAlgorithm());
            intake = new Intake(hardwareMap);
            shooter = new Shooter(hardwareMap);
            transit = new Transit(hardwareMap);
            vision = new Vision(hardwareMap, Alliance.BLUE, true);
            AutoHiveTarget audience = new AutoHiveTarget(
                    vision, CellCluster.BLUE_AUDIENCE, Blue3Tip::toCellFieldPose);
            AutoHiveTarget opposite = new AutoHiveTarget(
                    vision, CellCluster.BLUE_OPPOSITE_AUDIENCE, Blue3Tip::toCellFieldPose);
            follower.setPose(start);

            while (opModeInInit() && !isStopRequested()) {
                localizer.refresh();
                if (localizationReady()) follower.update();
                scheduler.run();
                showTelemetry();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            localizer.refresh();
            if (localizer.usingCachedSample()) localizer.refresh();
            if (!localizer.isFresh()) {
                reportPinpointStop();
                return;
            }

            follower.setPose(start);
            Command routine = autoRoutine(audience, opposite);
            scheduler.schedule(routine);
            while (opModeIsActive() && scheduler.isScheduled(routine)) {
                localizer.refresh();
                if (!localizationReady()) {
                    stopDrive();
                    reportPinpointStop();
                    break;
                }
                follower.update();
                scheduler.run();
                showTelemetry();
                idle();
            }
        } finally {
            scheduler.cancelAll();
            if (transit != null) transit.cancelFeeding();
            if (intake != null) {
                intake.stop();
                intake.raiseFlower();
            }
            if (shooter != null) shooter.stop();
            if (vision != null) vision.stop();
            if (follower != null) stopDrive();
            scheduler.reset();
        }
    }

    private static Pose2D toCellFieldPose(Pose pose) {
        // Visualizer -Y points toward the audience; the HIVE map uses +X.
        return new Pose2D(DistanceUnit.INCH,
                VISUALIZER_FIELD_CENTER_INCHES - pose.y(),
                pose.x() - VISUALIZER_FIELD_CENTER_INCHES,
                AngleUnit.RADIANS, pose.heading() + Math.PI / 2.0);
    }

    private boolean localizationReady() {
        return localizer.isFresh() || localizer.usingCachedSample();
    }

    private String pinpointStatus() {
        DeviceStatus status = pinpoint.getDeviceStatus();
        if (localizer.usingCachedSample()) return status + " (using last good sample)";
        if (status == DeviceStatus.READY && !localizer.isFresh()) return "READY (invalid pose or velocity)";
        return status.toString();
    }

    private void reportPinpointStop() {
        String status = pinpointStatus();
        RobotLog.ww("Blue3Tip", "Stopped on %s: Pinpoint %s", activePath, status);
        telemetry.addData("Autonomous stopped", "Pinpoint %s on %s", status, activePath);
        telemetry.update();
    }

    static final class CachedPinpointLocalizer implements Localizer {
        private final Localizer sensor;
        private final GoBildaPinpointDriver pinpoint;
        private MotionState state = MotionState.zero();
        private long lastGoodSampleNanos;
        private boolean fresh;
        private boolean usingCachedSample;

        CachedPinpointLocalizer(Localizer sensor, GoBildaPinpointDriver pinpoint) {
            this.sensor = sensor;
            this.pinpoint = pinpoint;
        }

        void refresh() {
            sensor.update();
            acceptSample(pinpoint.getDeviceStatus(), sensor.state(), System.nanoTime());
        }

        void acceptSample(DeviceStatus status, MotionState sample, long nowNanos) {
            usingCachedSample = status == DeviceStatus.FAULT_BAD_READ && fresh
                    && nowNanos - lastGoodSampleNanos <= PINPOINT_BAD_READ_HOLD_NANOS;
            fresh = status == DeviceStatus.READY && isFinite(sample);
            if (fresh) {
                state = sample;
                lastGoodSampleNanos = nowNanos;
            }
        }

        boolean isFresh() {
            return fresh;
        }

        boolean usingCachedSample() {
            return usingCachedSample;
        }

        private static boolean isFinite(MotionState sample) {
            if (sample == null) return false;
            Pose pose = sample.pose();
            Velocity velocity = sample.velocity();
            return pose != null && velocity != null
                    && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading())
                    && Double.isFinite(velocity.vx) && Double.isFinite(velocity.vy) && Double.isFinite(velocity.omega);
        }

        @Override
        public MotionState state() {
            return state;
        }

        @Override
        public void setPose(Pose pose) {
            sensor.setPose(pose);
            state = state.withPose(pose);
        }

        @Override
        public void update() {
            // Blue3Tip refreshes and validates the sample before calling Follower.update().
        }

        @Override
        public void reset() {
            fresh = false;
            usingCachedSample = false;
            state = MotionState.zero();
            sensor.reset();
        }
    }

    private void deployFlower() {
        if (intake.getFlowerState() == Intake.FlowerState.RAISED) intake.toggleFlower();
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private Command approachShot(String name, Path path, Pose endpoint, AutoHiveTarget target) {
        return followPath(name, path)
                .beforeStarting(() -> shooter.setDistanceTarget(target.distanceCmAt(endpoint)));
    }

    private Command followPath(String name, Path path) {
        return followPath(name, path, 0, false);
    }

    private Command followPath(String name, Path path, long waitMs, boolean deployAfterTurn) {
        return new CommandBase() {
            private ElapsedTime arrivalWait;

            @Override
            public void initialize() {
                arrivalWait = null;
                activePath = name;
                follower.follow(path);
            }

            @Override
            public void execute() {
                if (deployAfterTurn && follower.completion() >= FLOWER_DEPLOY_PROGRESS) deployFlower();
                if (waitMs > 0 && arrivalWait == null && !follower.isBusy()) {
                    arrivalWait = new ElapsedTime();
                }
            }

            @Override
            public boolean isFinished() {
                return !follower.isBusy()
                        && (waitMs == 0 || (arrivalWait != null && arrivalWait.milliseconds() >= waitMs));
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) stopDrive();
            }
        };
    }

    private Command collectDuring(Command deadline) {
        return new ParallelDeadlineGroup(deadline, new FunctionalCommand(
                this::requestCollection,
                this::requestCollection,
                interrupted -> {
                    intake.updateShotFeed(false);
                    transit.cancelFeeding();
                },
                () -> false, intake, transit));
    }

    private void requestCollection() {
        intake.updateInputs(true, false);
        transit.updateIntakeRequest(true, false);
    }

    private Command shoot(AutoHiveTarget target) {
        return new AutoShootCommand(follower, shooter, intake, transit, target,
                this::localizationReady, SHOT_DURATION_MS, SHOT_TIMEOUT_MS, false) {
            @Override
            public void execute() {
                super.execute();
                telemetry.addData("Shot", getStatus());
                telemetry.addData("Shot CELL", target.getCluster());
                telemetry.addData("Shot distance (cm)", "%.1f", target.getDistanceCm());
                telemetry.addData("Shot target (ticks/s)", "%.0f", shooter.getTargetVelocity());
                telemetry.addData("Shot pitch", "%.3f", shooter.getPitchPosition());
            }

            @Override
            public void end(boolean interrupted) {
                double distance = target.getDistanceCm();
                double velocity = shooter.getTargetVelocity();
                double pitch = shooter.getPitchPosition();
                super.end(interrupted);
                RobotLog.ii("Blue3Tip", "%s %s | %.1f cm, %.0f ticks/s, pitch %.3f | %s",
                        activePath, target.getCluster(), distance, velocity, pitch, getStatus());
            }
        };
    }

    private void showTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Path", activePath);
        telemetry.addData("Pinpoint", pinpointStatus());
        telemetry.addData("Pose (in, deg)", "%.1f, %.1f, %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Flower", intake.getFlowerState());
        telemetry.update();
    }

    public Path startToShoot1() {
        return line(start, startToShoot1).linear(start, startToShoot1);
    }

    public Path shoot1ToFlower1() {
        return curve(startToShoot1, shoot1ToFlower1Control1, shoot1ToFlower1Control2, shoot1ToFlower1)
                .heading(Interpolator.piecewise()
                        .until(FLOWER_DEPLOY_PROGRESS,
                                Interpolator.linear(shoot1ToFlower1Segment1Start, shoot1ToFlower1Segment1End))
                        .until(1, Interpolator.constant(shoot1ToFlower1Segment2Heading)));
    }

    public Path flower1ToShoot2() {
        return line(flower1ToShoot2Start, flower1ToShoot2).linear(flower1ToShoot2Start, flower1ToShoot2);
    }

    public Path shoot2ToPreload() {
        return curve(flower1ToShoot2, shoot2ToPreloadControl1, shoot2ToPreload)
                .heading(Interpolator.piecewise()
                        .until(0.4205,
                                Interpolator.linear(shoot2ToPreloadSegment1Start, shoot2ToPreloadSegment1End))
                        .until(1, Interpolator.constant(shoot2ToPreloadSegment2Heading)));
    }

    public Path preloadToShoot3() {
        return curve(preloadToShoot3Start, preloadToShoot3Control1, preloadToShoot3)
                .linear(preloadToShoot3Start, preloadToShoot3);
    }

    public Path shoot3ToGarden() {
        return curve(preloadToShoot3, shoot3ToGardenControl1, shoot3ToGardenControl2, shoot3ToGarden)
                .heading(Interpolator.piecewise()
                        .until(0.3339,
                                Interpolator.linear(shoot3ToGardenSegment1Start, shoot3ToGardenSegment1End))
                        .until(1, Interpolator.constant(shoot3ToGardenSegment2Heading)));
    }

    public Path gardenToShoot4() {
        return line(gardenToShoot4Start, gardenToShoot4).linear(gardenToShoot4Start, gardenToShoot4);
    }

    public Path shoot4ToFlower2() {
        return curve(gardenToShoot4, shoot4ToFlower2Control1, shoot4ToFlower2)
                .heading(Interpolator.piecewise()
                        .until(0.5,
                                Interpolator.linear(shoot4ToFlower2Segment1Start, shoot4ToFlower2Segment1End))
                        .until(1, Interpolator.constant(shoot4ToFlower2Segment2Heading)));
    }

    public Path flower2ToShoot5() {
        return curve(flower2ToShoot5Start, flower2ToShoot5Control1, flower2ToShoot5)
                .linear(flower2ToShoot5Start, flower2ToShoot5);
    }

    public Path shoot5ToPark() {
        return curve(flower2ToShoot5, shoot5ToParkControl1, shoot5ToPark)
                .heading(Interpolator.piecewise()
                        .until(0.3209,
                                Interpolator.linear(shoot5ToParkSegment1Start, shoot5ToParkSegment1End))
                        .until(1, Interpolator.constant(shoot5ToParkSegment2Heading)));
    }
}
