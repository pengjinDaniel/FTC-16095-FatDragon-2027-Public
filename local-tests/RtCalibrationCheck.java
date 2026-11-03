package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import sun.misc.Unsafe;

/** Replays continuous idle calibration, paused RT calibration and odometry aiming. */
public class RtCalibrationCheck {
    private static final Unsafe U = allocator();
    private static int checks;
    private static final Pose2D FIXED = pose(100, 40, 0);

    private static Unsafe allocator() {
        try {
            Field field = Unsafe.class.getDeclaredField("theUnsafe");
            field.setAccessible(true);
            return (Unsafe) field.get(null);
        } catch (Exception error) { throw new AssertionError(error); }
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

    private static void near(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-6,
                message + ": expected " + expected + ", got " + actual);
    }

    private static Pose2D pose(double x, double y, double heading) {
        return new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, heading);
    }

    private static class CaptureDrive extends Drive {
        double forward, strafe, turn;
        CaptureDrive() { super(null); }
        @Override public void moveRobot(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }

    private static class Fixture {
        final Vision vision = (Vision) U.allocateInstance(Vision.class);
        final CaptureDrive drive = (CaptureDrive) U.allocateInstance(CaptureDrive.class);
        final VisionCalibrationCheck.FakePinpoint pp = (VisionCalibrationCheck.FakePinpoint)
                U.allocateInstance(VisionCalibrationCheck.FakePinpoint.class);
        final Method clear = Vision.class.getDeclaredMethod("clearCurrentFrame");

        Fixture() throws Exception {
            pp.position = pose(0, 0, 0);
            pp.status = DeviceStatus.READY;
            set(drive, Drive.class, "pinpoint", pp);
            set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
            set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
            set(vision, Vision.class, "alliance", Alliance.BLUE);
            clear.setAccessible(true);
        }

        void frame(Pose2D visual, double timestamp, boolean rt, boolean stopped, long now) throws Exception {
            drive.periodic();
            clear.invoke(vision);
            if (visual != null) {
                VisionMath.HiveSolveResult result = new VisionMath.HiveSolveResult();
                result.correction = new VisionMath.HiveCorrection(visual, -30, 0);
                result.status = "Synthetic current vision";
                set(vision, Vision.class, "hiveSolveResult", result);
                set(vision, Vision.class, "frameTimestamp", timestamp);
            }
            vision.updateAimTarget(drive, stopped, rt, now);
        }

        void rt(Pose2D visual, double timestamp, long now) throws Exception {
            frame(visual, timestamp, true, false, now);
        }

        void initialize() throws Exception { frame(FIXED, 1, false, false, 0); }
    }

    private static class ReadyShooter extends Shooter {
        ReadyShooter() { super(null); }
        @Override public void stop() { }
        @Override public boolean setDistanceTarget(double cm) { return Double.isFinite(cm); }
        @Override public boolean isReady() { return true; }
        @Override public boolean isPitchReady() { return true; }
    }

    private static class QuietIntake extends Intake {
        QuietIntake() { super(null); }
        @Override public void updateInputs(boolean requested, boolean jam, boolean shoot) { }
        @Override public void clearBallCount() { }
        @Override public void suspend() { }
    }

    private static void continuousIdleCalibration() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.pp.headingVelocity = 90;
        for (int i = 2; i <= 4; i++) {
            f.frame(pose(100 + i, 40, i), i, false, false, i * 20_000_000L);
            check(f.pp.writes == i, "each idle fresh frame calibrates without waiting for a stable batch");
            near(f.drive.getPose().getX(DistanceUnit.CM), 100 + i, "idle calibration applies current visual X");
        }
        f.frame(FIXED, 4, false, false, 100_000_000L);
        check(f.pp.writes == 4, "duplicate idle frames cannot write twice");
        f.frame(null, 0, false, false, 120_000_000L);
        check(f.pp.writes == 4, "missing vision leaves odometry unchanged");
        f.frame(FIXED, 5, false, false, 140_000_000L);
        check(f.pp.writes == 5, "idle calibration resumes on the next valid frame");
    }

    private static void rtUsesOnlyOdometry() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(pose(-200, -200, -90), 2, 20_000_000L);
        check(f.vision.hasValidAimingTarget() && f.vision.isAimCalibrationReady()
                        && !f.vision.shouldPauseForAimCalibration(),
                "RT aims immediately from initialized odometry without five-frame sampling");
        check(f.pp.writes == 1, "the RT rising edge cannot write a vision correction");
        double error = f.vision.getYawErrorDegrees();
        double distance = f.vision.getTagCenterHorizontalDistanceInches();
        for (int i = 0; i < 8; i++) {
            f.rt(pose(-200 + i, -200, 90), 3 + i, 40_000_000L + i * 20_000_000L);
            near(f.vision.getYawErrorDegrees(), error, "held RT ignores changing visual yaw and position");
            near(f.vision.getTagCenterHorizontalDistanceInches(), distance, "held RT measures distance from OD");
            check(f.vision.getSelectedCluster().get() == CellCluster.BLUE_AUDIENCE,
                    "the OD field half selects the CELL during RT");
        }
        check(f.pp.writes == 1, "held RT never calibrates, including after five new frames");
        f.pp.position = pose(-100, 40, 15);
        f.rt(FIXED, 11, 220_000_000L);
        check(f.vision.getSelectedCluster().get() == CellCluster.BLUE_OPPOSITE_AUDIENCE,
                "OD motion changes the live CELL selection");
        near(f.vision.getYawErrorDegrees(), VisionMath.coordinateYawErrorDegrees(
                f.pp.position, -17.7, 12.75, VisionConstants.shooterYawDegrees),
                "current OD pose controls yaw during RT");
        for (long now : new long[]{240_000_000L, 340_000_000L, 1_000_000_000L}) {
            f.rt(null, 0, now);
            check(f.vision.hasValidAimingTarget(), "tag loss does not pause valid OD aiming");
        }
        f.frame(pose(-100, 40, 15), 11, false, false, 1_020_000_000L);
        check(f.pp.writes == 2, "RT release can immediately calibrate a still-fresh unprocessed frame");
        Fixture cold = new Fixture();
        cold.rt(FIXED, 1, 0);
        check(!cold.vision.hasValidAimingTarget() && cold.pp.writes == 0,
                "RT cannot initialize an unknown field pose even with visible tags");
    }

    private static void rtFaultAndStop() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(FIXED, 2, 40_000_000L);
        f.frame(FIXED, 3, true, true, 60_000_000L);
        f.rt(FIXED, 4, 80_000_000L);
        check(!f.vision.isAimCalibrationReady() && !f.vision.hasValidAimingTarget(),
                "B stays latched until RT release");
        check(f.pp.writes == 1, "held RT and latched B never calibrate");
        f.frame(FIXED, 5, false, false, 100_000_000L);
        f.rt(FIXED, 6, 120_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 2,
                "RT release calibrates once and the next RT press pauses again");
        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(FIXED, 7, 140_000_000L);
        check(!f.vision.hasValidAimingTarget() && f.pp.writes == 2,
                "Pinpoint faults invalidate OD aiming without recalibrating during RT");
        f.pp.status = DeviceStatus.READY;
        f.rt(FIXED, 8, 160_000_000L);
        check(!f.vision.hasValidAimingTarget() && f.pp.writes == 2,
                "a recovered Pinpoint still waits for RT release before visual initialization");
        f.frame(FIXED, 9, false, false, 180_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 3,
                "release restores field initialization from the next valid frame");
    }

    private static void odometryLockAndTagLossFeed() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        double heading = Math.IEEEremainder(Math.toDegrees(Math.atan2(
                12.75 * 2.54 - 40, 17.7 * 2.54 - 100)) - 180, 360);
        Pose2D aligned = pose(100, 40, heading);
        f.frame(aligned, 2, false, false, 10_000_000L);
        AutoAimController aim = new AutoAimController();
        ReadyShooter shooter = (ReadyShooter) U.allocateInstance(ReadyShooter.class);
        QuietIntake intake = (QuietIntake) U.allocateInstance(QuietIntake.class);
        Transit transit = (Transit) U.allocateInstance(Transit.class);
        double[] motorPower = {0};
        DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) { motorPower[0] = (double) args[0]; return null; }
                    throw new AssertionError(method);
                });
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, args) -> null);
        set(transit, Transit.class, "transitMotor", motor);
        set(transit, Transit.class, "limitServo", servo);
        set(transit, Transit.class, "feedReverseTimer", new ElapsedTime());
        set(transit, Transit.class, "transitState", Transit.TransitState.CLOSE);
        TransitConstants.limitPositionsCalibrated = true;
        TransitConstants.feedReverseTime = 0.0;
        Gamepad gamepad = (Gamepad) U.allocateInstance(Gamepad.class);
        ShootAssistCommand command = new ShootAssistCommand(
                f.drive, shooter, intake, transit, f.vision, aim, new GamepadEx(gamepad));
        command.initialize();
        f.rt(null, 0, 20_000_000L);
        command.execute();
        check(!aim.isLocked() && transit.getState() == Transit.TransitState.CLOSE,
                "OD aiming still requires the existing stable lock interval");
        set(aim, AutoAimController.class, "withinToleranceSinceNanos", System.nanoTime() - 1_000_000_000L);
        command.execute();
        check(aim.isLocked() && command.canPrepareToFire() && command.canFire()
                        && transit.getState() == Transit.TransitState.OPEN,
                "stable OD aim can feed without a visible tag");
        f.rt(pose(-200, -200, 90), 3, 60_000_000L);
        command.execute();
        check(aim.isLocked() && command.canFire() && f.pp.writes == 2,
                "visible vision cannot alter a locked OD shot while RT is held");
        f.rt(null, 0, 80_000_000L);
        command.execute();
        check(aim.isLocked() && command.canFire(), "tag loss preserves valid OD aim and the existing feed sequence");
        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(aligned, 4, 100_000_000L);
        command.execute();
        near(motorPower[0], 0, "invalid OD explicitly stops the committed feeder");
        check(!aim.isLocked() && transit.getState() == Transit.TransitState.CLOSE,
                "invalid field position clears lock and closes the feed gate");
        command.end(true);
        near(f.drive.turn, 0, "ending RT stops turning");
    }

    private static class Camera extends Limelight3A {
        Frame frame;
        Camera() { super(null, "test", null); }
        @Override public boolean isConnected() { return true; }
        @Override public LLResult getLatestResult() { return frame; }
    }

    private static class Tag extends LLResultTypes.FiducialResult {
        int id;
        Tag() throws Exception { super(null); }
        @Override public int getFiducialId() { return id; }
        @Override public double getTargetArea() { return .01; }
        @Override public Pose3D getTargetPoseRobotSpace() { return null; }
    }

    private static class Frame extends LLResult {
        double timestamp;
        int pipeline;
        long age;
        Tag tag;
        Tag otherTag;
        Frame() throws Exception { super(null); }
        @Override public boolean isValid() { return true; }
        @Override public int getPipelineIndex() { return pipeline; }
        @Override public long getStaleness() { return age; }
        @Override public double getTimestamp() { return timestamp; }
        @Override public double getBotposeAvgDist() { return 1; }
        @Override public double getBotposeSpan() { return 0; }
        @Override public double getBotposeAvgArea() { return 1; }
        @Override public double[] getStddevMt1() { return null; }
        @Override public List<LLResultTypes.FiducialResult> getFiducialResults() {
            return otherTag == null ? Collections.singletonList(tag) : java.util.Arrays.asList(tag, otherTag);
        }
        @Override public int getBotposeTagCount() { return otherTag == null ? 1 : 2; }
        @Override public Pose3D getBotpose() {
            return new Pose3D(new Position(DistanceUnit.CM, 100, 40, 0, 0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        }
    }

    private static void frozenStaleAndWrongPipeline() throws Exception {
        Fixture f = new Fixture();
        Camera camera = (Camera) U.allocateInstance(Camera.class);
        Frame frame = (Frame) U.allocateInstance(Frame.class);
        frame.tag = (Tag) U.allocateInstance(Tag.class);
        frame.tag.id = 38;
        frame.timestamp = 1;
        camera.frame = frame;
        set(f.vision, Vision.class, "limelight", camera);
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 0);
        check(f.drive.isFieldPoseInitialized() && f.pp.writes == 1,
                "one real SDK-style idle frame initializes the field pose");
        set(f.vision, Vision.class, "lastPhysicalFrameSeenNanos", System.nanoTime() - 300_000_000L);
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 20_000_000L);
        check(!f.vision.getCorrectedRobotFieldPose().isPresent() && f.vision.getPoseStatus().contains("frozen"),
                "a frozen camera is invalid even with fresh HTTP reception");
        frame.timestamp = 2;
        frame.age = VisionConstants.maxResultAgeMs + 1;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 40_000_000L);
        check(f.pp.writes == 1, "stale reception cannot calibrate odometry");
        frame.age = 0;
        frame.pipeline = 1;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 60_000_000L);
        check(f.pp.writes == 1, "the wrong alliance pipeline cannot calibrate odometry");
        frame.pipeline = 0;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 80_000_000L);
        check(f.pp.writes == 2 && f.vision.getCorrectedRobotFieldPose().isPresent(),
                "a new valid idle frame resumes calibration after camera faults");
    }

    private static void mixedAllianceTagsStayUsable() throws Exception {
        for (Alliance alliance : Alliance.values()) {
            Fixture f = new Fixture();
            Camera camera = (Camera) U.allocateInstance(Camera.class);
            Frame frame = (Frame) U.allocateInstance(Frame.class);
            frame.pipeline = alliance == Alliance.BLUE ? 0 : 1;
            frame.tag = (Tag) U.allocateInstance(Tag.class);
            frame.tag.id = alliance == Alliance.BLUE ? 38 : 30;
            frame.otherTag = (Tag) U.allocateInstance(Tag.class);
            frame.otherTag.id = alliance == Alliance.BLUE ? 30 : 38;
            frame.timestamp = 1;
            camera.frame = frame;
            set(f.vision, Vision.class, "alliance", alliance);
            set(f.vision, Vision.class, "pipelineIndex", frame.pipeline);
            set(f.vision, Vision.class, "limelight", camera);
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, false, 0);
            check(f.drive.isFieldPoseInitialized() && f.pp.writes == 1,
                    "mixed-alliance tags may initialize odometry for " + alliance);
            check(f.vision.getUsableTagCount() == 2 && f.vision.getVisibleTagIds().size() == 2,
                    "both alliances contribute usable HIVE detections");
            frame.timestamp = 2;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, true, 20_000_000L);
            check(f.vision.hasValidAimingTarget() && f.pp.writes == 1,
                    "mixed-alliance tags do not change RT odometry aiming or calibration pause");
            check(f.vision.getSelectedCluster().get().isForAlliance(alliance),
                    "mixed detections still aim at the driver's own CELL");
            frame.tag.id = frame.otherTag.id;
            frame.otherTag = null;
            frame.timestamp = 3;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, false, 40_000_000L);
            check(f.pp.writes == 2 && f.vision.getUsableTagCount() == 1,
                    "an opposing HIVE tag alone can provide valid localization");
            check(f.vision.getSelectedCluster().get().isForAlliance(alliance),
                    "opposing-only localization cannot select the opposing CELL");
            frame.tag.id = 29;
            frame.timestamp = 4;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, false, 60_000_000L);
            check(f.pp.writes == 2 && !f.vision.getCorrectedRobotFieldPose().isPresent(),
                    "unknown tag IDs still cannot establish HIVE geometry");
        }
    }

    public static void main(String[] args) throws Exception {
        continuousIdleCalibration();
        rtUsesOnlyOdometry();
        rtFaultAndStop();
        odometryLockAndTagLossFeed();
        frozenStaleAndWrongPipeline();
        mixedAllianceTagsStayUsable();
        System.out.println("PASS RtCalibrationCheck: " + checks
                + " idle calibration, paused RT, OD aim, freshness, lock and real-feed assertions");
    }
}
