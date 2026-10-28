package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.lang.reflect.Field;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import sun.misc.Unsafe;

/** Exercises direct pose writes, metadata rejection and odometry/headless continuation. */
public class VisionCalibrationCheck {
    private static int checks;
    private static final Unsafe ALLOCATOR = allocator();

    private static Unsafe allocator() {
        try {
            Field field = Unsafe.class.getDeclaredField("theUnsafe");
            field.setAccessible(true);
            return (Unsafe) field.get(null);
        } catch (ReflectiveOperationException error) {
            throw new AssertionError(error);
        }
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void close(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-6,
                message + ": expected " + expected + ", got " + actual);
    }

    private static void set(Object target, Class<?> owner, String name, Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Pose2D pose(double xCm, double yCm, double heading) {
        return new Pose2D(DistanceUnit.CM, xCm, yCm, AngleUnit.DEGREES, heading);
    }

    public static class FakePinpoint extends com.qualcomm.hardware.gobilda.GoBildaPinpointDriver {
        Pose2D position;
        DeviceStatus status;
        int writes;
        double velX, velY, headingVelocity;
        FakePinpoint() { super(null, false); }
        @Override public DeviceStatus getDeviceStatus() { return status; }
        @Override public Pose2D getPosition() { return position; }
        @Override public double getHeading(AngleUnit unit) { return position.getHeading(unit); }
        @Override public void setPosition(Pose2D value) { position = value; writes++; }
        @Override public void update() { }
        @Override public double getVelX(DistanceUnit unit) { return unit.fromMm(DistanceUnit.INCH.toMm(velX)); }
        @Override public double getVelY(DistanceUnit unit) { return unit.fromMm(DistanceUnit.INCH.toMm(velY)); }
        @Override public double getHeadingVelocity(
                org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit unit) {
            return unit.fromDegrees(headingVelocity);
        }
    }

    private static final class Samples {
        final Vision vision = (Vision) ALLOCATOR.allocateInstance(Vision.class);
        final Drive drive = (Drive) ALLOCATOR.allocateInstance(Drive.class);
        final FakePinpoint pinpoint = (FakePinpoint) ALLOCATOR.allocateInstance(FakePinpoint.class);

        Samples() throws Exception {
            pinpoint.position = pose(0, 0, 0);
            pinpoint.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
            set(drive, Drive.class, "pinpoint", pinpoint);
            set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
            set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
            set(vision, Vision.class, "alliance", Alliance.BLUE);
            set(vision, Vision.class, "observedClusterStates", Collections.emptyMap());
        }

        void prepare(Pose2D pose, double timestamp) throws Exception {
            VisionMath.HiveSolveResult result = new VisionMath.HiveSolveResult();
            result.correction = new VisionMath.HiveCorrection(pose, -30, 0);
            result.candidates[0] = result.correction;
            result.usableCandidateCount = 1;
            result.status = "Synthetic Limelight field pose";
            set(vision, Vision.class, "hiveSolveResult", result);
            set(vision, Vision.class, "frameTimestamp", timestamp);
        }

        void frame(Pose2D pose, double timestamp, boolean allowed, long now) throws Exception {
            prepare(pose, timestamp);
            vision.updateFieldTarget(drive, allowed, now);
        }

        void missingFrame(long now) throws Exception {
            set(vision, Vision.class, "hiveSolveResult", null);
            set(vision, Vision.class, "correctedRobotFieldPose", null);
            set(vision, Vision.class, "poseStatus", "No AprilTag detections");
            set(vision, Vision.class, "frameTimestamp", Double.NaN);
            vision.updateFieldTarget(drive, false, now);
        }
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


    private static class Camera extends com.qualcomm.hardware.limelightvision.Limelight3A {
        com.qualcomm.hardware.limelightvision.LLResult result;
        Camera() { super(null, "test", null); }
        @Override public boolean isConnected() { return true; }
        @Override public com.qualcomm.hardware.limelightvision.LLResult getLatestResult() { return result; }
    }

    private static class Tag extends com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult {
        double area;
        Tag() throws Exception { super(null); }
        @Override public int getFiducialId() { return 38; }
        @Override public double getTargetArea() { return area; }
        @Override public org.firstinspires.ftc.robotcore.external.navigation.Pose3D getTargetPoseRobotSpace() {
            return null;
        }
    }

    private static class Frame extends com.qualcomm.hardware.limelightvision.LLResult {
        int count;
        Tag tag;
        Frame() throws Exception { super(null); }
        @Override public int getPipelineIndex() { return 0; }
        @Override public long getStaleness() { return 0; }
        @Override public boolean isValid() { return true; }
        @Override public double getTimestamp() { return 1; }
        @Override public double getBotposeAvgDist() { return 0; }
        @Override public double getBotposeSpan() { return 0; }
        @Override public double getBotposeAvgArea() { return 0; }
        @Override public double[] getStddevMt1() { return null; }
        @Override public java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult>
                getFiducialResults() { return Collections.singletonList(tag); }
        @Override public int getBotposeTagCount() { return count; }
        @Override public org.firstinspires.ftc.robotcore.external.navigation.Pose3D getBotpose() {
            // Real SDK fallback when the JSON botpose array is absent.
            return createPose3DRobot(new double[0]);
        }
    }

    private static void checkEmptyPoseMetadata() throws Exception {
        int minimum = VisionConstants.minimumBotposeTagCount;
        try {
            VisionConstants.minimumBotposeTagCount = 0;
            for (boolean missingLocalization : new boolean[]{true, false}) {
                Samples samples = new Samples();
                Camera camera = (Camera) ALLOCATOR.allocateInstance(Camera.class);
                Frame frame = (Frame) ALLOCATOR.allocateInstance(Frame.class);
                Tag tag = (Tag) ALLOCATOR.allocateInstance(Tag.class);
                frame.count = missingLocalization ? 0 : 1;
                tag.area = missingLocalization ? 1 : 0;
                frame.tag = tag;
                camera.result = frame;
                set(samples.vision, Vision.class, "limelight", camera);
                samples.vision.update();
                samples.vision.updateAimTarget(samples.drive, false);
                check(!samples.drive.isFieldPoseInitialized() && !samples.vision.hasValidAimingTarget(),
                        "zero minimum must not turn absent botpose metadata into a valid origin");
            }
        } finally {
            VisionConstants.minimumBotposeTagCount = minimum;
        }
    }

    public static void main(String[] args) throws Exception {
        checkEmptyPoseMetadata();
        Samples empty = new Samples();
        empty.drive.periodic();
        check(!empty.drive.isFieldPoseInitialized(), "READY odometry alone cannot establish field coordinates");
        check(Double.isNaN(empty.drive.getPose().getX(DistanceUnit.CM)),
                "uninitialized odometry is never reported as field origin");
        close(VisionConstants.calibrationIntervalSeconds, 1.5, "production waits 1.5 seconds after success");

        for (boolean stopped : new boolean[]{false, true}) {
            Samples sample = new Samples();
            for (int i = 0; i < 3; i++) {
                sample.prepare(pose(-100, 34, 180), i + 1);
                sample.vision.updateAimTarget(sample.drive, stopped);
            }
            boolean permitted = !stopped;
            check(sample.drive.isFieldPoseInitialized() == permitted, "B stop pauses the stable calibration path");
            check(sample.pinpoint.writes == (permitted ? 1 : 0), "three permitted frames write exactly once");
            check(sample.vision.hasValidAimingTarget() == permitted, "aiming waits for field initialization");
        }

        Samples direct = new Samples();
        check(direct.drive.updateVisionPose(pose(100, 40, 0), 1, 0), "Drive accepts a finite external pose");
        check(direct.drive.isFieldPoseInitialized(), "successful external write establishes the field frame");
        double positionGate = DriveConstants.maximumVisionPositionDifferenceInches;
        double headingGate = DriveConstants.maximumVisionHeadingDifferenceDegrees;
        try {
            DriveConstants.maximumVisionPositionDifferenceInches = 0;
            DriveConstants.maximumVisionHeadingDifferenceDegrees = 0;
            check(direct.drive.updateVisionPose(pose(-1000, 500, -90), 2, 1),
                    "Drive writes a large field-frame correction without a disagreement gate");
            close(direct.drive.getPose().getX(DistanceUnit.CM), -1000, "external X is applied");
            close(direct.drive.getPose().getHeading(AngleUnit.DEGREES), -90, "external heading is applied");
        } finally {
            DriveConstants.maximumVisionPositionDifferenceInches = positionGate;
            DriveConstants.maximumVisionHeadingDifferenceDegrees = headingGate;
        }
        check(!direct.drive.updateVisionPose(pose(0, 0, 0), 2, 2), "the same physical frame cannot write twice");
        check(!direct.drive.updateVisionPose(pose(Double.NaN, 0, 0), 3, 3),
                "nonfinite external poses cannot overwrite valid odometry");
        check(direct.pinpoint.writes == 2, "rejected external writes leave the hardware write count unchanged");
        direct.pinpoint.velX = 3;
        direct.pinpoint.velY = 4;
        direct.pinpoint.headingVelocity = -20;
        close(direct.drive.getTranslationSpeedInchesPerSecond(), 5, "translation speed comes from odometry");
        close(direct.drive.getHeadingVelocityDegreesPerSecond(), -20, "rotation telemetry comes from Pinpoint IMU");
        direct.pinpoint.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        direct.drive.periodic();
        check(!direct.drive.isFieldPoseInitialized(), "Pinpoint fault invalidates the field registration");
        check(!direct.drive.updateVisionPose(pose(0, 0, 0), 3, 4), "an unavailable Pinpoint rejects external writes");
        direct.pinpoint.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
        check(direct.drive.updateVisionPose(pose(0, 0, 180), 3, 5), "a ready Pinpoint can establish a new field frame");
        checkHeadlessAndTagLossShooting();
        System.out.println("PASS VisionCalibrationCheck: " + checks
                + " direct overwrite, metadata, field registration, headless and tag-loss assertions");
    }

    private static class ReadyShooter extends org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter {
        double distance;
        ReadyShooter() { super(null); }
        @Override public boolean setDistanceTarget(double cm) { distance = cm; return Double.isFinite(cm); }
        @Override public boolean isReady() { return true; }
        @Override public boolean isPitchReady() { return true; }
    }

    private static class QuietIntake extends org.firstinspires.ftc.teamcode.subsystems.intake.Intake {
        QuietIntake() { super(null); }
        @Override public void updateInputs(boolean requested, boolean jam, boolean shoot) { }
    }

    private static class ReadyTransit extends org.firstinspires.ftc.teamcode.subsystems.transit.Transit {
        boolean feedRequested;
        ReadyTransit() { super(null); }
        @Override public void requestFeed(boolean permitted) { feedRequested = permitted; }
    }

    private static class LockedAim extends org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController {
        @Override public double calculateTurnPower(boolean valid, double error) { return valid ? .123 : 0; }
        @Override public boolean isLocked() { return true; }
    }

    private static void checkHeadlessAndTagLossShooting() throws Exception {
        Samples sample = new Samples();
        CaptureDrive drive = (CaptureDrive) ALLOCATOR.allocateInstance(CaptureDrive.class);
        set(drive, Drive.class, "pinpoint", sample.pinpoint);
        set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
        set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
        drive.periodic();
        sample.pinpoint.position = pose(0, 0, 90);
        drive.moveFieldCentric(1, 0, 0);
        close(drive.forward, 0, "headless driving is available before the first visual field fix");
        close(drive.strafe, 1, "pre-initialization headless driving uses the IMU");
        check(!drive.isFieldPoseInitialized(), "IMU headless driving does not initialize field coordinates");
        drive.resetDriverHeading();
        close(drive.getDriverHeadingRadians(), 0, "START can reset forward before tags are visible");
        sample.pinpoint.position = pose(0, 0, 0);
        drive.resetDriverHeading();
        drive.updateVisionPose(pose(100, 40, 0), 1, 0);
        ReadyShooter shooter = (ReadyShooter) ALLOCATOR.allocateInstance(ReadyShooter.class);
        QuietIntake intake = (QuietIntake) ALLOCATOR.allocateInstance(QuietIntake.class);
        ReadyTransit transit = (ReadyTransit) ALLOCATOR.allocateInstance(ReadyTransit.class);
        com.qualcomm.robotcore.hardware.Gamepad gamepad =
                (com.qualcomm.robotcore.hardware.Gamepad) ALLOCATOR.allocateInstance(
                        com.qualcomm.robotcore.hardware.Gamepad.class);
        com.arcrobotics.ftclib.gamepad.GamepadEx inputs = new com.arcrobotics.ftclib.gamepad.GamepadEx(gamepad);
        org.firstinspires.ftc.teamcode.commands.ShootAssistCommand command =
                new org.firstinspires.ftc.teamcode.commands.ShootAssistCommand(
                        drive, shooter, intake, transit, sample.vision, new LockedAim(), inputs);
        sample.prepare(pose(100, 40, 0), 2);
        sample.vision.updateFieldTarget(drive, false, 0);
        java.lang.reflect.Method clear = Vision.class.getDeclaredMethod("clearCurrentFrame");
        clear.setAccessible(true);
        clear.invoke(sample.vision);
        for (double heading : new double[]{0, 90, 180, -90}) {
            sample.pinpoint.position = pose(100, 40, heading);
            sample.vision.updateFieldTarget(drive, false, 1);
            for (float[] stick : new float[][]{{-.5f, 0}, {.5f, 0}, {0, -.5f}, {0, .5f}}) {
                gamepad.left_stick_y = stick[0];
                gamepad.left_stick_x = stick[1];
                drive.moveFieldCentric(inputs.getLeftY(), inputs.getLeftX(), .123);
                double expectedForward = drive.forward;
                double expectedStrafe = drive.strafe;
                command.execute();
                close(drive.forward, expectedForward, "RT forward uses the normal headless frame");
                close(drive.strafe, expectedStrafe, "RT strafe uses the normal headless frame");
                close(drive.turn, .123, "headless translation preserves automatic turn output");
                check(transit.feedRequested && Double.isFinite(shooter.distance),
                        "real shoot-assist control remains ready through tag loss");
            }
        }
        sample.pinpoint.position = pose(100, 40, 90);
        close(drive.getDriverHeadingRadians(), Math.PI / 2, "headless follows real IMU rotation");
        drive.updateVisionPose(pose(100, 40, -90), 3, 2_000_000_000L);
        close(drive.getDriverHeadingRadians(), Math.PI / 2, "vision overwrite cannot rotate the driver reference");
        drive.resetDriverHeading();
        close(drive.getDriverHeadingRadians(), 0, "L3 resets the headless forward direction");
        drive.clearVisionPose("Pinpoint unavailable");
        drive.moveFieldCentric(.6, -.4, .1);
        close(drive.forward, .6, "manual drive remains available before visual field initialization");
        close(drive.strafe, -.4, "missing heading uses robot-relative manual fallback");
    }

}
