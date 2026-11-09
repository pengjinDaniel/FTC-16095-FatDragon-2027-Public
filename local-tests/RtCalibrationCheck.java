package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchReadHistoryImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
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
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import sun.misc.Unsafe;

/** Replays continuous idle calibration, RT controls, read diagnostics and odometry aiming. */
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
            pp.xOffsetMm = DriveConstants.pinpointXOffsetMm;
            pp.yOffsetMm = DriveConstants.pinpointYOffsetMm;
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
        double distanceCm;
        double measuredVelocity;
        int stops;

        ReadyShooter() { super(null); }
        @Override public void stop() { stops++; }
        @Override public boolean setDistanceTarget(double cm) {
            distanceCm = cm;
            return Double.isFinite(cm);
        }
        @Override public boolean isReady() { return true; }
        @Override public boolean isPitchReady() { return true; }
        @Override public double getTargetVelocity() { return 1600; }
        @Override public double getRightVelocity() { return measuredVelocity; }
    }

    private static class FeedTimer extends ElapsedTime {
        double time;
        @Override public double seconds() { return time; }
        @Override public void reset() { time = 0; }
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
        f.pp.position = pose(101.5, 40.5, 1.5);
        f.pp.headingVelocity = 90;
        for (int i = 2; i <= 4; i++) {
            f.frame(pose(100 + i, 40, i), i, false, false, i * 20_000_000L);
            check(f.pp.writes == i, "each fresh idle frame calibrates immediately after the previous success");
            near(f.drive.getPose().getX(DistanceUnit.CM), 100 + i, "continuous vision corrects odometry X");
            near(f.drive.getPose().getY(DistanceUnit.CM), 40, "continuous vision corrects odometry Y");
            near(f.drive.getPose().getHeading(AngleUnit.DEGREES), i, "continuous vision corrects odometry heading");
        }
        f.frame(FIXED, 4, false, false, 100_000_000L);
        check(f.pp.writes == 4, "the same physical frame cannot calibrate twice");
        near(f.drive.getPose().getX(DistanceUnit.CM), 104, "duplicate frames preserve the last corrected position");
        f.frame(null, 0, false, false, 120_000_000L);
        check(f.pp.writes == 4, "missing vision leaves odometry unchanged");
        f.frame(pose(104, 40, 4), 5, false, false, 140_000_000L);
        check(f.pp.writes == 5, "tag reacquisition calibrates immediately without a waiting interval");
        check(!f.vision.getCalibrationStatus().contains("cooldown")
                        && !f.vision.getRtCalibrationStatus().contains("cooldown"),
                "telemetry no longer reports a successful-calibration cooldown");
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 0,
                "the compatibility getter reports no cooldown after success");
        f.frame(pose(105, 40, 4), 6, false, false, 160_000_000L);
        check(f.pp.writes == 6, "the next fresh frame calibrates only 20 ms after success");
        f.frame(null, 0, false, false, 180_000_000L);
        check(f.pp.writes == 6, "missing vision cannot calibrate");
        f.frame(pose(106, 40, 4), 7, false, false, 200_000_000L);
        check(f.pp.writes == 7, "a valid frame immediately retries after a missing frame");
        f.frame(pose(900, 900, 90), 8, false, false, 220_000_000L);
        check(f.pp.writes == 7, "a rejected jump cannot calibrate");
        f.frame(pose(107, 40, 4), 9, false, false, 240_000_000L);
        check(f.pp.writes == 8, "the next acceptable frame retries immediately after rejection");

        Fixture waiting = new Fixture();
        waiting.frame(null, 0, false, false, 0);
        check(waiting.pp.writes == 0 && !waiting.drive.isFieldPoseInitialized(),
                "missing initial vision leaves calibration available");
        waiting.pp.status = DeviceStatus.CALIBRATING;
        waiting.frame(FIXED, 1, false, false, 20_000_000L);
        check(waiting.pp.writes == 0, "an unavailable Pinpoint cannot calibrate");
        near(waiting.vision.getCalibrationCooldownRemainingSeconds(), 0,
                "an unavailable Pinpoint does not start a cooldown");
        waiting.pp.status = DeviceStatus.READY;
        waiting.frame(FIXED, 1, false, false, 40_000_000L);
        check(waiting.pp.writes == 1 && waiting.drive.isFieldPoseInitialized(),
                "the first accepted frame calibrates without five-frame sampling");

        Fixture restarted = new Fixture();
        restarted.initialize();
        check(restarted.pp.writes == 1 && restarted.drive.isFieldPoseInitialized(),
                "a new OpMode instance initializes on its first acceptable frame");
    }

    private static void rtCalibratesTwiceThenUsesOdometry() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(FIXED, 1, 20_000_000L);
        check(f.pp.writes == 1 && !f.vision.isAimCalibrationReady()
                        && f.vision.shouldPauseForAimCalibration(),
                "an idle frame already used before pressing RT cannot count again");
        check(f.vision.getCalibrationStatus().contains("Waiting for a new Limelight frame"),
                "a duplicate frame reports why RT is still waiting");
        f.rt(pose(101, 40, 2), 2, 40_000_000L);
        check(f.pp.writes == 2 && f.vision.hasValidAimingTarget() && !f.vision.isAimCalibrationReady()
                        && f.vision.shouldPauseForAimCalibration(),
                "the first accepted RT frame writes once and leaves calibration pending");
        near(f.drive.getPose().getX(DistanceUnit.CM), 101, "the first RT correction writes X");
        check(f.vision.getCalibrationStatus().contains("1/2"), "telemetry reports the first success");
        f.rt(pose(101, 40, 2), 2, 50_000_000L);
        check(f.pp.writes == 2 && !f.vision.isAimCalibrationReady(),
                "a duplicate of the first success cannot complete calibration");
        f.rt(pose(102, 40, 3), 3, 60_000_000L);
        check(f.pp.writes == 3 && f.vision.isAimCalibrationReady() && !f.vision.shouldPauseForAimCalibration(),
                "the second accepted RT frame completes calibration");
        near(f.drive.getPose().getX(DistanceUnit.CM), 102, "the second RT correction writes X");
        near(f.drive.getPose().getHeading(AngleUnit.DEGREES), 3, "the second RT correction writes heading");
        check(f.vision.getCalibrationStatus().contains("2/2"), "telemetry reports both successes");
        double error = f.vision.getYawErrorDegrees();
        double distance = f.vision.getTagCenterHorizontalDistanceInches();
        for (int i = 0; i < 8; i++) {
            f.rt(pose(-200 + i, -200, 90), 4 + i, 80_000_000L + i * 20_000_000L);
            near(f.vision.getYawErrorDegrees(), error, "held RT ignores later visual yaw and position");
            near(f.vision.getTagCenterHorizontalDistanceInches(), distance, "held RT measures distance from OD");
            check(f.vision.getSelectedCluster().get() == CellCluster.BLUE_AUDIENCE,
                    "the OD field half selects the CELL during RT");
        }
        check(f.pp.writes == 3, "held RT permits exactly two successful vision corrections");
        f.pp.position = pose(-100, 40, 15);
        f.rt(FIXED, 12, 240_000_000L);
        check(f.vision.getSelectedCluster().get() == CellCluster.BLUE_OPPOSITE_AUDIENCE,
                "OD motion changes the live CELL selection");
        near(f.vision.getYawErrorDegrees(), VisionMath.coordinateYawErrorDegrees(
                f.pp.position, -17.7, 12.75, VisionConstants.shooterYawDegrees),
                "current OD pose controls yaw during RT");
        for (long now : new long[]{260_000_000L, 340_000_000L, 1_000_000_000L}) {
            f.rt(null, 0, now);
            check(f.vision.hasValidAimingTarget() && f.vision.isAimCalibrationReady(),
                    "tag loss preserves calibrated OD aiming");
        }
        f.frame(pose(-100, 40, 15), 13, false, false, 1_020_000_000L);
        check(f.pp.writes == 4, "RT release immediately resumes continuous idle calibration");
        f.rt(pose(-99, 40, 16), 14, 1_040_000_000L);
        check(f.pp.writes == 5 && !f.vision.isAimCalibrationReady(), "a new RT session starts counting again");
        f.rt(pose(-98, 40, 16), 15, 1_060_000_000L);
        check(f.pp.writes == 6 && f.vision.isAimCalibrationReady(), "the new session completes on its second success");
        f.rt(pose(-97, 40, 16), 16, 4_000_000_000L);
        check(f.pp.writes == 6, "later acceptable frames stay paused while RT is held");
        f.frame(pose(-97, 40, 16), 16, false, false, 4_020_000_000L);
        check(f.pp.writes == 7, "release makes the next eligible frame available to idle calibration");

        Fixture cold = new Fixture();
        cold.rt(FIXED, 1, 0);
        check(cold.vision.hasValidAimingTarget() && !cold.vision.isAimCalibrationReady() && cold.pp.writes == 1,
                "the first acceptable RT frame initializes an unknown field pose and leaves one correction pending");
        cold.rt(FIXED, 2, 20_000_000L);
        check(cold.vision.isAimCalibrationReady() && cold.pp.writes == 2,
                "the second acceptable RT frame completes calibration after initialization");
    }

    private static void rtUsesIdleJumpGate() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(null, 0, 20_000_000L);
        check(f.vision.hasValidAimingTarget() && !f.vision.isAimCalibrationReady()
                        && !f.vision.shouldPauseForAimCalibration() && f.pp.writes == 1,
                "RT can aim from OD while waiting for a usable visual frame");
        Pose2D jumped = pose(200, 40, 30);
        f.rt(jumped, 2, 40_000_000L);
        f.rt(jumped, 2, 60_000_000L);
        f.rt(jumped, 3, 80_000_000L);
        check(f.pp.writes == 1 && !f.vision.isAimCalibrationReady()
                        && f.vision.shouldPauseForAimCalibration(),
                "a duplicate cannot advance the idle jump-rejection count during RT");
        f.rt(jumped, 4, 100_000_000L);
        check(f.pp.writes == 2 && !f.vision.isAimCalibrationReady(),
                "the idle jump override counts as one successful correction");
        near(f.drive.getPose().getX(DistanceUnit.CM), 200, "RT jump override writes the accepted pose");
        f.rt(jumped, 5, 120_000_000L);
        check(f.pp.writes == 3 && f.vision.isAimCalibrationReady(),
                "the next accepted frame completes the two corrections");
    }

    private static void failedVisionWritebackRetriesImmediately() throws Exception {
        for (boolean rt : new boolean[]{false, true}) {
            Fixture f = new Fixture();
            f.pp.failAfterPoseWrite = true;
            f.frame(FIXED, 1, rt, false, 0);
            check(!f.drive.isFieldPoseInitialized() && f.drive.getPinpointBadReadCount() == 1,
                    "a failed post-write read rejects and records the attempted calibration");
            check(f.drive.getLastPinpointReadFailure().contains("after vision write"),
                    "diagnostics identify the post-calibration read phase");
            if (rt) check(!f.vision.isAimCalibrationReady(), "failed writeback leaves RT calibration pending");
            near(f.vision.getCalibrationCooldownRemainingSeconds(), 0,
                    "a failed post-write read does not start a cooldown");
            f.pp.failAfterPoseWrite = false;
            f.pp.status = DeviceStatus.READY;
            f.frame(FIXED, 2, rt, false, 20_000_000L);
            check(f.vision.hasValidAimingTarget() && f.pp.writes == 2,
                    "the next verified frame retries a failed calibration immediately");
            if (rt) check(!f.vision.isAimCalibrationReady(), "a failed write does not count toward two successes");
            near(f.vision.getCalibrationCooldownRemainingSeconds(), 0,
                    "a successful retry also leaves no cooldown");
            f.frame(FIXED, 3, rt, false, 40_000_000L);
            check(f.pp.writes == 3 && f.vision.isAimCalibrationReady(),
                    "two verified writes complete RT calibration after the failed attempt");
            f.frame(FIXED, 4, rt, false, 60_000_000L);
            check(f.pp.writes == (rt ? 3 : 4), "RT stops after two successes while idle stays continuous");
        }
    }

    private static void rtFaultAndStop() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.frame(FIXED, 2, true, true, 40_000_000L);
        f.rt(FIXED, 3, 60_000_000L);
        check(!f.vision.isAimCalibrationReady() && !f.vision.hasValidAimingTarget(),
                "B stays latched until RT release");
        check(f.pp.writes == 1, "B prevents the pending RT correction");
        f.frame(FIXED, 4, false, false, 80_000_000L);
        check(f.pp.writes == 2, "RT release resumes calibration after B stop");
        f.rt(FIXED, 5, 100_000_000L);
        f.rt(FIXED, 6, 120_000_000L);
        check(f.vision.hasValidAimingTarget() && f.vision.isAimCalibrationReady() && f.pp.writes == 4,
                "the next RT session completes two calibrations after B stop");
        f.pp.position = pose(110, 42, 10);
        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(FIXED, 7, 140_000_000L);
        check(!f.vision.hasValidAimingTarget() && f.pp.writes == 4,
                "bad reads suspend OD aiming after the RT correction");
        check(!f.drive.isFieldPoseInitialized()
                        && Double.isNaN(f.drive.getPose().getX(DistanceUnit.CM)),
                "bad reads cannot expose a cached field pose for control");
        check(f.vision.getCalibrationStatus().contains("RT calibration complete"),
                "a temporary bad read does not rearm RT calibration");
        f.pp.status = DeviceStatus.READY;
        f.rt(FIXED, 8, 160_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 4,
                "a valid recovered Pinpoint resumes OD aiming without a vision rewrite");
        near(f.drive.getPose().getX(DistanceUnit.CM), 110, "recovery preserves the current OD position");
        near(f.drive.getDriverHeadingRadians(), Math.toRadians(10),
                "a bad read cannot reset the headless driving reference");
        check(f.pp.offsetReads == 2, "recovery verifies both device offsets once");
        f.frame(pose(110, 42, 10), 9, false, false, 180_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 5 && f.pp.offsetReads == 2,
                "RT release recalibrates immediately without rereading verified device offsets");
        f.pp.status = DeviceStatus.CALIBRATING;
        f.rt(FIXED, 10, 200_000_000L);
        check(!f.vision.isAimCalibrationReady() && f.pp.writes == 5,
                "an unavailable Pinpoint cannot complete a new RT correction");
        f.pp.status = DeviceStatus.READY;
        f.frame(FIXED, 11, false, false, 220_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 6,
                "idle vision immediately restores field registration after verified odometry recovery");
        f.rt(FIXED, 12, 240_000_000L);
        f.rt(FIXED, 13, 260_000_000L);
        check(f.pp.writes == 8 && f.vision.isAimCalibrationReady(), "RT can calibrate twice after recovery");
        f.frame(FIXED, 14, false, false, 280_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 9,
                "RT release resumes calibration immediately after recovery");
    }

    private static void recoveryChecksDeviceOffsets() throws Exception {
        Fixture reset = new Fixture();
        reset.initialize();
        reset.pp.status = DeviceStatus.FAULT_BAD_READ;
        reset.rt(null, 0, 20_000_000L);
        reset.pp.xOffsetMm = 0;
        reset.pp.yOffsetMm = 0;
        reset.pp.position = pose(0, 0, 0);
        reset.pp.status = DeviceStatus.READY;
        reset.rt(FIXED, 2, 40_000_000L);
        check(!reset.drive.isFieldPoseInitialized() && !reset.vision.hasValidAimingTarget()
                        && reset.pp.writes == 1,
                "READY with reset device offsets cannot reuse the old field frame");
        check(reset.drive.getLocalizationStatus().contains("offsets changed"),
                "reset device settings identify lost field registration");
        reset.frame(FIXED, 3, false, false, 2_000_000_000L);
        check(!reset.drive.isFieldPoseInitialized() && reset.pp.writes == 1,
                "idle vision cannot initialize field coordinates with lost device settings");

        Fixture retry = new Fixture();
        retry.initialize();
        retry.pp.status = DeviceStatus.FAULT_BAD_READ;
        retry.rt(null, 0, 20_000_000L);
        retry.pp.status = DeviceStatus.READY;
        retry.pp.failOffsetRead = true;
        retry.rt(FIXED, 2, 40_000_000L);
        check(!retry.vision.hasValidAimingTarget() && retry.pp.writes == 1,
                "a failed recovery offset read keeps aiming suspended");
        retry.pp.status = DeviceStatus.READY;
        retry.pp.failOffsetRead = false;
        retry.rt(null, 0, 60_000_000L);
        check(retry.vision.hasValidAimingTarget() && retry.pp.writes == 1,
                "the next valid configuration read can recover without vision calibration");
    }

    private static TimestampedI2cData bulkPacket(int loopTime, float[] values) {
        TimestampedI2cData packet = new TimestampedI2cData();
        packet.register = 18;
        packet.data = new byte[40];
        ByteBuffer bytes = ByteBuffer.wrap(packet.data).order(ByteOrder.LITTLE_ENDIAN);
        bytes.putInt(0, 1);
        bytes.putInt(4, loopTime);
        for (int i = 0; i < values.length; i++) bytes.putFloat(16 + 4 * i, values[i]);
        return packet;
    }

    private static void pinpointBadReadDiagnostics() throws Exception {
        float[][] values = {
                {1000, 400, 0, Float.NaN, 0, 0},
                {1000, 400, 0, 0, 0, 0},
                {7001, 400, 0, 0, 0, 0},
                {1000, 400, 121, 0, 0, 0},
                {1000, 400, 0, 11000, 0, 0}
        };
        String[] causes = {"NaN in VX", "loopTime=0", "X mm jump", "heading rad jump", "VX mm/s magnitude"};
        for (int i = 0; i < values.length; i++) {
            Fixture f = new Fixture();
            f.initialize();
            I2cDeviceSynchReadHistoryImpl history = new I2cDeviceSynchReadHistoryImpl();
            history.setHistoryQueueCapacity(2);
            set(f.drive, Drive.class, "pinpointReadHistory", history);
            f.pp.readHistory = history;
            f.pp.rawReply = bulkPacket(i == 1 ? 0 : 1000, values[i]);
            f.pp.status = DeviceStatus.FAULT_BAD_READ;
            f.rt(null, 0, 20_000_000L);
            check(f.drive.getPinpointBadReadCount() == 1 && f.drive.getPinpointReadCount() == 3,
                    "read counters include normal polling and the initial vision writeback read");
            check(f.drive.getLastPinpointReadFailure().contains(causes[i]),
                    "diagnostics retain the raw rejected field: " + causes[i]);
            check(!f.vision.hasValidAimingTarget(), "raw diagnostics never enable a rejected pose");
            f.rt(null, 0, 40_000_000L);
            check(f.drive.getPinpointBadReadCount() == 2
                            && f.drive.getPinpointReadDiagnostics().contains("streak 2"),
                    "consecutive rejected cycles update the streak");
            f.pp.rawReply = bulkPacket(1000, new float[]{1000, 400, 0, 0, 0, 0});
            f.pp.status = DeviceStatus.READY;
            f.rt(null, 0, 60_000_000L);
            check(f.vision.hasValidAimingTarget() && f.pp.writes == 1,
                    "capturing raw diagnostics does not prevent recovery or recalibrate");
            check(f.drive.getLastPinpointReadFailure().contains(causes[i])
                            && f.drive.getPinpointReadDiagnostics().contains("streak 0"),
                    "the last failure remains visible after a successful recovery");
        }

        Fixture crc = new Fixture();
        crc.initialize();
        I2cDeviceSynchReadHistoryImpl history = new I2cDeviceSynchReadHistoryImpl();
        history.setHistoryQueueCapacity(2);
        set(crc.drive, Drive.class, "pinpointReadHistory", history);
        crc.pp.readHistory = history;
        TimestampedI2cData packet = bulkPacket(1000, new float[]{1000, 400, 0, 0, 0, 0});
        Method checksum = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class
                .getDeclaredMethod("computeCRC8", byte[].class);
        checksum.setAccessible(true);
        byte expected = (Byte) checksum.invoke(crc.pp, (Object) packet.data);
        packet.data = java.util.Arrays.copyOf(packet.data, 41);
        packet.data[40] = (byte) (expected ^ 1);
        crc.pp.rawReply = packet;
        crc.pp.status = DeviceStatus.FAULT_BAD_READ;
        crc.rt(null, 0, 20_000_000L);
        check(crc.drive.getLastPinpointReadFailure().contains("Bulk CRC mismatch"),
                "raw CRC diagnostics agree with the actual SDK checksum implementation");
        crc.pp.rawReply = null;
        crc.rt(null, 0, 40_000_000L);
        check(crc.drive.getLastPinpointReadFailure().contains("No fresh I2C reply"),
                "an absent capture is reported without inventing a corrupt field");
    }

    private static void odometryTrackingFeed() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        double heading = Math.IEEEremainder(Math.toDegrees(Math.atan2(
                12.75 * 2.54 - 40, 17.7 * 2.54 - 100)) - 180, 360);
        Pose2D aligned = pose(100, 40, heading);
        f.pp.position = aligned;
        f.frame(aligned, 2, false, false, 10_000_000L);
        AutoAimController aim = new AutoAimController();
        ReadyShooter shooter = (ReadyShooter) U.allocateInstance(ReadyShooter.class);
        shooter.measuredVelocity = shooter.getTargetVelocity();
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
        FeedTimer timer = new FeedTimer();
        set(transit, Transit.class, "feedReverseTimer", timer);
        set(transit, Transit.class, "transitState", Transit.TransitState.CLOSE);
        TransitConstants.limitPositionsCalibrated = true;
        TransitConstants.feedReverseTime = 0.1;
        Gamepad gamepad = (Gamepad) U.allocateInstance(Gamepad.class);
        ShootAssistCommand command = new ShootAssistCommand(
                f.drive, shooter, intake, transit, f.vision, aim, new GamepadEx(gamepad));
        command.initialize();
        f.pp.position = pose(100, 40, heading + 10);
        f.rt(aligned, 2, 20_000_000L);
        command.execute();
        near(f.drive.turn, 0, "automatic turn waits for an eligible frame when usable vision is present");
        near(motorPower[0], TransitConstants.feedReversePower, "RT prepares the feeder before alignment");
        check(!command.canFire() && transit.getState() == Transit.TransitState.CLOSE,
                "the gate stays closed during reverse preparation");
        check(shooter.stops == 0, "pressing RT preserves shooter pre-spin");
        f.rt(null, 0, 25_000_000L);
        command.execute();
        check(Math.abs(f.drive.turn) > 0 && !command.canFire(),
                "OD can turn toward the target while the camera has no usable frame");

        f.pp.position = aligned;
        f.rt(null, 0, 30_000_000L);
        command.execute();
        check(!command.canPrepareToFire() && transit.getState() == Transit.TransitState.CLOSE,
                "alignment cannot release automatic feed before the RT correction");
        f.rt(aligned, 3, 40_000_000L);
        command.execute();
        check(!f.vision.isAimCalibrationReady() && !command.canPrepareToFire() && !command.canFire(),
                "one successful correction leaves automatic feed waiting");
        f.rt(aligned, 4, 60_000_000L);
        command.execute();
        check(f.vision.isAimCalibrationReady() && !command.canFire(),
                "the feeder remains closed while reverse preparation is still running");
        f.rt(null, 0, 70_000_000L);
        timer.time = 0.05;
        command.execute();
        check(command.canPrepareToFire() && !command.canFire(),
                "a clear firing window still waits for the reverse pulse to finish");
        double initialDistance = shooter.distanceCm;
        f.pp.position = pose(130, 40, heading + 10);
        f.rt(null, 0, 80_000_000L);
        timer.time = 0.11;
        command.execute();
        near(motorPower[0], 0, "a bump before reverse ends holds the feeder");
        check(transit.getState() == Transit.TransitState.CLOSE, "the gate remains closed after the bump");
        check(Math.abs(f.drive.turn) > 0 && shooter.distanceCm > initialDistance,
                "the displaced pose updates steering and shot distance");
        near(Math.hypot(f.drive.forward, f.drive.strafe), 0, "neutral sticks do not pull back to the old pose");

        double newHeading = f.vision.getDesiredHeadingDegrees();
        f.pp.position = pose(130, 40, newHeading);
        gamepad.left_stick_y = -0.5f;
        f.rt(null, 0, 90_000_000L);
        command.execute();
        f.rt(null, 0, 95_000_000L);
        command.execute();
        check(command.canPrepareToFire() && command.canFire()
                        && transit.getState() == Transit.TransitState.OPEN,
                "moving with a clear aim window feeds immediately after preparation");
        near(Math.hypot(f.drive.forward, f.drive.strafe), 0.5, "RT preserves manual translation");
        near(motorPower[0], TransitConstants.feedPower, "the feeder drives forward on the first usable window");
        f.rt(pose(-200, -200, 90), 5, 100_000_000L);
        command.execute();
        check(command.canFire() && f.pp.writes == 4,
                "later visual frames cannot alter a calibrated OD shot while RT is held");

        f.pp.position = pose(130, 40, newHeading + 10);
        f.rt(null, 0, 110_000_000L);
        command.execute();
        check(!command.canFire() && transit.getState() == Transit.TransitState.CLOSE,
                "losing alignment while feeding closes the gate");
        near(motorPower[0], 0, "losing alignment while feeding stops transit");
        f.pp.position = pose(130, 40, newHeading);
        f.rt(null, 0, 120_000_000L);
        command.execute();
        f.rt(null, 0, 130_000_000L);
        command.execute();
        near(motorPower[0], TransitConstants.feedPower, "reacquisition resumes feed without another reverse pulse");

        shooter.measuredVelocity = 1300;
        command.execute();
        near(motorPower[0], 0, "current wheel speed can pause feeding despite a prior ready flag");
        shooter.measuredVelocity = shooter.getTargetVelocity();
        command.execute();
        near(motorPower[0], TransitConstants.feedPower, "feed resumes at the current target speed");

        f.pp.position = pose(130, 40, newHeading + DriveConstants.aimLockToleranceDegrees * 0.99);
        f.rt(null, 0, 135_000_000L);
        command.execute();
        check(Math.abs(f.vision.getYawErrorDegrees()) <= DriveConstants.aimLockToleranceDegrees
                        && !command.canFire(),
                "a moving heading can be aligned now but outside the window at ball release");
        f.pp.position = pose(130, 40, newHeading);
        f.rt(null, 0, 137_000_000L);
        command.execute();
        f.rt(null, 0, 139_000_000L);
        command.execute();
        check(command.canFire(), "a settled heading resumes feed after a sweep");

        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(aligned, 6, 140_000_000L);
        command.execute();
        near(motorPower[0], 0, "invalid OD stops the feeder");
        check(!command.canPrepareToFire() && transit.getState() == Transit.TransitState.CLOSE,
                "invalid field position clears the firing window and closes the feed gate");
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
        f.vision.updateAimTarget(f.drive, false, true, 20_000_000L);
        check(!f.vision.getCorrectedRobotFieldPose().isPresent() && f.vision.getPoseStatus().contains("frozen"),
                "a frozen camera is invalid even with fresh HTTP reception");
        frame.timestamp = 2;
        frame.age = VisionConstants.maxResultAgeMs + 1;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, true, 40_000_000L);
        check(f.pp.writes == 1 && !f.vision.isAimCalibrationReady(), "stale reception cannot complete RT calibration");
        frame.age = 0;
        frame.pipeline = 1;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, true, 60_000_000L);
        check(f.pp.writes == 1 && !f.vision.isAimCalibrationReady(), "wrong pipeline cannot complete RT calibration");
        frame.pipeline = 0;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, true, 80_000_000L);
        check(f.pp.writes == 2 && !f.vision.isAimCalibrationReady(),
                "the first fresh frame on the correct pipeline counts once");
        frame.timestamp = 3;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, true, 100_000_000L);
        check(f.pp.writes == 3 && f.vision.isAimCalibrationReady(),
                "two fresh frames on the correct pipeline complete the pending RT calibration");
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
            check(f.vision.hasValidAimingTarget() && !f.vision.isAimCalibrationReady() && f.pp.writes == 2,
                    "mixed-alliance tags permit the first RT correction");
            frame.timestamp = 3;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, true, 40_000_000L);
            check(f.vision.isAimCalibrationReady() && f.pp.writes == 3,
                    "another fresh mixed-alliance frame completes RT calibration");
            check(f.vision.getSelectedCluster().get().isForAlliance(alliance),
                    "mixed detections still aim at the driver's own CELL");
            frame.tag.id = frame.otherTag.id;
            frame.otherTag = null;
            frame.timestamp = 4;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, false, 60_000_000L);
            check(f.pp.writes == 4 && f.vision.getUsableTagCount() == 1,
                    "an opposing HIVE tag remains usable for continuous idle calibration");
            check(f.vision.getSelectedCluster().get().isForAlliance(alliance),
                    "opposing-only localization cannot select the opposing CELL");
            frame.tag.id = 29;
            frame.timestamp = 5;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, false, 80_000_000L);
            check(f.pp.writes == 4 && !f.vision.getCorrectedRobotFieldPose().isPresent(),
                    "unknown tag IDs still cannot establish HIVE geometry");
        }
    }

    public static void main(String[] args) throws Exception {
        continuousIdleCalibration();
        failedVisionWritebackRetriesImmediately();
        rtCalibratesTwiceThenUsesOdometry();
        rtUsesIdleJumpGate();
        rtFaultAndStop();
        recoveryChecksDeviceOffsets();
        pinpointBadReadDiagnostics();
        odometryTrackingFeed();
        frozenStaleAndWrongPipeline();
        mixedAllianceTagsStayUsable();
        System.out.println("PASS RtCalibrationCheck: " + checks
                + " continuous calibration, RT, read diagnostics, OD aim, freshness, lock and real-feed assertions");
    }
}
