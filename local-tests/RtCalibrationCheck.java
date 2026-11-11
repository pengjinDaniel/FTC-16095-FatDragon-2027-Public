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
import org.firstinspires.ftc.teamcode.subsystems.localization.PinpointPoseEstimator;
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
        boolean pitchReady;
        int stopCount;

        ReadyShooter() { super(null); }
        @Override public void stop() { stopCount++; }
        @Override public boolean setDistanceTarget(double cm) {
            distanceCm = cm;
            return Double.isFinite(cm);
        }
        @Override public boolean isReady() { return true; }
        @Override public boolean isPitchReady() { return pitchReady; }
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

    private static void rtCalibratesOnceThenUsesOdometry() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(FIXED, 1, 20_000_000L);
        check(f.pp.writes == 1 && f.vision.hasValidAimingTarget() && !f.vision.isAimCalibrationReady()
                        && !f.vision.shouldPauseForAimCalibration(),
                "an already-used idle frame does not count, while odometry can still aim");
        check(f.vision.getCalibrationStatus().contains("new Limelight frame"),
                "telemetry reports the need for a new frame");
        f.rt(pose(101, 40, 2), 2, 40_000_000L);
        check(f.pp.writes == 2 && f.vision.isAimCalibrationReady(),
                "the first successful RT calibration makes the aim ready");
        near(f.drive.getPose().getX(DistanceUnit.CM), 101, "the single calibration updates X");
        double error = f.vision.getYawErrorDegrees();
        double distance = f.vision.getTargetHorizontalDistanceInches();
        for (int i = 0; i < 8; i++) {
            f.rt(pose(-200 + i, -200, 90), 3 + i, 60_000_000L + i * 20_000_000L);
            near(f.vision.getYawErrorDegrees(), error, "held RT ignores visual yaw and position");
            near(f.vision.getTargetHorizontalDistanceInches(), distance, "held RT measures distance from OD");
            check(f.vision.getSelectedCluster().get() == CellCluster.BLUE_AUDIENCE,
                    "the OD field half selects the CELL during RT");
        }
        check(f.pp.writes == 2, "held RT permits one successful vision correction");

        f.pp.position = pose(-100, 40, 15);
        f.rt(FIXED, 11, 240_000_000L);
        check(f.vision.getSelectedCluster().get() == CellCluster.BLUE_OPPOSITE_AUDIENCE,
                "OD motion changes the live CELL selection");
        near(f.vision.getYawErrorDegrees(), VisionMath.coordinateYawErrorDegrees(
                f.pp.position, -17.7, 12.75, VisionConstants.shooterYawDegrees),
                "current OD pose controls yaw during RT");
        f.rt(null, 0, 1_000_000_000L);
        check(f.vision.hasValidAimingTarget() && f.vision.isAimCalibrationReady(),
                "tag loss preserves calibrated OD aiming");
        f.frame(pose(-99, 40, 16), 12, false, false, 1_020_000_000L);
        check(f.pp.writes == 3, "RT release immediately resumes continuous idle calibration");
        f.rt(pose(-98, 40, 16), 13, 1_040_000_000L);
        check(f.pp.writes == 4 && f.vision.isAimCalibrationReady(),
                "the next RT session accepts one new correction");
        f.rt(pose(-97, 40, 16), 14, 1_060_000_000L);
        check(f.pp.writes == 4, "later RT frames are ignored again");
        f.frame(pose(-97, 40, 16), 14, false, false, 1_080_000_000L);
        check(f.pp.writes == 5, "a fresh frame becomes eligible when RT is released");

        Fixture cold = new Fixture();
        cold.rt(null, 0, 0);
        check(!cold.vision.hasValidAimingTarget() && !cold.vision.isAimCalibrationReady()
                        && cold.vision.shouldPauseForAimCalibration() && cold.pp.writes == 0,
                "a cold start waits for a usable frame before aiming");
        cold.rt(FIXED, 1, 20_000_000L);
        check(cold.vision.isAimCalibrationReady() && cold.pp.writes == 1,
                "the first successful RT frame can establish the field pose");
    }

    private static void rtBreaksIdleJumpSequence() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        Pose2D jumped = pose(200, 40, 30);
        f.frame(jumped, 2, false, false, 20_000_000L);
        f.rt(jumped, 3, 40_000_000L);
        f.rt(jumped, 3, 60_000_000L);
        f.rt(jumped, 4, 80_000_000L);
        check(f.pp.writes == 1 && !f.vision.isAimCalibrationReady(),
                "idle rejects and duplicate frames cannot advance the RT jump gate");
        f.rt(jumped, 5, 100_000_000L);
        check(f.pp.writes == 2 && f.vision.isAimCalibrationReady(),
                "the existing jump override can supply the single successful RT correction");
        near(f.drive.getPose().getX(DistanceUnit.CM), 200, "the override writes the accepted pose");
        f.rt(jumped, 6, 120_000_000L);
        check(f.pp.writes == 2, "the override consumes the RT correction");
    }

    private static void failedVisionWritebackRetriesImmediately() throws Exception {
        Fixture f = new Fixture();
        f.pp.failAfterPoseWrite = true;
        f.rt(FIXED, 1, 0);
        check(!f.drive.isFieldPoseInitialized() && !f.vision.isAimCalibrationReady()
                        && f.drive.getPinpointBadReadCount() == 1,
                "a failed post-write read rejects and records the attempted calibration");
        check(f.drive.getLastPinpointReadFailure().contains("after vision write"),
                "diagnostics identify the post-calibration read phase");
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 0,
                "a failed post-write read does not start a cooldown");

        f.pp.failAfterPoseWrite = false;
        f.pp.status = DeviceStatus.READY;
        f.rt(FIXED, 2, 20_000_000L);
        check(f.vision.isAimCalibrationReady() && f.pp.writes == 2,
                "a failed write does not consume the single successful correction");
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 0,
                "a successful retry also leaves no cooldown");
        f.rt(FIXED, 3, 40_000_000L);
        check(f.pp.writes == 2, "a successful retry completes the RT calibration");
        f.frame(FIXED, 3, false, false, 60_000_000L);
        check(f.pp.writes == 3, "idle calibration resumes after the retry");
    }

    private static void rtFaultAndStop() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.frame(FIXED, 2, true, true, 40_000_000L);
        f.rt(FIXED, 3, 60_000_000L);
        check(!f.vision.isAimCalibrationReady() && !f.vision.hasValidAimingTarget(),
                "B stays latched until RT release");
        check(f.pp.writes == 1, "B leaves vision calibration paused");
        f.frame(FIXED, 4, false, false, 80_000_000L);
        check(f.pp.writes == 2, "RT release resumes calibration after B stop");
        f.rt(FIXED, 5, 100_000_000L);
        check(f.vision.hasValidAimingTarget() && f.vision.isAimCalibrationReady() && f.pp.writes == 3,
                "the next RT session can calibrate once after B stop");

        f.pp.position = pose(110, 42, 10);
        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(FIXED, 6, 120_000_000L);
        check(!f.vision.hasValidAimingTarget() && f.pp.writes == 3,
                "bad reads suspend OD aiming under RT");
        check(!f.drive.isFieldPoseInitialized()
                        && Double.isNaN(f.drive.getPose().getX(DistanceUnit.CM)),
                "bad reads cannot expose a cached field pose for control");
        f.pp.status = DeviceStatus.READY;
        f.rt(FIXED, 7, 140_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 3,
                "a valid recovered Pinpoint resumes OD aiming without a vision rewrite");
        near(f.drive.getPose().getX(DistanceUnit.CM), 110, "recovery preserves the current OD position");
        near(f.drive.getDriverHeadingRadians(), Math.toRadians(10),
                "a bad read cannot reset the headless driving reference");
        check(f.pp.offsetReads == 2, "recovery verifies both device offsets once");
        f.frame(pose(110, 42, 10), 8, false, false, 160_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 4 && f.pp.offsetReads == 2,
                "RT release recalibrates immediately without rereading verified device offsets");

        f.pp.status = DeviceStatus.CALIBRATING;
        f.rt(FIXED, 9, 180_000_000L);
        check(!f.vision.isAimCalibrationReady() && f.pp.writes == 4,
                "an unavailable Pinpoint invalidates the field pose");
        f.pp.status = DeviceStatus.READY;
        f.rt(FIXED, 10, 200_000_000L);
        check(f.vision.isAimCalibrationReady() && f.pp.writes == 5,
                "verified odometry recovery can complete a pending RT calibration");
        f.pp.status = DeviceStatus.CALIBRATING;
        f.rt(FIXED, 11, 220_000_000L);
        f.pp.status = DeviceStatus.READY;
        f.rt(FIXED, 12, 240_000_000L);
        check(!f.vision.hasValidAimingTarget() && f.pp.writes == 5,
                "losing the field anchor cannot rearm a completed RT calibration");
        f.frame(FIXED, 13, false, false, 260_000_000L);
        check(f.vision.hasValidAimingTarget() && f.pp.writes == 6,
                "idle vision restores the field pose after a completed RT session");
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

    private static void odometryAimAndCommittedFeed() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        double heading = Math.IEEEremainder(Math.toDegrees(Math.atan2(
                12.75 * 2.54 - 40, 17.7 * 2.54 - 100)) - 180, 360);
        Pose2D aligned = pose(100, 40, heading);
        f.pp.position = aligned;
        f.frame(aligned, 2, false, false, 10_000_000L);
        AutoAimController aim = new AutoAimController();
        ReadyShooter shooter = (ReadyShooter) U.allocateInstance(ReadyShooter.class);
        QuietIntake intake = (QuietIntake) U.allocateInstance(QuietIntake.class);
        Transit transit = (Transit) U.allocateInstance(Transit.class);
        double[] motorPower = {0};
        double[] gatePosition = {TransitConstants.limitClosed};
        DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (proxy, method, args) -> {
                    if (!method.getName().equals("setPower")) throw new AssertionError(method);
                    motorPower[0] = (double) args[0];
                    if (motorPower[0] < 0 && transit.getFiringState() == Transit.FiringState.REVERSING) {
                        near(gatePosition[0], TransitConstants.limitOpen, "shot reverse power follows the open gate command");
                    }
                    return null;
                });
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (!method.getName().equals("setPosition")) throw new AssertionError(method);
                    gatePosition[0] = (double) args[0];
                    return null;
                });
        set(transit, Transit.class, "transitMotor", motor);
        set(transit, Transit.class, "limitServo", servo);
        FeedTimer timer = new FeedTimer();
        set(transit, Transit.class, "feedReverseTimer", timer);
        set(transit, Transit.class, "transitState", Transit.TransitState.CLOSE);
        TransitConstants.limitPositionsCalibrated = true;
        TransitConstants.feedReversePower = -0.4;
        TransitConstants.feedReverseTime = 0.1;
        Gamepad gamepad = (Gamepad) U.allocateInstance(Gamepad.class);
        ShootAssistCommand command = new ShootAssistCommand(
                f.drive, shooter, intake, transit, f.vision, aim, new GamepadEx(gamepad));
        command.initialize();
        f.pp.position = pose(100, 40, heading + 10);
        f.rt(aligned, 2, 20_000_000L);
        command.execute();
        check(Math.abs(f.drive.turn) > 0 && f.pp.writes == 2 && !f.vision.isAimCalibrationReady(),
                "RT can turn from OD while waiting for a new correction");
        near(motorPower[0], 0, "transit stays stopped while turning toward the target");
        check(shooter.stopCount == 1, "pressing RT resets shooter pre-spin");
        f.rt(null, 0, 25_000_000L);
        command.execute();
        check(Math.abs(f.drive.turn) > 0 && !command.canFire(),
                "OD can turn toward the target while the camera has no usable frame");

        f.pp.position = aligned;
        f.rt(null, 0, 30_000_000L);
        command.execute();
        set(aim, AutoAimController.class, "withinToleranceSinceNanos", System.nanoTime() - 1_000_000_000L);
        shooter.pitchReady = true;
        command.execute();
        check(aim.isLocked() && !command.canPrepareToFire(),
                "an OD lock still waits for the single correction before feeding");
        f.rt(aligned, 3, 40_000_000L);
        command.execute();
        check(f.vision.isAimCalibrationReady() && !aim.isLocked() && !command.canPrepareToFire(),
                "the successful correction starts a fresh aim lock interval");
        shooter.pitchReady = false;
        f.rt(null, 0, 60_000_000L);
        set(aim, AutoAimController.class, "withinToleranceSinceNanos", System.nanoTime() - 1_000_000_000L);
        command.execute();
        check(aim.isLocked() && !command.canPrepareToFire()
                        && transit.getState() == Transit.TransitState.CLOSE,
                "RT waits for the pitch servo even when aim and speed are ready");
        near(motorPower[0], 0, "transit waits for pitch to settle");
        shooter.pitchReady = true;
        command.execute();
        check(command.canPrepareToFire() && transit.getFiringState() == Transit.FiringState.REVERSING
                        && transit.getState() == Transit.TransitState.OPEN,
                "calibrated aim, ready speed and settled pitch open the gate before reversing");
        near(motorPower[0], -0.4, "an open gate starts the configured reverse pulse");

        double initialDistance = shooter.distanceCm;
        f.pp.position = pose(130, 40, heading + 10);
        f.rt(pose(-200, -200, 90), 5, 100_000_000L);
        timer.time = 0.05;
        command.execute();
        check(!aim.isLocked() && f.pp.writes == 3 && shooter.distanceCm > initialDistance,
                "after one correction, new OD geometry updates the shot without another write");
        near(motorPower[0], -0.4, "losing alignment does not restart or cancel the reverse pulse");

        timer.time = 0.1;
        command.execute();
        check(command.canFire() && transit.getState() == Transit.TransitState.OPEN,
                "the committed shot advances at the reverse deadline");
        near(motorPower[0], TransitConstants.feedPower, "the deadline switches transit to forward feed");
        command.execute();
        near(motorPower[0], TransitConstants.feedPower, "forward feed continues while the shot is held");

        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(aligned, 6, 120_000_000L);
        command.execute();
        check(!aim.isLocked() && !command.canFire(), "invalid field position clears aim lock and cancels feeding");
        check(Double.isNaN(shooter.distanceCm), "invalid odometry clears the shooter distance target");
        near(motorPower[0], 0, "invalid odometry stops transit");
        near(gatePosition[0], TransitConstants.limitClosed, "invalid odometry closes the gate");

        command.end(true);
        check(shooter.stopCount == 2, "releasing RT stops the shooter");
        near(f.drive.turn, 0, "ending RT stops turning");

        f.drive.clearVisionPose("Fusion test");
        f.pp.status = DeviceStatus.READY;
        f.pp.position = pose(0, 0, 0);
        f.frame(null, 0, false, false, System.nanoTime());
        PinpointPoseEstimator estimator = new PinpointPoseEstimator();
        set(f.drive, Drive.class, "poseEstimator", estimator);
        long now = System.nanoTime();
        for (int offsetMs = -300; offsetMs <= 0; offsetMs += 50) {
            estimator.updateOdometry(now + offsetMs * 1_000_000L, f.pp.position);
        }
        check(!f.drive.addTimedVisionPose(aligned, 10, now - 280_000_000L, now, .1, false),
                "one MT1 frame cannot anchor fusion");
        check(f.drive.addTimedVisionPose(aligned, 11, now - 230_000_000L, now, .1, false),
                "the next consistent frame anchors fusion");
        Frame fusionFrame = installFusionCamera(f);
        fusionFrame.pose = new Pose3D(new Position(DistanceUnit.CM, 100, 40, 0, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, heading, 0, 0, 0));
        fusedVisionFrame(f, now - 175_000_000L, false);
        fusionFrame.timestamp++;
        command.initialize();
        fusedVisionFrame(f, now, true);
        check(f.vision.isAimCalibrationReady() && f.vision.shouldResetAimControl(),
                "the single fused RT correction restarts the aim lock");
        command.execute();
        f.rt(null, 0, System.nanoTime());
        set(aim, AutoAimController.class, "withinToleranceSinceNanos", System.nanoTime() - 1_000_000_000L);
        command.execute();
        timer.time = 0.1;
        command.execute();
        check(command.canFire(), "a healthy fused pose can lock and feed during a short tag loss");
        f.pp.status = DeviceStatus.FAULT_BAD_READ;
        f.rt(null, 0, System.nanoTime());
        command.execute();
        check(!command.canFire() && !aim.isLocked(), "a fused OD fault cancels a committed shot");
        near(f.drive.turn, 0, "a fused OD fault stops automatic turning");
        near(motorPower[0], 0, "a fused OD fault stops transit");
        near(gatePosition[0], TransitConstants.limitClosed, "a fused OD fault closes the gate");
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
        double captureLatency;
        Pose3D pose;
        Tag tag;
        Tag otherTag;
        Frame() throws Exception { super(null); }
        @Override public boolean isValid() { return true; }
        @Override public int getPipelineIndex() { return pipeline; }
        @Override public long getStaleness() { return age; }
        @Override public double getCaptureLatency() { return captureLatency; }
        @Override public double getTargetingLatency() { return 5; }
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
            return pose != null ? pose : new Pose3D(new Position(DistanceUnit.CM, 100, 40, 0, 0),
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
        check(f.pp.writes == 1 && !f.vision.getCorrectedRobotFieldPose().isPresent(),
                "stale reception cannot calibrate in idle");
        frame.age = 0;
        frame.pipeline = 1;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 60_000_000L);
        check(f.pp.writes == 1 && !f.vision.getCorrectedRobotFieldPose().isPresent(),
                "the wrong pipeline cannot calibrate in idle");
        frame.pipeline = 0;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, true, 80_000_000L);
        check(f.pp.writes == 2 && f.vision.isAimCalibrationReady(),
                "a fresh frame on the correct pipeline completes RT calibration");
        frame.timestamp = 3;
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, true, 90_000_000L);
        check(f.pp.writes == 2, "a later frame cannot recalibrate under the same RT press");
        f.vision.update();
        f.vision.updateAimTarget(f.drive, false, false, 100_000_000L);
        check(f.pp.writes == 3 && f.vision.isAimCalibrationReady(),
                "the first eligible frame calibrates when RT is released");
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
            check(f.vision.hasValidAimingTarget() && f.vision.isAimCalibrationReady() && f.pp.writes == 2,
                    "a mixed-alliance frame can supply the single legacy RT correction");
            frame.timestamp = 3;
            f.vision.update();
            f.vision.updateAimTarget(f.drive, false, false, 40_000_000L);
            check(f.vision.isAimCalibrationReady() && f.pp.writes == 3,
                    "mixed-alliance tags remain usable after RT release");
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

    private static void fusedEstimatorReplaysOdometry() {
        PinpointPoseEstimator estimator = new PinpointPoseEstimator();
        long ms = 1_000_000L;
        for (int i = 0; i <= 10; i++) {
            estimator.updateOdometry(i * 50 * ms, pose(i * 5, 0, 0));
        }
        check(!estimator.addVisionMeasurement(pose(200, 312.5, 90), 125 * ms, 500 * ms, .1, false),
                "the first delayed observation is only an anchor candidate");
        check(estimator.addVisionMeasurement(pose(200, 327.5, 90), 275 * ms, 500 * ms, .1, false),
                "anchor agreement compares observations after compensating for odometry motion");
        near(estimator.getEstimatedPose().getX(DistanceUnit.CM), 200, "delayed replay rotates local X motion");
        near(estimator.getEstimatedPose().getY(DistanceUnit.CM), 350, "delayed replay retains later motion");
        near(estimator.getEstimatedPose().getHeading(AngleUnit.DEGREES), 90, "vision anchors field heading");
        check(!estimator.addVisionMeasurement(pose(200, 327.5, 90), 275 * ms, 500 * ms, .1, false),
                "a capture timestamp cannot be accepted twice");
        check(!estimator.addVisionMeasurement(pose(900, 900, 90), 300 * ms, 500 * ms, .1, false),
                "an innovation outlier does not overwrite the current estimate");
        check(estimator.addVisionMeasurement(pose(200, 333, 90), 325 * ms, 500 * ms, .1, false),
                "a consistent delayed observation updates the seeded EKF");
        double y = estimator.getEstimatedPose().getY(DistanceUnit.CM);
        check(y > 350 && y < 350.6, "the EKF blends the correction and retains later motion");

        estimator.setVisionPaused(true, 500 * ms);
        check(!estimator.addVisionMeasurement(pose(200, 338, 90), 375 * ms, 500 * ms, .1, true),
                "an acceptable measurement cannot change the fused pose under RT");
        near(estimator.getEstimatedPose().getY(DistanceUnit.CM), y, "pausing preserves the current estimate");
        for (int time = 550; time <= 2000; time += 50) {
            estimator.updateOdometry(time * ms, pose(50, 0, 0));
        }
        check(estimator.isUsable(2000 * ms),
                "intentional vision pause can continue past the normal vision age limit");
        estimator.setVisionPaused(false, 2000 * ms);
        check(!estimator.isUsable(2000 * ms), "releasing RT restores the normal vision age limit");
        estimator.setVisionPaused(true, 2000 * ms);
        check(!estimator.isUsable(2000 * ms), "an expired pose cannot start a new RT session");
        check(!estimator.addVisionMeasurement(pose(0, 0, 0), 1850 * ms, 2000 * ms, .1, true),
                "even a new anchor must wait for RT release");
        estimator.setVisionPaused(false, 2000 * ms);
        check(!estimator.addVisionMeasurement(pose(0, 0, 0), 1900 * ms, 2000 * ms, .1, true),
                "re-anchoring an expired estimate waits for confirmation");
        check(estimator.addVisionMeasurement(pose(0, 0, 0), 1950 * ms, 2000 * ms, .1, true)
                        && estimator.isUsable(2000 * ms),
                "a consistent pair recovers an expired field frame");
        estimator.updateOdometry(2300 * ms, pose(0, 0, 0));
        check(!estimator.isUsable(2300 * ms), "a long odometry gap clears the old anchor");

        estimator.updateOdometry(2350 * ms, pose(0, 0, 0));
        estimator.updateOdometry(2400 * ms, pose(0, 0, 0));
        check(!estimator.addVisionMeasurement(pose(0, 0, 179), 2300 * ms, 2400 * ms, .1, false),
                "an anchor candidate may lie at the heading wrap");
        check(estimator.addVisionMeasurement(pose(0, 0, -179), 2350 * ms, 2400 * ms, .1, false)
                        && estimator.addVisionMeasurement(pose(0, 0, 179), 2400 * ms, 2400 * ms, .1, false),
                "anchor agreement and innovation both wrap the heading difference");
        check(Math.abs(estimator.getEstimatedPose().getHeading(AngleUnit.DEGREES)) >= 179,
                "a correction across the wrap remains close to 180 degrees");
    }

    private static void fusedVisionFrame(Fixture f, long observedNanos, boolean rt) throws Exception {
        f.vision.update();
        set(f.vision, Vision.class, "currentResultObservedNanos", observedNanos);
        f.vision.updateAimTarget(f.drive, false, rt, observedNanos);
    }

    private static Frame installFusionCamera(Fixture f) throws Exception {
        Camera camera = (Camera) U.allocateInstance(Camera.class);
        Frame frame = (Frame) U.allocateInstance(Frame.class);
        frame.tag = (Tag) U.allocateInstance(Tag.class);
        frame.tag.id = 38;
        frame.otherTag = (Tag) U.allocateInstance(Tag.class);
        frame.otherTag.id = 39;
        frame.timestamp = 1;
        frame.captureLatency = 15;
        camera.frame = frame;
        set(f.vision, Vision.class, "limelight", camera);
        return frame;
    }

    private static void fusedVisionQualifiesFrames() throws Exception {
        Fixture f = new Fixture();
        PinpointPoseEstimator estimator = new PinpointPoseEstimator();
        set(f.drive, Drive.class, "poseEstimator", estimator);
        f.drive.resetDriverHeading();
        long ms = 1_000_000L;
        long start = System.nanoTime() - 500 * ms;
        for (int time = 0; time <= 500; time += 50) {
            estimator.updateOdometry(start + time * ms, f.pp.position);
        }
        Frame frame = installFusionCamera(f);
        frame.otherTag.id = 30;

        fusedVisionFrame(f, start + 50 * ms, false);
        check(f.vision.getCalibrationStatus().contains("two usable tags"),
                "mixed alliances cannot update fusion");
        frame.otherTag.id = 39;
        frame.timestamp = 2;
        fusedVisionFrame(f, start + 75 * ms, false);
        fusedVisionFrame(f, start + 200 * ms, false);
        check(!f.vision.isAimCalibrationReady(), "a repeated frame cannot progress HIVE stability");
        frame.timestamp = 3;
        fusedVisionFrame(f, start + 230 * ms, false);
        check(!f.vision.isAimCalibrationReady(), "the first stable frame still needs a second frame");
        frame.timestamp = 4;
        fusedVisionFrame(f, start + 260 * ms, true);
        check(f.vision.isAimCalibrationReady() && f.vision.hasValidAimingTarget()
                        && f.vision.wasCorrectionApplied() && f.vision.shouldResetAimControl(),
                "the first accepted fused correction completes RT calibration and restarts aim lock");
        check(f.pp.writes == 0, "fusion never overwrites the Pinpoint hardware pose");
        near(f.drive.getDriverHeadingRadians(), 0, "fusion preserves the separate driver heading");

        frame.timestamp = 5;
        frame.pose = new Pose3D(new Position(DistanceUnit.CM, 100.5, 40, 0, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        fusedVisionFrame(f, start + 300 * ms, true);
        check(f.vision.isAimCalibrationReady() && !f.vision.wasCorrectionApplied(),
                "after the first correction, RT ignores another fresh qualifying frame");
        near(f.drive.getPose().getX(DistanceUnit.CM), 100, "RT keeps the once-calibrated field pose");
        fusedVisionFrame(f, start + 315 * ms, false);
        check(f.vision.wasCorrectionApplied() && !f.vision.shouldResetAimControl(),
                "RT release resumes small visual corrections");
        frame.timestamp = 6;
        fusedVisionFrame(f, start + 320 * ms, true);
        check(f.vision.wasCorrectionApplied() && f.vision.isAimCalibrationReady()
                        && f.vision.shouldResetAimControl(),
                "the next RT press can fuse one new frame and reset the aim lock again");
        frame.timestamp = 3;
        fusedVisionFrame(f, start + 330 * ms, false);
        check(!f.vision.wasCorrectionApplied(), "an out-of-order camera frame cannot update fusion");
        frame.timestamp = 7;
        frame.captureLatency = 220;
        fusedVisionFrame(f, start + 360 * ms, false);
        check(!f.vision.wasCorrectionApplied(), "a new packet with an old capture cannot update fusion");

        frame.captureLatency = 15;
        double pivotCm = DistanceUnit.INCH.toCm(VisionConstants.hivePivotHeightInches);
        for (int tilt = 1; tilt <= 4; tilt++) {
            double phi = Math.toRadians(tilt);
            double rawX = Math.cos(phi) * 100.5 - Math.sin(phi) * pivotCm;
            double rawZ = -Math.sin(phi) * 100.5 - Math.cos(phi) * pivotCm + pivotCm;
            frame.pose = new Pose3D(new Position(DistanceUnit.CM, rawX, 40, rawZ, 0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, tilt, 0, 0));
            frame.timestamp++;
            fusedVisionFrame(f, start + (360 + tilt * 30) * ms, false);
        }
        check(!f.vision.wasCorrectionApplied() && f.vision.getCalibrationStatus().contains("stable HIVE"),
                "a slow HIVE flip cannot pass the stability window via small frame-to-frame changes");
    }

    public static void main(String[] args) throws Exception {
        continuousIdleCalibration();
        failedVisionWritebackRetriesImmediately();
        rtCalibratesOnceThenUsesOdometry();
        rtBreaksIdleJumpSequence();
        rtFaultAndStop();
        recoveryChecksDeviceOffsets();
        pinpointBadReadDiagnostics();
        odometryAimAndCommittedFeed();
        frozenStaleAndWrongPipeline();
        mixedAllianceTagsStayUsable();
        fusedEstimatorReplaysOdometry();
        fusedVisionQualifiesFrames();
        System.out.println("PASS RtCalibrationCheck: " + checks
                + " continuous calibration, RT, read diagnostics, OD aim, freshness, lock and real-feed assertions");
    }
}
