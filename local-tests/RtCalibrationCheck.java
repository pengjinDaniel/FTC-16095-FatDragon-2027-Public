package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import sun.misc.Unsafe;

/** Replays idle three-frame calibration, success-only cooldown and RT odometry aiming. */
public class RtCalibrationCheck {
    private static final Unsafe U = allocator();
    private static int checks;
    private static final Pose2D FIXED = pose(100, 40, 0);

    private static Unsafe allocator() {
        try {
            Field f = Unsafe.class.getDeclaredField("theUnsafe");
            f.setAccessible(true);
            return (Unsafe) f.get(null);
        } catch (Exception e) { throw new AssertionError(e); }
    }

    private static void set(Object object, Class<?> owner, String name, Object value) throws Exception {
        Field f = owner.getDeclaredField(name);
        f.setAccessible(true);
        f.set(object, value);
    }

    private static void check(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-6,
                message + ": expected " + expected + ", got " + actual);
    }

    private static Pose2D pose(double x, double y, double h) {
        return new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, h);
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

    private static class FlakyPinpoint extends VisionCalibrationCheck.FakePinpoint {
        boolean failNextWrite;
        @Override public void setPosition(Pose2D value) {
            super.setPosition(value);
            if (failNextWrite) {
                failNextWrite = false;
                status = DeviceStatus.CALIBRATING;
            }
        }
    }

    private static class Fixture {
        final Vision vision = (Vision) U.allocateInstance(Vision.class);
        final CaptureDrive drive = (CaptureDrive) U.allocateInstance(CaptureDrive.class);
        final FlakyPinpoint pp = (FlakyPinpoint) U.allocateInstance(FlakyPinpoint.class);
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

        void frame(Pose2D visual, double hive, double timestamp, Pose2D odometry,
                   boolean rt, boolean stopped, long now) throws Exception {
            if (odometry != null) pp.position = odometry;
            drive.periodic();
            clear.invoke(vision);
            if (visual != null) {
                VisionMath.HiveSolveResult result = new VisionMath.HiveSolveResult();
                result.correction = new VisionMath.HiveCorrection(visual, hive, 0);
                result.candidates[0] = result.correction;
                result.usableCandidateCount = 1;
                result.status = "Synthetic current vision";
                set(vision, Vision.class, "hiveSolveResult", result);
                set(vision, Vision.class, "frameTimestamp", timestamp);
                set(vision, Vision.class, "lastPhysicalFrameTimestamp", timestamp);
            }
            vision.updateAimTarget(drive, stopped, rt, now);
        }

        void normal(Pose2D visual, double hive, double timestamp, Pose2D odometry, long now) throws Exception {
            frame(visual, hive, timestamp, odometry, false, false, now);
        }

        void rt(Pose2D visual, double hive, double timestamp, Pose2D odometry, long now) throws Exception {
            frame(visual, hive, timestamp, odometry, true, false, now);
        }

        void initialize() throws Exception {
            for (int i = 0; i < 3; i++) {
                normal(FIXED, -30, i + 1, pose(0, 0, 0), i * 20_000_000L);
            }
        }
    }

    private static class ReadyShooter extends Shooter {
        ReadyShooter() { super(null); }
        @Override public void stop() { }
        @Override public boolean setDistanceTarget(double distance) { return Double.isFinite(distance); }
        @Override public boolean isReady() { return true; }
        @Override public boolean isPitchReady() { return true; }
    }

    private static class QuietIntake extends Intake {
        QuietIntake() { super(null); }
        @Override public void updateInputs(boolean requested, boolean jam, boolean shoot) { }
    }

    private static class ReadyTransit extends Transit {
        boolean feeding;
        ReadyTransit() { super(null); }
        @Override public TransitState getState() { return TransitState.CLOSE; }
        @Override public void cancelFeeding() { feeding = false; }
        @Override public void requestFeed(boolean permitted) { feeding = permitted; }
    }

    private static class LockedAim extends AutoAimController {
        int resets;
        @Override public double calculateTurnPower(boolean valid, double error) { return valid ? .123 : 0; }
        @Override public boolean isLocked() { return true; }
        @Override public void reset() { resets++; super.reset(); }
    }



    private static void threeFramesThenCooldown() throws Exception {
        Fixture f = new Fixture();
        f.normal(FIXED, -30, 1, pose(0, 0, 0), 0);
        check(f.pp.writes == 0 && !f.drive.isFieldPoseInitialized(), "first idle frame cannot overwrite the pose");
        for (int i = 0; i < 10; i++) f.normal(FIXED, -30, 1, null, 1_000_000L + i);
        check(f.pp.writes == 0, "reading one frame repeatedly cannot confirm calibration");
        f.normal(FIXED, -30, 2, null, 20_000_000L);
        check(f.pp.writes == 0, "second idle frame cannot overwrite the pose");
        f.normal(FIXED, -30, 3, null, 40_000_000L);
        check(f.pp.writes == 1 && f.drive.isFieldPoseInitialized(), "third stable idle frame writes once");
        near(VisionConstants.calibrationIntervalSeconds, 1.5, "production success cooldown is 1.5 seconds");
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 1.5, "successful write starts cooldown");
        long deadline = 1_540_000_000L;
        Pose2D next = pose(200, -100, 40);
        f.normal(next, 30, 4, null, 100_000_000L);
        f.normal(next, 30, 5, null, deadline - 1);
        check(f.pp.writes == 1, "cooldown blocks both collection and writes");
        f.normal(next, 30, 5, null, deadline);
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 0, "cooldown expires from the last successful write");
        f.normal(next, 30, 6, null, deadline + 20_000_000L);
        f.normal(next, 30, 7, null, deadline + 40_000_000L);
        check(f.pp.writes == 1, "frames observed during cooldown cannot join the next window");
        f.normal(next, 30, 8, null, deadline + 60_000_000L);
        check(f.pp.writes == 2, "three fresh post-cooldown frames write once");
        near(f.drive.getPose().getX(DistanceUnit.CM), 200, "large absolute odometry disagreement is still overwritten");
        near(f.drive.getPose().getHeading(AngleUnit.DEGREES), 40, "latest visual heading is applied");
        long secondDeadline = deadline + 60_000_000L + 1_500_000_000L;
        f.normal(next, 30, 8, null, secondDeadline);
        check(f.pp.writes == 2, "an already accepted physical frame cannot overwrite twice");
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 0, "duplicates do not restart cooldown");
    }

    private static void failuresRetryWithoutCooldown() throws Exception {
        Fixture f = new Fixture();
        f.normal(FIXED, -30, 1, pose(0, 0, 0), 0);
        f.normal(null, 0, 0, null, 20_000_000L);
        near(f.vision.getCalibrationCooldownRemainingSeconds(), 0, "missing vision cannot start cooldown");
        for (int i = 0; i < 3; i++) f.normal(FIXED, -30, i + 2, null, 40_000_000L + i * 20_000_000L);
        check(f.pp.writes == 1, "valid samples retry immediately after missing vision");

        Fixture outlier = new Fixture();
        outlier.normal(FIXED, -30, 1, pose(0, 0, 0), 0);
        outlier.normal(pose(102, 40, 0), -30, 2, null, 20_000_000L);
        outlier.normal(pose(104, 40, 0), -30, 3, null, 40_000_000L);
        check(outlier.pp.writes == 0, "the full three-frame spread is checked against its anchor");
        near(outlier.vision.getCalibrationCooldownRemainingSeconds(), 0, "unstable observations cannot start cooldown");
        outlier.normal(pose(104, 40, 0), -30, 4, null, 60_000_000L);
        outlier.normal(pose(104, 40, 0), -30, 5, null, 80_000_000L);
        check(outlier.pp.writes == 1, "a replacement stable window retries without a 1.5-second delay");

        Fixture failedWrite = new Fixture();
        failedWrite.normal(FIXED, -30, 1, pose(0, 0, 0), 0);
        failedWrite.normal(FIXED, -30, 2, null, 20_000_000L);
        failedWrite.pp.failNextWrite = true;
        failedWrite.normal(FIXED, -30, 3, null, 40_000_000L);
        check(!failedWrite.drive.isFieldPoseInitialized(), "a failed write readback cannot establish field coordinates");
        near(failedWrite.vision.getCalibrationCooldownRemainingSeconds(), 0, "a failed write cannot start cooldown");
        failedWrite.pp.status = DeviceStatus.READY;
        for (int i = 0; i < 3; i++) failedWrite.normal(FIXED, -30, i + 4, null, 60_000_000L + i * 20_000_000L);
        check(failedWrite.pp.writes == 2 && failedWrite.drive.isFieldPoseInitialized(),
                "a failed write retries a new stable window immediately");
    }

    private static void motionAndFrameContinuity() throws Exception {
        Fixture moving = new Fixture();
        double offset = Math.toRadians(20);
        for (int i = 0; i < 3; i++) {
            double dx = i * 2, dy = i;
            Pose2D odometry = pose(500 + dx, -200 + dy, 170 + i * 20);
            Pose2D visual = pose(100 + Math.cos(offset) * dx - Math.sin(offset) * dy,
                    40 + Math.sin(offset) * dx + Math.cos(offset) * dy, -170 + i * 20);
            moving.normal(visual, -30, i + 1, odometry, i * 20_000_000L);
        }
        check(moving.pp.writes == 1, "motion compensation accepts translation, turning, heading wrap and a large field offset");
        near(moving.drive.getPose().getHeading(AngleUnit.DEGREES), -130, "the latest moving frame is applied");

        Fixture heading = new Fixture();
        heading.normal(FIXED, -30, 1, pose(0, 0, 0), 0);
        heading.normal(pose(100, 40, 10), -30, 2, null, 20_000_000L);
        heading.normal(FIXED, -30, 3, null, 40_000_000L);
        heading.normal(FIXED, -30, 4, null, 60_000_000L);
        check(heading.pp.writes == 0, "a visual heading jump restarts the window");
        heading.normal(FIXED, -30, 5, null, 80_000_000L);
        check(heading.pp.writes == 1, "stable heading observations recover without cooldown");

        Fixture hive = new Fixture();
        hive.normal(FIXED, -30, 1, pose(0, 0, 0), 0);
        hive.normal(FIXED, -25, 2, null, 20_000_000L);
        hive.normal(FIXED, -25, 3, null, 40_000_000L);
        check(hive.pp.writes == 0, "HIVE-angle changes restart the window");
        hive.normal(FIXED, -25, 4, null, 60_000_000L);
        check(hive.pp.writes == 1, "stable new HIVE geometry can calibrate");

        Fixture gap = new Fixture();
        gap.normal(FIXED, -30, 10, pose(0, 0, 0), 0);
        gap.normal(FIXED, -30, 11, null, 20_000_000L);
        gap.normal(FIXED, -30, 1, null, 240_000_000L);
        gap.normal(FIXED, -30, 2, null, 260_000_000L);
        check(gap.pp.writes == 0, "a stale gap and camera clock restart cannot reuse old samples");
        gap.normal(FIXED, -30, 3, null, 280_000_000L);
        check(gap.pp.writes == 1, "three new frames recover after a camera restart");
    }

    private static void rtNeverCalibratesAndClearsPartialWindows() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.normal(pose(150, -50, 20), 30, 4, null, 1_560_000_000L);
        f.normal(pose(150, -50, 20), 30, 5, null, 1_580_000_000L);
        int before = f.pp.writes;
        f.rt(pose(-500, 400, 90), -30, 6, null, 1_600_000_000L);
        CellCluster held = f.vision.getSelectedCluster().orElse(null);
        double openingX = f.vision.getTargetXInches();
        for (int i = 0; i < 6; i++) {
            f.rt(pose(-500, 400, 90), -30, i + 7, null, 1_620_000_000L + i * 20_000_000L);
            check(f.pp.writes == before && !f.vision.wasCorrectionApplied(), "RT never writes even when stable tags are visible");
        }
        f.rt(null, 0, 0, pose(-90, 40, 10), 4_000_000_000L);
        check(f.vision.isAimCalibrationReady() && f.vision.hasValidAimingTarget(), "long RT holds keep aiming through tag loss without a search timeout");
        check(f.vision.getSelectedCluster().orElse(null) == held, "RT retains the selected CELL across the field center");
        near(f.vision.getTargetXInches(), openingX, "RT retains HIVE geometry");
        near(f.drive.getPose().getX(DistanceUnit.CM), -90, "Pinpoint continues moving during RT");
        f.normal(pose(200, -100, 40), -30, 13, null, 4_020_000_000L);
        f.normal(pose(200, -100, 40), -30, 14, null, 4_040_000_000L);
        check(f.pp.writes == before, "release cannot reuse pre-RT or RT-held samples");
        f.normal(pose(200, -100, 40), -30, 15, null, 4_060_000_000L);
        check(f.pp.writes == before + 1, "three fresh release frames resume calibration");
    }

    private static void cooldownRunsWhileRtIsHeld() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(FIXED, -30, 4, null, 60_000_000L);
        f.normal(FIXED, -30, 5, null, 1_000_000_000L);
        check(f.pp.writes == 1, "RT release cannot bypass an unfinished success cooldown");
        near(f.vision.getCalibrationCooldownRemainingSeconds(), .54, "cooldown is measured from success rather than RT release");
        f.normal(FIXED, -30, 6, null, 1_540_000_000L);
        f.normal(FIXED, -30, 7, null, 1_560_000_000L);
        check(f.pp.writes == 1, "expiry starts a new three-frame window");
        f.normal(FIXED, -30, 8, null, 1_580_000_000L);
        check(f.pp.writes == 2, "cooldown elapsed during RT is not restarted");
    }

    private static void rtUsesNormalPidImmediately() throws Exception {
        Fixture f = new Fixture();
        f.initialize();
        f.rt(null, 0, 0, null, 60_000_000L);
        AutoAimController aim = new AutoAimController();
        AutoAimController reference = new AutoAimController();
        ReadyTransit transit = (ReadyTransit) U.allocateInstance(ReadyTransit.class);
        Gamepad gamepad = (Gamepad) U.allocateInstance(Gamepad.class);
        gamepad.left_stick_y = -.3f;
        ShootAssistCommand command = new ShootAssistCommand(f.drive,
                (ReadyShooter) U.allocateInstance(ReadyShooter.class),
                (QuietIntake) U.allocateInstance(QuietIntake.class), transit, f.vision,
                aim, new GamepadEx(gamepad));
        command.initialize();
        double expected = reference.calculateTurnPower(true, f.vision.getYawErrorDegrees());
        command.execute();
        near(f.drive.turn, expected, "RT uses normal PID immediately rather than fixed-direction search");
        check(Math.hypot(f.drive.forward, f.drive.strafe) > 0, "RT manual translation remains headless");
        double heading = f.vision.getDesiredHeadingDegrees();
        f.rt(null, 0, 0, pose(100, 40, heading), 80_000_000L);
        command.execute(); command.execute();
        near(f.drive.turn, 0, "RT can stop at its existing coordinate target without a new tag");
        set(aim, AutoAimController.class, "withinToleranceSinceNanos",
                System.nanoTime() - (long) ((DriveConstants.aimLockStableTimeSeconds + .01) * 1_000_000_000L));
        command.execute();
        check(aim.isLocked() && transit.feeding && command.canPrepareToFire(), "odometry aim can lock and fire through tag loss");
        f.pp.status = DeviceStatus.CALIBRATING;
        f.rt(null, 0, 0, null, 100_000_000L);
        command.execute();
        near(f.drive.turn, 0, "Pinpoint loss stops automatic rotation");
        check(!transit.feeding, "Pinpoint loss cancels feeding");
    }

    private static void coldRtAndBStop() throws Exception {
        Fixture cold = new Fixture();
        for (int i = 0; i < 5; i++) cold.rt(FIXED, -30, i + 1, pose(0, 0, 0), i * 20_000_000L);
        check(cold.pp.writes == 0 && !cold.drive.isFieldPoseInitialized() && !cold.vision.hasValidAimingTarget(),
                "RT cannot initialize a cold field pose even with visible tags");
        for (int i = 0; i < 3; i++) cold.normal(FIXED, -30, i + 6, null, 100_000_000L + i * 20_000_000L);
        check(cold.pp.writes == 1 && cold.drive.isFieldPoseInitialized(), "release allows three-frame field initialization");
        cold.rt(null, 0, 0, null, 160_000_000L);
        check(cold.vision.isAimCalibrationReady(), "the next RT press needs no new visual confirmation");

        Fixture stopped = new Fixture();
        stopped.initialize();
        stopped.frame(FIXED, -30, 4, null, true, true, 60_000_000L);
        stopped.rt(FIXED, -30, 5, null, 80_000_000L);
        check(!stopped.vision.hasValidAimingTarget() && stopped.pp.writes == 1, "B stop stays latched through the RT hold");
        stopped.normal(FIXED, -30, 6, null, 100_000_000L);
        stopped.rt(null, 0, 0, null, 120_000_000L);
        check(stopped.vision.hasValidAimingTarget(), "RT release clears B stop without requiring a new RT correction");
    }

    private static void pinpointFaultDiagnosticsAndRecovery() throws Exception {
        for (DeviceStatus status : DeviceStatus.values()) {
            if (status == DeviceStatus.READY) continue;
            Fixture fault = new Fixture();
            fault.initialize();
            fault.pp.status = status;
            fault.normal(FIXED, -30, 4, null, 2_000_000_000L);
            check(!fault.drive.isFieldPoseInitialized() && fault.pp.writes == 1,
                    "a Pinpoint fault invalidates field coordinates without a vision write");
            check(fault.vision.getCalibrationStatus().contains("Pinpoint")
                            && fault.vision.getCalibrationStatus().contains(status.name()),
                    "valid vision must report the actual Pinpoint blocker: " + status);
            check(!Double.isFinite(fault.drive.getPose().getX(DistanceUnit.CM)),
                    "a failed device cannot expose stale field coordinates to control");
            check(fault.drive.getOdometryStatus() == status
                            && fault.drive.getRawOdometryPose() == fault.pp.position,
                    "diagnostics expose cached device values without making them valid for control");
            fault.pp.status = DeviceStatus.READY;
            for (int i = 0; i < 3; i++) {
                fault.normal(FIXED, -30, i + 5, null, 2_020_000_000L + i * 20_000_000L);
            }
            check(fault.drive.isFieldPoseInitialized() && fault.pp.writes == 2,
                    "a recovered device automatically reinitializes after three fresh frames");
        }
        Fixture invalid = new Fixture();
        invalid.initialize();
        invalid.normal(FIXED, -30, 4, pose(Double.NaN, 0, 0), 2_000_000_000L);
        check(!invalid.drive.isFieldPoseInitialized() && invalid.pp.writes == 1,
                "READY with non-finite odometry still blocks calibration");
        check(invalid.vision.getCalibrationStatus().contains("Pinpoint")
                        && invalid.vision.getCalibrationStatus().contains("invalid X/Y/H"),
                "invalid odometry must not be labeled as a Limelight failure");
        for (int i = 0; i < 3; i++) {
            invalid.normal(FIXED, -30, i + 5, pose(0, 0, 0), 2_020_000_000L + i * 20_000_000L);
        }
        check(invalid.drive.isFieldPoseInitialized() && invalid.pp.writes == 2,
                "finite odometry recovery still requires the existing three-frame window");
    }

    public static void main(String[] args) throws Exception {
        threeFramesThenCooldown();
        failuresRetryWithoutCooldown();
        motionAndFrameContinuity();
        rtNeverCalibratesAndClearsPartialWindows();
        cooldownRunsWhileRtIsHeld();
        rtUsesNormalPidImmediately();
        coldRtAndBStop();
        pinpointFaultDiagnosticsAndRecovery();
        System.out.println("PASS RtCalibrationCheck: " + checks
                + " idle three-frame calibration, success-only cooldown, retry, RT pause and odometry PID assertions");
    }
}
