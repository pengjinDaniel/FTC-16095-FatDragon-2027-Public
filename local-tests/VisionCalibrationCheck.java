package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.lang.reflect.Field;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import sun.misc.Unsafe;

/** Exercises the production calibration policy and Pinpoint writes without robot hardware. */
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

    private static void set(Object target, Class<?> owner, String name, Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Pose2D pose(double xCm, double yCm, double heading) {
        return new Pose2D(DistanceUnit.CM, xCm, yCm, AngleUnit.DEGREES, heading);
    }

    public static class FakePinpoint extends GoBildaPinpointDriver {
        Pose2D position;
        DeviceStatus status;
        int writes;

        private FakePinpoint() { super(null, false); }
        @Override public DeviceStatus getDeviceStatus() { return status; }
        @Override public Pose2D getPosition() { return position; }
        @Override public double getHeading(AngleUnit unit) { return position.getHeading(unit); }
        @Override public void setPosition(Pose2D candidate) { position = candidate; writes++; }
        @Override public void update() { }
    }

    private static class Samples {
        final Vision vision;
        final Drive drive;
        final FakePinpoint pinpoint;
        double timestamp;

        Samples() throws Exception {
            vision = (Vision) ALLOCATOR.allocateInstance(Vision.class);
            drive = (Drive) ALLOCATOR.allocateInstance(Drive.class);
            pinpoint = (FakePinpoint) ALLOCATOR.allocateInstance(FakePinpoint.class);
            pinpoint.position = pose(0, 0, 0);
            pinpoint.status = GoBildaPinpointDriver.DeviceStatus.READY;
            set(drive, Drive.class, "pinpoint", pinpoint);
            set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
            set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
            set(drive, Drive.class, "lastAcceptedVisionTimestamp", Double.NaN);
            set(vision, Vision.class, "alliance", Alliance.BLUE);
            set(vision, Vision.class, "observedClusterStates", Collections.emptyMap());
        }

        void prepare(double xCm, double yCm, double heading) throws Exception {
            set(vision, Vision.class, "correctedRobotFieldPose", pose(xCm, yCm, heading));
            set(vision, Vision.class, "frameTimestamp", ++timestamp);
        }

        void frame(double xCm, double yCm, double heading, boolean allowed, long now)
                throws Exception {
            prepare(xCm, yCm, heading);
            vision.updateFieldTarget(drive, allowed, now);
        }

        void missingFrame(long now) throws Exception {
            set(vision, Vision.class, "correctedRobotFieldPose", null);
            set(vision, Vision.class, "frameTimestamp", Double.NaN);
            vision.updateFieldTarget(drive, true, now);
        }
    }

    public static void main(String[] args) throws Exception {
        for (boolean stopped : new boolean[]{false, true}) {
            for (boolean rt : new boolean[]{false, true}) {
                Samples sample = new Samples();
                Pose2D previous = sample.pinpoint.position;
                sample.prepare(100, 200, 90);
                sample.vision.updateAimTarget(sample.drive, stopped, rt);
                boolean expected = !stopped && !rt;
                check(sample.pinpoint.writes == (expected ? 1 : 0),
                        "RT and B block writes; otherwise calibration is automatic");
                check(sample.drive.isFieldPoseInitialized() == expected,
                        "RT exclusion also applies before field initialization");
                if (!expected) {
                    check(sample.pinpoint.position == previous, "blocked calibration preserves XYH");
                }
            }
        }

        Samples normal = new Samples();
        normal.frame(200, -30, 90, true, 0);
        check(normal.pinpoint.writes == 1 && normal.drive.isFieldPoseInitialized(),
                "the first valid vision pose initializes an arbitrary field location");
        Pose2D initial = normal.pinpoint.position;
        normal.frame(201, -29, 91, true, 1_499_999_999L);
        check(normal.pinpoint.writes == 1 && normal.pinpoint.position == initial,
                "one nanosecond before 1.5 seconds, XYH remains untouched");
        check(normal.vision.getCalibrationCooldownRemainingSeconds() > 0,
                "success cooldown remains active before the deadline");
        normal.frame(201, -29, 91, true, 1_500_000_000L);
        check(normal.pinpoint.writes == 2, "exactly 1.5 seconds permits the next correction");
        normal.vision.updateFieldTarget(normal.drive, true, 3_000_000_000L);
        check(normal.pinpoint.writes == 2, "a repeated physical frame cannot write twice");
        check(normal.vision.getCalibrationCooldownRemainingSeconds() == 0,
                "a duplicate frame does not restart the success cooldown");
        normal.frame(202, -28, 92, true, 3_000_000_001L);
        check(normal.pinpoint.writes == 3, "the next distinct frame retries immediately");
        normal.frame(203, -27, 93, false, 3_200_000_000L);
        normal.frame(203, -27, 93, false, 5_000_000_000L);
        check(normal.pinpoint.writes == 3, "paused calibration never writes");
        normal.frame(203, -27, 93, true, 5_000_000_001L);
        check(normal.pinpoint.writes == 4, "elapsed cooldown does not restart on RT release");

        Samples retry = new Samples();
        retry.frame(100, 100, -30, true, 0);
        Pose2D previous = retry.pinpoint.position;
        retry.frame(110, 100, -30, true, 1_500_000_000L);
        check(retry.pinpoint.writes == 1 && retry.pinpoint.position == previous,
                "an out-of-gate frame never overwrites existing XYH");
        check(!retry.vision.isCorrectedPoseAccepted(), "a rejected frame is not visually accepted");
        check(retry.vision.getCalibrationCooldownRemainingSeconds() == 0,
                "a failed correction starts no cooldown");
        retry.frame(104.9, 100, -25.1, true, 1_500_000_001L);
        check(retry.pinpoint.writes == 2, "a failed correction retries on the very next fresh frame");

        for (double[] candidate : new double[][]{
                {4.999, 0, 4.999, 1},
                {5, 0, 0, 0},
                {5.001, 0, 0, 0},
                {0, 0, 5, 0},
                {0, 0, 5.001, 0},
                {4, 4, 0, 0},
                {3, 4, 0, 0},
                {-4.999, 0, -4.999, 1},
                {0, -5, 0, 0},
                {0, 0, -5, 0}}) {
            Samples boundary = new Samples();
            boundary.frame(0, 0, 0, true, 0);
            Pose2D before = boundary.pinpoint.position;
            boundary.frame(candidate[0], candidate[1], candidate[2], true, 1_500_000_000L);
            boolean accepted = candidate[3] == 1;
            check(boundary.pinpoint.writes == (accepted ? 2 : 1),
                    "strict 5 cm / 5 degree boundary: " + java.util.Arrays.toString(candidate));
            if (!accepted) {
                check(boundary.pinpoint.position == before, "rejection preserves every XYH component");
            }
        }

        Samples jumps = new Samples();
        jumps.frame(0, 0, 0, true, 0);
        Pose2D beforeJumps = jumps.pinpoint.position;
        for (int i = 1; i <= 20; i++) {
            jumps.frame(100, 100, 90, true, 1_500_000_000L + i);
            check(jumps.pinpoint.writes == 1 && jumps.pinpoint.position == beforeJumps,
                    "repeated rejected frames must never force a Pinpoint write");
        }
        check(jumps.drive.getRejectedVisionFrameCount() == 20,
                "distinct rejected frames are counted without an override");
        jumps.vision.updateFieldTarget(jumps.drive, true, 1_500_000_030L);
        check(jumps.drive.getRejectedVisionFrameCount() == 20,
                "a duplicate rejected frame cannot advance the counter");
        jumps.frame(1, 1, 1, true, 1_500_000_031L);
        check(jumps.pinpoint.writes == 2, "a valid frame immediately recovers after many rejections");

        Samples wrapped = new Samples();
        wrapped.frame(0, 0, 179, true, 0);
        wrapped.frame(0, 0, -179, true, 1_500_000_000L);
        check(wrapped.pinpoint.writes == 2, "heading wrap uses the shortest two-degree difference");
        wrapped.frame(0, 0, -174, true, 3_000_000_000L);
        check(wrapped.pinpoint.writes == 2, "exactly five degrees is rejected after heading wrap");

        Samples missing = new Samples();
        missing.frame(100, 100, 45, true, 0);
        missing.missingFrame(1_500_000_000L);
        check(missing.pinpoint.writes == 1 && missing.vision.hasValidAimingTarget(),
                "missing vision preserves Pinpoint and coordinate aiming");
        missing.frame(101, 100, 46, true, 1_500_000_001L);
        check(missing.pinpoint.writes == 2, "missing vision adds no retry delay");

        Samples invalid = new Samples();
        Pose2D beforeInvalid = invalid.pinpoint.position;
        invalid.frame(Double.NaN, 0, 0, true, 0);
        check(invalid.pinpoint.writes == 0 && invalid.pinpoint.position == beforeInvalid,
                "a nonfinite pose cannot overwrite uninitialized odometry");
        check(invalid.vision.getCalibrationCooldownRemainingSeconds() == 0,
                "failed initialization starts no cooldown");
        invalid.frame(150, -100, 45, true, 1);
        check(invalid.pinpoint.writes == 1, "a valid frame can initialize immediately after failure");
        invalid.pinpoint.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        invalid.frame(151, -100, 46, true, 1_500_000_001L);
        check(invalid.pinpoint.writes == 1 && !invalid.vision.hasValidAimingTarget(),
                "Pinpoint not READY blocks writes and aiming");
        invalid.pinpoint.status = GoBildaPinpointDriver.DeviceStatus.READY;
        invalid.frame(151, -100, 46, true, 1_500_000_002L);
        check(invalid.pinpoint.writes == 2, "READY recovery retries without a failure cooldown");

        System.out.println("PASS VisionCalibrationCheck: " + checks
                + " production RT, cooldown, strict gate and no-overwrite assertions");
    }
}
