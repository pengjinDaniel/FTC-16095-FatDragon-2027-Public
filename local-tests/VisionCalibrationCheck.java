package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.lang.reflect.Field;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import sun.misc.Unsafe;

/** Checks the October 9 calibration policy using the production Vision and Drive methods. */
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

    private static Pose2D pose(double x, double heading) {
        return new Pose2D(DistanceUnit.INCH, x, 0, AngleUnit.DEGREES, heading);
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
            pinpoint.position = pose(0, 0);
            pinpoint.status = GoBildaPinpointDriver.DeviceStatus.READY;
            set(drive, Drive.class, "pinpoint", pinpoint);
            set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
            set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
            set(drive, Drive.class, "lastAcceptedVisionTimestamp", Double.NaN);
            set(vision, Vision.class, "alliance", Alliance.BLUE);
            set(vision, Vision.class, "observedClusterStates", Collections.emptyMap());
        }

        void frame(double x, double heading, boolean started, boolean stopped, boolean requested)
                throws Exception {
            set(vision, Vision.class, "correctedRobotFieldPose", pose(x, heading));
            set(vision, Vision.class, "frameTimestamp", ++timestamp);
            vision.updateAimTarget(drive, started, stopped, requested);
        }

        void missingFrame() throws Exception {
            set(vision, Vision.class, "correctedRobotFieldPose", null);
            set(vision, Vision.class, "frameTimestamp", Double.NaN);
            vision.updateAimTarget(drive, true, false, true);
        }
    }

    public static void main(String[] args) throws Exception {
        for (boolean started : new boolean[]{false, true}) {
            for (boolean stopped : new boolean[]{false, true}) {
                for (boolean requested : new boolean[]{false, true}) {
                    Samples sample = new Samples();
                    sample.frame(4, 0, started, stopped, requested);
                    boolean expected = !started || (!stopped && requested);
                    check(sample.pinpoint.writes == (expected ? 1 : 0),
                            "INIT always calibrates; after START a request is required and B blocks it");
                    check(sample.vision.isCorrectedPoseAccepted() == expected,
                            "visual acceptance follows the physical frame actually written");
                }
            }
        }

        Samples normal = new Samples();
        normal.frame(4, 0, false, false, false);
        check(normal.pinpoint.writes == 1 && normal.drive.isFieldPoseInitialized(),
                "the first usable frame initializes without a consecutive-frame prerequisite");
        normal.frame(5, 0, true, false, false);
        check(normal.pinpoint.writes == 1 && normal.vision.hasValidAimingTarget(),
                "unrequested calibration leaves Pinpoint aiming active");
        check(!normal.vision.isCorrectedPoseAccepted(), "an unwritten new frame is not accepted");
        normal.frame(5, 0, true, false, true);
        normal.frame(6, 0, true, false, true);
        check(normal.pinpoint.writes == 3, "success adds no cooldown to the next requested fresh frame");
        normal.vision.updateAimTarget(normal.drive, true, false, true);
        check(normal.pinpoint.writes == 3 && normal.vision.isCorrectedPoseAccepted(),
                "re-reading an accepted frame neither writes again nor loses its acceptance");

        Samples jumps = new Samples();
        jumps.frame(4, 0, false, false, false);
        jumps.frame(30, 90, true, false, true);
        check(jumps.pinpoint.writes == 1 && jumps.drive.getRejectedVisionFrameCount() == 1,
                "the first out-of-gate frame is rejected");
        jumps.vision.updateAimTarget(jumps.drive, true, false, true);
        check(jumps.drive.getRejectedVisionFrameCount() == 1,
                "one repeated physical frame cannot advance the override counter");
        jumps.frame(30, 90, true, false, true);
        check(jumps.pinpoint.writes == 1 && jumps.drive.getRejectedVisionFrameCount() == 2,
                "the second fresh out-of-gate frame remains rejected");
        jumps.frame(30, 90, true, false, true);
        check(jumps.pinpoint.writes == 2 && jumps.drive.wasVisionCorrectionForced(),
                "the third fresh out-of-gate frame restores the October 9 forced override");
        check(jumps.drive.hasAcceptedVisionFrame(jumps.timestamp),
                "the forced frame is tracked as the accepted frame");
        jumps.frame(30, 90, true, false, true);
        check(jumps.pinpoint.writes == 3 && !jumps.drive.wasVisionCorrectionForced(),
                "the next in-gate frame writes normally and clears the forced flag");

        Samples interrupted = new Samples();
        interrupted.frame(4, 0, false, false, false);
        interrupted.frame(30, 0, true, false, true);
        interrupted.frame(30, 0, true, false, true);
        interrupted.missingFrame();
        check(interrupted.drive.getRejectedVisionFrameCount() == 0,
                "missing corrected vision resets the rejection sequence");
        check(interrupted.vision.hasValidAimingTarget(),
                "losing AprilTags preserves coordinate aiming from initialized Pinpoint");
        interrupted.frame(30, 0, true, false, true);
        check(interrupted.pinpoint.writes == 1 && interrupted.drive.getRejectedVisionFrameCount() == 1,
                "a new rejection sequence cannot inherit the earlier two rejected frames");

        Samples boundary = new Samples();
        boundary.frame(0, 0, false, false, false);
        boundary.frame(12, 30, true, false, true);
        check(boundary.pinpoint.writes == 2 && !boundary.drive.wasVisionCorrectionForced(),
                "12 inches and 30 degrees are inclusive correction boundaries");

        Samples wrapped = new Samples();
        wrapped.frame(4, 179, false, false, false);
        wrapped.frame(4, -179, true, false, true);
        check(wrapped.pinpoint.writes == 2, "heading wrap is a two-degree difference");

        Samples invalid = new Samples();
        invalid.frame(Double.NaN, 0, false, false, false);
        check(invalid.pinpoint.writes == 0 && !invalid.drive.isFieldPoseInitialized(),
                "a nonfinite corrected pose cannot initialize localization");
        invalid.frame(4, 0, false, false, false);
        check(invalid.pinpoint.writes == 1, "the next valid frame can retry immediately");
        System.out.println("PASS VisionCalibrationCheck: " + checks
                + " production calibration, frame identity and override assertions");
    }
}
