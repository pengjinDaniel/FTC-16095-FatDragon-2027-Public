package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Forward-projects sphere silhouettes to test held-ball ranging and rear-facing control. */
public final class PollenChaseCheck {
    private static int checks;
    private static final double CAMERA_FORWARD = -12.715;
    private static final double CAMERA_HEIGHT = 20.7415;
    private static final double PITCH_UP = 50.0;
    private static final double YAW = 180.0;
    private static final double EDGE = 17.0; // Reported center-to-rear-edge distance.

    private static final class Sample {
        double tx, ty;
        List<List<Double>> corners;
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void close(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-7,
                message + ": expected " + expected + ", got " + actual);
    }

    private static PollenChaseController controller(double yaw, double cameraRight) {
        return new PollenChaseController(CAMERA_FORWARD, cameraRight, CAMERA_HEIGHT,
                PITCH_UP, yaw, EDGE, 1.0);
    }

    private static double centeredHeight(double gap, double yaw) {
        double sign = Math.cos(Math.toRadians(yaw)) < 0.0 ? -1.0 : 1.0;
        double cameraHorizontal = EDGE + PollenChaseController.POLLEN_RADIUS_CM + gap
                - sign * CAMERA_FORWARD;
        return CAMERA_HEIGHT + cameraHorizontal * Math.tan(Math.toRadians(PITCH_UP));
    }

    private static Sample project(double gap, double approachRight, double height,
                                  double yawDegrees, double cameraRight) {
        double yaw = Math.toRadians(yawDegrees), pitch = Math.toRadians(PITCH_UP);
        double sign = Math.cos(yaw) < 0.0 ? -1.0 : 1.0;
        double worldForward = sign * (EDGE + PollenChaseController.POLLEN_RADIUS_CM + gap);
        double worldRight = sign * approachRight;
        double deltaForward = worldForward - CAMERA_FORWARD;
        double deltaRight = worldRight - cameraRight;
        double deltaUp = height - CAMERA_HEIGHT;
        double cameraHorizontal = Math.cos(yaw) * deltaForward + Math.sin(yaw) * deltaRight;
        double opticalRight = -Math.sin(yaw) * deltaForward + Math.cos(yaw) * deltaRight;
        double opticalForward = Math.cos(pitch) * cameraHorizontal + Math.sin(pitch) * deltaUp;
        double opticalUp = -Math.sin(pitch) * cameraHorizontal + Math.cos(pitch) * deltaUp;

        // A tangent to the sphere defines each bounding edge of its perspective silhouette.
        double xAngle = Math.atan2(opticalRight, opticalForward);
        double yAngle = Math.atan2(opticalUp, opticalForward);
        double xRadius = Math.asin(PollenChaseController.POLLEN_RADIUS_CM
                / Math.hypot(opticalRight, opticalForward));
        double yRadius = Math.asin(PollenChaseController.POLLEN_RADIUS_CM
                / Math.hypot(opticalUp, opticalForward));
        double lowX = Math.tan(xAngle - xRadius), highX = Math.tan(xAngle + xRadius);
        double lowY = Math.tan(yAngle - yRadius), highY = Math.tan(yAngle + yRadius);
        double minX = 320.0 + PollenChaseController.FOCAL_X_PIXELS * lowX;
        double maxX = 320.0 + PollenChaseController.FOCAL_X_PIXELS * highX;
        double minY = 240.0 - PollenChaseController.FOCAL_Y_PIXELS * highY;
        double maxY = 240.0 - PollenChaseController.FOCAL_Y_PIXELS * lowY;
        Sample sample = new Sample();
        sample.tx = Math.toDegrees(Math.atan((lowX + highX) / 2.0));
        sample.ty = Math.toDegrees(Math.atan((lowY + highY) / 2.0));
        sample.corners = Arrays.asList(Arrays.asList(minX, minY), Arrays.asList(maxX, minY),
                Arrays.asList(maxX, maxY), Arrays.asList(minX, maxY));
        return sample;
    }

    private static void update(PollenChaseController chase, Sample sample, int frame, long ms) {
        chase.update(true, frame, 10.0, sample.tx, sample.ty, sample.corners, ms * 1_000_000L);
    }

    private static void acquire(PollenChaseController chase, Sample sample) {
        for (int frame = 1; frame <= 3; frame++) update(chase, sample, frame, frame * 20L);
    }

    private static void stopped(PollenChaseController chase, String message) {
        close(chase.getForwardPower(), 0.0, message + " forward");
        close(chase.getTurnPower(), 0.0, message + " turn");
    }

    public static void main(String[] args) {
        PollenChaseController chase = controller(YAW, 0.0);
        check(chase.getConfigurationError() == null, "upward 50 degrees and yaw 180 are supported");
        Sample centered = project(80.0, 0.0, centeredHeight(80.0, YAW), YAW, 0.0);
        update(chase, centered, 1, 20L);
        stopped(chase, "first frame does not move");
        update(chase, centered, 1, 25L);
        update(chase, centered, 1, 30L);
        stopped(chase, "repeated HTTP results do not acquire a physical target");
        update(chase, centered, 2, 40L);
        stopped(chase, "two fresh frames do not move");
        update(chase, centered, 3, 60L);
        close(chase.getGapCm(), 80.0, "rear edge, camera translation and ball radius are applied");
        close(chase.getBallHeightCm(), centeredHeight(80.0, YAW), "raised ball height is estimated");
        close(chase.getForwardPower(), -0.50,
                "far rear target reaches the 0.50 reverse power limit");
        close(chase.getTurnPower(), 0.0, "a centered rear target does not turn");

        for (double height : new double[] {105.0, 165.0}) {
            chase = controller(YAW, 0.0);
            acquire(chase, project(80.0, 0.0, height, YAW, 0.0));
            close(chase.getGapCm(), 80.0, "different held heights preserve horizontal distance");
            close(chase.getBallHeightCm(), height, "height is not constrained to a ground-ball plane");
            check(chase.getForwardPower() < 0.0, "both raised targets remain chaseable");
        }

        chase = controller(YAW, 0.0);
        acquire(chase, project(40.0, 0.0, 55.0, YAW, 0.0));
        update(chase, project(15.0, 0.0, 55.0, YAW, 0.0), 4, 80L);
        check(chase.getForwardPower() < 0.0, "15 cm continues toward the 3 cm setpoint");
        update(chase, project(13.0, 0.0, 55.0, YAW, 0.0), 5, 100L);
        close(chase.getForwardPower(), -0.12, "reverse approach slows near the target");
        update(chase, project(3.1, 0.0, 35.0, YAW, 0.0), 6, 120L);
        close(chase.getForwardPower(), -0.07, "friction floor permits final approach");
        update(chase, project(3.0, 0.0, 35.0, YAW, 0.0), 7, 140L);
        check(chase.getState() == PollenChaseController.State.ARRIVED, "stop at exactly 3 cm");
        stopped(chase, "3 cm stops translation and rotation");
        update(chase, centered, 8, 160L);
        stopped(chase, "arrival stays latched when a new/farther target appears");
        chase.update(false, 9.0, 0.0, 0.0, 0.0, null, 180_000_000L);
        check(chase.getState() == PollenChaseController.State.ARRIVED,
                "arrival stays latched through target loss");

        chase = controller(YAW, 0.0);
        acquire(chase, project(80.0, 30.0, centeredHeight(80.0, YAW), YAW, 0.0));
        close(chase.getForwardPower(), 0.0, "large bearing aligns before reversing");
        check(chase.getTurnPower() > 0.0, "rear-camera image-right produces a positive chassis turn");
        update(chase, project(80.0, -30.0, centeredHeight(80.0, YAW), YAW, 0.0), 4, 80L);
        check(chase.getTurnPower() < 0.0, "rear-camera image-left reverses the chassis turn");

        chase = controller(YAW, 5.0);
        acquire(chase, project(80.0, 0.0, centeredHeight(80.0, YAW), YAW, 5.0));
        close(chase.getBearingDegrees(), 0.0, "camera lateral offset aims at robot centerline");
        close(chase.getGapCm(), 80.0, "camera lateral offset preserves the stopping gap");
        chase = controller(0.0, 0.0);
        acquire(chase, project(80.0, 0.0, centeredHeight(80.0, 0.0), 0.0, 0.0));
        check(chase.getForwardPower() > 0.0, "yaw 0 produces forward motion");

        chase = controller(YAW, 0.0);
        acquire(chase, centered);
        chase.update(false, 4.0, 0.0, centered.tx, centered.ty, centered.corners, 80_000_000L);
        stopped(chase, "lost/invalid/wrong-pipeline target stops immediately");
        update(chase, centered, 5, 100L);
        stopped(chase, "target loss requires reacquisition");
        chase.reset();
        acquire(chase, centered);
        chase.update(true, 4.0, 101.0, centered.tx, centered.ty, centered.corners, 80_000_000L);
        check(chase.getState() == PollenChaseController.State.STALE_FRAME, "old data is rejected");
        stopped(chase, "stale data stops");
        chase.reset();
        acquire(chase, centered);
        chase.update(true, 3.0, 0.0, centered.tx, centered.ty, centered.corners, 161_000_000L);
        check(chase.getState() == PollenChaseController.State.FROZEN_FRAME,
                "fresh HTTP reception cannot hide a frozen physical frame");
        stopped(chase, "frozen data stops");

        for (List<List<Double>> badCorners : Arrays.<List<List<Double>>>asList(
                null, Collections.emptyList(),
                Arrays.asList(Arrays.asList(0.0, 100.0), Arrays.asList(50.0, 100.0),
                        Arrays.asList(50.0, 150.0), Arrays.asList(0.0, 150.0)),
                Arrays.asList(Arrays.asList(Double.NaN, 100.0), Arrays.asList(350.0, 100.0),
                        Arrays.asList(350.0, 150.0), Arrays.asList(300.0, 150.0)))) {
            chase = controller(YAW, 0.0);
            for (int frame = 1; frame <= 3; frame++) {
                chase.update(true, frame, 0.0, 0.0, 0.0, badCorners, frame * 20_000_000L);
            }
            check(chase.getState() == PollenChaseController.State.INVALID_CONTOUR,
                    "missing, clipped and nonfinite contours cannot supply distance");
            stopped(chase, "invalid contour stops");
        }
        chase = controller(YAW, 0.0);
        acquire(chase, centered);
        chase.update(true, 4.0, 0.0, Double.NaN, 0.0, centered.corners, 80_000_000L);
        stopped(chase, "nonfinite angles stop");

        PollenChaseController[] invalid = {
                new PollenChaseController(CAMERA_FORWARD, 0.0, CAMERA_HEIGHT, PITCH_UP, YAW, Double.NaN, 1.0),
                new PollenChaseController(CAMERA_FORWARD, 0.0, CAMERA_HEIGHT, Double.NaN, YAW, EDGE, 1.0),
                new PollenChaseController(CAMERA_FORWARD, 0.0, CAMERA_HEIGHT, PITCH_UP, 90.0, EDGE, 1.0),
                new PollenChaseController(CAMERA_FORWARD, 0.0, CAMERA_HEIGHT, PITCH_UP, YAW, EDGE, 0.0)
        };
        for (PollenChaseController invalidSetup : invalid) {
            acquire(invalidSetup, centered);
            check(invalidSetup.getConfigurationError() != null, "invalid setup is explained");
            stopped(invalidSetup, "unconfigured geometry cannot drive");
        }
        System.out.println("PASS PollenChaseCheck: " + checks + " held-ball geometry and motion-gating assertions");
    }
}
