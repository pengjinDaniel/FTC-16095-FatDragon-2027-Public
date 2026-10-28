package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import sun.misc.Unsafe;

/** Replays visual HIVE geometry through unconditional Pinpoint calibration and odometry continuation. */
public class HiveGeometryCheck {
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

    private static void close(double actual, double expected, double tolerance, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
                message + ": expected " + expected + ", got " + actual);
    }

    private static void set(Object target, Class<?> owner, String name, Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Pose2D pose(double x, double y, double heading) {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    private static VisionMath.HiveSolveResult solve(double x, double y, double heading, double theta) {
        return solve(x, y, heading, theta, 0, 0);
    }

    private static VisionMath.HiveSolveResult solve(
            double x, double y, double heading, double theta, double mapXErrorMeters, double mapZErrorMeters
    ) {
        double phi = Math.toRadians(-theta);
        double yaw = Math.toRadians(heading);
        double height = 43.9497007874; // Independent official CAD pivot, not the production constant.
        Position position = new Position(DistanceUnit.INCH,
                Math.cos(phi) * x - height * Math.sin(phi) + mapXErrorMeters / .0254, y,
                height - Math.sin(phi) * x - height * Math.cos(phi) + mapZErrorMeters / .0254, 0);
        YawPitchRollAngles orientation = new YawPitchRollAngles(AngleUnit.RADIANS,
                Math.atan2(Math.sin(yaw), Math.cos(phi) * Math.cos(yaw)),
                Math.asin(Math.sin(phi) * Math.cos(yaw)),
                Math.atan2(Math.sin(phi) * Math.sin(yaw), Math.cos(phi)), 0);
        return VisionMath.solveHiveFlatPose(new Pose3D(position, orientation),
                VisionConstants.hivePivotHeightInches,
                VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees);
    }

    private static VisionMath.HiveSolveResult noisyMirror() {
        Pose3D raw = new Pose3D(new Position(DistanceUnit.METER,
                -0.4921738642316256, 1.016, 0.11145986099331764, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 90, 0, 26, 0));
        return VisionMath.solveHiveFlatPose(raw, VisionConstants.hivePivotHeightInches,
                VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees);
    }

    private static final class RuntimePath {
        final Vision vision;
        final Drive drive;
        final VisionCalibrationCheck.FakePinpoint pinpoint;

        RuntimePath(Alliance alliance) throws Exception {
            vision = (Vision) ALLOCATOR.allocateInstance(Vision.class);
            drive = (Drive) ALLOCATOR.allocateInstance(Drive.class);
            pinpoint = (VisionCalibrationCheck.FakePinpoint)
                    ALLOCATOR.allocateInstance(VisionCalibrationCheck.FakePinpoint.class);
            pinpoint.position = pose(0, 0, 0);
            pinpoint.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
            set(drive, Drive.class, "pinpoint", pinpoint);
            set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
            set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
            set(vision, Vision.class, "alliance", alliance);
            set(vision, Vision.class, "observedClusterStates", Collections.emptyMap());
        }

        void frame(VisionMath.HiveSolveResult solve, double timestamp, boolean allowed, long now)
                throws Exception {
            set(vision, Vision.class, "hiveSolveResult", solve);
            set(vision, Vision.class, "frameTimestamp", timestamp);
            vision.updateFieldTarget(drive, allowed, now);
        }

        void calibrate(VisionMath.HiveSolveResult solve, double timestamp, long now) throws Exception {
            for (int i = 0; i < 3; i++) {
                frame(solve, timestamp - (2 - i) * .1, true, now + i * 20_000_000L);
            }
        }
    }

    public static void main(String[] args) throws Exception {
        VisionMath.HiveSolveResult visual = noisyMirror();
        check(visual.usableCandidateCount == 1 && visual.correction != null,
                "full visual orientation provides one pose without a Pinpoint reference");
        close(visual.correction.robotPose.getHeading(AngleUnit.DEGREES), 90, 1e-6,
                "visual correction preserves heading");
        close(visual.correction.hiveAngleDegrees, -26, 1e-6,
                "visual HIVE tilt uses both pitch and roll");
        check(Math.abs(visual.correction.robotPose.getX(DistanceUnit.INCH)) < .2,
                "orientation noise does not select the old mirrored floor root");

        for (double x : new double[]{-30, -3, 0, 3, 30}) {
            for (double heading : new double[]{-180, -135, -90, -45, 0, 45, 90, 135}) {
                for (double theta : new double[]{-35, -30, 0, 30, 35}) {
                    VisionMath.HiveSolveResult result = solve(x, 40, heading, theta);
                    VisionMath.HiveCorrection corrected =
                            result.correction;
                    check(corrected != null, "consistent CAD pose must be solvable: x=" + x + ", h=" + heading
                            + ", tilt=" + theta + ", status=" + result.status
                            + ", roots=" + java.util.Arrays.toString(result.candidatePhiDegrees));
                    close(corrected.robotPose.getX(DistanceUnit.INCH), x, .001, "CAD pose X");
                    if (x == 0) {
                        check(CellCluster.forFieldX(Alliance.BLUE,
                                corrected.robotPose.getX(DistanceUnit.INCH)).get()
                                == CellCluster.BLUE_AUDIENCE,
                                "floating-point X=0 must keep the deterministic audience CELL");
                    }
                    close(Math.IEEEremainder(corrected.robotPose.getHeading(AngleUnit.DEGREES)
                            - heading, 360), 0, .001, "CAD pose heading");
                }
            }
        }

        for (double theta : new double[]{-35, 35}) {
            for (double xCm : new double[]{-5, -2, 0, 2, 5}) {
                for (double mapXError : new double[]{-2e-8, 2e-8}) {
                    VisionMath.HiveSolveResult result =
                            solve(xCm / 2.54, 40, 90, theta, mapXError, 5e-10);
                    VisionMath.HiveCorrection correction = result.correction;
                    if (xCm == 0) {
                        check(correction == null || correction.robotPose.getX(DistanceUnit.INCH) == 0,
                                "center-line map rounding must wait or preserve deterministic X=0");
                    } else {
                        check(correction != null
                                && correction.robotPose.getX(DistanceUnit.INCH) * xCm > 0,
                                "map rounding at the tilt boundary must not leave only the mirrored root");
                    }
                }
            }
        }
        check(solve(30, 40, 90, 36).usableCandidateCount == 0,
                "the numerical tolerance must still reject a genuinely excessive HIVE tilt");

        for (double side : new double[]{-1, 1}) {
            double[] reference = {side * 16.52, 12.75};
            for (double[] sample : new double[][]{
                    {-30, 16.519923}, {0, 21.456559}, {30, 20.643755}}) {
                double[] target = VisionMath.cellTargetForHiveTilt(reference,
                        VisionConstants.cellTargetReferenceHeightInches,
                        VisionConstants.hivePivotHeightInches,
                        VisionConstants.cellTargetReferenceTiltDegrees, side * sample[0]);
                close(target[0], side * sample[1], .002, "independent CAD/SDK target X");
                close(target[1], 12.75, 1e-9, "rotation preserves CELL Y");
            }
        }

        for (CellCluster cluster : CellCluster.values()) {
            double side = cluster == CellCluster.RED_AUDIENCE || cluster == CellCluster.BLUE_AUDIENCE
                    ? 1.0 : -1.0;
            double tagY = cluster.isForAlliance(Alliance.BLUE) ? 12.75 : -12.75;
            // Independent centroids of the four fmap translations, rotated about the CAD pivot.
            for (double[] sample : new double[][]{
                    {0.0, 14.2690594488}, {-30.0, 13.1064522228}, {30.0, 11.6082837189}}) {
                double[] center = VisionMath.tagClusterCenterForHiveTilt(cluster, side * sample[0]);
                close(center[0], side * sample[1], .0001, "four-tag center follows HIVE tilt");
                close(center[1], tagY, 1e-9, "four-tag center keeps alliance Y");
            }
            double[] fallback = VisionMath.tagClusterCenterForHiveTilt(cluster, Double.NaN);
            close(fallback[0], side * 13.1064522228, .0001, "unknown tilt uses nominal HIGH geometry");
        }
        check(VisionMath.tagClusterCenterForHiveTilt(null, 0.0) == null,
                "missing CELL has no four-tag center");

        for (Alliance alliance : Alliance.values()) {
            for (double side : new double[]{-1, 1}) {
                for (double distanceCm : new double[]{67.0, 89.5, 112.0}) {
                    RuntimePath runtime = new RuntimePath(alliance);
                    double x = side * 13.1064522228;
                    double y = (alliance == Alliance.BLUE ? 1 : -1) * (12.75 + distanceCm / 2.54);
                    check(runtime.drive.updateVisionPose(pose(x, y, 90), 1, 0), "measured pose initializes");
                    runtime.frame(solve(x, y, 90, -side * 30.0), 2, false, 0);
                    close(DistanceUnit.INCH.toCm(runtime.vision.getTagCenterHorizontalDistanceInches()),
                            distanceCm, .002, "production shot distance matches the four-tag measurement");
                    close(runtime.vision.getTargetXInches(), side * 16.52, .002,
                            "aiming retains the CELL opening center");
                }
            }
        }

        RuntimePath cold = new RuntimePath(Alliance.BLUE);
        cold.calibrate(solve(0, 0, 180, -30), 1, 0);
        check(cold.drive.isFieldPoseInitialized() && cold.vision.hasValidAimingTarget(),
                "center field coordinates initialize from the visual pose");
        close(cold.drive.getPose().getX(DistanceUnit.INCH), 0, .001, "center X");
        close(cold.drive.getPose().getY(DistanceUnit.INCH), 0, .001, "center Y");
        close(Math.abs(cold.drive.getPose().getHeading(AngleUnit.DEGREES)), 180, .001,
                "opposite-audience intake heading remains 180 degrees");

        RuntimePath warm = new RuntimePath(Alliance.BLUE);
        warm.drive.updateVisionPose(pose(100, 40, 0), 1, 0);
        warm.calibrate(solve(-30, 40, 180, 30), 2, 100_000_000L);
        close(warm.drive.getPose().getX(DistanceUnit.INCH), -30, .001,
                "a previous wrong pose cannot reject current Limelight localization");
        check(warm.vision.getSelectedCluster().get() == CellCluster.BLUE_OPPOSITE_AUDIENCE,
                "a confirmed visual position changes the selected half");

        for (Alliance alliance : Alliance.values()) {
            RuntimePath runtime = new RuntimePath(alliance);
            double y = alliance == Alliance.BLUE ? 40 : -40;
            runtime.calibrate(solve(30, y, 180, -30), 1, 0);
            check(runtime.drive.isFieldPoseInitialized(), "three stable visual frames initialize");
            close(runtime.vision.getTargetXInches(), 16.52, .002, "HIGH opening reference");
            runtime.frame(solve(30, y, 180, 0), 2, false, 100_000_000L);
            check(!runtime.drive.hasAcceptedVisionFrame(2), "B stop pauses visual pose overwrites");
            close(runtime.vision.getTargetXInches(), 21.456559, .002,
                    "the aim point follows HIVE tilt while pose writes are paused");
            check(runtime.vision.getTargetHiveState() == HiveState.MIDDLE,
                    "HIVE state follows the current visual observation");
            Method clear = Vision.class.getDeclaredMethod("clearCurrentFrame");
            clear.setAccessible(true);
            clear.invoke(runtime.vision);
            runtime.vision.updateFieldTarget(runtime.drive, false, 200_000_000L);
            check(runtime.drive.isFieldPoseInitialized() && runtime.vision.hasValidAimingTarget(),
                    "tag loss preserves initialized odometry and aiming");
            check(Double.isFinite(runtime.vision.getTagCenterHorizontalDistanceInches()),
                    "tag loss retains shot distance from cached HIVE geometry");
            runtime.pinpoint.position = pose(-30, y, 180);
            runtime.frame(solve(-30, y, 180, 30), 3, false, 300_000_000L);
            check(!runtime.drive.hasAcceptedVisionFrame(3),
                    "a paused overwrite keeps using odometry");
            close(runtime.vision.getTargetXInches(), -16.52, .002,
                    "crossing the field switches to the opposite opening");
            check(runtime.vision.getSelectedCluster().get()
                            == CellCluster.forFieldX(alliance, -30).get(),
                    "Red and Blue both select the current odometry half");
        }
        System.out.println("PASS HiveGeometryCheck: " + checks
                + " visual-orientation, independent CAD/SDK, B-stop exclusion and odometry-continuation assertions");
    }
}
