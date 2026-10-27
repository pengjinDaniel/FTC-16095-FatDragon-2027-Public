package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

/** Replays mirrored HIVE roots and SDK/CAD opening geometry through the production vision path. */
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
            pinpoint.status = GoBildaPinpointDriver.DeviceStatus.READY;
            set(drive, Drive.class, "pinpoint", pinpoint);
            set(drive, Drive.class, "driverHeadingReferenceRadians", Double.NaN);
            set(drive, Drive.class, "lastVisionTimestamp", Double.NaN);
            set(drive, Drive.class, "lastAcceptedVisionTimestamp", Double.NaN);
            set(vision, Vision.class, "alliance", alliance);
            set(vision, Vision.class, "observedClusterStates", Collections.emptyMap());
        }

        void frame(VisionMath.HiveSolveResult solve, double timestamp, boolean allowed, long now)
                throws Exception {
            set(vision, Vision.class, "hiveSolveResult", solve);
            set(vision, Vision.class, "frameTimestamp", timestamp);
            vision.updateFieldTarget(drive, allowed, now);
        }
    }

    public static void main(String[] args) throws Exception {
        VisionMath.HiveSolveResult ambiguous = noisyMirror();
        check(ambiguous.usableCandidateCount == 2, "both noisy roots remain geometrically usable");
        check(ambiguous.correction == null, "an ambiguous initial frame must not initialize mirrored X");
        VisionMath.HiveCorrection positive =
                VisionMath.selectHiveCorrection(ambiguous, pose(3, 40, 90));
        close(positive.robotPose.getX(DistanceUnit.INCH), 3, .01, "Pinpoint selects positive X");
        close(positive.robotPose.getHeading(AngleUnit.DEGREES), 90, 1e-6, "heading is preserved");
        VisionMath.HiveCorrection negative =
                VisionMath.selectHiveCorrection(ambiguous, pose(-3, 40, 90));
        close(negative.robotPose.getX(DistanceUnit.INCH), -3, .01, "Pinpoint selects negative X");
        check(VisionMath.selectHiveCorrection(ambiguous, pose(0, 40, 90)) == null,
                "equidistant roots remain unresolved");
        check(VisionMath.selectHiveCorrection(ambiguous, null) == null,
                "a previous selection cannot leak into a frame without a reference");
        check(VisionMath.selectHiveCorrection(ambiguous, pose(Double.NaN, 40, 90)) == null,
                "an invalid reference cannot select a root");

        for (double x : new double[]{-30, -3, 0, 3, 30}) {
            for (double heading : new double[]{-180, -135, -90, -45, 0, 45, 90, 135}) {
                for (double theta : new double[]{-35, -30, 0, 30, 35}) {
                    VisionMath.HiveSolveResult result = solve(x, 40, heading, theta);
                    VisionMath.HiveCorrection corrected =
                            VisionMath.selectHiveCorrection(result, pose(x, 40, heading));
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
                    VisionMath.HiveCorrection correction = VisionMath.selectHiveCorrection(
                            result, pose(xCm / 2.54, 40, 90));
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

        RuntimePath cold = new RuntimePath(Alliance.BLUE);
        cold.frame(noisyMirror(), 1, true, 0);
        check(cold.pinpoint.writes == 0 && !cold.drive.isFieldPoseInitialized(),
                "an ambiguous first frame cannot write Pinpoint");
        check(!cold.vision.hasValidAimingTarget(), "uninitialized ambiguous pose cannot aim");

        RuntimePath warm = new RuntimePath(Alliance.BLUE);
        check(warm.drive.calibrateFieldPose(pose(3, 40, 90), 1), "independent pose initializes Pinpoint");
        warm.frame(noisyMirror(), 2, true, 1_500_000_000L);
        check(warm.pinpoint.position.getX(DistanceUnit.INCH) > 0,
                "the 30 cm gate no longer admits the opposite HIVE root");
        check(warm.vision.getSelectedCluster().get() == CellCluster.BLUE_AUDIENCE,
                "the mirrored-root replay keeps the correct CELL");
        check(warm.vision.getYawErrorDegrees() > 0, "the replay requests the correct left turn");

        for (Alliance alliance : Alliance.values()) {
            RuntimePath runtime = new RuntimePath(alliance);
            double y = alliance == Alliance.BLUE ? 40 : -40;
            runtime.frame(solve(30, y, 90, -30), 1, true, 0);
            check(runtime.pinpoint.writes == 1, "a unique first frame still initializes");
            close(runtime.vision.getTargetXInches(), 16.52, .002, "HIGH opening reference");
            runtime.frame(solve(30, y, 90, 0), 2, false, 100_000_000L);
            check(runtime.pinpoint.writes == 1, "RT continues to block Pinpoint writes");
            close(runtime.vision.getTargetXInches(), 21.456559, .002,
                    "the aim point follows HIVE tilt while RT is held");
            check(runtime.vision.getTargetHiveState() == HiveState.MIDDLE,
                    "HIVE state follows the accepted observation");
            double lastTarget = runtime.vision.getTargetXInches();
            Method clear = Vision.class.getDeclaredMethod("clearCurrentFrame");
            clear.setAccessible(true);
            clear.invoke(runtime.vision);
            runtime.vision.updateFieldTarget(runtime.drive, false, 200_000_000L);
            close(runtime.vision.getTargetXInches(), lastTarget, 1e-9,
                    "tag loss preserves the last known opening angle");
            runtime.frame(solve(100, y, 90, 30), 3, false, 300_000_000L);
            close(runtime.vision.getTargetXInches(), lastTarget, 1e-9,
                    "an out-of-gate observation cannot move the cached target");
            runtime.frame(solve(100, y, 90, 30), 4, true, 1_500_000_000L);
            check(runtime.pinpoint.writes == 1, "an out-of-gate observation still cannot write pose");
            check(runtime.drive.getRejectedVisionFrameCount() == 1,
                    "normal calibration rejection reporting remains intact");
        }
        System.out.println("PASS HiveGeometryCheck: " + checks
                + " mirrored-root, independent CAD/SDK, RT and cached-angle assertions");
    }
}
