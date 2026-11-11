package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchReadHistory;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Drive extends SubsystemBase {
    private static final long DRIVER_HEADING_HOLD_NANOS = 100_000_000L;

    public final DcMotorEx leftFrontMotor;
    public final DcMotorEx leftBackMotor;
    public final DcMotorEx rightFrontMotor;
    public final DcMotorEx rightBackMotor;
    private static final Pose2D UNKNOWN_POSE = new Pose2D(
            DistanceUnit.INCH, Double.NaN, Double.NaN, AngleUnit.RADIANS, Double.NaN);
    private final GoBildaPinpointDriver pinpoint;
    private boolean fieldPoseInitialized;
    private boolean pinpointReadInterrupted;
    private boolean pinpointConfigurationLost;
    private I2cDeviceSynchReadHistory pinpointReadHistory;
    private int pinpointFirmwareVersion;
    private long pinpointReadCount;
    private long pinpointBadReadCount;
    private int pinpointBadReadStreak;
    private long lastPinpointReadStartedNanos;
    private double pinpointReadDurationMs;
    private double pinpointReadIntervalMs;
    private long lastPinpointBadReadNanos;
    private String lastPinpointBadReadReason = "No bad reads";
    private double driverHeadingReferenceRadians = Double.NaN;
    private boolean periodicBadRead;
    private double lastValidDriverHeadingRadians = Double.NaN;
    private long lastValidDriverHeadingNanos;
    private double lastVisionTimestamp = Double.NaN;
    private double lastProcessedVisionTimestamp = Double.NaN;
    private int rejectedVisionFrameCount;
    private String localizationStatus = "Waiting for Limelight field initialization";

    public Drive(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.leftFrontMotorName);
        leftBackMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.leftBackMotorName);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.rightFrontMotorName);
        rightBackMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.rightBackMotorName);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DriveConstants.pinpointName);
        leftFrontMotor.setDirection(DriveConstants.leftFrontMotorDirection);
        leftBackMotor.setDirection(DriveConstants.leftBackMotorDirection);
        rightFrontMotor.setDirection(DriveConstants.rightFrontMotorDirection);
        rightBackMotor.setDirection(DriveConstants.rightBackMotorDirection);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
        pinpointFirmwareVersion = pinpoint.getDeviceVersion();
        if (pinpoint.getDeviceClient() instanceof I2cDeviceSynchReadHistory) {
            pinpointReadHistory = (I2cDeviceSynchReadHistory) pinpoint.getDeviceClient();
            pinpointReadHistory.setHistoryQueueCapacity(2);
        }
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                DriveConstants.pinpointForwardPodDirection, DriveConstants.pinpointStrafePodDirection);
        pinpoint.setOffsets(DriveConstants.pinpointXOffsetMm, DriveConstants.pinpointYOffsetMm,
                DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        stop();
    }

    @Override
    public void periodic() {
        long readStarted = readPinpoint("periodic");
        periodicBadRead = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
        if (periodicBadRead) {
            // Preserve the field registration while the interrupted read awaits validation.
            localizationStatus = "Pinpoint bad read; waiting for valid odometry";
            return;
        }
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpointBadReadStreak = 0;
            clearVisionPose("Pinpoint not ready: " + pinpoint.getDeviceStatus());
            pinpointReadInterrupted = true;
            return;
        }
        if (!isFinitePose(pinpoint.getPosition())) {
            pinpointBadReadStreak = 0;
            clearVisionPose("Pinpoint returned invalid X/Y/H");
            pinpointReadInterrupted = true;
            return;
        }
        if (pinpointReadInterrupted) {
            // Read configuration only after an interruption, not on every control loop.
            double xOffset = pinpoint.getXOffset(DistanceUnit.MM);
            double yOffset = pinpoint.getYOffset(DistanceUnit.MM);
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
                    recordPinpointBadRead("Offset-register read rejected by SDK CRC check (LOCAL_TEST)",
                            readStarted);
                } else {
                    clearVisionPose("Pinpoint not ready during offset verification");
                    return;
                }
                localizationStatus = "Pinpoint bad read; waiting for valid device offsets";
                return;
            }
            if (!Double.isFinite(xOffset) || !Double.isFinite(yOffset)
                    || !Double.isFinite(DriveConstants.pinpointXOffsetMm)
                    || !Double.isFinite(DriveConstants.pinpointYOffsetMm)
                    || Math.abs(xOffset - DriveConstants.pinpointXOffsetMm) > 1e-3
                    || Math.abs(yOffset - DriveConstants.pinpointYOffsetMm) > 1e-3) {
                pinpointConfigurationLost = true;
                clearVisionPose("Pinpoint offsets changed; restart OpMode to restore configuration");
                pinpointBadReadStreak = 0;
                return;
            }
            pinpointReadInterrupted = false;
            localizationStatus = fieldPoseInitialized
                    ? "Pinpoint read recovered; continuing field odometry"
                    : "Pinpoint read recovered; waiting for vision calibration";
        }
        if (!Double.isFinite(driverHeadingReferenceRadians)) {
            resetDriverHeading();
        }
        lastValidDriverHeadingRadians = getDriverHeadingRadians();
        lastValidDriverHeadingNanos = readStarted;
        pinpointBadReadStreak = 0;
    }

    private long readPinpoint(String phase) {
        Pose2D previousPose = null;
        double previousHeading = Double.NaN;
        if (pinpointReadHistory != null) {
            previousPose = pinpoint.getPosition();
            previousHeading = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
            pinpointReadHistory.getHistoryQueue().clear();
        }
        long readStarted = System.nanoTime();
        pinpointReadIntervalMs = pinpointReadCount == 0 ? 0.0
                : (readStarted - lastPinpointReadStartedNanos) / 1_000_000.0;
        lastPinpointReadStartedNanos = readStarted;
        pinpointReadCount++;
        pinpoint.update();
        pinpointReadDurationMs = (System.nanoTime() - readStarted) / 1_000_000.0;
        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            recordPinpointBadRead(phase + ": " + describePinpointBadRead(previousPose, previousHeading),
                    readStarted);
        }
        return readStarted;
    }

    private void recordPinpointBadRead(String reason, long readStarted) {
        pinpointReadInterrupted = true;
        pinpointBadReadCount++;
        pinpointBadReadStreak++;
        lastPinpointBadReadNanos = System.nanoTime();
        pinpointReadDurationMs = (lastPinpointBadReadNanos - readStarted) / 1_000_000.0;
        lastPinpointBadReadReason = String.format(Locale.US, "%s; read %.1f ms / gap %.1f ms",
                reason, pinpointReadDurationMs, pinpointReadIntervalMs);
    }

    private String describePinpointBadRead(Pose2D previousPose, double previousHeading) {
        if (pinpointReadHistory == null) return "SDK BAD_READ; raw I2C history unavailable";
        TimestampedI2cData packet = null;
        for (TimestampedI2cData sample : pinpointReadHistory.getHistoryQueue()) packet = sample;
        if (packet == null || packet.data == null) return "No fresh I2C reply captured";
        // FTC Hardware 12.0.0 defaults to register 18 with ten 4-byte bulk values.
        if (packet.register != 18 || packet.data.length < 40) {
            return String.format(Locale.US, "Unexpected I2C reply: register %d / %d bytes",
                    packet.register, packet.data.length);
        }
        if (packet.data.length == 41) {
            int crc = 0x90;
            for (int i = 0; i < 40; i++) {
                crc ^= packet.data[i] & 0xff;
                for (int bit = 0; bit < 8; bit++) {
                    crc = ((crc & 0x80) != 0 ? (crc << 1) ^ 0x31 : crc << 1) & 0xff;
                }
            }
            if (crc != (packet.data[40] & 0xff)) return "Bulk CRC mismatch";
        }
        ByteBuffer data = ByteBuffer.wrap(packet.data).order(ByteOrder.LITTLE_ENDIAN);
        int loopTime = data.getInt(4);
        if (loopTime <= 1) return "loopTime=" + loopTime + " us; no valid sensor sample";
        if (packet.data.length == 40 && previousPose != null) {
            String[] names = {"X mm", "Y mm", "heading rad", "VX mm/s", "VY mm/s", "turn rad/s"};
            double[] previous = {previousPose.getX(DistanceUnit.MM), previousPose.getY(DistanceUnit.MM),
                    previousHeading, 0, 0, 0};
            double[] limits = {5000, 5000, 120, 10000, 10000, 120};
            for (int i = 0; i < names.length; i++) {
                float value = data.getFloat(16 + 4 * i);
                if (Float.isNaN(value)) return "NaN in " + names[i];
                double magnitude = i < 3 ? Math.abs(value - previous[i]) : Math.abs(value);
                if (magnitude > limits[i]) {
                    return String.format(Locale.US, "%s %s %.1f > SDK limit %.0f", names[i],
                            i < 3 ? "jump" : "magnitude", magnitude, limits[i]);
                }
            }
        }
        if ((data.getInt(0) & (1 << 5)) != 0) return "Raw device BAD_READ flag";
        return "SDK BAD_READ; captured packet has no matching trigger";
    }

    public long getPinpointReadCount() { return pinpointReadCount; }
    public long getPinpointBadReadCount() { return pinpointBadReadCount; }

    public String getPinpointReadDiagnostics() {
        double percent = pinpointReadCount == 0 ? 0.0 : 100.0 * pinpointBadReadCount / pinpointReadCount;
        return String.format(Locale.US, "%s; bad cycles %d/%d (%.2f%%), streak %d; read %.1f ms, gap %.1f ms; FW %s / LOCAL",
                pinpoint.getDeviceStatus(), pinpointBadReadCount, pinpointReadCount, percent,
                pinpointBadReadStreak, pinpointReadDurationMs, pinpointReadIntervalMs,
                pinpointFirmwareVersion > 0 ? Integer.toString(pinpointFirmwareVersion) : "unknown");
    }

    public String getLastPinpointReadFailure() {
        return pinpointBadReadCount == 0 ? "No bad reads"
                : String.format(Locale.US, "%.1f s ago: %s",
                        (System.nanoTime() - lastPinpointBadReadNanos) / 1_000_000_000.0,
                        lastPinpointBadReadReason);
    }

    // Vision establishes the field frame; Pinpoint tracks movement between vision overwrites.
    public Pose2D getPose() {
        return isFieldPoseInitialized() ? pinpoint.getPosition() : UNKNOWN_POSE;
    }

    // A local odometry frame is sufficient for comparing motion between visual observations.
    public Pose2D getOdometryPose() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getPosition() : UNKNOWN_POSE;
    }

    public GoBildaPinpointDriver.DeviceStatus getOdometryStatus() {
        return pinpoint.getDeviceStatus();
    }

    // Diagnostics may inspect cached device values even when control cannot use them.
    public Pose2D getRawOdometryPose() {
        return pinpoint.getPosition();
    }

    public boolean updateVisionPose(Pose2D pose, double frameTimestamp, long nowNanos) {
        if (!DriveConstants.pinpointConfigurationConfirmed) {
            localizationStatus = "Pinpoint configuration needs confirmation";
            resetVisionRejectionSequence();
            return false;
        }
        if (pinpointConfigurationLost || pinpointReadInterrupted) {
            localizationStatus = pinpointConfigurationLost
                    ? "Pinpoint offsets changed; restart OpMode to restore configuration"
                    : "Waiting for valid Pinpoint data and offset verification";
            return false;
        }
        if (!isFinitePose(pose) || !Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0) {
            localizationStatus = "Invalid Limelight field pose";
            resetVisionRejectionSequence();
            return false;
        }
        if (frameTimestamp == lastProcessedVisionTimestamp) return false;
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            clearVisionPose("Pinpoint not ready for vision overwrite: " + pinpoint.getDeviceStatus());
            pinpointReadInterrupted = true;
            return false;
        }
        lastProcessedVisionTimestamp = frameTimestamp;
        if (!Double.isFinite(DriveConstants.maximumVisionPositionDifferenceInches)
                || DriveConstants.maximumVisionPositionDifferenceInches < 0.0
                || !Double.isFinite(DriveConstants.maximumVisionHeadingDifferenceDegrees)
                || DriveConstants.maximumVisionHeadingDifferenceDegrees < 0.0
                || DriveConstants.visionRejectedFramesBeforeOverride < 1) {
            localizationStatus = "Invalid vision correction gates";
            resetVisionRejectionSequence();
            return false;
        }
        Pose2D predictedPose = pinpoint.getPosition();
        boolean initialized = fieldPoseInitialized;
        double positionDifference = isFinitePose(predictedPose) ? Math.hypot(
                pose.getX(DistanceUnit.INCH) - predictedPose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH) - predictedPose.getY(DistanceUnit.INCH)) : Double.NaN;
        double headingDifference = isFinitePose(predictedPose) ? Math.abs(Math.IEEEremainder(
                pose.getHeading(AngleUnit.DEGREES) - predictedPose.getHeading(AngleUnit.DEGREES),
                360.0)) : Double.NaN;
        boolean outsideGate = initialized && (
                positionDifference > DriveConstants.maximumVisionPositionDifferenceInches
                || headingDifference > DriveConstants.maximumVisionHeadingDifferenceDegrees);
        if (outsideGate) {
            rejectedVisionFrameCount++;
            if (rejectedVisionFrameCount < DriveConstants.visionRejectedFramesBeforeOverride) {
                localizationStatus = "Rejected vision jump " + rejectedVisionFrameCount
                        + "/" + DriveConstants.visionRejectedFramesBeforeOverride + "; using Pinpoint";
                return false;
            }
        }
        double previousHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        pinpoint.setPosition(pose);
        readPinpoint("after vision write");
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            clearVisionPose("Pinpoint not ready after vision overwrite: " + pinpoint.getDeviceStatus());
            pinpointReadInterrupted = true;
            return false;
        }
        if (!isFinitePose(pinpoint.getPosition())) {
            clearVisionPose("Pinpoint returned invalid X/Y/H after vision overwrite");
            pinpointReadInterrupted = true;
            return false;
        }
        if (Double.isFinite(driverHeadingReferenceRadians) && Double.isFinite(previousHeading)) {
            // A field-heading correction must not change the driver's headless reference.
            double correction = Math.IEEEremainder(
                    pose.getHeading(AngleUnit.RADIANS) - previousHeading, 2.0 * Math.PI);
            driverHeadingReferenceRadians = Math.IEEEremainder(
                    driverHeadingReferenceRadians + correction, 2.0 * Math.PI);
        } else {
            driverHeadingReferenceRadians = pose.getHeading(AngleUnit.RADIANS);
        }
        fieldPoseInitialized = true;
        pinpointReadInterrupted = false;
        lastVisionTimestamp = frameTimestamp;
        resetVisionRejectionSequence();
        localizationStatus = outsideGate ? "Forced vision override"
                : initialized ? "Vision correction applied" : "Initialized from first corrected frame";
        return true;
    }

    public void resetVisionRejectionSequence() {
        rejectedVisionFrameCount = 0;
    }

    public void clearVisionPose(String reason) {
        fieldPoseInitialized = false;
        pinpointReadInterrupted = false;
        lastVisionTimestamp = Double.NaN;
        lastProcessedVisionTimestamp = Double.NaN;
        resetVisionRejectionSequence();
        driverHeadingReferenceRadians = Double.NaN;
        lastValidDriverHeadingRadians = Double.NaN;
        localizationStatus = reason;
    }

    private static boolean isFinitePose(Pose2D pose) {
        return pose != null
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.RADIANS));
    }

    public boolean hasAcceptedVisionFrame(double timestamp) {
        return isFieldPoseInitialized() && Double.isFinite(timestamp) && timestamp == lastVisionTimestamp;
    }

    public String getLocalizationStatus() { return localizationStatus; }

    public boolean isFieldPoseInitialized() {
        return fieldPoseInitialized && !pinpointReadInterrupted
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public double getTranslationSpeedInchesPerSecond() {
        return isFieldPoseInitialized() ? Math.hypot(
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)) : Double.NaN;
    }

    public double getHeadingVelocityDegreesPerSecond() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) : Double.NaN;
    }

    public void resetDriverHeading() {
        driverHeadingReferenceRadians = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getHeading(AngleUnit.RADIANS) : Double.NaN;
        lastValidDriverHeadingRadians = Double.NaN;
    }

    public double getDriverHeadingRadians() {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(driverHeadingReferenceRadians)) {
            return Double.NaN;
        }
        return Math.IEEEremainder(
                pinpoint.getHeading(AngleUnit.RADIANS) - driverHeadingReferenceRadians, 2.0 * Math.PI);
    }

    public void moveFieldCentric(double forward, double strafe, double turn) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            stop();
            return;
        }
        double heading = getDriverHeadingRadians();
        // A brief periodic fault can reuse the last valid angle for manual translation.
        if (!Double.isFinite(heading) && periodicBadRead
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                && System.nanoTime() - lastValidDriverHeadingNanos <= DRIVER_HEADING_HOLD_NANOS) {
            heading = lastValidDriverHeadingRadians;
        }
        if (!Double.isFinite(heading)) {
            moveRobot(forward, strafe, turn);
            return;
        }
        // Field heading is CCW-positive; joystick strafe is right-positive.
        double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double robotStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        moveRobot(robotForward, robotStrafe, turn);
    }

    public void stop() {
        moveRobot(0, 0, 0);
    }

    public void moveRobot(double forward, double strafe, double turn) {
        double rotX = strafe;
        double rotY = forward;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        setMotorPowers(
                leftFrontPower,
                leftBackPower,
                rightFrontPower,
                rightBackPower
        );
    }

    private void setMotorPowers(
            double leftFrontPower,
            double leftBackPower,
            double rightFrontPower,
            double rightBackPower
    ) {
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }
}
