package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Drive extends SubsystemBase {
    public final DcMotorEx leftFrontMotor;
    public final DcMotorEx leftBackMotor;
    public final DcMotorEx rightFrontMotor;
    public final DcMotorEx rightBackMotor;
    private static final Pose2D UNKNOWN_POSE = new Pose2D(
            DistanceUnit.INCH, Double.NaN, Double.NaN, AngleUnit.RADIANS, Double.NaN);
    private final GoBildaPinpointDriver pinpoint;
    private boolean fieldPoseInitialized;
    private double driverHeadingReferenceRadians = Double.NaN;
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
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            clearVisionPose("Pinpoint not ready: " + pinpoint.getDeviceStatus());
        } else if (!isFinitePose(pinpoint.getPosition())) {
            clearVisionPose("Pinpoint returned invalid X/Y/H");
        } else if (!Double.isFinite(driverHeadingReferenceRadians)) {
            resetDriverHeading();
        }
    }

    // Vision establishes the field frame; Pinpoint tracks movement between vision overwrites.
    public Pose2D getPose() {
        return fieldPoseInitialized ? pinpoint.getPosition() : UNKNOWN_POSE;
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
        if (!isFinitePose(pose) || !Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0) {
            localizationStatus = "Invalid Limelight field pose";
            resetVisionRejectionSequence();
            return false;
        }
        if (frameTimestamp == lastProcessedVisionTimestamp) return false;
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            clearVisionPose("Pinpoint not ready for vision overwrite: " + pinpoint.getDeviceStatus());
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
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            clearVisionPose("Pinpoint not ready after vision overwrite: " + pinpoint.getDeviceStatus());
            return false;
        }
        if (!isFinitePose(pinpoint.getPosition())) {
            clearVisionPose("Pinpoint returned invalid X/Y/H after vision overwrite");
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
        lastVisionTimestamp = Double.NaN;
        lastProcessedVisionTimestamp = Double.NaN;
        resetVisionRejectionSequence();
        driverHeadingReferenceRadians = Double.NaN;
        localizationStatus = reason;
    }

    private static boolean isFinitePose(Pose2D pose) {
        return pose != null
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.RADIANS));
    }

    public boolean hasAcceptedVisionFrame(double timestamp) {
        return fieldPoseInitialized && Double.isFinite(timestamp) && timestamp == lastVisionTimestamp;
    }

    public String getLocalizationStatus() { return localizationStatus; }

    public boolean isFieldPoseInitialized() { return fieldPoseInitialized; }

    public double getTranslationSpeedInchesPerSecond() {
        return fieldPoseInitialized ? Math.hypot(
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)) : Double.NaN;
    }

    public double getHeadingVelocityDegreesPerSecond() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) : Double.NaN;
    }

    public void resetDriverHeading() {
        driverHeadingReferenceRadians = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getHeading(AngleUnit.RADIANS) : Double.NaN;
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
