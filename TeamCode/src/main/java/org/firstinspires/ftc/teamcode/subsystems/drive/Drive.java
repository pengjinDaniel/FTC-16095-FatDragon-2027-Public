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
    private final GoBildaPinpointDriver pinpoint;
    private boolean fieldPoseInitialized;
    private double driverHeadingReferenceRadians = Double.NaN;
    private double lastVisionTimestamp = Double.NaN;
    private double lastAcceptedVisionTimestamp = Double.NaN;
    private int rejectedVisionFrameCount;
    private double visionPositionDifferenceInches = Double.NaN;
    private double visionHeadingDifferenceDegrees = Double.NaN;
    private String visionCorrectionStatus = "Waiting for corrected vision";

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

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                DriveConstants.pinpointForwardPodDirection,
                DriveConstants.pinpointStrafePodDirection
        );
        pinpoint.setOffsets(
                DriveConstants.pinpointXOffsetMm,
                DriveConstants.pinpointYOffsetMm,
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM
        );
        pinpoint.resetPosAndIMU();

        stop();
    }

    @Override
    public void periodic() {
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            fieldPoseInitialized = false;
            lastVisionTimestamp = Double.NaN;
            lastAcceptedVisionTimestamp = Double.NaN;
            resetVisionRejectionSequence();
            visionCorrectionStatus = "Pinpoint not ready";
            driverHeadingReferenceRadians = Double.NaN;
        } else if (!Double.isFinite(driverHeadingReferenceRadians)) {
            resetDriverHeading();
        }
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public void setPose(Pose2D pose) {
        if (!isFinitePose(pose)) {
            throw new IllegalArgumentException("Pose must be finite");
        }
        double previousHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        pinpoint.setPosition(pose);
        // setPosition writes hardware registers; refresh the SDK bulk-read cache.
        pinpoint.update();
        fieldPoseInitialized = pinpoint.getDeviceStatus()
                == GoBildaPinpointDriver.DeviceStatus.READY;
        if (fieldPoseInitialized && Double.isFinite(driverHeadingReferenceRadians)
                && Double.isFinite(previousHeading)) {
            // A localization correction must not rotate the driver's reference frame.
            double correction = Math.IEEEremainder(
                    pose.getHeading(AngleUnit.RADIANS) - previousHeading, 2.0 * Math.PI);
            driverHeadingReferenceRadians = Math.IEEEremainder(
                    driverHeadingReferenceRadians + correction, 2.0 * Math.PI);
        }
    }

    public boolean calibrateFieldPose(Pose2D pose, double visionTimestamp) {
        if (!DriveConstants.pinpointConfigurationConfirmed
                || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            visionCorrectionStatus = "Pinpoint not ready for vision";
            resetVisionRejectionSequence();
            return false;
        }
        if (!isFinitePose(pose) || !Double.isFinite(visionTimestamp) || visionTimestamp <= 0.0) {
            visionCorrectionStatus = "Invalid corrected vision pose";
            resetVisionRejectionSequence();
            return false;
        }
        // Count rejected physical frames once, even when the same frame is read repeatedly.
        if (visionTimestamp == lastVisionTimestamp) {
            return false;
        }
        lastVisionTimestamp = visionTimestamp;
        if (!Double.isFinite(DriveConstants.maximumVisionPositionDifferenceInches)
                || DriveConstants.maximumVisionPositionDifferenceInches < 0.0
                || !Double.isFinite(DriveConstants.maximumVisionHeadingDifferenceDegrees)
                || DriveConstants.maximumVisionHeadingDifferenceDegrees < 0.0) {
            visionCorrectionStatus = "Invalid vision correction gates";
            resetVisionRejectionSequence();
            return false;
        }
        Pose2D predictedPose = getPose();
        boolean initialized = fieldPoseInitialized;
        visionPositionDifferenceInches = isFinitePose(predictedPose) ? Math.hypot(
                pose.getX(DistanceUnit.INCH) - predictedPose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH) - predictedPose.getY(DistanceUnit.INCH)) : Double.NaN;
        visionHeadingDifferenceDegrees = isFinitePose(predictedPose) ? Math.abs(Math.IEEEremainder(
                pose.getHeading(AngleUnit.DEGREES) - predictedPose.getHeading(AngleUnit.DEGREES),
                360.0)) : Double.NaN;
        boolean outsideGate = initialized && (
                visionPositionDifferenceInches > DriveConstants.maximumVisionPositionDifferenceInches
                || visionHeadingDifferenceDegrees > DriveConstants.maximumVisionHeadingDifferenceDegrees);
        if (outsideGate) {
            rejectedVisionFrameCount++;
            visionCorrectionStatus = "Rejected vision jump; using Pinpoint";
            return false;
        }
        setPose(pose);
        if (fieldPoseInitialized) {
            lastAcceptedVisionTimestamp = visionTimestamp;
            resetVisionRejectionSequence();
            visionCorrectionStatus = initialized ? "Vision correction applied"
                    : "Initialized from first corrected frame";
        } else {
            visionCorrectionStatus = "Pinpoint not ready after vision write";
        }
        return fieldPoseInitialized;
    }

    /** Read-only confidence check; never writes Pinpoint, including while R2 is held. */
    public boolean isVisionPoseWithinGate(Pose2D pose) {
        Pose2D predicted = getPose();
        return isFinitePose(pose) && isFinitePose(predicted)
                && Double.isFinite(DriveConstants.maximumVisionPositionDifferenceInches)
                && Double.isFinite(DriveConstants.maximumVisionHeadingDifferenceDegrees)
                && Math.hypot(pose.getX(DistanceUnit.INCH)-predicted.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH)-predicted.getY(DistanceUnit.INCH))
                    <= DriveConstants.maximumVisionPositionDifferenceInches
                && Math.abs(Math.IEEEremainder(pose.getHeading(AngleUnit.DEGREES)
                    -predicted.getHeading(AngleUnit.DEGREES),360.0))
                    <= DriveConstants.maximumVisionHeadingDifferenceDegrees;
    }

    private static boolean isFinitePose(Pose2D pose) {
        return pose != null
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.RADIANS));
    }

    public void resetVisionRejectionSequence() {
        rejectedVisionFrameCount = 0;
    }

    public boolean hasAcceptedVisionFrame(double timestamp) {
        return fieldPoseInitialized && Double.isFinite(timestamp)
                && timestamp == lastAcceptedVisionTimestamp;
    }

    public String getVisionCorrectionStatus() { return visionCorrectionStatus; }
    public int getRejectedVisionFrameCount() { return rejectedVisionFrameCount; }
    // Retain the diagnostic API; corrections never bypass the jump gates.
    public boolean wasVisionCorrectionForced() { return false; }
    public double getVisionPositionDifferenceInches() { return visionPositionDifferenceInches; }
    public double getVisionHeadingDifferenceDegrees() { return visionHeadingDifferenceDegrees; }

    public boolean isFieldPoseInitialized() {
        return fieldPoseInitialized;
    }

    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        return pinpoint.getDeviceStatus();
    }

    public double getTranslationSpeedInchesPerSecond() {
        return Math.hypot(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH)
        );
    }

    public double getHeadingVelocityDegreesPerSecond() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) : Double.NaN;
    }

    public void resetDriverHeading() {
        driverHeadingReferenceRadians = pinpoint.getDeviceStatus()
                == GoBildaPinpointDriver.DeviceStatus.READY
                ? pinpoint.getHeading(AngleUnit.RADIANS) : Double.NaN;
    }

    public double getDriverHeadingRadians() {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(driverHeadingReferenceRadians)) {
            return Double.NaN;
        }
        return Math.IEEEremainder(
                pinpoint.getHeading(AngleUnit.RADIANS) - driverHeadingReferenceRadians,
                2.0 * Math.PI);
    }

    public void moveFieldCentric(double forward, double strafe, double turn) {
        double heading = getDriverHeadingRadians();
        if (!Double.isFinite(heading) || !Double.isFinite(forward)
                || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            stop();
            return;
        }
        // Pinpoint heading is CCW-positive; joystick strafe is right-positive.
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
