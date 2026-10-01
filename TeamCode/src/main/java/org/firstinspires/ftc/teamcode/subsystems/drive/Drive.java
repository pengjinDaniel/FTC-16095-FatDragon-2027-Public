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
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public void setPose(Pose2D pose) {
        if (pose == null
                || !Double.isFinite(pose.getX(DistanceUnit.INCH))
                || !Double.isFinite(pose.getY(DistanceUnit.INCH))
                || !Double.isFinite(pose.getHeading(AngleUnit.RADIANS))) {
            throw new IllegalArgumentException("Pose must be finite");
        }
        pinpoint.setPosition(pose);
    }

    public void setXYInches(double xInches, double yInches) {
        if (!Double.isFinite(xInches) || !Double.isFinite(yInches)) {
            throw new IllegalArgumentException("Pose coordinates must be finite");
        }
        pinpoint.setPosX(xInches, DistanceUnit.INCH);
        pinpoint.setPosY(yInches, DistanceUnit.INCH);
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

    public double getHeadingSpeedDegreesPerSecond() {
        return Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
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
