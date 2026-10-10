
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="Robot: Auto Drive By Gyro", group="Robot")
@Disabled
public class RobotAutoDriveByGyro_Linear extends LinearOpMode {

    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private IMU             imu         = null;

    private double          headingError  = 0;

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.2;
    static final double     HEADING_THRESHOLD       = 1.0 ;
    static final double     P_TURN_GAIN            = 0.02;
    static final double     P_DRIVE_GAIN           = 0.03;


    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();


        driveStraight(DRIVE_SPEED, 24.0, 0.0);
        turnToHeading( TURN_SPEED, -45.0);
        holdHeading( TURN_SPEED, -45.0, 0.5);

        driveStraight(DRIVE_SPEED, 17.0, -45.0);
        turnToHeading( TURN_SPEED,  45.0);
        holdHeading( TURN_SPEED,  45.0, 0.5);

        driveStraight(DRIVE_SPEED, 17.0, 45.0);
        turnToHeading( TURN_SPEED,   0.0);
        holdHeading( TURN_SPEED,   0.0, 1.0);

        driveStraight(DRIVE_SPEED,-48.0, 0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }



    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        if (opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed);

                sendTelemetry(true);
            }

            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
