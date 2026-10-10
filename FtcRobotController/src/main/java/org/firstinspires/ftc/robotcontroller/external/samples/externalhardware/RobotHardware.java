
package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class RobotHardware {

    private LinearOpMode myOpMode = null;

    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor armMotor = null;
    private Servo   leftHand = null;
    private Servo   rightHand = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init()    {
        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void driveRobot(double Drive, double Turn) {
        double left  = Drive + Turn;
        double right = Drive - Turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        setDrivePower(left, right);
    }

    public void setDrivePower(double leftWheel, double rightWheel) {
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }

    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }
}
