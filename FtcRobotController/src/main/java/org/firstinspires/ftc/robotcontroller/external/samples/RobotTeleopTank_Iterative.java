
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Robot: Teleop Tank", group="Robot")
@Disabled
public class RobotTeleopTank_Iterative extends OpMode{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftArm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;
    public static final double ARM_UP_POWER    =  0.50 ;
    public static final double ARM_DOWN_POWER  = -0.25 ;

    @Override
    public void init() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftArm    = hardwareMap.get(DcMotor.class, "left_arm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        rightClaw = hardwareMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        telemetry.addData(">", "Robot Ready.  Press START.");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double left;
        double right;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        leftDrive.setPower(left);
        rightDrive.setPower(right);

        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        leftClaw.setPosition(MID_SERVO + clawOffset);
        rightClaw.setPosition(MID_SERVO - clawOffset);

        if (gamepad1.y)
            leftArm.setPower(ARM_UP_POWER);
        else if (gamepad1.a)
            leftArm.setPower(ARM_DOWN_POWER);
        else
            leftArm.setPower(0.0);

        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    @Override
    public void stop() {
    }
}
