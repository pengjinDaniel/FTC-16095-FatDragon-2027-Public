package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

@TeleOp(name = "Shooter Test", group = "TeleOp")
public class ShooterTest extends OpMode {
    private static final double maxTestPower = 0.8;
    private static final double triggerDeadband = 0.05;

    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private boolean stopLatched;

    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);

        leftShooterMotor.setDirection(ShooterConstants.leftShooterDirection);
        rightShooterMotor.setDirection(ShooterConstants.rightShooterDirection);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopMotors();
    }

    @Override
    public void loop() {
        double motorPower = getMotorPower();
        double leftVelocity = Math.abs(leftShooterMotor.getVelocity());
        double rightVelocity = Math.abs(rightShooterMotor.getVelocity());

        leftShooterMotor.setPower(motorPower);
        rightShooterMotor.setPower(motorPower);

        telemetry.addLine("Hold right trigger to run. Press B to stop.");
        telemetry.addLine("After pressing B, release the trigger before restarting.");
        telemetry.addData("Power", "%.2f", motorPower);
        telemetry.addData("Left velocity", "%.1f ticks/s", leftVelocity);
        telemetry.addData("Right velocity", "%.1f ticks/s", rightVelocity);
        telemetry.addData("Velocity difference", "%.1f ticks/s",
                Math.abs(leftVelocity - rightVelocity));
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    private double getMotorPower() {
        if (gamepad1.b) {
            stopLatched = true;
        }

        if (gamepad1.right_trigger <= triggerDeadband) {
            stopLatched = false;
            return 0.0;
        }

        if (stopLatched) {
            return 0.0;
        }

        return Range.scale(
                gamepad1.right_trigger,
                triggerDeadband,
                1.0,
                0.0,
                maxTestPower
        );
    }

    private void stopMotors() {
        leftShooterMotor.setPower(0.0);
        rightShooterMotor.setPower(0.0);
    }
}
