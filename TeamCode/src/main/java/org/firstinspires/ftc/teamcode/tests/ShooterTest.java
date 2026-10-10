package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

@Config
@TeleOp(name = "Shooter Test", group = "TeleOp")
public class ShooterTest extends OpMode {
    public static double power = 0.0;
    public static double pitchPosition = ShooterConstants.pitchStartPosition;
    public static double maxTestPower = 0.8;

    private static final double triggerDeadband = 0.05;

    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private Servo pitchServo;
    private FtcDashboard dashboard;
    private boolean stopLatched;

    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);

        leftShooterMotor.setDirection(ShooterConstants.leftShooterDirection);
        rightShooterMotor.setDirection(ShooterConstants.rightShooterDirection);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopLatched = false;
        stopMotors();

        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);
        if (Double.isFinite(pitchPosition)) {
            pitchServo.setPosition(Range.clip(pitchPosition,
                    ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition));
        }
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double motorPower = getMotorPower();
        double leftVelocity = Math.abs(leftShooterMotor.getVelocity());
        double rightVelocity = Math.abs(rightShooterMotor.getVelocity());

        leftShooterMotor.setPower(motorPower);
        rightShooterMotor.setPower(motorPower);
        if (!stopLatched && Double.isFinite(pitchPosition)) {
            pitchServo.setPosition(Range.clip(pitchPosition,
                    ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition));
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("requestedPower", power);
        packet.put("power", motorPower);
        packet.put("pitchPosition", pitchServo.getPosition());
        packet.put("leftVelocity", leftVelocity);
        packet.put("rightVelocity", rightVelocity);
        packet.put("velocityDifference", Math.abs(leftVelocity - rightVelocity));
        packet.put("stopLatched", stopLatched);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("Set power and pitchPosition in Dashboard > ShooterTest.");
        telemetry.addLine("Hold RT to apply the fixed power. B stops until RT is released.");
        telemetry.addData("Requested power", "%.3f", power);
        telemetry.addData("Power", "%.3f", motorPower);
        telemetry.addData("Pitch position", "%.3f", pitchServo.getPosition());
        telemetry.addData("Left velocity", "%.1f ticks/s", leftVelocity);
        telemetry.addData("Right velocity", "%.1f ticks/s", rightVelocity);
        telemetry.addData("Velocity difference", "%.1f ticks/s",
                Math.abs(leftVelocity - rightVelocity));
        telemetry.addData("Stop latched", stopLatched);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    private double getMotorPower() {
        if (gamepad1.b) {
            stopLatched = true;
            return 0.0;
        }

        if (gamepad1.right_trigger <= triggerDeadband) {
            stopLatched = false;
            return 0.0;
        }

        if (stopLatched || !Double.isFinite(power) || !Double.isFinite(maxTestPower)
                || !Double.isFinite(pitchPosition)) {
            return 0.0;
        }

        return Range.clip(power, 0.0, Range.clip(maxTestPower, 0.0, 1.0));
    }

    private void stopMotors() {
        if (leftShooterMotor != null) {
            leftShooterMotor.setPower(0.0);
        }
        if (rightShooterMotor != null) {
            rightShooterMotor.setPower(0.0);
        }
    }
}
