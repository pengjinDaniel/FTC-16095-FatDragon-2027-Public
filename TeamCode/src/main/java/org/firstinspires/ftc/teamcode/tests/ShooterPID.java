package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

@Config
@TeleOp(name = "Shooter PID", group = "Tests")
public class ShooterPID extends OpMode {
    public static double targetVelocity = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double maxTestPower = 0.8;

    private static final double triggerDeadband = 0.05;

    private final PIDFController leftController = new PIDFController(0.0, 0.0, 0.0, 0.0);
    private final PIDFController rightController = new PIDFController(0.0, 0.0, 0.0, 0.0);
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private FtcDashboard dashboard;
    private boolean stopLatched;
    private double previousTargetVelocity = Double.NaN;

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
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double leftVelocity = Math.abs(leftShooterMotor.getVelocity());
        double rightVelocity = Math.abs(rightShooterMotor.getVelocity());
        double leftPower = 0.0;
        double rightPower = 0.0;
        boolean running = canRun(leftVelocity, rightVelocity);

        if (running) {
            if (targetVelocity != previousTargetVelocity) {
                leftController.reset();
                rightController.reset();
                previousTargetVelocity = targetVelocity;
            }
            leftController.setPIDF(kP, kI, kD, kF);
            rightController.setPIDF(kP, kI, kD, kF);
            leftPower = leftController.calculate(leftVelocity, targetVelocity);
            rightPower = rightController.calculate(rightVelocity, targetVelocity);
            if (!Double.isFinite(leftPower) || !Double.isFinite(rightPower)) {
                stopLatched = true;
                running = false;
            } else {
                double powerLimit = Range.clip(maxTestPower, 0.0, 1.0);
                leftPower = Range.clip(leftPower, 0.0, powerLimit);
                rightPower = Range.clip(rightPower, 0.0, powerLimit);
            }
        }

        if (!running) {
            leftPower = 0.0;
            rightPower = 0.0;
            leftController.reset();
            rightController.reset();
            previousTargetVelocity = Double.NaN;
        }
        leftShooterMotor.setPower(leftPower);
        rightShooterMotor.setPower(rightPower);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetVelocity", targetVelocity);
        packet.put("leftVelocity", leftVelocity);
        packet.put("rightVelocity", rightVelocity);
        packet.put("leftError", targetVelocity - leftVelocity);
        packet.put("rightError", targetVelocity - rightVelocity);
        packet.put("leftPower", leftPower);
        packet.put("rightPower", rightPower);
        packet.put("running", running);
        packet.put("stopLatched", stopLatched);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("Set targetVelocity (ticks/s) and kP/kI/kD/kF in Dashboard > ShooterPID.");
        telemetry.addLine("Hold RT to run. B stops until RT is released.");
        telemetry.addData("Target velocity", "%.1f ticks/s", targetVelocity);
        telemetry.addData("Left velocity", "%.1f ticks/s", leftVelocity);
        telemetry.addData("Right velocity", "%.1f ticks/s", rightVelocity);
        telemetry.addData("Left error", "%.1f ticks/s", targetVelocity - leftVelocity);
        telemetry.addData("Right error", "%.1f ticks/s", targetVelocity - rightVelocity);
        telemetry.addData("Left power", "%.3f", leftPower);
        telemetry.addData("Right power", "%.3f", rightPower);
        telemetry.addData("Running", running);
        telemetry.addData("Stop latched", stopLatched);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    private boolean canRun(double leftVelocity, double rightVelocity) {
        if (gamepad1.b) {
            stopLatched = true;
            return false;
        }

        if (gamepad1.right_trigger <= triggerDeadband) {
            stopLatched = false;
            return false;
        }

        return !stopLatched
                && Double.isFinite(targetVelocity) && targetVelocity > 0.0
                && Double.isFinite(maxTestPower) && maxTestPower > 0.0
                && Double.isFinite(kP) && Double.isFinite(kI)
                && Double.isFinite(kD) && Double.isFinite(kF)
                && Double.isFinite(leftVelocity) && Double.isFinite(rightVelocity);
    }

    private void stopMotors() {
        if (leftShooterMotor != null) {
            leftShooterMotor.setPower(0.0);
        }
        if (rightShooterMotor != null) {
            rightShooterMotor.setPower(0.0);
        }
        leftController.reset();
        rightController.reset();
        previousTargetVelocity = Double.NaN;
    }
}
