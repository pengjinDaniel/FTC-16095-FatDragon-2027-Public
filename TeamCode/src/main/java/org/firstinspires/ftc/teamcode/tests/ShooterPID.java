package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

@Config
@TeleOp(name = "Shooter PID", group = "Tests")
public class ShooterPID extends OpMode {
    public static double targetVelocity = 0.0;
    public static double maxTestPower = 0.8;
    public static double pitchPosition = 0.52;
    public static boolean usePid = false;

    private static final double referencePower = 0.8;
    private static final double triggerDeadband = 0.05;

    private final PIDFController leftController = new PIDFController(
            ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD, ShooterConstants.kF);
    private final PIDFController rightController = new PIDFController(
            ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD, ShooterConstants.kF);
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private Servo pitchServo;
    private FtcDashboard dashboard;
    private boolean stopLatched;
    private boolean aWasPressed;
    private double previousTargetVelocity = Double.NaN;

    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);

        // Match DashTuner's raw power signs without an additional direction inversion.
        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopLatched = false;
        aWasPressed = false;
        usePid = false;
        stopMotors();
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double leftVelocity = Math.abs(leftShooterMotor.getVelocity());
        double rightVelocity = Math.abs(rightShooterMotor.getVelocity());
        double leftPower = 0.0;
        double rightPower = 0.0;
        boolean captureRequested = gamepad1.a && !aWasPressed;
        aWasPressed = gamepad1.a;
        boolean running = canRun(leftVelocity, rightVelocity);

        if (running) {
            pitchServo.setPosition(Range.clip(pitchPosition,
                    ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition));
            if (captureRequested && !usePid && leftVelocity > 0.0 && rightVelocity > 0.0) {
                targetVelocity = 0.5 * leftVelocity + 0.5 * rightVelocity;
                usePid = true;
            }

            double powerLimit = Range.clip(maxTestPower, 0.0, referencePower);
            if (usePid) {
                if (targetVelocity != previousTargetVelocity) {
                    leftController.reset();
                    rightController.reset();
                    previousTargetVelocity = targetVelocity;
                }
                leftController.setPIDF(ShooterConstants.kP, ShooterConstants.kI,
                        ShooterConstants.kD, ShooterConstants.kF);
                rightController.setPIDF(ShooterConstants.kP, ShooterConstants.kI,
                        ShooterConstants.kD, ShooterConstants.kF);
                leftPower = leftController.calculate(leftVelocity, targetVelocity);
                rightPower = rightController.calculate(rightVelocity, targetVelocity);
                if (!Double.isFinite(leftPower) || !Double.isFinite(rightPower)) {
                    stopLatched = true;
                    running = false;
                } else {
                    leftPower = Range.clip(leftPower, 0.0, powerLimit);
                    rightPower = Range.clip(rightPower, 0.0, powerLimit);
                }
            } else {
                leftPower = Math.min(referencePower, powerLimit);
                rightPower = leftPower;
                leftController.reset();
                rightController.reset();
                previousTargetVelocity = Double.NaN;
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
        packet.put("mode", usePid ? "PID" : "POWER REFERENCE");
        packet.put("targetVelocity", targetVelocity);
        packet.put("leftVelocity", leftVelocity);
        packet.put("rightVelocity", rightVelocity);
        packet.put("leftError", targetVelocity - leftVelocity);
        packet.put("rightError", targetVelocity - rightVelocity);
        packet.put("leftPower", leftPower);
        packet.put("rightPower", rightPower);
        packet.put("pitchPosition", pitchServo.getPosition());
        packet.put("running", running);
        packet.put("stopLatched", stopLatched);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("Hold RT to run. Reference: left +0.8 / right +0.8, pitch 0.52.");
        telemetry.addLine("Wait for stable speeds, then press A to capture their average and enable PID.");
        telemetry.addLine("Set usePid=false in Dashboard > ShooterPID to repeat the power reference.");
        telemetry.addLine("Set kP/kI/kD/kF in Dashboard > ShooterConstants. B stops until RT is released.");
        telemetry.addData("Mode", usePid ? "PID" : "POWER REFERENCE");
        telemetry.addData("Target velocity", "%.1f ticks/s", targetVelocity);
        telemetry.addData("Left velocity", "%.1f ticks/s", leftVelocity);
        telemetry.addData("Right velocity", "%.1f ticks/s", rightVelocity);
        telemetry.addData("Left error", "%.1f ticks/s", targetVelocity - leftVelocity);
        telemetry.addData("Right error", "%.1f ticks/s", targetVelocity - rightVelocity);
        telemetry.addData("Left power", "%.3f", leftPower);
        telemetry.addData("Right power", "%.3f", rightPower);
        telemetry.addData("Pitch position", "%.3f", pitchServo.getPosition());
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
                && Double.isFinite(maxTestPower) && maxTestPower > 0.0
                && Double.isFinite(pitchPosition)
                && Double.isFinite(leftVelocity) && Double.isFinite(rightVelocity)
                && (!usePid || (Double.isFinite(targetVelocity) && targetVelocity > 0.0
                && Double.isFinite(ShooterConstants.kP) && Double.isFinite(ShooterConstants.kI)
                && Double.isFinite(ShooterConstants.kD) && Double.isFinite(ShooterConstants.kF)));
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
