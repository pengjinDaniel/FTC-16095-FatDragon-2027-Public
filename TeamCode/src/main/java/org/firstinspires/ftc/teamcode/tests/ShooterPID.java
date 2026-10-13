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

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

@Config
@TeleOp(name = "Shooter PID", group = "Tests")
public class ShooterPID extends OpMode {
    public static double targetVelocity = 1640.0;
    public static double maxTestPower = 1.0;
    public static double pitchPosition = 0.52;

    private static final double feedPower = 0.7;
    private static final double velocityStep = 100.0;
    private static final double triggerDeadband = 0.05;

    private final PIDFController leftController = new PIDFController(
            ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD, ShooterConstants.kF);
    private final PIDFController rightController = new PIDFController(
            ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD, ShooterConstants.kF);
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private DcMotor intakeMotor;
    private DcMotor transitMotor;
    private Servo pitchServo;
    private Servo limitServo;
    private Drive drive;
    private FtcDashboard dashboard;
    private boolean stopLatched;
    private boolean dpadLeftWasPressed;
    private boolean dpadUpWasPressed;
    private boolean dpadDownWasPressed;
    private boolean leftStickWasPressed;
    private boolean limitOpen;
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
        dpadLeftWasPressed = false;
        dpadUpWasPressed = false;
        dpadDownWasPressed = false;
        leftStickWasPressed = false;
        targetVelocity = 1640.0;
        maxTestPower = 1.0;
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        transitMotor = hardwareMap.get(DcMotor.class, TransitConstants.transitMotorName);
        intakeMotor.setDirection(IntakeConstants.intakeMotorDirection);
        transitMotor.setDirection(TransitConstants.transitMotorDirection);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transitMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
        limitOpen = false;
        limitServo.setPosition(TransitConstants.limitClosed);
        drive = new Drive(hardwareMap);
        stopMotors();
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {
        drive.periodic();
        telemetry.addData("Pinpoint", drive.getPinpointStatus());
        telemetry.addData("Target velocity", "%.1f ticks/s", targetVelocity);
        telemetry.addData("Limit", "CLOSED");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.periodic();
        drive.resetDriverHeading();
    }

    @Override
    public void loop() {
        drive.periodic();
        if (gamepad1.left_stick_button && !leftStickWasPressed) {
            drive.resetDriverHeading();
        }
        leftStickWasPressed = gamepad1.left_stick_button;
        drive.moveFieldCentric(applyDeadband(-gamepad1.left_stick_y),
                applyDeadband(gamepad1.left_stick_x), applyDeadband(gamepad1.right_stick_x));

        if (gamepad1.dpad_left && !dpadLeftWasPressed) {
            limitOpen = !limitOpen;
        }
        dpadLeftWasPressed = gamepad1.dpad_left;
        limitServo.setPosition(limitOpen ? TransitConstants.limitOpen : TransitConstants.limitClosed);
        if (gamepad1.dpad_up && !dpadUpWasPressed) {
            targetVelocity += velocityStep;
        }
        if (gamepad1.dpad_down && !dpadDownWasPressed) {
            targetVelocity = Math.max(0.0, targetVelocity - velocityStep);
        }
        dpadUpWasPressed = gamepad1.dpad_up;
        dpadDownWasPressed = gamepad1.dpad_down;

        double leftVelocity = Math.abs(leftShooterMotor.getVelocity());
        double rightVelocity = Math.abs(rightShooterMotor.getVelocity());
        double leftPower = 0.0;
        double rightPower = 0.0;
        boolean running = canRun(leftVelocity, rightVelocity);

        if (running) {
            pitchServo.setPosition(Range.clip(pitchPosition,
                    ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition));
            double powerLimit = Range.clip(maxTestPower, 0.0, 1.0);
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
        intakeMotor.setPower(running ? feedPower : 0.0);
        transitMotor.setPower(running ? feedPower : 0.0);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("mode", "PID");
        packet.put("targetVelocity", targetVelocity);
        packet.put("leftVelocity", leftVelocity);
        packet.put("rightVelocity", rightVelocity);
        packet.put("leftError", targetVelocity - leftVelocity);
        packet.put("rightError", targetVelocity - rightVelocity);
        packet.put("leftPower", leftPower);
        packet.put("rightPower", rightPower);
        packet.put("pitchPosition", pitchServo.getPosition());
        packet.put("intakePower", intakeMotor.getPower());
        packet.put("transitPower", transitMotor.getPower());
        packet.put("limitOpen", limitOpen);
        packet.put("driverHeadingRadians", drive.getDriverHeadingRadians());
        packet.put("pinpointStatus", drive.getPinpointStatus().toString());
        packet.put("running", running);
        packet.put("stopLatched", stopLatched);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("Hold RT: shooter PID, intake/transit +0.7. B stops until RT is released.");
        telemetry.addLine("D-pad Up/Down: target +/-100 ticks/s. D-pad Left: toggle limit.");
        telemetry.addLine("Left stick: field-centric move. Right stick X: turn. Left stick button: reset heading.");
        telemetry.addLine("Set kP/kI/kD/kF in Dashboard > ShooterConstants. PID output limit: 1.0.");
        telemetry.addData("Mode", "PID");
        telemetry.addData("Target velocity", "%.1f ticks/s", targetVelocity);
        telemetry.addData("Left velocity", "%.1f ticks/s", leftVelocity);
        telemetry.addData("Right velocity", "%.1f ticks/s", rightVelocity);
        telemetry.addData("Left error", "%.1f ticks/s", targetVelocity - leftVelocity);
        telemetry.addData("Right error", "%.1f ticks/s", targetVelocity - rightVelocity);
        telemetry.addData("Left power", "%.3f", leftPower);
        telemetry.addData("Right power", "%.3f", rightPower);
        telemetry.addData("Pitch position", "%.3f", pitchServo.getPosition());
        telemetry.addData("Intake power", "%.2f", intakeMotor.getPower());
        telemetry.addData("Transit power", "%.2f", transitMotor.getPower());
        telemetry.addData("Limit", limitOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Driver heading", "%.1f deg", Math.toDegrees(drive.getDriverHeadingRadians()));
        telemetry.addData("Pinpoint", drive.getPinpointStatus());
        telemetry.addData("Running", running);
        telemetry.addData("Stop latched", stopLatched);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
        limitOpen = false;
        if (limitServo != null) {
            limitServo.setPosition(TransitConstants.limitClosed);
        }
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
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
                && Double.isFinite(targetVelocity) && targetVelocity > 0.0
                && Double.isFinite(ShooterConstants.kP) && Double.isFinite(ShooterConstants.kI)
                && Double.isFinite(ShooterConstants.kD) && Double.isFinite(ShooterConstants.kF);
    }

    private void stopMotors() {
        if (leftShooterMotor != null) {
            leftShooterMotor.setPower(0.0);
        }
        if (rightShooterMotor != null) {
            rightShooterMotor.setPower(0.0);
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (transitMotor != null) {
            transitMotor.setPower(0.0);
        }
        if (drive != null) {
            drive.stop();
        }
        leftController.reset();
        rightController.reset();
        previousTargetVelocity = Double.NaN;
    }
}
