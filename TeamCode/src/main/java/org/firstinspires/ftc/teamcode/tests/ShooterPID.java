package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

@TeleOp(name = "Shooter PID", group = "Tests")
public class ShooterPID extends OpMode {
    private static final String diagnosticVersion = "ShooterPID-RT-FEED-4";
    private static final double velocityStep = 100.0;
    private static final double pitchStep = 0.01;

    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Drive drive;
    private FtcDashboard dashboard;
    private boolean stopLatched;
    private boolean feedRequested;
    private boolean manualLimitOpen;
    private boolean rightBumperWasPressed;
    private boolean dpadLeftWasPressed;
    private boolean dpadRightWasPressed;
    private boolean dpadUpWasPressed;
    private boolean dpadDownWasPressed;
    private boolean leftStickWasPressed;
    private String controlStatus = "NOT STARTED";

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        stopLatched = false;
        feedRequested = false;
        manualLimitOpen = false;
        rightBumperWasPressed = false;
        dpadLeftWasPressed = false;
        dpadRightWasPressed = false;
        dpadUpWasPressed = false;
        dpadDownWasPressed = false;
        leftStickWasPressed = false;
        stopMotors();
    }

    @Override
    public void init_loop() {
        drive.periodic();
        telemetry.addData("Version", diagnosticVersion);
        telemetry.addData("Drive", drive.getLocalizationStatus());
        telemetry.addData("Target velocity", "%.1f ticks/s", ShooterConstants.targetVelocity);
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
        intake.periodic();
        boolean requested = gamepad1.right_trigger > TeleOpConstants.triggerThreshold;
        boolean intakeRequested = gamepad1.left_trigger > TeleOpConstants.triggerThreshold;
        boolean jamRequested = gamepad1.left_bumper;

        if (gamepad1.b) {
            stopLatched = true;
        } else if (!requested) {
            stopLatched = false;
        }
        if (!stopLatched && !requested && gamepad1.right_bumper && !rightBumperWasPressed) {
            manualLimitOpen = transit.getState() != Transit.TransitState.OPEN;
        }
        double pitchAdjustment = 0.0;
        if (gamepad1.dpad_right && !dpadRightWasPressed) {
            pitchAdjustment += pitchStep;
        }
        if (gamepad1.dpad_left && !dpadLeftWasPressed) {
            pitchAdjustment -= pitchStep;
        }
        if (!stopLatched && pitchAdjustment != 0.0) {
            shooter.setPitchPosition(shooter.getPitchPosition() + pitchAdjustment);
        }
        if (gamepad1.dpad_up && !dpadUpWasPressed) {
            ShooterConstants.targetVelocity += velocityStep;
        }
        if (gamepad1.dpad_down && !dpadDownWasPressed) {
            ShooterConstants.targetVelocity = Math.max(0.0, ShooterConstants.targetVelocity - velocityStep);
        }
        rightBumperWasPressed = gamepad1.right_bumper;
        dpadLeftWasPressed = gamepad1.dpad_left;
        dpadRightWasPressed = gamepad1.dpad_right;
        dpadUpWasPressed = gamepad1.dpad_up;
        dpadDownWasPressed = gamepad1.dpad_down;

        if (gamepad1.left_stick_button && !leftStickWasPressed) {
            drive.resetDriverHeading();
        }
        leftStickWasPressed = gamepad1.left_stick_button;

        if (stopLatched) {
            stopMotors();
            feedRequested = false;
            controlStatus = "STOP LATCHED: RELEASE RT";
        } else {
            drive.moveFieldCentric(applyDeadband(-gamepad1.left_stick_y),
                    applyDeadband(gamepad1.left_stick_x), applyDeadband(gamepad1.right_stick_x));
            intake.updateInputs(intakeRequested, jamRequested, requested);
            if (requested && !jamRequested) {
                feedRequested = true;
                manualLimitOpen = false;
                shooter.setTargetVelocity(ShooterConstants.targetVelocity);
                shooter.periodic();
                transit.requestFeed(shooter.isReady() && shooter.isPitchReady());
                if (!shooter.hasTargetVelocity()) {
                    controlStatus = "INVALID SHARED CONFIGURATION";
                } else if (transit.getFiringState() == Transit.FiringState.REVERSING
                        || transit.getFiringState() == Transit.FiringState.FEEDING) {
                    controlStatus = transit.getFiringState().name();
                } else if (!transit.areLimitPositionsCalibrated()) {
                    controlStatus = "WAITING: LIMIT POSITIONS INVALID";
                } else if (!shooter.isReady()) {
                    controlStatus = "WAITING: SHOOTER SPEED";
                } else if (!shooter.isPitchReady()) {
                    controlStatus = "WAITING: PITCH SETTLING";
                } else {
                    controlStatus = "WAITING: FEED CONFIGURATION";
                }
            } else {
                if (jamRequested) {
                    shooter.stop();
                } else {
                    shooter.idle();
                }
                feedRequested = false;
                controlStatus = jamRequested ? "UNJAM" : "STANDBY";
            }
        }

        if (!stopLatched && !feedRequested) {
            if (transit.getFiringState() != Transit.FiringState.IDLE) {
                transit.cancelFeeding();
            }
            if (jamRequested) {
                transit.forwardForIntakeReverse();
            } else if (intakeRequested && !requested) {
                transit.forward();
            } else {
                transit.stopMotor();
            }
            if (manualLimitOpen) {
                transit.open();
            } else {
                transit.close();
            }
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("version", diagnosticVersion);
        packet.put("controlStatus", controlStatus);
        packet.put("rtInput", gamepad1.right_trigger);
        packet.put("ltInput", gamepad1.left_trigger);
        packet.put("rightBumperHeld", gamepad1.right_bumper);
        packet.put("dpadLeftHeld", gamepad1.dpad_left);
        packet.put("dpadRightHeld", gamepad1.dpad_right);
        packet.put("bHeld", gamepad1.b);
        packet.put("powerLimit", ShooterConstants.maxPower);
        packet.put("leftPidOutput", shooter.getLeftPidOutput());
        packet.put("rightPidOutput", shooter.getRightPidOutput());
        packet.put("kP", ShooterConstants.kP);
        packet.put("kI", ShooterConstants.kI);
        packet.put("kD", ShooterConstants.kD);
        packet.put("kF", ShooterConstants.kF);
        packet.put("mode", shooter.getControlMode());
        packet.put("targetVelocity", ShooterConstants.targetVelocity);
        packet.put("leftVelocity", shooter.getLeftVelocity());
        packet.put("rightVelocity", shooter.getRightVelocity());
        packet.put("leftPower", shooter.getLeftPower());
        packet.put("rightPower", shooter.getRightPower());
        packet.put("shooterReady", shooter.isReady());
        packet.put("pitchReady", shooter.isPitchReady());
        packet.put("limitPositionsValid", transit.areLimitPositionsCalibrated());
        packet.put("limitCommand", transit.getState().name());
        packet.put("pitchPosition", shooter.getPitchPosition());
        packet.put("intakePower", intake.getRequestedPower());
        packet.put("transitPower", transit.getRequestedPower());
        packet.put("feedRequested", feedRequested);
        packet.put("firingState", transit.getFiringState().name());
        packet.put("feedReverseProgress", transit.getFeedReverseTimerProgress());
        packet.put("running", shooter.hasTargetVelocity());
        packet.put("stopLatched", stopLatched);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("RT: shared shooter PID + intake + transit feed. LT: intake. LB: unjam. B: stop until RT release.");
        telemetry.addLine("D-pad Up/Down: shared target +/-100 ticks/s. D-pad Right/Left: pitch +/-0.01.");
        telemetry.addLine("RB: toggle limit OPEN/CLOSE while RT is released.");
        telemetry.addLine("After shooter-ready and pitch-ready, open the gate, reverse transit, then feed.");
        telemetry.addLine("Dashboard > ShooterConstants: shared target, PIDF and power limit. Pitch starts at pitchStartPosition.");
        telemetry.addData("Version", diagnosticVersion);
        telemetry.addData("Control status", controlStatus);
        telemetry.addData("RT input / B held", "%.3f / %b", gamepad1.right_trigger, gamepad1.b);
        telemetry.addData("Target velocity", "%.1f ticks/s", ShooterConstants.targetVelocity);
        telemetry.addData("Left / right velocity", "%.1f / %.1f ticks/s",
                shooter.getLeftVelocity(), shooter.getRightVelocity());
        telemetry.addData("Left / right PID output", "%.3f / %.3f",
                shooter.getLeftPidOutput(), shooter.getRightPidOutput());
        telemetry.addData("Left / right power", "%.3f / %.3f", shooter.getLeftPower(), shooter.getRightPower());
        telemetry.addData("Shooter / pitch ready", "%b / %b", shooter.isReady(), shooter.isPitchReady());
        telemetry.addData("Pitch position", "%.3f", shooter.getPitchPosition());
        telemetry.addData("Limit command / positions valid", "%s / %b",
                transit.getState(), transit.areLimitPositionsCalibrated());
        telemetry.addData("Feed request / state", "%b / %s", feedRequested, transit.getFiringState());
        telemetry.addData("Intake / transit power", "%.2f / %.2f",
                intake.getRequestedPower(), transit.getRequestedPower());
        telemetry.addData("Stop latched", stopLatched);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
        if (intake != null) {
            intake.raiseFlower();
        }
        CommandScheduler.getInstance().reset();
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }

    private void stopMotors() {
        manualLimitOpen = false;
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (transit != null) transit.cancelFeeding();
        if (drive != null) drive.stop();
    }
}
