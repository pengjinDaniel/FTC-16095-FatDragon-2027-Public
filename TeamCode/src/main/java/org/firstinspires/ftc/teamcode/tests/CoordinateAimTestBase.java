package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public abstract class CoordinateAimTestBase extends OpMode {
    private Drive drive;
    private Vision vision;
    private AutoAimController autoAim;
    private PositionHoldController positionHold;
    private boolean started;
    private boolean aimRequested;
    private boolean manualTranslation;
    private boolean stopLatched;
    private boolean driverHeadingResetPressed;
    private double turnPower;
    private FtcDashboard dashboard;
    private long lastLoopNanos;
    private double loopDtSeconds;

    protected abstract Alliance getAlliance();

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        started = false;
        aimRequested = false;
        stopLatched = false;
        driverHeadingResetPressed = false;
        turnPower = 0.0;
        lastLoopNanos = 0;
        loopDtSeconds = 0.0;
        dashboard = FtcDashboard.getInstance();
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, getAlliance());
        autoAim = new AutoAimController();
        positionHold = new PositionHoldController();
        drive.stop();
    }

    @Override
    public void init_loop() {
        updateSensors();
        showTelemetry();
    }

    @Override
    public void start() {
        drive.periodic();
        drive.resetDriverHeading();
        driverHeadingResetPressed = gamepad1.left_stick_button;
        started = true;
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        loopDtSeconds = lastLoopNanos == 0 ? 0.0 : (now - lastLoopNanos) / 1_000_000_000.0;
        lastLoopNanos = now;
        updateSensors();
        if (gamepad1.left_stick_button && !driverHeadingResetPressed) {
            drive.resetDriverHeading();
        }
        driverHeadingResetPressed = gamepad1.left_stick_button;
        aimRequested = gamepad1.right_trigger > TeleOpConstants.triggerThreshold;
        double forward = -applyDeadband(gamepad1.left_stick_y);
        double strafe = applyDeadband(gamepad1.left_stick_x);
        manualTranslation = forward != 0.0 || strafe != 0.0;

        if (gamepad1.b) {
            stopLatched = true;
        } else if (!aimRequested) {
            stopLatched = false;
        }

        if (stopLatched) {
            autoAim.reset();
            positionHold.reset();
            turnPower = 0.0;
            drive.stop();
        } else if (aimRequested) {
            if (vision.wasCorrectionApplied()) {
                autoAim.reset();
                positionHold.reset();
            }
            if (manualTranslation || !vision.isAimCalibrationReady()) {
                positionHold.reset();
            } else {
                positionHold.update(
                        drive.getPose(),
                        drive.isFieldPoseInitialized(),
                        drive.getTranslationSpeedInchesPerSecond()
                );
                forward = positionHold.getForwardPower();
                strafe = positionHold.getStrafePower();
            }
            turnPower = autoAim.calculateTurnPower(
                    vision.hasValidAimingTarget(),
                    vision.getYawErrorDegrees()
            );
            if (manualTranslation) {
                drive.moveFieldCentric(forward, strafe, turnPower);
            } else {
                drive.moveRobot(forward, strafe, turnPower);
            }
        } else {
            autoAim.reset();
            positionHold.reset();
            turnPower = applyDeadband(gamepad1.right_stick_x);
            drive.moveFieldCentric(forward, strafe, turnPower);
        }
        showTelemetry();
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (vision != null) {
            vision.stop();
        }
        CommandScheduler.getInstance().reset();
    }

    private void updateSensors() {
        drive.periodic();
        vision.update();
        vision.updateAimTarget(drive, stopLatched || gamepad1.b,
                gamepad1.right_trigger > TeleOpConstants.triggerThreshold);
    }

    private void showTelemetry() {
        Pose2D pose = drive.getPose();
        String selectedCell = vision.getSelectedCluster().map(Enum::name).orElse("NONE");
        String controlMode = !started ? "INIT" : stopLatched ? "STOPPED"
                : aimRequested ? (vision.isAimCalibrationReady() ? "AIMING" : "WAITING_FOR_POSE") : "MANUAL";

        telemetry.addLine("RT: aim with calibration paused | Release RT: 3 stable frames, 1.5 s after success | B: stop | L3: reset forward");
        telemetry.addData("Control Mode", controlMode);
        telemetry.addData("Target Status", vision.getTargetStatus());
        telemetry.addData("RT Calibration", vision.getRtCalibrationStatus());
        telemetry.addData("Pinpoint X/Y/H (in / deg)", "%.1f, %.1f / %+.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("CELL / Target XY (in)", "%s / %.1f, %.1f",
                selectedCell, vision.getTargetXInches(), vision.getTargetYInches());
        telemetry.addData("Desired Heading (deg)", "%+.1f", vision.getDesiredHeadingDegrees());
        telemetry.addData("Yaw Error (deg)", "%+.1f", vision.getYawErrorDegrees());
        telemetry.addData("Turn Power raw/applied", "%+.3f / %+.3f",
                autoAim.getRawTurnPower(), turnPower);
        telemetry.addData("Heading Velocity (deg/s)", "%+.1f",
                drive.getHeadingVelocityDegreesPerSecond());
        telemetry.addData("Effective Aim Turn Limit", "%.2f", getEffectiveMaxTurnPower());
        telemetry.addData("Aim PIDF", "%.3f / %.3f / %.3f / %.3f",
                DriveConstants.autoAimKp, DriveConstants.autoAimKi,
                DriveConstants.autoAimKd, DriveConstants.autoAimKf);
        telemetry.addData("Aim Locked", vision.isAimCalibrationReady() && autoAim.isLocked());
        telemetry.addData("Calibration / Wait (s)", "%s / %.1f",
                vision.getCalibrationStatus(), vision.getCalibrationCooldownRemainingSeconds());
        telemetry.update();

        if (dashboard != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("aimErrorDeg", vision.getYawErrorDegrees());
            packet.put("turnPowerRaw", autoAim.getRawTurnPower());
            packet.put("turnPowerApplied", turnPower);
            packet.put("headingVelocityDegPerSec", drive.getHeadingVelocityDegreesPerSecond());
            packet.put("effectiveMaxTurnPower", getEffectiveMaxTurnPower());
            packet.put("fieldXIn", pose.getX(DistanceUnit.INCH));
            packet.put("fieldYIn", pose.getY(DistanceUnit.INCH));
            packet.put("headingDeg", pose.getHeading(AngleUnit.DEGREES));
            packet.put("targetXIn", vision.getTargetXInches());
            packet.put("targetYIn", vision.getTargetYInches());
            packet.put("selectedCell", selectedCell);
            packet.put("desiredHeadingDeg", vision.getDesiredHeadingDegrees());
            packet.put("aimRequested", aimRequested);
            packet.put("aimLocked", vision.isAimCalibrationReady() && autoAim.isLocked());
            packet.put("validAimTarget", vision.hasValidAimingTarget());
            packet.put("visionCalibrationStatus", vision.getCalibrationStatus());
            packet.put("visionCalibrationCooldownSec", vision.getCalibrationCooldownRemainingSeconds());
            packet.put("loopDtSec", loopDtSeconds);
            packet.put("autoAimKp", DriveConstants.autoAimKp);
            packet.put("autoAimKi", DriveConstants.autoAimKi);
            packet.put("autoAimKd", DriveConstants.autoAimKd);
            packet.put("autoAimKf", DriveConstants.autoAimKf);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private static double getEffectiveMaxTurnPower() {
        return AutoAimController.getEffectiveMaxTurnPower();
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) <= DriveConstants.joystickDeadband ? 0.0 : value;
    }
}
