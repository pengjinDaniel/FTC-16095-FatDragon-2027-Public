package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import java.util.Map;

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
    private long aimStartNanos = -1;
    private double timeToLockSeconds = -1.0;
    private long lastLoopNanos;
    private double loopDtSeconds;

    protected abstract Alliance getAlliance();

    @Override
    public void init() {
        started = false;
        aimRequested = false;
        stopLatched = false;
        driverHeadingResetPressed = false;
        turnPower = 0.0;
        aimStartNanos = -1;
        timeToLockSeconds = -1.0;
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

        if (!aimRequested || stopLatched) {
            aimStartNanos = -1;
            timeToLockSeconds = -1.0;
        } else if (aimStartNanos < 0) {
            aimStartNanos = System.nanoTime();
        }

        if (stopLatched) {
            autoAim.reset();
            positionHold.reset();
            turnPower = 0.0;
            drive.stop();
        } else if (aimRequested) {
            if (manualTranslation) {
                positionHold.reset();
            } else {
                positionHold.update(
                        drive.getPose(),
                        drive.getPinpointStatus(),
                        drive.getTranslationSpeedInchesPerSecond()
                );
                forward = positionHold.getForwardPower();
                strafe = positionHold.getStrafePower();
            }
            turnPower = autoAim.calculateTurnPower(
                    vision.hasValidAimingTarget(),
                    vision.getYawErrorDegrees(),
                    drive.getHeadingVelocityDegreesPerSecond()
            );
            double maxTurnPower = getEffectiveMaxTurnPower();
            turnPower = Range.clip(turnPower, -maxTurnPower, maxTurnPower);
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
        if (aimStartNanos >= 0 && timeToLockSeconds < 0.0 && autoAim.isLocked()) {
            timeToLockSeconds = (System.nanoTime() - aimStartNanos) / 1_000_000_000.0;
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
    }

    private void updateSensors() {
        drive.periodic();
        vision.update();
        boolean correctFromVision = gamepad1.right_trigger <= TeleOpConstants.triggerThreshold;
        vision.setAimSelectionHeld(started && !stopLatched && !gamepad1.b
                && !correctFromVision);
        vision.updateFieldTarget(drive, correctFromVision);
    }

    private void showTelemetry() {
        telemetry.addLine("Field-relative drive | L3: reset forward | RT: aim (pause calibration) | release RT: calibrate | B: stop");
        telemetry.addData("Control Mode", stopLatched ? "STOPPED"
                : aimRequested ? "COORDINATE CELL AIM" : "MANUAL DRIVE");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Target Status", vision.getTargetStatus());
        telemetry.addData("Vision Pose Status", vision.getPoseStatus());
        telemetry.addData("Pipeline expected/actual", "%d / %d",
                vision.getExpectedPipelineIndex(), vision.getActualPipelineIndex());
        telemetry.addData("MT1 Tag Count / Age (ms)", "%d / %d",
                vision.getBotposeTagCount(), vision.getResultAgeMs());
        telemetry.addData("Area-qualified Alliance Tags", vision.getUsableTagCount());
        telemetry.addData("Raw MT1 (centered map; NOT field XY)", vision.getRawRobotFieldPose()
                .map(Object::toString).orElse("NONE"));
        telemetry.addData("Vision Field Pose (center 72/72 in)", vision.getCorrectedRobotFieldPose()
                .map(Object::toString).orElse("NONE"));
        telemetry.addData("HIVE Angle (deg)", vision.getHiveAngleDegrees());
        telemetry.addData("Robot Level Error (deg)", vision.getRobotLevelErrorDegrees());
        telemetry.addData("Field Pose Initialized", drive.isFieldPoseInitialized());
        telemetry.addData("Pinpoint Configuration Confirmed", DriveConstants.pinpointConfigurationConfirmed);
        telemetry.addData("Vision Correction Applied", vision.wasCorrectionApplied());
        telemetry.addData("Vision Raw / Solve / Accepted", "%b / %b / %b",
                vision.isRawResultValid(), vision.isHiveSolveSuccessful(), vision.isCorrectedPoseAccepted());
        telemetry.addData("Physical Frame Age (ms)", vision.getPhysicalFrameAgeMs());
        telemetry.addData("HIVE phi roots / level errors", vision.getHiveCandidateDiagnostics());
        telemetry.addData("Usable HIVE Roots", vision.getUsableHiveCandidateCount());
        telemetry.addData("MT1 Avg Dist / Span (m)", "%.2f / %.2f",
                vision.getAverageTagDistanceMeters(), vision.getTagSpanMeters());
        telemetry.addData("MT1 Avg Area (%) / Stddev", "%.3f / %s",
                vision.getAverageTagArea(), vision.getMt1StandardDeviationDiagnostics());
        telemetry.addData("Vision Correction Status", drive.getVisionCorrectionStatus());
        telemetry.addData("Vision Difference (in / deg)", "%.2f / %.2f",
                drive.getVisionPositionDifferenceInches(), drive.getVisionHeadingDifferenceDegrees());
        telemetry.addData("Rejected New Frames", drive.getRejectedVisionFrameCount());
        telemetry.addData("Visible Tags", vision.getVisibleTagIds());
        telemetry.addData("CELL States", vision.getObservedClusterStates());
        telemetry.addData("Selected CELL",
                vision.getSelectedCluster().map(Enum::name).orElse("NONE"));
        telemetry.addData("Valid Aim Target", vision.hasValidAimingTarget());
        telemetry.addData("Valid HIGH Target", vision.hasValidShootingTarget());
        for (Map.Entry<Integer, Pose3D> entry
                : vision.getVisibleTagPosesRobotSpace().entrySet()) {
            Pose3D tagPose = entry.getValue();
            Position position = tagPose.getPosition().toUnit(DistanceUnit.INCH);
            telemetry.addData("Tag " + entry.getKey() + " XYZ/roll",
                    "%.2f, %.2f, %.2f in / %.1f deg",
                    position.x, position.y, position.z,
                    tagPose.getOrientation() == null ? Double.NaN
                            : tagPose.getOrientation().getRoll(AngleUnit.DEGREES));
        }
        telemetry.addData("CELL Target Field XY (in)", "%.2f, %.2f",
                vision.getTargetXInches(), vision.getTargetYInches());
        telemetry.addData("Desired Heading (deg)", vision.getDesiredHeadingDegrees());
        telemetry.addData("Yaw Error (deg)", vision.getYawErrorDegrees());
        telemetry.addData("Horizontal Distance (in)", vision.getTargetHorizontalDistanceInches());
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("Raw Aim Turn Power", autoAim.getRawTurnPower());
        telemetry.addData("Aim D Turn Power", autoAim.getDerivativeTurnPower());
        telemetry.addData("Heading Velocity (deg/s)", drive.getHeadingVelocityDegreesPerSecond());
        telemetry.addData("Effective Aim Turn Limit", getEffectiveMaxTurnPower());
        telemetry.addData("Aim Locked", autoAim.isLocked());
        telemetry.addData("Time to First Lock (s)", timeToLockSeconds);
        telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
        telemetry.addData("Driver Heading (deg)", Math.toDegrees(drive.getDriverHeadingRadians()));
        telemetry.addData("Driver Heading Ready", Double.isFinite(drive.getDriverHeadingRadians()));
        Pose2D pose = drive.getPose();
        telemetry.addData("Pinpoint field X/Y/H (center 72/72 in)", "%.2f, %.2f in / %.1f deg",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position Hold Calibrated", DriveConstants.positionHoldCalibrated);
        telemetry.addData("Manual Translation", manualTranslation);
        telemetry.addData("Position Error (in)", positionHold.getPositionErrorInches());
        telemetry.update();

        if (dashboard != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("aimErrorDeg", vision.getYawErrorDegrees());
            packet.put("turnPowerRaw", autoAim.getRawTurnPower());
            packet.put("turnPowerD", autoAim.getDerivativeTurnPower());
            packet.put("headingVelocityDegPerSec", drive.getHeadingVelocityDegreesPerSecond());
            packet.put("turnPowerApplied", turnPower);
            packet.put("effectiveMaxTurnPower", getEffectiveMaxTurnPower());
            packet.put("headingDeg", pose.getHeading(AngleUnit.DEGREES));
            packet.put("driverHeadingDeg", Math.toDegrees(drive.getDriverHeadingRadians()));
            packet.put("driverHeadingReady", Double.isFinite(drive.getDriverHeadingRadians()));
            packet.put("desiredHeadingDeg", vision.getDesiredHeadingDegrees());
            packet.put("aimRequested", aimRequested);
            packet.put("aimLocked", autoAim.isLocked());
            packet.put("aimElapsedSec", aimStartNanos < 0 ? 0.0
                    : (System.nanoTime() - aimStartNanos) / 1_000_000_000.0);
            packet.put("timeToLockSec", timeToLockSeconds);
            packet.put("validAimTarget", vision.hasValidAimingTarget());
            packet.put("visionCorrectionApplied", vision.wasCorrectionApplied());
            packet.put("loopDtSec", loopDtSeconds);
            packet.put("autoAimKp", DriveConstants.autoAimKp);
            packet.put("autoAimKi", DriveConstants.autoAimKi);
            packet.put("autoAimKd", DriveConstants.autoAimKd);
            packet.put("autoAimKf", DriveConstants.autoAimKf);
            double batteryVoltage = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (Double.isFinite(voltage) && voltage > 0.0) {
                    batteryVoltage = Math.min(batteryVoltage, voltage);
                }
            }
            if (Double.isFinite(batteryVoltage)) {
                packet.put("batteryVoltage", batteryVoltage);
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private static double getEffectiveMaxTurnPower() {
        double controllerLimit = DriveConstants.autoAimMaxTurnPower;
        double testLimit = DriveConstants.autoAimTestMaxTurnPower;
        if (!Double.isFinite(controllerLimit) || !Double.isFinite(testLimit)
                || controllerLimit < 0.0 || testLimit < 0.0) {
            return 0.0;
        }
        return Math.min(1.0, Math.min(controllerLimit, testLimit));
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) <= DriveConstants.joystickDeadband ? 0.0 : value;
    }
}
