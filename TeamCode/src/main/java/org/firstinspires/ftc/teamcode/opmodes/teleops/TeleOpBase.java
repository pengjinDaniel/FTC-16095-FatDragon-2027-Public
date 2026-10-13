package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Map;

public abstract class TeleOpBase extends CommandOpMode {
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Vision vision;
    private AutoAimController autoAim;
    private ShootAssistCommand shootAssistCommand;
    private FtcDashboard dashboard;
    private boolean started;
    private boolean aimRequested;
    private boolean stopLatched;
    private long aimStartNanos = -1;
    private double timeToLockSeconds = -1.0;
    private long lastLoopNanos;
    private double loopDtSeconds;

    protected abstract Alliance getAlliance();

    protected Vision createVision() {
        return new Vision(hardwareMap, getAlliance());
    }

    @Override
    public void initialize() {
        started = false;
        aimRequested = false;
        stopLatched = false;
        aimStartNanos = -1;
        timeToLockSeconds = -1.0;
        lastLoopNanos = 0;
        loopDtSeconds = 0.0;
        dashboard = FtcDashboard.getInstance();
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        vision = createVision();
        autoAim = new AutoAimController();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        shootAssistCommand = new ShootAssistCommand(
                drive,
                shooter,
                intake,
                transit,
                vision,
                autoAim,
                gamepadEx
        );
        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx));
        intake.setDefaultCommand(new IntakeCommand(intake, transit, gamepadEx));

        // The scheduler refreshes sensors before buttons and command execution.
        CommandScheduler.getInstance().addButton(() -> vision.updateFieldTarget(
                drive, aimRequested));

        new Trigger(
                () -> gamepadEx.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenActive(drive::resetDriverHeading);

        new Trigger(
                () -> gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenActive(intake::toggleFlower);

        new Trigger(
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                        > TeleOpConstants.triggerThreshold
        ).whileActiveContinuous(shootAssistCommand);
    }

    @Override
    public void run() {
        long now = System.nanoTime();
        loopDtSeconds = lastLoopNanos == 0 ? 0.0 : (now - lastLoopNanos) / 1_000_000_000.0;
        lastLoopNanos = now;
        aimRequested = gamepad1.right_trigger > TeleOpConstants.triggerThreshold;

        if (gamepad1.b) {
            stopLatched = true;
        } else if (!aimRequested) {
            stopLatched = false;
        }

        vision.setAimSelectionHeld(aimRequested && !stopLatched);

        if (!aimRequested || stopLatched) {
            aimStartNanos = -1;
            timeToLockSeconds = -1.0;
        } else if (aimStartNanos < 0) {
            aimStartNanos = now;
        }

        if (stopLatched) {
            shootAssistCommand.cancel();
            autoAim.reset();
            stopMechanisms();
            drive.periodic();
            vision.update();
            vision.updateFieldTarget(drive, false);
        } else {
            super.run();
        }

        if (aimStartNanos >= 0 && timeToLockSeconds < 0.0 && autoAim.isLocked()) {
            timeToLockSeconds = (System.nanoTime() - aimStartNanos) / 1_000_000_000.0;
        }
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("Field-relative drive | L3: reset forward | RT: aim/shoot | B: stop");
        telemetry.addData("Control Mode", !started ? "INITIALIZING" : stopLatched ? "STOPPED"
                : aimRequested ? "COORDINATE CELL AIM" : "MANUAL DRIVE");
        telemetry.addData("Stop Latched", stopLatched);
        Pose2D pose = drive.getPose();
        telemetry.addData("Pinpoint X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Driver Heading (deg)", Math.toDegrees(drive.getDriverHeadingRadians()));
        telemetry.addData("Driver Heading Ready", Double.isFinite(drive.getDriverHeadingRadians()));
        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Target Status", vision.getTargetStatus());
        telemetry.addData("Vision Pose Status", vision.getPoseStatus());
        telemetry.addData("Localization Mode", vision.getLocalizationMode());
        telemetry.addData("Calibration Tags Used", vision.getLocalizationTagCount());
        if (vision.getLocalizationMode() == Vision.LocalizationMode.SINGLE_TAG) {
            telemetry.addData("Single Tag expected / used", "%d / %d",
                    vision.getExpectedCalibrationTagId(), vision.getCalibrationTagId());
        }
        telemetry.addData("Pipeline expected/actual", "%d / %d",
                vision.getExpectedPipelineIndex(), vision.getActualPipelineIndex());
        telemetry.addData("MT1 Tag Count / Age (ms)", "%d / %d",
                vision.getBotposeTagCount(), vision.getResultAgeMs());
        telemetry.addData("Area-qualified Alliance Tags", vision.getUsableTagCount());
        telemetry.addData("Raw Localization Pose", vision.getRawRobotFieldPose()
                .map(Object::toString).orElse("NONE"));
        telemetry.addData("Corrected Vision Pose", vision.getCorrectedRobotFieldPose()
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
        telemetry.addData("Detected Tags (all)", vision.getDetectedTagIds());
        telemetry.addData("Single Tag delta XY / yaw vs MT1", vision.getTagPoseAgreement());
        telemetry.addData("HIVE Correction Yaw Delta (deg)", vision.getRawToCorrectedHeadingDeltaDegrees());
        telemetry.addData("Alliance Tag Visible", vision.hasVisibleAllianceTag());
        telemetry.addData("Tag Z (in)", vision.getVisibleTagHeightsInches());
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
        telemetry.addData("Cluster States", vision.getObservedClusterStates());
        telemetry.addData(
                "Selected Cluster",
                vision.getSelectedCluster().map(Enum::name).orElse("NONE")
        );
        telemetry.addData("RT Latched CELL",
                vision.getLatchedAimCluster().map(Enum::name).orElse("NONE"));
        telemetry.addData("Representative Tag (display only)", vision.getSelectedTagId());
        telemetry.addData("Hive State", vision.getTargetHiveState());
        telemetry.addData("Valid Aim Target", vision.hasValidAimingTarget());
        telemetry.addData("Valid HIGH Target", vision.hasValidShootingTarget());
        telemetry.addData(
                "Representative Tag Pose (robot)",
                vision.getRepresentativeTagPoseRobotSpace()
                        .map(Object::toString)
                        .orElse("NONE")
        );
        telemetry.addData("CELL Target Field XY (in)", "%.2f, %.2f",
                vision.getTargetXInches(), vision.getTargetYInches());
        telemetry.addData("Desired Heading (deg)", vision.getDesiredHeadingDegrees());
        telemetry.addData(
                "Horizontal Distance (in)",
                vision.getTargetHorizontalDistanceInches()
        );
        telemetry.addData("Yaw Error (deg)", vision.getYawErrorDegrees());
        telemetry.addData("Shooter Mode", shooter.getControlMode());
        telemetry.addData("Shooter Fixed Power", ShooterConstants.fixedPower);
        telemetry.addData("Shooter Left / Right Power", "%.3f / %.3f",
                shooter.getLeftPower(), shooter.getRightPower());
        telemetry.addData("Shooter Left", shooter.getLeftVelocity());
        telemetry.addData("Shooter Right", shooter.getRightVelocity());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.addData("Shooter Pitch", shooter.getPitchPosition());
        telemetry.addData("Pitch Settled", shooter.isPitchReady());
        telemetry.addData("Aim Locked", autoAim.isLocked());
        telemetry.addData("Raw Aim Turn Power", autoAim.getRawTurnPower());
        telemetry.addData("Aim D Turn Power", autoAim.getDerivativeTurnPower());
        telemetry.addData("Heading Velocity (deg/s)", drive.getHeadingVelocityDegreesPerSecond());
        telemetry.addData("Turn Power", getAppliedTurnPower());
        telemetry.addData("Effective Aim Turn Limit", ShootAssistCommand.getEffectiveMaxTurnPower());
        telemetry.addData("Time to First Lock (s)", timeToLockSeconds);
        telemetry.addData("Position Hold Calibrated", DriveConstants.positionHoldCalibrated);
        telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
        telemetry.addData("Translation Speed (in/s)", drive.getTranslationSpeedInchesPerSecond());
        telemetry.addData("Position Locked", shootAssistCommand.isPositionLocked());
        telemetry.addData("Position Error (in)", shootAssistCommand.getPositionErrorInches());
        telemetry.addData("Manual Translation", shootAssistCommand.isManualTranslation());
        telemetry.addData("Shoot Requested", shootAssistCommand.isActive());
        telemetry.addData("Can Prepare", shootAssistCommand.canPrepareToFire());
        telemetry.addData("Can Fire", shootAssistCommand.canFire());
        telemetry.addData("Firing State", shootAssistCommand.getFiringState());
        telemetry.addData(
                "Limit Lead",
                "%.3f / %.3f s (%.0f%%)",
                shootAssistCommand.getLimitOpenTimerElapsedSeconds(),
                TransitConstants.LIMIT_OPEN_LEAD_TIME_SECONDS,
                100.0 * shootAssistCommand.getLimitOpenTimerProgress()
        );
        telemetry.addData("Intake Power", intake.getRequestedPower());
        telemetry.addData("Ball Count", intake.getBallCount());
        telemetry.addData("CDS Distance (cm)", intake.getDistanceCm());
        telemetry.addData("CDS Ball Present", intake.isBallPresent());
        telemetry.addData("Transit Power", transit.getRequestedPower());
        telemetry.addData("Limit Servo", transit.getState());
        telemetry.addData("Limit Calibrated", transit.areLimitPositionsCalibrated());
        telemetry.addData("Flower State", intake.getFlowerState());
        telemetry.addData("Flower Calibrated", intake.areFlowerPositionsCalibrated());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("aimErrorDeg", vision.getYawErrorDegrees());
        packet.put("turnPowerRaw", autoAim.getRawTurnPower());
        packet.put("turnPowerD", autoAim.getDerivativeTurnPower());
        packet.put("headingVelocityDegPerSec", drive.getHeadingVelocityDegreesPerSecond());
        packet.put("turnPowerApplied", getAppliedTurnPower());
        packet.put("effectiveMaxTurnPower", ShootAssistCommand.getEffectiveMaxTurnPower());
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
        packet.put("localizationMode", vision.getLocalizationMode().name());
        packet.put("localizationTagCount", vision.getLocalizationTagCount());
        packet.put("expectedCalibrationTagId", vision.getExpectedCalibrationTagId());
        packet.put("calibrationTagId", vision.getCalibrationTagId());
        packet.put("detectedTagIds", vision.getDetectedTagIds().toString());
        packet.put("mt1TagCount", vision.getBotposeTagCount());
        packet.put("qualifiedTagCount", vision.getUsableTagCount());
        packet.put("singleTagVsMt1", vision.getTagPoseAgreement().toString());
        packet.put("hiveCorrectionYawDeltaDeg", vision.getRawToCorrectedHeadingDeltaDegrees());
        packet.put("visionPoseStatus", vision.getPoseStatus());
        packet.put("visionCorrectionStatus", drive.getVisionCorrectionStatus());
        packet.put("selectedCell", vision.getSelectedCluster().map(Enum::name).orElse("NONE"));
        packet.put("loopDtSec", loopDtSeconds);
        packet.put("autoAimKp", DriveConstants.autoAimKp);
        packet.put("autoAimKi", DriveConstants.autoAimKi);
        packet.put("autoAimKd", DriveConstants.autoAimKd);
        packet.put("autoAimKf", DriveConstants.autoAimKf);
        packet.put("mode", shooter.getControlMode());
        packet.put("fixedPower", ShooterConstants.fixedPower);
        packet.put("leftVelocity", shooter.getLeftVelocity());
        packet.put("rightVelocity", shooter.getRightVelocity());
        packet.put("velocityDifference", Math.abs(shooter.getLeftVelocity() - shooter.getRightVelocity()));
        packet.put("leftPower", shooter.getLeftPower());
        packet.put("rightPower", shooter.getRightPower());
        packet.put("pitchPosition", shooter.getPitchPosition());
        packet.put("shooterReady", shooter.isReady());
        packet.put("canFire", shootAssistCommand.canFire());
        packet.put("stopLatched", stopLatched);
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

    private double getAppliedTurnPower() {
        if (!started || stopLatched || !Double.isFinite(drive.getDriverHeadingRadians())) {
            return 0.0;
        }
        if (aimRequested) {
            return shootAssistCommand.getTurnPower();
        }
        return Math.abs(gamepad1.right_stick_x) > DriveConstants.joystickDeadband
                ? gamepad1.right_stick_x : 0.0;
    }

    private void stopMechanisms() {
        if (drive != null) drive.stop();
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (transit != null) {
            transit.stopMotor();
            transit.close();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            while (opModeInInit() && !isStopRequested()) {
                drive.periodic();
                vision.update();
                vision.updateFieldTarget(drive, true);
                showTelemetry();
                idle();
            }
            if (isStopRequested()) {
                return;
            }
            waitForStart();
            drive.periodic();
            drive.resetDriverHeading();
            started = true;
            while (!isStopRequested() && opModeIsActive()) {
                run();
            }
        } finally {
            if (shootAssistCommand != null) shootAssistCommand.cancel();
            reset();
            stopMechanisms();
            if (intake != null) intake.raiseFlower();
            if (vision != null) vision.stop();
        }
    }
}
