package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class TeleOpBase extends CommandOpMode {
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Vision vision;
    private AutoAimController autoAim;
    private ShootAssistCommand shootAssistCommand;

    protected abstract Alliance getAlliance();

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        vision = new Vision(hardwareMap, getAlliance());
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
        super.run();
        vision.updateFieldTarget(drive, !drive.isFieldPoseInitialized() || gamepad1.b);

        Pose2D pose = drive.getPose();
        telemetry.addData("Pinpoint X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Target Status", vision.getTargetStatus());
        telemetry.addData("Vision Pose Status", vision.getPoseStatus());
        telemetry.addData("Pipeline expected/actual", "%d / %d",
                vision.getExpectedPipelineIndex(), vision.getActualPipelineIndex());
        telemetry.addData("MT1 Tag Count / Age (ms)", "%d / %d",
                vision.getBotposeTagCount(), vision.getResultAgeMs());
        telemetry.addData("Area-qualified Alliance Tags", vision.getUsableTagCount());
        telemetry.addData("Raw MT1 Pose", vision.getRawRobotFieldPose()
                .map(Object::toString).orElse("NONE"));
        telemetry.addData("Corrected Vision Pose", vision.getCorrectedRobotFieldPose()
                .map(Object::toString).orElse("NONE"));
        telemetry.addData("HIVE Angle (deg)", vision.getHiveAngleDegrees());
        telemetry.addData("Robot Level Error (deg)", vision.getRobotLevelErrorDegrees());
        telemetry.addData("Field Pose Initialized", drive.isFieldPoseInitialized());
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
        telemetry.addData("Rejected New Frames / Forced", "%d / %b",
                drive.getRejectedVisionFrameCount(), drive.wasVisionCorrectionForced());
        telemetry.addData("Visible Tags", vision.getVisibleTagIds());
        telemetry.addData("Alliance Tag Visible", vision.hasVisibleAllianceTag());
        telemetry.addData("Tag Z (in)", vision.getVisibleTagHeightsInches());
        telemetry.addData("Cluster States", vision.getObservedClusterStates());
        telemetry.addData(
                "Selected Cluster",
                vision.getSelectedCluster().map(Enum::name).orElse("NONE")
        );
        telemetry.addData("Selected Tag", vision.getSelectedTagId());
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
        telemetry.addData("Shooter Target", shooter.getTargetVelocity());
        telemetry.addData("Shooter Left", shooter.getLeftVelocity());
        telemetry.addData("Shooter Right", shooter.getRightVelocity());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.addData("Shooter Pitch", shooter.getPitchPosition());
        telemetry.addData("Pitch Settled", shooter.isPitchReady());
        telemetry.addData("Aim Locked", autoAim.isLocked());
        telemetry.addData("Position Hold Calibrated",
                org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.positionHoldCalibrated);
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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            if (drive != null) drive.stop();
            if (shooter != null) shooter.stop();
            if (intake != null) {
                intake.stop();
                intake.raiseFlower();
            }
            if (transit != null) {
                transit.stopMotor();
                transit.close();
            }
            if (vision != null) vision.stop();
        }
    }
}
