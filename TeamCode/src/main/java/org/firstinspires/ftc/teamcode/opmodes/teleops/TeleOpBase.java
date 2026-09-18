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

public abstract class TeleOpBase extends CommandOpMode {
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Vision vision;
    private AutoAimController autoAim;
    private ShootAssistCommand shootAssistCommand;

    protected abstract Alliance getAlliance();

    @Override
    public void initialize() {
        Drive drive = new Drive(hardwareMap);
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
        intake.setDefaultCommand(new IntakeCommand(intake, gamepadEx));

        new Trigger(
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                        > TeleOpConstants.triggerThreshold
        ).whileActiveContinuous(shootAssistCommand);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Visible Tags", vision.getVisibleTagIds());
        telemetry.addData("Alliance Tag Visible", vision.hasVisibleAllianceTag());
        telemetry.addData("Tag Z (in)", vision.getVisibleTagHeightsInches());
        telemetry.addData("Calibrated Tag Transforms", vision.getCalibratedTransformTagIds());
        telemetry.addData("Cluster States", vision.getObservedClusterStates());
        telemetry.addData(
                "Selected Cluster",
                vision.getSelectedCluster().map(Enum::name).orElse("NONE")
        );
        telemetry.addData("Selected Tag", vision.getSelectedTagId());
        telemetry.addData("Hive State", vision.getTargetHiveState());
        telemetry.addData("Valid HIGH Target", vision.hasValidShootingTarget());
        telemetry.addData(
                "Representative Tag Pose (robot)",
                vision.getRepresentativeTagPoseRobotSpace()
                        .map(Object::toString)
                        .orElse("NONE")
        );
        telemetry.addData(
                "CELL Center Pose (robot)",
                vision.getTargetCellPoseRobotSpace()
                        .map(Object::toString)
                        .orElse("NONE")
        );
        telemetry.addData("3D Range (in)", vision.getTargetRange3DInches());
        telemetry.addData(
                "Horizontal Distance (in)",
                vision.getTargetHorizontalDistanceInches()
        );
        telemetry.addData("Yaw Error (deg)", vision.getYawErrorDegrees());
        telemetry.addData("Shooter Target", shooter.getTargetVelocity());
        telemetry.addData("Shooter Left", shooter.getLeftVelocity());
        telemetry.addData("Shooter Right", shooter.getRightVelocity());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.addData("Aim Locked", autoAim.isLocked());
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
        telemetry.addData("Limit Servo", transit.getState());
        telemetry.update();
    }
}
