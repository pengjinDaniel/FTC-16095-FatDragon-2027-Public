package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit.FiringState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public class ShootAssistCommand extends CommandBase {
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final Vision vision;
    private final AutoAimController autoAim;
    private final PositionHoldController positionHold = new PositionHoldController();
    private final GamepadEx gamepadEx;
    private final boolean autoAimEnabled;

    private boolean canPrepareToFire;
    private boolean manualTranslation;

    public ShootAssistCommand(
            Drive drive,
            Shooter shooter,
            Intake intake,
            Transit transit,
            Vision vision,
            AutoAimController autoAim,
            GamepadEx gamepadEx
    ) {
        this(drive, shooter, intake, transit, vision, autoAim, gamepadEx, true);
    }

    public ShootAssistCommand(
            Drive drive,
            Shooter shooter,
            Intake intake,
            Transit transit,
            Vision vision,
            AutoAimController autoAim,
            GamepadEx gamepadEx,
            boolean autoAimEnabled
    ) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.transit = transit;
        this.vision = vision;
        this.autoAim = autoAim;
        this.gamepadEx = gamepadEx;
        this.autoAimEnabled = autoAimEnabled;
        addRequirements(drive, shooter, intake, transit);
    }

    @Override
    public void initialize() {
        shooter.stop();
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        resetFiringSequence(FiringState.WAITING);
    }

    @Override
    public void execute() {
        double forward = applyDeadband(gamepadEx.getLeftY());
        double strafe = applyDeadband(gamepadEx.getLeftX());
        manualTranslation = forward != 0.0 || strafe != 0.0;
        if (!autoAimEnabled || manualTranslation) {
            positionHold.reset();
        } else {
            positionHold.update(drive.getPose(), drive.getPinpointStatus(),
                    drive.getTranslationSpeedInchesPerSecond());
            forward = positionHold.getForwardPower();
            strafe = positionHold.getStrafePower();
        }

        boolean intakeRequested = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                > org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants.triggerThreshold;
        boolean jamRequested = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        intake.updateInputs(intakeRequested, jamRequested);
        double turnPower = updateShootRequest(jamRequested);
        if (autoAimEnabled) {
            drive.moveRobot(forward, strafe, turnPower);
        } else {
            drive.moveFieldCentric(forward, strafe, applyDeadband(gamepadEx.getRightX()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        resetFiringSequence(FiringState.IDLE);
        shooter.stop();
        intake.suspend();
        drive.stop();
    }

    public boolean canFire() {
        return transit.getFiringState() == FiringState.FEEDING;
    }

    public boolean canPrepareToFire() {
        return canPrepareToFire;
    }

    public FiringState getFiringState() {
        return transit.getFiringState();
    }

    public boolean isManualTranslation() {
        return manualTranslation;
    }

    private double updateShootRequest(boolean jamRequested) {
        if (jamRequested) {
            shooter.stop();
            autoAim.reset();
            resetFiringSequence(FiringState.WAITING);
            transit.reverse();
            return 0.0;
        }

        boolean validAimingTarget = vision.hasValidAimingTarget();
        double autoAimTurn = autoAimEnabled ? autoAim.calculateTurnPower(
                validAimingTarget,
                vision.getYawErrorDegrees()
        ) : 0.0;

        double distanceCm = validAimingTarget ? DistanceUnit.INCH.toCm(
                vision.getTargetHorizontalDistanceInches()) : Double.NaN;
        if (!shooter.setDistanceTarget(distanceCm)) {
            // Invalid position/calibration also cancels an already committed feed.
            resetFiringSequence(FiringState.WAITING);
            return validAimingTarget ? autoAimTurn : 0.0;
        }

        canPrepareToFire = shooter.isReady()
                && (!autoAimEnabled || autoAim.isLocked())
                && shooter.isPitchReady();
        transit.requestFeed(canPrepareToFire);

        return validAimingTarget ? autoAimTurn : 0.0;
    }

    private void resetFiringSequence(FiringState resetState) {
        if (transit.getState() == Transit.TransitState.OPEN) {
            intake.clearBallCount();
        }
        canPrepareToFire = false;
        transit.cancelFeeding();
        if (resetState != FiringState.IDLE) {
            transit.requestFeed(false);
        }
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }
}
