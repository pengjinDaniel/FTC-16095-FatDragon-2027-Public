package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit.FiringState;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public class ShootAssistCommand extends CommandBase {
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final Vision vision;
    private final AutoAimController autoAim;
    private final GamepadEx gamepadEx;
    private final boolean autoAimEnabled;

    private boolean canPrepareToFire;
    private boolean manualTranslation;
    private boolean fedSinceReset;
    private boolean aimWindowReady;
    private CellCluster previousAimCluster;
    private double previousYawErrorDegrees = Double.NaN;
    private long previousAimSampleNanos;

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
        if (!autoAimEnabled) shooter.stop();
        autoAim.reset();
        manualTranslation = false;
        resetFiringSequence(FiringState.WAITING);
    }

    @Override
    public void execute() {
        double forward = applyDeadband(gamepadEx.getLeftY());
        double strafe = applyDeadband(gamepadEx.getLeftX());
        manualTranslation = forward != 0.0 || strafe != 0.0;

        boolean intakeRequested = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                > org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants.triggerThreshold;
        boolean jamRequested = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean intakeReverseRequested = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        intake.updateInputs(intakeRequested, jamRequested, true);
        double turnPower = updateShootRequest(jamRequested, intakeReverseRequested);
        if (intakeReverseRequested && !jamRequested) {
            transit.forwardForIntakeReverse();
        }
        if (autoAimEnabled && !manualTranslation) {
            drive.moveRobot(forward, strafe, turnPower);
        } else {
            drive.moveFieldCentric(forward, strafe,
                    autoAimEnabled ? turnPower : applyDeadband(gamepadEx.getRightX()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        autoAim.reset();
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

    public boolean isAimWindowReady() {
        return aimWindowReady;
    }

    private double updateShootRequest(boolean jamRequested, boolean intakeReverseRequested) {
        if (jamRequested) {
            shooter.stop();
            autoAim.reset();
            resetFiringSequence(FiringState.WAITING);
            transit.reverse();
            return 0.0;
        }

        boolean validAimingTarget = vision.hasValidAimingTarget();
        double yawErrorDegrees = vision.getYawErrorDegrees();
        boolean poseCorrected = autoAimEnabled && vision.wasCorrectionApplied();
        double autoAimTurn = 0.0;
        if (autoAimEnabled) {
            if (poseCorrected) autoAim.reset();
            autoAimTurn = autoAim.calculateTurnPower(
                    validAimingTarget && !vision.shouldPauseForAimCalibration(), yawErrorDegrees);
        }

        double distanceCm = validAimingTarget ? DistanceUnit.INCH.toCm(
                vision.getTargetHorizontalDistanceInches()) : Double.NaN;
        if (!shooter.setDistanceTarget(distanceCm)) {
            resetFiringSequence(FiringState.WAITING);
            return validAimingTarget ? autoAimTurn : 0.0;
        }

        if (autoAimEnabled && intakeReverseRequested) {
            resetFiringSequence(FiringState.WAITING);
            return autoAimTurn;
        }

        canPrepareToFire = shooter.isReady() && shooter.isPitchReady();
        if (autoAimEnabled) {
            aimWindowReady = updateAimWindow(yawErrorDegrees, poseCorrected);
            double targetVelocity = shooter.getTargetVelocity();
            canPrepareToFire = canPrepareToFire && aimWindowReady
                    && Math.abs(shooter.getRightVelocity() - targetVelocity)
                    <= targetVelocity * ShooterConstants.readyExitTolerance;
            transit.requestTrackedFeed(canPrepareToFire);
            fedSinceReset |= canFire();
        } else {
            transit.requestFeed(canPrepareToFire);
        }

        return autoAimTurn;
    }

    private boolean updateAimWindow(double yawErrorDegrees, boolean poseCorrected) {
        if (!vision.isAimCalibrationReady() || !Double.isFinite(yawErrorDegrees)) {
            previousYawErrorDegrees = Double.NaN;
            return false;
        }

        CellCluster cluster = vision.getSelectedCluster().orElse(null);
        long now = System.nanoTime();
        double elapsedSeconds = (now - previousAimSampleNanos) / 1_000_000_000.0;
        double yawChange = Math.IEEEremainder(yawErrorDegrees - previousYawErrorDegrees, 360.0);
        boolean comparable = !poseCorrected && cluster == previousAimCluster;
        previousAimCluster = cluster;
        previousYawErrorDegrees = yawErrorDegrees;
        previousAimSampleNanos = now;

        double lookaheadSeconds = DriveConstants.autoAimShotLookaheadSeconds;
        double toleranceDegrees = DriveConstants.aimLockToleranceDegrees;
        if (!comparable || elapsedSeconds <= 0.0 || !Double.isFinite(yawChange)
                || !Double.isFinite(lookaheadSeconds) || lookaheadSeconds < 0.0
                || !Double.isFinite(toleranceDegrees) || toleranceDegrees < 0.0) {
            return false;
        }
        // The yaw-error rate includes the target bearing change during translation.
        double predictedError = yawErrorDegrees + yawChange * lookaheadSeconds / elapsedSeconds;
        return Math.abs(yawErrorDegrees) <= toleranceDegrees
                && Math.abs(predictedError) <= toleranceDegrees;
    }

    private void resetFiringSequence(FiringState resetState) {
        if (fedSinceReset || transit.getState() == Transit.TransitState.OPEN) {
            intake.clearBallCount();
        }
        fedSinceReset = false;
        aimWindowReady = false;
        previousYawErrorDegrees = Double.NaN;
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
