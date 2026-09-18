package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public class ShootAssistCommand extends CommandBase {
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final Vision vision;
    private final AutoAimController autoAim;
    private final GamepadEx gamepadEx;
    private final ElapsedTime limitOpenTimer = new ElapsedTime();

    private boolean active;
    private boolean canPrepareToFire;
    private FiringState firingState = FiringState.IDLE;

    public enum FiringState {
        IDLE,
        WAITING,
        LIMIT_OPENING,
        FEEDING
    }

    public ShootAssistCommand(
            Drive drive,
            Shooter shooter,
            Intake intake,
            Transit transit,
            Vision vision,
            AutoAimController autoAim,
            GamepadEx gamepadEx
    ) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.transit = transit;
        this.vision = vision;
        this.autoAim = autoAim;
        this.gamepadEx = gamepadEx;
        addRequirements(drive, shooter, intake, transit);
    }

    @Override
    public void initialize() {
        active = true;
        shooter.idle();
        intake.idle();
        transit.close();
        autoAim.reset();
        resetFiringSequence(FiringState.WAITING);
    }

    @Override
    public void execute() {
        double forward = applyDeadband(gamepadEx.getLeftY());
        double strafe = applyDeadband(gamepadEx.getLeftX());
        double autoAimTurn = updateShootRequest();

        drive.moveRobot(forward, strafe, autoAimTurn);
    }

    @Override
    public void end(boolean interrupted) {
        active = false;
        autoAim.reset();
        resetFiringSequence(FiringState.IDLE);
        shooter.idle();
        drive.stop();
    }

    public boolean isActive() {
        return active;
    }

    public boolean canFire() {
        return firingState == FiringState.FEEDING;
    }

    public boolean canPrepareToFire() {
        return canPrepareToFire;
    }

    public FiringState getFiringState() {
        return firingState;
    }

    public double getLimitOpenTimerElapsedSeconds() {
        if (firingState == FiringState.LIMIT_OPENING) {
            return limitOpenTimer.seconds();
        }
        if (firingState == FiringState.FEEDING) {
            return TransitConstants.LIMIT_OPEN_LEAD_TIME_SECONDS;
        }
        return 0.0;
    }

    public double getLimitOpenTimerProgress() {
        double leadTime = TransitConstants.LIMIT_OPEN_LEAD_TIME_SECONDS;
        if (firingState == FiringState.FEEDING) {
            return 1.0;
        }
        if (firingState != FiringState.LIMIT_OPENING
                || !Double.isFinite(leadTime)
                || leadTime <= 0.0) {
            return 0.0;
        }
        return Math.min(1.0, limitOpenTimer.seconds() / leadTime);
    }

    private double updateShootRequest() {
        boolean validShootingTarget = vision.hasValidShootingTarget();
        double autoAimTurn = autoAim.calculateTurnPower(
                validShootingTarget,
                vision.getYawErrorDegrees()
        );

        if (validShootingTarget) {
            boolean targetAccepted = shooter.setTargetFromDistance(
                    vision.getTargetHorizontalDistanceInches()
            );
            if (!targetAccepted) {
                shooter.idle();
            }
        }

        canPrepareToFire = active
                && validShootingTarget
                && vision.getTargetHiveState() == HiveState.HIGH
                && shooter.isReady()
                && autoAim.isLocked();
        updateFiringSequence(canPrepareToFire);

        return validShootingTarget ? autoAimTurn : 0.0;
    }

    private void updateFiringSequence(boolean readyToOpenLimit) {
        if (!readyToOpenLimit) {
            resetFiringSequence(FiringState.WAITING);
            return;
        }

        double leadTime = TransitConstants.LIMIT_OPEN_LEAD_TIME_SECONDS;
        if (!Double.isFinite(leadTime) || leadTime < 0.0) {
            resetFiringSequence(FiringState.WAITING);
            return;
        }

        if (firingState != FiringState.LIMIT_OPENING
                && firingState != FiringState.FEEDING) {
            transit.open();
            firingState = FiringState.LIMIT_OPENING;
            limitOpenTimer.reset();
        } else {
            transit.open();
        }

        if (firingState == FiringState.LIMIT_OPENING
                && limitOpenTimer.seconds() >= leadTime) {
            firingState = FiringState.FEEDING;
        }

        boolean feeding = firingState == FiringState.FEEDING;
        shooter.setFeeding(feeding);
        if (feeding) {
            intake.active();
        } else {
            intake.idle();
        }
    }

    private void resetFiringSequence(FiringState resetState) {
        canPrepareToFire = false;
        firingState = resetState;
        limitOpenTimer.reset();
        shooter.setFeeding(false);
        transit.close();
        intake.idle();
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }
}
