package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

public class DriveCommand extends CommandBase {
    private final Drive drive;
    private final GamepadEx gamepadEx;

    public DriveCommand(Drive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double forward = applyDeadband(gamepadEx.getLeftY());
        double strafe = applyDeadband(gamepadEx.getLeftX());
        double turn = applyDeadband(gamepadEx.getRightX());

        drive.moveRobot(forward, strafe, turn);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }
}
