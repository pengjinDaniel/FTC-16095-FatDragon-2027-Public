package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

@TeleOp(name = "Drive Test", group = "teleop")
public class DriveTest extends CommandOpMode {
    @Override
    public void initialize() {
        Drive drive = new Drive(hardwareMap);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx));
    }
}
