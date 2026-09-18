package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final GamepadEx gamepadEx;

    public IntakeCommand(Intake intake, GamepadEx gamepadEx) {
        this.intake = intake;
        this.gamepadEx = gamepadEx;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        updateIntakeRequest();
    }

    @Override
    public void execute() {
        updateIntakeRequest();
    }

    @Override
    public void end(boolean interrupted) {
        intake.idle();
    }

    private void updateIntakeRequest() {
        boolean intakeRequested = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                > TeleOpConstants.triggerThreshold;
        if (intakeRequested) {
            intake.active();
        } else {
            intake.idle();
        }
    }
}
