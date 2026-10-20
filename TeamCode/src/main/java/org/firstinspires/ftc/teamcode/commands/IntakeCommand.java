package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final Transit transit;
    private final GamepadEx gamepadEx;

    public IntakeCommand(Intake intake, Transit transit, GamepadEx gamepadEx) {
        this.intake = intake;
        this.transit = transit;
        this.gamepadEx = gamepadEx;
        addRequirements(intake, transit);
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
        intake.suspend();
        transit.stopMotor();
    }

    private void updateIntakeRequest() {
        boolean intakeRequested = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                > TeleOpConstants.triggerThreshold;
        boolean jamRequested = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        intake.updateInputs(intakeRequested, jamRequested);
        transit.updateIntakeRequest(intakeRequested, jamRequested);
    }
}
