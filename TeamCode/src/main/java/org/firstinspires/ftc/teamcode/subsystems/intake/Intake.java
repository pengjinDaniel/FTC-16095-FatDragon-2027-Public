package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private double requestedPower = IntakeConstants.idlePower;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        intakeMotor.setDirection(IntakeConstants.intakeMotorDirection);
        idle();
    }

    public void idle() {
        setPower(IntakeConstants.idlePower);
    }

    public void active() {
        setPower(IntakeConstants.activePower);
    }

    public void stop() {
        setPower(0.0);
    }

    public double getRequestedPower() {
        return requestedPower;
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(requestedPower);
    }

    private void setPower(double power) {
        requestedPower = power;
        intakeMotor.setPower(requestedPower);
    }
}
