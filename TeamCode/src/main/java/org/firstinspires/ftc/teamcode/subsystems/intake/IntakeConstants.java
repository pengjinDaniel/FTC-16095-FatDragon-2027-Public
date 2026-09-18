package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeConstants {
    public static String intakeMotorName = "intakemotor";
    public static DcMotorSimple.Direction intakeMotorDirection = DcMotorSimple.Direction.REVERSE;

    public static double idlePower = 0.3;
    public static double activePower = 0.9;

    private IntakeConstants() {
    }
}
