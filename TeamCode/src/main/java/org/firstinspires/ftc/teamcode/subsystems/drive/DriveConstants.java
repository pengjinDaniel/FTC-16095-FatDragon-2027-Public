package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveConstants {
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    public static DcMotorSimple.Direction leftFrontMotorDirection =
            DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftBackMotorDirection =
            DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection =
            DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightBackMotorDirection =
            DcMotorSimple.Direction.FORWARD;
    public static double joystickDeadband = 0.03;
    public static double strafingBalance = 1.1;
}
