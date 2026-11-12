package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";

    public static DcMotorSimple.Direction leftShooterDirection =
            DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightShooterDirection =
            DcMotorSimple.Direction.REVERSE;

    public static double idlePower = 0.55;
    public static double maxPower = 1;
    public static double targetVelocity = 1700.0;
    public static String pitchServoName = "pitchServo";
    public static double pitchMinPosition = 0.2;
    public static double pitchMaxPosition = 1.0;
    public static double pitchStartPosition = 0.5;
    public static double pitchSettleTimeSeconds = 0.2;
    public static double pitchChangeResetThreshold = 0.005;
    public static double[][] nearShotPoints = {
            {57.7, 1550, 0.6},
            {75.0, 1570, 0.5},
            {95.8, 1590, 0.45},
            {113.5, 1620, 0.4},
            {121.3, 1630, 0.35}
    };

    public static double[][] farShotPoints = {
            {130.9, 1700, 0.37},
            {152.1, 1720, 0.3},
            {163.0, 1740, 0.28},
            {183.4, 1750, 0.26},
            {215.7, 1870, 0.24}
    };
    public static boolean velocityControlCalibrated = true;
    public static double kP = 0.004;  //0.004
    public static double kI = 0.0000495;
    public static double kD = 0.0;
    public static double kF = 0.000459 35;  //0.00063
    public static double readyEnterTolerance = 0.02;
    public static double readyExitTolerance = 0.025;
    public static double readyStableTimeSeconds = 0.05;
    public static double targetChangeResetThreshold = 30.0;

    private ShooterConstants() {
    }
}
