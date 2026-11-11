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
            {55.8, 1540, 0.67},
            {64.4, 1590, 0.55},
            {96.0, 1620, 0.46},
            {113.2, 1640, 0.43},
            {120.5, 1670, 0.36}
    };

    public static double[][] farShotPoints = {
            {130.9, 1700, 0.37},
            {152.1, 1720, 0.3},
            {163.0, 1740, 0.28},
            {183.4, 1750, 0.26},
            {215.7, 1870, 0.24}
    };

    // Enables the selected shared PID configuration; verify its gains on the robot.
    public static boolean velocityControlCalibrated = true;
    public static double kP = 0.004;  //0.004
    public static double kI = 0.0000475;
    public static double kD = 0.0;
    // Feedforward is power per tick/s; leave feedback headroom at the shot targets.
    public static double kF = 0.00046;  //0.00063

    // Fractional speed tolerances; fixed power uses a stable measured reference.
    public static double readyEnterTolerance = 0.02;
    public static double readyExitTolerance = 0.025;
    public static double readyStableTimeSeconds = 0.05;
    // Cumulative target changes above this many ticks/s restart PID and readiness timing.
    public static double targetChangeResetThreshold = 30.0;

    private ShooterConstants() {
    }
}
