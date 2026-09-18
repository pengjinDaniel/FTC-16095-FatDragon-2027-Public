package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";

    public static DcMotorSimple.Direction leftShooterDirection =
            DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightShooterDirection =
            DcMotorSimple.Direction.REVERSE;

    public static double idlePower = 0.1;
    public static double maxPower = 1.0;

    // Provisional 2026 starting values. Retune on the 2027 shooter.
    public static double minVelocity = 940.0;
    public static double maxVelocity = 1520.0;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0005;

    public static double readyEnterTolerance = 40.0;
    public static double readyExitTolerance = 80.0;
    public static double readyStableTimeSeconds = 0.20;
    public static double targetChangeResetThreshold = 40.0;

    // Leave at zero until a loaded-shot test demonstrates that a feed boost is needed.
    public static double feedLoadBoost = 0.0;

    // Populate both arrays with matching, ascending, measured 2027 values before firing.
    public static double[] distancePointsInches = new double[0];
    public static double[] velocityPointsTicksPerSecond = new double[0];

    private ShooterConstants() {
    }
}
