package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";

    public static DcMotorSimple.Direction leftShooterDirection =
            DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightShooterDirection =
            DcMotorSimple.Direction.FORWARD;

    public static double idlePower = 0.0;
    public static double maxPower = 1.0;
    public static String pitchServoName = "pitchServo";
    public static double pitchMinPosition = 0.0;
    public static double pitchMaxPosition = 0.53;
    public static double pitchStartPosition = 0.1;
    public static double pitchSettleTimeSeconds = 0.2;
    public static double pitchChangeResetThreshold = 0.005;
    public static boolean velocityControlCalibrated = false;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0005;

    public static double readyEnterTolerance = Double.NaN;
    public static double readyExitTolerance = Double.NaN;
    public static double readyStableTimeSeconds = Double.NaN;
    public static double targetChangeResetThreshold = Double.NaN;

    public static double[] distancePointsInches = new double[0];
    public static double[] velocityPointsTicksPerSecond = new double[0];
    public static double[] pitchPoints = new double[0];

    private ShooterConstants() {
    }
}
