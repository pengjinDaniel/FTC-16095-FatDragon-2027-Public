package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";

    public static DcMotorSimple.Direction leftShooterDirection =
            DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightShooterDirection =
            DcMotorSimple.Direction.FORWARD;

    public static double idlePower = 0.0;
    public static double maxPower = 1.0;
    public static String pitchServoName = "pitchServo";
    public static double pitchMinPosition = 0.0;
    public static double pitchMaxPosition = 0.5;
    public static double pitchStartPosition = 0.1;
    public static double pitchSettleTimeSeconds = 0.2;
    public static double pitchChangeResetThreshold = 0.005;

    // Set measured gains and readiness thresholds before enabling velocity control.
    public static boolean velocityControlCalibrated = false;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double readyEnterTolerance = Double.NaN;
    public static double readyExitTolerance = Double.NaN;
    public static double readyStableTimeSeconds = Double.NaN;
    public static double targetChangeResetThreshold = Double.NaN;

    // Populate all three arrays with matching, ascending, measured 2027 values before firing.
    public static double[] distancePointsInches = new double[0];
    public static double[] velocityPointsTicksPerSecond = new double[0];
    public static double[] pitchPoints = new double[0];

    private ShooterConstants() {
    }
}
