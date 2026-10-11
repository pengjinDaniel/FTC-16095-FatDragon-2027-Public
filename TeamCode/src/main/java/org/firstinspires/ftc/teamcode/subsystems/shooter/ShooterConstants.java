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
    public static double maxPower = 0.8;
    public static double fixedPower = 0.8;
    public static String pitchServoName = "pitchServo";
    public static double pitchMinPosition = 0.0;
    public static double pitchMaxPosition = 0.53;
    public static double pitchStartPosition = 0.52;
    public static double pitchSettleTimeSeconds = 0.2;
    public static double pitchChangeResetThreshold = 0.005;
    public static boolean velocityControlCalibrated = false;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0005;

    // Fractional speed tolerances; fixed power uses a stable measured reference.
    public static double readyEnterTolerance = 0.05;
    public static double readyExitTolerance = 0.05;
    public static double readyStableTimeSeconds = 0.05;
    public static double targetChangeResetThreshold = 0.0;

    public static double[] distancePointsInches = new double[0];
    public static double[] velocityPointsTicksPerSecond = new double[0];
    public static double[] pitchPoints = new double[0];

    private ShooterConstants() {
    }
}
