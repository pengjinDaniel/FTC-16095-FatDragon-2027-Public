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
    public static double pitchMinPosition = 0;
    public static double pitchMaxPosition = 0.55;
    public static double pitchStartPosition = 0.5;
    public static double pitchSettleTimeSeconds = 0.2;
    public static double pitchChangeResetThreshold = 0.005;

    // Measure horizontally from robot center to the current CELL's opening center, in cm.
    // Remeasure distances previously calibrated to the four-AprilTag centroid.
    // Velocity values are encoder ticks/s; pitch values are servo positions, not angles.
    public static double centerDistanceCm = 54.1;
    public static double centerVelocityTicksPerSecond = 1495.0;
    public static double centerPitchPosition = 0.54;
    public static double wallDistanceCm = 154.0;
    public static double wallVelocityTicksPerSecond = 1660.0;
    public static double wallPitchPosition = 0.27;

    // Extra points: {distanceCm, velocityTicksPerSecond, pitchServoPosition}.
    // Interpolated starting values, not measurements; replace each row with measured data.
    // Points may be in any order, but distances must be unique (including center/wall).
    // Use an empty array to return to center/wall-only interpolation.
    public static double[][] additionalShotPoints = {
            {89.5, 1530.0, 0.36},
            {115.3, 1560.0, 0.34},
            {76.4, 1510.0, 0.43},
            {132.4, 1610, 0.29}
    };

    // Enables the selected shared PID configuration; verify its gains on the robot.
    public static boolean velocityControlCalibrated = true;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    // Feedforward is power per tick/s; leave feedback headroom at the shot targets.
    public static double kF = 0.0006;

    // Fractional speed tolerances; fixed power uses a stable measured reference.
    public static double readyEnterTolerance = 0.06;
    public static double readyExitTolerance = 0.06;
    public static double readyStableTimeSeconds = 0.05;
    // Cumulative target changes above this many ticks/s restart PID and readiness timing.
    public static double targetChangeResetThreshold = 30.0;

    private ShooterConstants() {
    }
}
