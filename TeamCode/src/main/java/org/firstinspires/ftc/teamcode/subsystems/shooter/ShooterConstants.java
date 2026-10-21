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

    public static double idlePower = 0.5;
    public static double maxPower = 1;
    public static double targetVelocity = 1640.0;
    public static String pitchServoName = "pitchServo";
    public static double pitchMinPosition = 0;
    public static double pitchMaxPosition = 0.55;
    public static double pitchStartPosition = 0.5;
    public static double pitchSettleTimeSeconds = 0.2;
    public static double pitchChangeResetThreshold = 0.005;

    // Record the same horizontal robot-to-CELL distance shown by Blue/Red, in cm.
    // Velocity values are encoder ticks/s; pitch values are servo positions, not angles.
    public static double centerDistanceCm = 67.0;
    public static double centerVelocityTicksPerSecond = 1590.0;
    public static double centerPitchPosition = 0.52;
    public static double wallDistanceCm = 112.0;
    public static double wallVelocityTicksPerSecond = 1780.0;
    public static double wallPitchPosition = 0.37;

    // Enables the selected shared PID configuration; verify its gains on the robot.
    public static boolean velocityControlCalibrated = true;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0005;

    // Fractional speed tolerances; fixed power uses a stable measured reference.
    public static double readyEnterTolerance = 0.06;
    public static double readyExitTolerance = 0.06;
    public static double readyStableTimeSeconds = 0.05;
    // Cumulative target changes above this many ticks/s restart PID and readiness timing.
    public static double targetChangeResetThreshold = 30.0;

    private ShooterConstants() {
    }
}
