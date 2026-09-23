package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveConstants {
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftBackMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightBackMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static double joystickDeadband = 0.03;

    public static String pinpointName = "pinpoint";
    // Measure both offsets and verify both pod signs on the 2027 robot before enabling hold.
    public static double pinpointXOffsetMm = 0.0;
    public static double pinpointYOffsetMm = 0.0;
    public static GoBildaPinpointDriver.EncoderDirection pinpointForwardPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection pinpointStrafePodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static boolean positionHoldCalibrated = false;
    public static double positionHoldKp = 0.04;
    public static double positionHoldKi = 0.0;
    public static double positionHoldKd = 0.0;
    public static double positionHoldMaxPower = 0.25;
    public static double positionLockEnterToleranceInches = 0.75;
    public static double positionLockExitToleranceInches = 0.75;
    public static double positionLockMaxSpeedInchesPerSecond = 2.0;
    public static double positionLockStableTimeSeconds = 0.15;

    // Provisional starting values. Tune the yaw sign and gains on the 2027 robot.
    public static double autoAimKp = 0.02;
    public static double autoAimKi = 0.0;
    public static double autoAimKd = 0.0;
    public static double autoAimKf = 0.0;
    public static double autoAimYawToTurnSign = 1.0;
    public static double autoAimMaxTurnPower = 0.4;
    public static double aimLockToleranceDegrees = 1.5;
    public static double aimUnlockToleranceDegrees = 2.5;
    public static double aimLockStableTimeSeconds = 0.15;
}
