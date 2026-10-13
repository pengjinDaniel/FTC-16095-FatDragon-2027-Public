package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
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
    // User confirmed 4-Bar pods, offsets, and both encoder directions.
    public static boolean pinpointConfigurationConfirmed = true;
    public static double pinpointXOffsetMm = -96.0;
    public static double pinpointYOffsetMm = -74.0;
    public static GoBildaPinpointDriver.EncoderDirection pinpointForwardPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection pinpointStrafePodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // Broad old-pose sanity bounds; VisionCalibrationFilter independently rejects unstable frames.
    public static double maximumVisionPositionDifferenceInches = 48.0;
    public static double maximumVisionHeadingDifferenceDegrees = 60.0;
    public static boolean positionHoldCalibrated = false;
    public static double positionHoldKp = 0.04;
    public static double positionHoldKi = 0.0;
    public static double positionHoldKd = 0.0;
    public static double positionHoldMaxPower = 0.25;
    public static double positionLockEnterToleranceInches = 0.75;
    public static double positionLockExitToleranceInches = 0.75;
    public static double positionLockMaxSpeedInchesPerSecond = 2.0;
    public static double positionLockStableTimeSeconds = 0.15;
    // Radian gains; preserve the former combined P/F response while enabling D.
    public static volatile double autoAimKp = -1.5;
    public static volatile double autoAimKi = 0.0;
    public static volatile double autoAimKd = -0.19;
    // Keep F at zero because the heading controller uses a zero setpoint.
    public static volatile double autoAimKf = 0.0;
    public static volatile double autoAimYawToTurnSign = 1.0;
    public static volatile double autoAimMaxTurnPower = 1;
    public static volatile double autoAimTestMaxTurnPower = 1;
    public static volatile double aimLockToleranceDegrees = 1.5;
    public static volatile double aimUnlockToleranceDegrees = 2.5;
    public static volatile double aimLockStableTimeSeconds = 0.15;
}
