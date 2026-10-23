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
    // After initialization, vision must differ from Pinpoint by less than 5 cm and 5 degrees.
    public static double maximumVisionPositionDifferenceInches = 5.0 / 2.54;
    public static double maximumVisionHeadingDifferenceDegrees = 5.0;
    public static boolean positionHoldCalibrated = false;
    public static double positionHoldKp = 0.04;
    public static double positionHoldKi = 0.0;
    public static double positionHoldKd = 0.0;
    public static double positionHoldMaxPower = 0.25;
    public static double positionLockEnterToleranceInches = 0.75;
    public static double positionLockExitToleranceInches = 0.75;
    public static double positionLockMaxSpeedInchesPerSecond = 2.0;
    public static double positionLockStableTimeSeconds = 0.15;
    // P/I/D use radian error; F preserves the original error-scaled feedforward.
    public static volatile double autoAimKp = -1;
    public static volatile double autoAimKi = 0.0;
    public static volatile double autoAimKd = -0.01;
    public static volatile double autoAimKf = 0.0;
    public static volatile double autoAimYawToTurnSign = 1.0;
    public static volatile double autoAimMaxTurnPower = 0.7;
    public static volatile double autoAimTestMaxTurnPower = 0.7;
    public static volatile double aimLockToleranceDegrees = 10.0;
    public static volatile double aimUnlockToleranceDegrees = 10.0;
    public static volatile double aimLockStableTimeSeconds = 0.05;
}
