package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class DriveConstants {
    // User-measured robot footprint in mm (2026-11-04): forward/back length, side-to-side width.
    public static double robotLengthMm = 419.0;
    public static double robotWidthMm = 368.0;

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
    public static boolean pinpointConfigurationConfirmed = true;
    // Effective offset validated on this robot with the centered Pedro Pose Test.
    public static double pinpointXOffsetMm = 98.5;
    public static double pinpointYOffsetMm = -74.0;
    public static GoBildaPinpointDriver.EncoderDirection pinpointForwardPodDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection pinpointStrafePodDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static double maximumVisionPositionDifferenceInches = 1.0;
    public static double maximumVisionHeadingDifferenceDegrees = 5.0;
    public static int visionRejectedFramesBeforeOverride = 3;
    // Select before INIT after validating the fusion in Limelight Aim Test Blue/Red.
    public static volatile boolean fusedTeleOpLocalizationEnabled = false;
    public static boolean positionHoldCalibrated = true;
    public static double positionHoldKp = 0.12;
    public static double positionHoldKi = 0.0;
    public static double positionHoldKd = 0.007;
    public static double positionHoldMaxPower = 0.6;
    public static double positionLockEnterToleranceInches = 0.75;
    public static double positionLockExitToleranceInches = 0.75;
    public static double positionLockMaxSpeedInchesPerSecond = 2.0;
    public static double positionLockStableTimeSeconds = 0.15;
    public static volatile double autoAimKp = -1.05;
    public static volatile double autoAimKi = -0.0375;
    public static volatile double autoAimKd = -0.017;
    public static volatile double autoAimKf = 0.0;
    public static volatile double autoAimKs = 0.05; //0.054
    public static volatile double autoAimYawToTurnSign = 1.0;
    public static volatile double autoAimMaxTurnPower = 0.65;
    public static volatile double autoAimTestMaxTurnPower = 0.65;
    public static volatile double autoAimAcquisitionTurnPower = 0.65;
    public static volatile double aimLockToleranceDegrees = 3.5;
    public static volatile double aimUnlockToleranceDegrees = 5.0;
    public static volatile double aimLockStableTimeSeconds = 0.1;
}

