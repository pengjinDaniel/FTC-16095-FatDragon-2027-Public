package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeConstants {
    public static String intakeMotorName = "intakeMotor";
    public static DcMotorSimple.Direction intakeMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static String colorSensorName = "cds";
    public static String leftFlowerServoName = "leftFlowerServo";
    public static String rightFlowerServoName = "rightFlowerServo";

    public static double activePower = 0.9;
    public static double fullReversePower = -0.5;
    public static double jamPower = -0.9;
    public static int maxBallCount = 4;
    public static double ballEnterDistanceCm = 2.0;
    public static double ballExitDistanceCm = 2.5;

    // Populate measured positions before enabling the flower servos.
    public static boolean flowerPositionsCalibrated = false;
    public static double leftFlowerRaisedPosition = Double.NaN;
    public static double rightFlowerRaisedPosition = Double.NaN;
    public static double leftFlowerDeployedPosition = Double.NaN;
    public static double rightFlowerDeployedPosition = Double.NaN;

    private IntakeConstants() {
    }
}
