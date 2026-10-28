package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class IntakeConstants {
    public static String intakeMotorName = "intakeMotor";
    public static DcMotorSimple.Direction intakeMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static String colorSensorName = "cds";
    public static String leftFlowerServoName = "leftFlowerServo";
    public static String rightFlowerServoName = "rightFlowerServo";

    public static double activePower = 1;
    public static double feedPower = 0.6;
    public static double fullReversePower = -1;
    public static double jamPower = -1;
    public static int maxBallCount = 4;
    public static double ballEnterDistanceCm = 2.0;
    public static double ballExitDistanceCm = 2.5;

    public static boolean flowerPositionsCalibrated = true;
    public static double leftFlowerRaisedPosition = 0.5;
    public static double rightFlowerRaisedPosition = 0.5;
    public static double leftFlowerDeployedPosition = 0.2;
    public static double rightFlowerDeployedPosition = 0.2;

    private IntakeConstants() {
    }
}
