package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class TransitConstants {
    public static String transitMotorName = "transitMotor";
    public static DcMotorSimple.Direction transitMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static double forwardPower = 0.7;
    public static double reversePower = -0.7;
    public static String limitServoName = "limitServo";
    public static boolean limitPositionsCalibrated = true;
    public static double limitClosed = 0.4;
    public static double limitOpen = 0.7;
    public static double LIMIT_OPEN_LEAD_TIME_SECONDS = 0.150;

    private TransitConstants() {
    }
}
