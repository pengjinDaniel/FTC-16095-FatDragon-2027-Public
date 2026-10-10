package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TransitConstants {
    public static String transitMotorName = "transitMotor";
    public static DcMotorSimple.Direction transitMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static double forwardPower = 0.8;
    public static double reversePower = -0.8;
    public static String limitServoName = "limitServo";
    public static boolean limitPositionsCalibrated = false;
    public static double limitClosed = Double.NaN;
    public static double limitOpen = Double.NaN;
    public static double LIMIT_OPEN_LEAD_TIME_SECONDS = 0.150;

    private TransitConstants() {
    }
}
