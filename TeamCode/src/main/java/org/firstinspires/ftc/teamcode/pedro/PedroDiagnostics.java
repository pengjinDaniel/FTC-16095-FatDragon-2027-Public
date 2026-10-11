package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Stop partially initialized motors and retain a useful configuration error on the phone. */
public final class PedroDiagnostics {
    private PedroDiagnostics() {}
    public static String failure(HardwareMap hardwareMap, RuntimeException failure) {
        int stopFailures = 0;
        String[] names = {DriveConstants.leftFrontMotorName, DriveConstants.leftBackMotorName,
                DriveConstants.rightFrontMotorName, DriveConstants.rightBackMotorName};
        for (String name : names) {
            if (name == null || name.trim().isEmpty() || hardwareMap == null) continue;
            try {
                DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
                if (motor != null) motor.setPower(0);
            } catch (RuntimeException stopFailure) { stopFailures++; }
        }
        String message = failure.getClass().getSimpleName() + ": " + failure.getMessage();
        if (stopFailures > 0) message += "; motor stop communication errors=" + stopFailures;
        return message;
    }
}
