
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@TeleOp(name = "Sensor: BNO055 IMU Calibration", group = "Sensor")
@Disabled
public class SensorBNO055IMUCalibration extends LinearOpMode
    {

    BNO055IMU imu;

    Orientation angles;


    @Override public void runOpMode() {

        telemetry.log().setCapacity(12);
        telemetry.log().add("");
        telemetry.log().add("Please refer to the calibration instructions");
        telemetry.log().add("contained in the Adafruit IMU calibration");
        telemetry.log().add("sample OpMode.");
        telemetry.log().add("");
        telemetry.log().add("When sufficient calibration has been reached,");
        telemetry.log().add("press the 'A' button to write the current");
        telemetry.log().add("calibration data to a file.");
        telemetry.log().add("");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        telemetry.log().add("Waiting for start...");

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        telemetry.log().add("...started...");

        while (opModeIsActive()) {

            if (gamepad1.a) {

                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

                String filename = "AdafruitIMUCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                while (gamepad1.a) {
                    telemetry.update();
                    idle();
                }
            }

            telemetry.update();
        }
    }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
                {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
