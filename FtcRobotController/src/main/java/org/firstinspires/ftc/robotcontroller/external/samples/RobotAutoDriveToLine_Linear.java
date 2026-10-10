
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


@Autonomous(name="Robot: Auto Drive To Line", group="Robot")
@Disabled
public class RobotAutoDriveToLine_Linear extends LinearOpMode {

    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;

    NormalizedColorSensor colorSensor;

    static final double     WHITE_THRESHOLD = 0.5;
    static final double     APPROACH_SPEED  = 0.25;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        colorSensor.setGain(15);

        while (opModeInInit()) {

            telemetry.addData("Status", "Ready to drive to white line.");

            getBrightness();
        }

        leftDrive.setPower(APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);

        while (opModeIsActive() && (getBrightness() < WHITE_THRESHOLD)) {
            sleep(5);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    double getBrightness() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
}
