
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Sensor: digital channel", group = "Sensor")
@Disabled
public class SensorDigitalTouch extends LinearOpMode {
    DigitalChannel digitalTouch;

    @Override
    public void runOpMode() {

        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (digitalTouch.getState() == false) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }

            telemetry.update();
        }
    }
}
