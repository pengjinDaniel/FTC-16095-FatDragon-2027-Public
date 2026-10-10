
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Sensor: REV touch sensor", group = "Sensor")
@Disabled
public class SensorTouch extends LinearOpMode {
    TouchSensor touchSensor;

    @Override
    public void runOpMode() {

        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");

        waitForStart();

        while (opModeIsActive()) {

            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.update();
        }
    }
}
