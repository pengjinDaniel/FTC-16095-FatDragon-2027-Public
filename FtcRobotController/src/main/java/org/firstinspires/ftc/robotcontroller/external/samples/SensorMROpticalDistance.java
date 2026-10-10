
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Sensor: MR ODS", group = "Sensor")
@Disabled
public class SensorMROpticalDistance extends LinearOpMode {

  OpticalDistanceSensor odsSensor;

  @Override
  public void runOpMode() {

    odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");

    waitForStart();

    while (opModeIsActive()) {

      telemetry.addData("Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("Normal", odsSensor.getLightDetected());

      telemetry.update();
    }
  }
}
