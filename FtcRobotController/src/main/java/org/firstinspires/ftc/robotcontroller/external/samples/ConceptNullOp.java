
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Concept: NullOp", group = "Concept")
@Disabled
public class ConceptNullOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {
  }

  @Override
  public void start() {
    runtime.reset();
  }

  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  @Override
  public void stop() {

  }
}
