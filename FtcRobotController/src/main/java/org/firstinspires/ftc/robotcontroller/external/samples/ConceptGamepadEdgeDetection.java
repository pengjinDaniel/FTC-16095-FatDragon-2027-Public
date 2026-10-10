






package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
@TeleOp(name="Concept: Gamepad Edge Detection", group ="Concept")
public class ConceptGamepadEdgeDetection extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetryButtonData();

            sleep(2000);
        }
    }

    public void telemetryButtonData() {
        telemetry.addData("Gamepad 1 Left Bumper Pressed", gamepad1.leftBumperWasPressed());
        telemetry.addData("Gamepad 1 Left Bumper Released", gamepad1.leftBumperWasReleased());
        telemetry.addData("Gamepad 1 Left Bumper Status", gamepad1.left_bumper);

        telemetry.addLine();

        telemetry.addData("Gamepad 1 Right Bumper Pressed", gamepad1.rightBumperWasPressed());
        telemetry.addData("Gamepad 1 Right Bumper Released", gamepad1.rightBumperWasReleased());
        telemetry.addData("Gamepad 1 Right Bumper Status", gamepad1.right_bumper);

        telemetry.addLine();

        telemetry.addData("Gamepad 1 Left Trigger Pressed", gamepad1.leftTriggerWasPressed());
        telemetry.addData("Gamepad 1 Left Trigger Released", gamepad1.leftTriggerWasReleased());
        telemetry.addData("Gamepad 1 Left Trigger Status", gamepad1.left_trigger_pressed);

        telemetry.addLine();

        telemetry.addData("Gamepad 1 Right Trigger Pressed", gamepad1.rightTriggerWasPressed());
        telemetry.addData("Gamepad 1 Right Trigger Released", gamepad1.rightTriggerWasReleased());
        telemetry.addData("Gamepad 1 Right Trigger Status", gamepad1.right_trigger_pressed);

        telemetry.addLine("\nTelemetry is updated every 2 seconds.");

        telemetry.update();
    }
}
