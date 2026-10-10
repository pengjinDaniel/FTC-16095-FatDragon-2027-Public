
package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Concept: Robot Hardware Class", group="Robot")
@Disabled
public class ConceptExternalHardwareClass extends LinearOpMode {

    RobotHardware   robot       = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive        = 0;
        double turn         = 0;
        double arm          = 0;
        double handOffset   = 0;

        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            robot.driveRobot(drive, turn);

            if (gamepad1.right_bumper)
                handOffset += robot.HAND_SPEED;
            else if (gamepad1.left_bumper)
                handOffset -= robot.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            robot.setHandPositions(handOffset);

            if (gamepad1.y)
                arm = robot.ARM_UP_POWER;
            else if (gamepad1.a)
                arm = robot.ARM_DOWN_POWER;
            else
                arm = 0;

            robot.setArmPower(arm);

            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Arm Power",  "%.2f", arm);
            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.update();

            sleep(50);
        }
    }
}
