package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp(name="Concept: Gamepad Rumble", group ="Concept")
public class ConceptGamepadRumble extends LinearOpMode
{
    boolean lastA = false;
    boolean lastLB = false;
    boolean highLevel = false;
    boolean secondHalf = false;

    Gamepad.RumbleEffect customRumbleEffect;
    ElapsedTime runtime = new ElapsedTime();

    final double HALF_TIME = 60.0;
    final double TRIGGER_THRESHOLD  = 0.75;

    @Override
    public void runOpMode()
    {
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 0.0, 250)
                .addStep(0.0, 0.0, 250)
                .addStep(1.0, 0.0, 250)
                .build();

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            boolean currentA = gamepad1.a ;
            boolean currentLB = gamepad1.left_bumper ;

            telemetry.addData(">", "Are we RUMBLING? %s\n", gamepad1.isRumbling() ? "YES" : "no" );

            if ((runtime.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(customRumbleEffect);
                secondHalf =true;
            }

            if (!secondHalf) {
                telemetry.addData(">", "Halftime Alert Countdown: %3.0f Sec \n", (HALF_TIME - runtime.seconds()) );
            }


            if (currentLB) {
                gamepad1.rumble(gamepad1.left_trigger, gamepad1.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);

                telemetry.addData(">", "Squeeze triggers to control rumbles");
                telemetry.addData("> : Rumble", "Left: %.0f%%   Right: %.0f%%", gamepad1.left_trigger * 100, gamepad1.right_trigger * 100);
            } else {
                if (lastLB) {
                    gamepad1.stopRumble();
                }

                telemetry.addData(">", "Hold Left-Bumper to test Manual Rumble");
                telemetry.addData(">", "Press A (Cross) for three blips");
                telemetry.addData(">", "Squeeze right trigger slowly for 1 blip");
            }
            lastLB = currentLB;


            if (currentA && !lastA) {
                if (!gamepad1.isRumbling())
                    gamepad1.rumbleBlips(3);
            }
            lastA = currentA;


            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!highLevel) {
                    gamepad1.rumble(0.9, 0, 200);
                    highLevel = true;
                }
            } else {
                highLevel = false;
            }

            telemetry.update();
            sleep(10);
        }
    }
}
