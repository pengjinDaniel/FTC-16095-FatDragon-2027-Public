
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Concept: Sound Resources", group="Concept")
@Disabled
public class ConceptSoundsASJava extends LinearOpMode {

    private boolean goldFound;
    private boolean silverFound;

    private boolean isX = false;
    private boolean isB = false;

    private boolean wasX = false;
    private boolean WasB = false;

    @Override
    public void runOpMode() {

        int silverSoundID = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        int goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());

        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);

        telemetry.addData("gold resource",   goldFound ?   "Found" : "NOT found\n Add gold.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", silverFound ? "Found" : "Not found\n Add silver.wav to /src/main/res/raw" );

        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X, B to play sounds.");
        telemetry.update();

        while (opModeIsActive()) {

            if (silverFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                telemetry.addData("Playing", "Resource Silver");
                telemetry.update();
            }

            if (goldFound && (isB = gamepad1.b) && !WasB) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                telemetry.addData("Playing", "Resource Gold");
                telemetry.update();
            }

            wasX = isX;
            WasB = isB;
        }
    }
}
