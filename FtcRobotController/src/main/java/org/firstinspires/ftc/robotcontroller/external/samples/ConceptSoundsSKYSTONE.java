
package org.firstinspires.ftc.robotcontroller.external.samples;

import android.content.Context;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="SKYSTONE Sounds", group="Concept")
@Disabled
public class ConceptSoundsSKYSTONE extends LinearOpMode {

    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;

    @Override
    public void runOpMode() {

        int     soundIndex      = 0;
        int     soundID         = -1;
        boolean was_dpad_up     = false;
        boolean was_dpad_down   = false;

        Context myApp = hardwareMap.appContext;

        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        while (!isStopRequested()) {

            if (gamepad1.dpad_down && !was_dpad_down) {
                soundIndex = (soundIndex + 1) % sounds.length;
            }

            if (gamepad1.dpad_up && !was_dpad_up) {
                soundIndex = (soundIndex + sounds.length - 1) % sounds.length;
            }

            if (gamepad1.right_bumper && !soundPlaying) {

                if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0){

                    soundPlaying = true;

                    SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }} );
                }
            }

            was_dpad_up     = gamepad1.dpad_up;
            was_dpad_down   = gamepad1.dpad_down;

            telemetry.addData("", "Use DPAD up/down to choose sound.");
            telemetry.addData("", "Press Right Bumper to play sound.");
            telemetry.addData("", "");
            telemetry.addData("Sound >", sounds[soundIndex]);
            telemetry.addData("Status >", soundPlaying ? "Playing" : "Stopped");
            telemetry.update();
        }
    }
}
