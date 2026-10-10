
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;


@TeleOp(name="Concept: Sound Files", group="Concept")
@Disabled
public class ConceptSoundsOnBotJava extends LinearOpMode {

    private String soundPath = "/FIRST/blocks/sounds";
    private File goldFile   = new File("/sdcard" + soundPath + "/gold.wav");
    private File silverFile = new File("/sdcard" + soundPath + "/silver.wav");

    private boolean isX = false;
    private boolean isB = false;

    private boolean wasX = false;
    private boolean WasB = false;

    @Override
    public void runOpMode() {

        boolean goldFound   = goldFile.exists();
        boolean silverFound = silverFile.exists();

        telemetry.addData("gold sound",   goldFound ?   "Found" : "NOT Found \nCopy gold.wav to " + soundPath  );
        telemetry.addData("silver sound", silverFound ? "Found" : "NOT Found \nCopy silver.wav to " + soundPath );

        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X or B to play sounds.");
        telemetry.update();

        while (opModeIsActive()) {

            if (silverFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverFile);
                telemetry.addData("Playing", "Silver File");
                telemetry.update();
            }

            if (goldFound && (isB = gamepad1.b) && !WasB) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldFile);
                telemetry.addData("Playing", "Gold File");
                telemetry.update();
            }

            wasX = isX;
            WasB = isB;
        }
    }
}
