package org.firstinspires.ftc.robotcontroller.external.samples;







import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Concept: LED Stick", group = "Concept")
@Disabled
public class ConceptLEDStick extends OpMode {
    private boolean wasUp;
    private boolean wasDown;
    private int brightness = 5;
    private final static double END_GAME_TIME = 120 - 30;

    private SparkFunLEDStick ledStick;

    @Override
    public void init() {
        ledStick = hardwareMap.get(SparkFunLEDStick.class, "back_leds");
        ledStick.setBrightness(brightness);
        ledStick.setColor(Color.GREEN);
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        telemetry.addLine("Hold the A button to turn blue");
        telemetry.addLine("Hold the B button to turn red");
        telemetry.addLine("Hold the left bumper to turn off");
        telemetry.addLine("Use DPAD Up/Down to change brightness");
        
        if (getRuntime() > END_GAME_TIME) {
            int[] ledColors = {Color.RED, Color.YELLOW, Color.RED, Color.YELLOW, Color.RED,
                    Color.YELLOW, Color.RED, Color.YELLOW, Color.RED, Color.YELLOW};
            ledStick.setColors(ledColors);
        } else if (gamepad1.a) {
            ledStick.setColor(Color.BLUE);
        } else if (gamepad1.b) {
            ledStick.setColor(Color.RED);
        } else if (gamepad1.left_bumper) {
            ledStick.turnAllOff();
        } else {
            ledStick.setColor(Color.GREEN);
        }

        int newBrightness = brightness;
        if (gamepad1.dpad_up && !wasUp) {
            newBrightness = brightness + 1;
        } else if (gamepad1.dpad_down && !wasDown) {
            newBrightness = brightness - 1;
        }
        if (newBrightness != brightness) {
            brightness = Range.clip(newBrightness, 0, 31);
            ledStick.setBrightness(brightness);
        }
        telemetry.addData("Brightness", brightness);

        wasDown = gamepad1.dpad_down;
        wasUp = gamepad1.dpad_up;
    }
}
