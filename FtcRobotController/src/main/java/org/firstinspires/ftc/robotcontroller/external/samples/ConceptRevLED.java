package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp(name = "Concept: RevLED", group = "Concept")
@Disabled
public class ConceptRevLED extends OpMode {
    LED frontLED_red;
    LED frontLED_green;

    @Override
    public void init() {
        frontLED_green = hardwareMap.get(LED.class, "front_led_green");
        frontLED_red = hardwareMap.get(LED.class, "front_led_red");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLED_red.on();
        } else {
            frontLED_red.off();
        }
        if (gamepad1.b) {
            frontLED_green.on();
        } else {
            frontLED_green.off();
        }
    }
}
