package org.firstinspires.ftc.robotcontroller.external.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Gamepad;

@Utility(name = "Test Gamepad", description = "Test gamepads on your driver station")
@SuppressWarnings("unused")
public class UtilityTestGamepad extends OpMode {

    public static final String SEPARATOR = "  •  ";

    @Override
    public void init(){
        telemetry.addData("Instructions", "Setup your gamepads and then press the play button");
    }

    @Override
    public void loop() {
        display_gamepad(gamepad1, "Gamepad 1");
        display_gamepad(gamepad2, "Gamepad 2");
    }

    void display_gamepad(Gamepad gamepad, String name) {
        telemetry.addLine("---" + name + "---");
        telemetry.addData("Left Joystick",  "(% 1.2f, %1.2f)", gamepad.left_stick_x, gamepad.left_stick_y);
        telemetry.addData("Left Trigger", "% 1.2f", gamepad.left_trigger);
        telemetry.addData("Right Joystick",  "(% 1.2f, %1.2f)", gamepad.right_stick_x, gamepad.right_stick_y);
        telemetry.addData("Right Trigger", "% 1.2f", gamepad.right_trigger);

        switch(gamepad.type()){
            case SONY_PS4:
            case SONY_PS4_SUPPORTED_BY_KERNEL:

                if (gamepad.touchpad_finger_1) {
                    telemetry.addData("Touchpad Finger1", "(% 1.2f, %1.2f)", gamepad.touchpad_finger_1_x, gamepad.touchpad_finger_1_y);
                } else {
                    telemetry.addData("Touchpad Finger1", false);
                }

                if (gamepad.touchpad_finger_2) {
                    telemetry.addData("Touchpad Finger2", "(% 1.2f, %1.2f)", gamepad.touchpad_finger_2_x, gamepad.touchpad_finger_2_y);
                } else {
                    telemetry.addData("Touchpad Finger2", false);
                }

                telemetry.addData("Buttons", ps_buttons_to_string(gamepad));
                break;
            case LOGITECH_F310:
            case XBOX_360:
            case UNKNOWN:
            default:
                telemetry.addData("Buttons", xbox_buttons_to_string(gamepad));
                break;
        }
    }

    String xbox_buttons_to_string(Gamepad gamepad){
        String buttons = generic_buttons_to_string(gamepad);
        if (gamepad.back) buttons += "back" + SEPARATOR;
        if (gamepad.start) buttons += "start" + SEPARATOR;
        if (gamepad.guide) buttons += "guide" + SEPARATOR;
        if (gamepad.a) buttons += "a" + SEPARATOR;
        if (gamepad.b) buttons += "b" + SEPARATOR;
        if (gamepad.x) buttons += "x" + SEPARATOR;
        if (gamepad.y) buttons += "y" + SEPARATOR;

        return buttons;
    }

    String ps_buttons_to_string(Gamepad gamepad){
        String buttons = generic_buttons_to_string(gamepad);
        if (gamepad.cross) buttons += "cross" + SEPARATOR;
        if (gamepad.circle) buttons += "circle" + SEPARATOR;
        if (gamepad.square) buttons += "square" + SEPARATOR;
        if (gamepad.triangle) buttons += "triangle" + SEPARATOR;
        if (gamepad.ps) buttons += "ps" + SEPARATOR;
        if (gamepad.share) buttons += "share" + SEPARATOR;
        if (gamepad.options) buttons += "options" + SEPARATOR;
        if (gamepad.touchpad) buttons += "touchpad" + SEPARATOR;

        return buttons;
    }

    String generic_buttons_to_string(Gamepad gamepad){
        String buttons = "";
        if (gamepad.dpad_up) buttons += "dpad_up" + SEPARATOR;
        if (gamepad.dpad_down) buttons += "dpad_down" + SEPARATOR;
        if (gamepad.dpad_left) buttons += "dpad_left" + SEPARATOR;
        if (gamepad.dpad_right) buttons += "dpad_right" + SEPARATOR;
        if (gamepad.left_bumper) buttons += "left_bumper" + SEPARATOR;
        if (gamepad.right_bumper) buttons += "right_bumper" + SEPARATOR;
        if (gamepad.left_stick_button) buttons += "left stick button" + SEPARATOR;
        if (gamepad.right_stick_button) buttons += "right stick button" + SEPARATOR;
        return buttons;
    }

}
