package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="BlinkinExample")
@Disabled
public class SampleRevBlinkinLedDriver extends OpMode {

    private final static int LED_PERIOD = 10;

    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    @Override
    public void init()
    {
        displayKind = DisplayKind.AUTO;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop()
    {
        handleGamepad();

        if (displayKind == DisplayKind.AUTO) {
            doAutoDisplay();
        } else {
        }
    }

    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a) {
            setDisplayKind(DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            setDisplayKind(DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

    protected void setDisplayKind(DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
}
