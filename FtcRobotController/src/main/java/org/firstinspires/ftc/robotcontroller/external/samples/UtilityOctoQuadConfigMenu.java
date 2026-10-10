
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Stack;

@TeleOp(name = "OctoQuad Configuration Tool", group="OctoQuad")
@Disabled
public class UtilityOctoQuadConfigMenu extends LinearOpMode
{
    TelemetryMenu.MenuElement rootMenu = new TelemetryMenu.MenuElement("OctoQuad Config Menu", true);
    TelemetryMenu.MenuElement menuHwInfo = new TelemetryMenu.MenuElement("Hardware Information", false);
    TelemetryMenu.EnumOption optionI2cResetMode;
    TelemetryMenu.EnumOption optionChannelBankConfig;

    TelemetryMenu.MenuElement menuEncoderDirections = new TelemetryMenu.MenuElement("Set Encoder Directions", false);
    TelemetryMenu.BooleanOption[] optionsEncoderDirections = new TelemetryMenu.BooleanOption[OctoQuad.NUM_ENCODERS];

    TelemetryMenu.MenuElement menuVelocityIntervals = new TelemetryMenu.MenuElement("Velocity Measurement Intervals", false);
    TelemetryMenu.IntegerOption[] optionsVelocityIntervals = new TelemetryMenu.IntegerOption[OctoQuad.NUM_ENCODERS];

    TelemetryMenu.MenuElement menuAbsParams = new TelemetryMenu.MenuElement("Abs. Encoder Pulse Width Params", false);
    TelemetryMenu.IntegerOption[] optionsAbsParamsMax = new TelemetryMenu.IntegerOption[OctoQuad.NUM_ENCODERS];
    TelemetryMenu.IntegerOption[] optionsAbsParamsMin = new TelemetryMenu.IntegerOption[OctoQuad.NUM_ENCODERS];
    TelemetryMenu.BooleanOption[] optionsAbsParamsWrapTracking = new TelemetryMenu.BooleanOption[OctoQuad.NUM_ENCODERS];

    TelemetryMenu.OptionElement optionProgramToFlash;
    TelemetryMenu.OptionElement optionSendToRAM;

    TelemetryMenu.StaticClickableOption optionExit;

    OctoQuad octoquad;

    boolean error = false;

    @Override
    public void runOpMode()
    {
        octoquad = hardwareMap.getAll(OctoQuad.class).get(0);

        if(octoquad.getChipId() != OctoQuad.OCTOQUAD_CHIP_ID)
        {
            telemetry.addLine("Error: cannot communicate with OctoQuad. Check your wiring and configuration and try again");
            telemetry.update();

            error = true;
        }
        else
        {
            if(octoquad.getFirmwareVersion().maj != OctoQuad.SUPPORTED_FW_VERSION_MAJ)
            {
                telemetry.addLine("Error: The OctoQuad is running a different major firmware version than this driver was built for. Cannot run configuration tool");
                telemetry.update();

                error = true;
            }
        }

        if(error)
        {
            waitForStart();
            return;
        }

        telemetry.addLine("Retrieving current configuration from OctoQuad");
        telemetry.update();

        optionExit = new TelemetryMenu.StaticClickableOption("Exit configuration menu")
        {
            @Override
            void onClick()
            {
                requestOpModeStop();
            }
        };

        optionI2cResetMode = new TelemetryMenu.EnumOption("I2C Reset Mode", OctoQuad.I2cRecoveryMode.values(), octoquad.getI2cRecoveryMode());
        optionChannelBankConfig = new TelemetryMenu.EnumOption("Channel Bank Modes", OctoQuad.ChannelBankConfig.values(), octoquad.getChannelBankConfig());

        menuHwInfo.addChild(new TelemetryMenu.StaticItem("Board Firmware: v" + octoquad.getFirmwareVersion()));

        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            optionsEncoderDirections[i] = new TelemetryMenu.BooleanOption(
                    String.format("Encoder %d direction", i),
                    octoquad.getSingleEncoderDirection(i) == OctoQuad.EncoderDirection.REVERSE,
                    "-",
                    "+");
        }
        menuEncoderDirections.addChildren(optionsEncoderDirections);

        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            optionsVelocityIntervals[i] = new TelemetryMenu.IntegerOption(
                    String.format("Chan %d velocity intvl", i),
                    OctoQuad.MIN_VELOCITY_MEASUREMENT_INTERVAL_MS,
                    OctoQuad.MAX_VELOCITY_MEASUREMENT_INTERVAL_MS,
                    octoquad.getSingleVelocitySampleInterval(i));
        }
        menuVelocityIntervals.addChildren(optionsVelocityIntervals);

        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            OctoQuad.ChannelPulseWidthParams params = octoquad.getSingleChannelPulseWidthParams(i);

            optionsAbsParamsMax[i] = new TelemetryMenu.IntegerOption(
                    String.format("Chan %d max pulse length", i),
                    OctoQuad.MIN_PULSE_WIDTH_US,
                    OctoQuad.MAX_PULSE_WIDTH_US,
                    params.max_length_us);

            optionsAbsParamsMin[i] = new TelemetryMenu.IntegerOption(
                    String.format("Chan %d min pulse length", i),
                    OctoQuad.MIN_PULSE_WIDTH_US,
                    OctoQuad.MAX_PULSE_WIDTH_US,
                    params.min_length_us);

            optionsAbsParamsWrapTracking[i] = new TelemetryMenu.BooleanOption(
                    String.format("Chan %d wrap tracking enabled", i),
                    octoquad.getSingleChannelPulseWidthTracksWrap(i),
                    "yes",
                    "no");
        }
        menuAbsParams.addChildren(optionsAbsParamsMin);
        menuAbsParams.addChildren(optionsAbsParamsMax);
        menuAbsParams.addChildren(optionsAbsParamsWrapTracking);

        optionProgramToFlash = new TelemetryMenu.OptionElement()
        {
            String name = "Program Settings to FLASH";
            long lastClickTime = 0;

            @Override
            protected String getDisplayText()
            {
                if(lastClickTime == 0)
                {
                    return name;
                }
                else
                {
                    if(System.currentTimeMillis() - lastClickTime < 1000)
                    {
                        return name + " **OK**";
                    }
                    else
                    {
                        lastClickTime = 0;
                        return name;
                    }
                }
            }

            @Override
            void onClick()
            {
                sendSettingsToRam();
                octoquad.saveParametersToFlash();
                lastClickTime = System.currentTimeMillis();
            }
        };

        optionSendToRAM = new TelemetryMenu.OptionElement()
        {
            String name = "Send Settings to RAM";
            long lastClickTime = 0;

            @Override
            protected String getDisplayText()
            {
                if(lastClickTime == 0)
                {
                    return name;
                }
                else
                {
                    if(System.currentTimeMillis() - lastClickTime < 1000)
                    {
                        return name + " **OK**";
                    }
                    else
                    {
                        lastClickTime = 0;
                        return name;
                    }
                }
            }

            @Override
            void onClick()
            {
                sendSettingsToRam();
                lastClickTime = System.currentTimeMillis();
            }
        };

        rootMenu.addChild(menuHwInfo);
        rootMenu.addChild(optionI2cResetMode);
        rootMenu.addChild(optionChannelBankConfig);
        rootMenu.addChild(menuEncoderDirections);
        rootMenu.addChild(menuVelocityIntervals);
        rootMenu.addChild(menuAbsParams);
        rootMenu.addChild(optionProgramToFlash);
        rootMenu.addChild(optionSendToRAM);
        rootMenu.addChild(optionExit);

        TelemetryMenu menu = new TelemetryMenu(telemetry, rootMenu);

        while (!isStopRequested())
        {
            menu.loop(gamepad1);
            telemetry.update();
            sleep(20);
        }
    }

    void sendSettingsToRam()
    {
        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            octoquad.setSingleEncoderDirection(i, optionsEncoderDirections[i].getValue() ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            octoquad.setSingleVelocitySampleInterval(i, optionsVelocityIntervals[i].getValue());

            OctoQuad.ChannelPulseWidthParams params = new OctoQuad.ChannelPulseWidthParams();
            params.max_length_us = optionsAbsParamsMax[i].getValue();
            params.min_length_us = optionsAbsParamsMin[i].getValue();

            octoquad.setSingleChannelPulseWidthParams(i, params);
            octoquad.setSingleChannelPulseWidthTracksWrap(i, optionsAbsParamsWrapTracking[i].val);
        }

        octoquad.setI2cRecoveryMode((OctoQuad.I2cRecoveryMode) optionI2cResetMode.getValue());
        octoquad.setChannelBankConfig((OctoQuad.ChannelBankConfig) optionChannelBankConfig.getValue());
    }


    public static class TelemetryMenu
    {
        private final MenuElement root;
        private MenuElement currentLevel;

        private boolean dpadUpPrev;
        private boolean dpadDnPrev;
        private boolean dpadRightPrev;
        private boolean dpadLeftPrev;
        private boolean xPrev;
        private boolean lbPrev;

        private int selectedIdx = 0;
        private Stack<Integer> selectedIdxStack = new Stack<>();

        private final Telemetry telemetry;

        public TelemetryMenu(Telemetry telemetry, MenuElement root)
        {
            this.root = root;
            this.currentLevel = root;
            this.telemetry = telemetry;

            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.setMsTransmissionInterval(50);
        }

        public void loop(Gamepad gamepad)
        {
            boolean dpadUp = gamepad.dpad_up;
            boolean dpadDn = gamepad.dpad_down;
            boolean dpadRight = gamepad.dpad_right;
            boolean dpadLeft = gamepad.dpad_left;
            boolean x = gamepad.x;
            boolean lb = gamepad.left_bumper;

            ArrayList<Element> children = currentLevel.children();
            Element currentSelection = children.get(selectedIdx);

            if (currentSelection instanceof OptionElement)
            {
                if (dpadRight && !dpadRightPrev)
                {
                    ((OptionElement) currentSelection).onRightInput();
                }
                else if (dpadLeft && !dpadLeftPrev)
                {
                    ((OptionElement) currentSelection).onLeftInput();
                }
            }

            if (dpadUp && !dpadUpPrev)
            {
                selectedIdx--;
            }
            else if (dpadDn && !dpadDnPrev)
            {
                selectedIdx++;
            }

            if (selectedIdx >= children.size())
            {
                selectedIdx = children.size()-1;
            }
            else if (selectedIdx < 0)
            {
                selectedIdx = 0;
            }

            else if (x && !xPrev)
            {
                if (currentSelection instanceof SpecialUpElement)
                {
                    if (currentLevel != root)
                    {
                        selectedIdx = selectedIdxStack.pop();

                        currentLevel = currentLevel.parent();
                    }
                }
                else if (currentSelection instanceof OptionElement)
                {
                    ((OptionElement) currentSelection).onClick();
                }
                else if (currentSelection instanceof MenuElement)
                {
                    selectedIdxStack.push(selectedIdx);

                    selectedIdx = 0;

                    currentLevel = (MenuElement) currentSelection;
                }
            }

            else if (lb && !lbPrev)
            {
                if (currentLevel != root)
                {
                    selectedIdx = selectedIdxStack.pop();

                    currentLevel = currentLevel.parent();
                }
            }

            dpadUpPrev = dpadUp;
            dpadDnPrev = dpadDn;
            dpadRightPrev = dpadRight;
            dpadLeftPrev = dpadLeft;
            xPrev = x;
            lbPrev = lb;

            StringBuilder builder = new StringBuilder();
            builder.append("<font color='#119af5' face=monospace>");
            builder.append("Navigate items.....dpad up/down\n")
                    .append("Select.............X or Square\n")
                    .append("Edit option........dpad left/right\n")
                    .append("Up one level.......left bumper\n");
            builder.append("</font>");
            builder.append("\n");

            builder.append("<font face=monospace>");
            builder.append("Current Menu: ").append(currentLevel.name).append("\n");

            for (int i = 0; i < children.size(); i++)
            {
                if (selectedIdx == i)
                {
                    builder.append("[<font color=green face=monospace>•</font>] ");
                }
                else
                {
                    builder.append("[ ] ");
                }

                Element e = children.get(i);

                if (e instanceof MenuElement)
                {
                    builder.append("> ");
                }

                builder.append(e.getDisplayText());

                builder.append("\n");
            }

            builder.append("</font>");

            String menu = builder.toString();

            telemetry.addLine(menu);
        }

        public static class MenuElement extends Element
        {
            private String name;
            private ArrayList<Element> children = new ArrayList<>();

            public MenuElement(String name, boolean isRoot)
            {
                this.name = name;

                if (!isRoot)
                {
                    children.add(new SpecialUpElement());
                }
            }

            public void addChild(Element child)
            {
                child.setParent(this);
                children.add(child);
            }

            public void addChildren(Element[] children)
            {
                for (Element e : children)
                {
                    e.setParent(this);
                    this.children.add(e);
                }
            }

            @Override
            protected String getDisplayText()
            {
                return name;
            }

            private ArrayList<Element> children()
            {
                return children;
            }
        }

        public static abstract class OptionElement extends Element
        {
            void onClick() {}

            protected void onLeftInput() {}

            protected void onRightInput() {}
        }

        public static class EnumOption extends OptionElement
        {
            protected int idx = 0;
            protected Enum[] e;
            protected String name;

            public EnumOption(String name, Enum[] e)
            {
                this.e = e;
                this.name = name;
            }

            public EnumOption(String name, Enum[] e, Enum def)
            {
                this(name, e);
                idx = def.ordinal();
            }

            @Override
            public void onLeftInput()
            {
                idx++;

                if(idx > e.length-1)
                {
                    idx = 0;
                }
            }

            @Override
            public void onRightInput()
            {
                idx--;

                if(idx < 0)
                {
                    idx = e.length-1;
                }
            }

            @Override
            public void onClick()
            {
            }

            @Override
            protected String getDisplayText()
            {
                return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, e[idx].name());
            }

            public Enum getValue()
            {
                return e[idx];
            }
        }

        public static class IntegerOption extends OptionElement
        {
            protected int i;
            protected int min;
            protected int max;
            protected String name;

            public IntegerOption(String name, int min, int max, int def)
            {
                this.name = name;
                this.min = min;
                this.max = max;
                this.i = def;
            }

            @Override
            public void onLeftInput()
            {
                i--;

                if(i < min)
                {
                    i = max;
                }
            }

            @Override
            public void onRightInput()
            {
                i++;

                if(i > max)
                {
                    i = min;
                }
            }

            @Override
            public void onClick()
            {
            }

            @Override
            protected String getDisplayText()
            {
                return String.format("%s: <font color='#e37c07' face=monospace>%d</font>", name, i);
            }

            public int getValue()
            {
                return i;
            }
        }

        static class BooleanOption extends OptionElement
        {
            private String name;
            private boolean val = true;

            private String customTrue;
            private String customFalse;

            BooleanOption(String name, boolean def)
            {
                this.name = name;
                this.val = def;
            }

            BooleanOption(String name, boolean def, String customTrue, String customFalse)
            {
                this(name, def);
                this.customTrue = customTrue;
                this.customFalse = customFalse;
            }

            @Override
            public void onLeftInput()
            {
                val = !val;
            }

            @Override
            public void onRightInput()
            {
                val = !val;
            }

            @Override
            public void onClick()
            {
                val = !val;
            }

            @Override
            protected String getDisplayText()
            {
                String valStr;

                if(customTrue != null && customFalse != null)
                {
                    valStr = val ? customTrue : customFalse;
                }
                else
                {
                    valStr = val ? "true" : "false";
                }

                return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, valStr);
            }

            public boolean getValue()
            {
                return val;
            }
        }

        public static class StaticItem extends OptionElement
        {
            private String name;

            public StaticItem(String name)
            {
                this.name = name;
            }

            @Override
            protected String getDisplayText()
            {
                return name;
            }
        }

        public static abstract class StaticClickableOption extends OptionElement
        {
            private String name;

            public StaticClickableOption(String name)
            {
                this.name = name;
            }

            abstract void onClick();

            @Override
            protected String getDisplayText()
            {
                return name;
            }
        }

        private static abstract class Element
        {
            private MenuElement parent;

            protected void setParent(MenuElement parent)
            {
                this.parent = parent;
            }

            protected MenuElement parent()
            {
                return parent;
            }

            protected abstract String getDisplayText();
        }

        private static class SpecialUpElement extends Element
        {
            @Override
            protected String getDisplayText()
            {
                return "<font color='#119af5' face=monospace>.. ↰ Up One Level</font>";
            }
        }
    }
}
