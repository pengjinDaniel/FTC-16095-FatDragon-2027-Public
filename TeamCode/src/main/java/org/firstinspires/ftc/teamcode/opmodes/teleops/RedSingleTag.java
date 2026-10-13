package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/** Same Red controls and PID; only the vision calibration source changes. */
@TeleOp(name = "Red Single Tag", group = "TeleOp")
public class RedSingleTag extends Red {
    @Override
    protected Vision createVision() {
        return new Vision(hardwareMap, getAlliance(), Vision.LocalizationMode.SINGLE_TAG);
    }
}
