package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/** Same Blue controls and PID; only the vision calibration source changes. */
@TeleOp(name = "Blue Single Tag", group = "TeleOp")
public class BlueSingleTag extends Blue {
    @Override
    protected Vision createVision() {
        return new Vision(hardwareMap, getAlliance(), Vision.LocalizationMode.SINGLE_TAG);
    }
}
