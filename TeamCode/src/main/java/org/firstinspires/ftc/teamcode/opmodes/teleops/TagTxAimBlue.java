package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.TagTxVision;

@TeleOp(name="Tag TX Aim Blue", group="TeleOp")
public class TagTxAimBlue extends TagTxAim {
    @Override protected TagTxVision.Alliance targetAlliance() { return TagTxVision.Alliance.BLUE; }
}
