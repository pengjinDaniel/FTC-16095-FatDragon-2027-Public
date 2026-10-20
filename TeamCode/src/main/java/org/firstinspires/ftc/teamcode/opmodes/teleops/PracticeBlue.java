package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Practice Blue", group = "TeleOp")
public class PracticeBlue extends Blue {
    @Override
    protected boolean isAutoAimEnabled() {
        return false;
    }
}
