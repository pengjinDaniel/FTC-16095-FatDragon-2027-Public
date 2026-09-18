package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@TeleOp(name = "BIOBUZZ TeleOp Blue", group = "TeleOp")
public class Blue extends TeleOpBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
