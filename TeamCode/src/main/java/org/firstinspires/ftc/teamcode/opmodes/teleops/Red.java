package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@TeleOp(name = "Red", group = "TeleOp")
public class Red extends TeleOpBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
