package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@Autonomous(name = "Blue 2.5 Tip", group = "Auto")
public class BlueTwoPointFiveTip extends TwoPointFiveTip {
    public BlueTwoPointFiveTip() {
        super(Alliance.BLUE);
    }
}
