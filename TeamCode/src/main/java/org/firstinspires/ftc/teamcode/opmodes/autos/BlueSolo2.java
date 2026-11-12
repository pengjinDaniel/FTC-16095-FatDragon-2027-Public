package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@Autonomous(name = "BlueSolo2", group = "Auto")
public class BlueSolo2 extends RedSolo2 {
    public BlueSolo2() {
        super(Alliance.BLUE);
    }
}
