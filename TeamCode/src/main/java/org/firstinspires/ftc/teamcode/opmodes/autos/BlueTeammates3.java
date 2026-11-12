package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@Autonomous(name = "BlueTeammates3", group = "Auto")
public class BlueTeammates3 extends RedTeammates3 {
    public BlueTeammates3() {
        super(Alliance.BLUE);
    }
}
