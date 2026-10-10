package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@TeleOp(name = "Limelight Aim Test Blue", group = "Test")
public class CoordinateAimBlueTest extends CoordinateAimTestBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
