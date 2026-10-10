package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
@TeleOp(name="Pedro Aim Red",group="Test")
public final class PedroCoordinateAimRed extends PedroCoordinateAimBase {
    @Override protected Alliance alliance(){return Alliance.RED;}
}
