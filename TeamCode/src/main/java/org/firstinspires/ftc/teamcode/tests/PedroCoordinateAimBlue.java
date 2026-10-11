package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
@TeleOp(name="Pedro Aim Blue",group="Test")
public final class PedroCoordinateAimBlue extends PedroCoordinateAimBase {
    @Override protected Alliance alliance(){return Alliance.BLUE;}
}
