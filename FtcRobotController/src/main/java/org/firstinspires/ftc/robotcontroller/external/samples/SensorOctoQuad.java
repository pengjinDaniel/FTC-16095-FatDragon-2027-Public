
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name = "OctoQuad Basic", group="OctoQuad")
public class SensorOctoQuad extends LinearOpMode {

    private final int ODO_LEFT  = 0;
    private final int ODO_RIGHT = 1;
    private final int ODO_PERP  = 2;

    private OctoQuad    octoquad;

    private int         posLeft;
    private int         posRight;
    private int         posPerp;
    private int         velLeft;
    private int         velRight;
    private int         velPerp;

    @Override
    public void runOpMode() {

        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        telemetry.addData("OctoQuad Firmware Version ", octoquad.getFirmwareVersion());

        octoquad.setSingleEncoderDirection(ODO_LEFT,  OctoQuad.EncoderDirection.REVERSE);
        octoquad.setSingleEncoderDirection(ODO_RIGHT, OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(ODO_PERP,  OctoQuad.EncoderDirection.FORWARD);

        octoquad.setAllVelocitySampleIntervals(50);

        octoquad.saveParametersToFlash();

        telemetry.addLine("\nPress START to read encoder values");
        telemetry.update();

        waitForStart();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(100);

        octoquad.resetAllPositions();

        while (opModeIsActive()) {
            telemetry.addData(">", "Press X to Reset Encoders\n");

            if (gamepad1.x) {
                octoquad.resetAllPositions();
            }

            readOdometryPods();

            telemetry.addData("Left  P", "%7d   V :%6d CPS ", posLeft, velLeft);
            telemetry.addData("Right P", "%7d   V :%6d CPS ", posRight, velRight);
            telemetry.addData("Perp  P", "%7d   V :%6d CPS ", posPerp, velPerp);
            telemetry.update();
        }
    }

    private void readOdometryPods() {
        posLeft  = octoquad.readSinglePosition_Caching(ODO_LEFT);
        posRight = octoquad.readSinglePosition_Caching(ODO_RIGHT);
        posPerp  = octoquad.readSinglePosition_Caching(ODO_PERP);
        velLeft  = octoquad.readSingleVelocity_Caching(ODO_LEFT)  * 20;
        velRight = octoquad.readSingleVelocity_Caching(ODO_RIGHT) * 20;
        velPerp  = octoquad.readSingleVelocity_Caching(ODO_PERP)  * 20;
    }
}
