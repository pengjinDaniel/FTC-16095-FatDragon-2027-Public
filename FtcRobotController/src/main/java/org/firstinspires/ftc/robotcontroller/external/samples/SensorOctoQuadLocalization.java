
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Disabled
@TeleOp(name="OctoQuad Localizer", group="OctoQuad")
public class SensorOctoQuadLocalization extends LinearOpMode
{
    static final int DEADWHEEL_PORT_X = 0;
    static final int DEADWHEEL_PORT_Y = 1;
    static final OctoQuad.EncoderDirection DEADWHEEL_X_DIR = OctoQuad.EncoderDirection.FORWARD;
    static final OctoQuad.EncoderDirection DEADWHEEL_Y_DIR = OctoQuad.EncoderDirection.REVERSE;
    static final float X_TICKS_PER_MM  =  12.34f;
    static final float Y_TICKS_PER_MM  =  12.34f;
    static final float TCP_OFFSET_X_MM =  123.4f;
    static final float TCP_OFFSET_Y_MM =  123.4f;
    static final float IMU_SCALAR      =    1.0f;

    static final double RAD2DEG = 180/Math.PI;

    int badPackets = 0;
    int totalPackets = 0;
    boolean warn = false;

    OctoQuad.LocalizerDataBlock localizer = new OctoQuad.LocalizerDataBlock();

    public void runOpMode()
    {
        OctoQuad oq = hardwareMap.get(OctoQuad.class, "octoquad");

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        oq.setSingleEncoderDirection(DEADWHEEL_PORT_X, DEADWHEEL_X_DIR);
        oq.setSingleEncoderDirection(DEADWHEEL_PORT_Y, DEADWHEEL_Y_DIR);
        oq.setLocalizerPortX(DEADWHEEL_PORT_X);
        oq.setLocalizerPortY(DEADWHEEL_PORT_Y);
        oq.setLocalizerCountsPerMM_X(X_TICKS_PER_MM);
        oq.setLocalizerCountsPerMM_Y(Y_TICKS_PER_MM);
        oq.setLocalizerTcpOffsetMM_X(TCP_OFFSET_X_MM);
        oq.setLocalizerTcpOffsetMM_Y(TCP_OFFSET_Y_MM);
        oq.setLocalizerImuHeadingScalar(IMU_SCALAR);
        oq.setLocalizerVelocityIntervalMS(25);
        oq.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        oq.resetLocalizerAndCalibrateIMU();

        while (opModeInInit())
        {
            telemetry.addData("OQ Firmware Version", oq.getFirmwareVersionString());
            telemetry.addData("Localizer Status", oq.getLocalizerStatus());
            telemetry.addData("Heading Axis Detection", oq.getLocalizerHeadingAxisChoice());
            telemetry.addLine(" ");

            warnIfNotTuned();

            telemetry.addLine("Press Play to start navigating");
            telemetry.update();

            sleep(100);
        }

        while (!isStopRequested() && (oq.getLocalizerStatus() != OctoQuad.LocalizerStatus.RUNNING))
        {
            telemetry.addLine("Waiting for IMU Calibration to complete\n");
            telemetry.addData("Localizer Status", oq.getLocalizerStatus());
            telemetry.update();
            sleep(100);
        }

        oq.setLocalizerPose(0, 0, 0);

        oq.resetAllPositions();

        while (opModeIsActive())
        {
            if (gamepad1.crossWasPressed())
            {
                oq.setLocalizerPose(200, 200, (float)(Math.PI/2.0f));
            }

            oq.readLocalizerData(localizer);

            if (localizer.crcOk)
            {
                warnIfNotTuned();

                telemetry.addData("Localizer Status", localizer.localizerStatus);
                telemetry.addData("Heading deg", "%.2f", localizer.heading_rad * RAD2DEG);
                telemetry.addData("Rotation dps", "%.2f", localizer.velHeading_radS * RAD2DEG);
                telemetry.addData("X Pos mm", localizer.posX_mm);
                telemetry.addData("Y Pos mm", localizer.posY_mm);
                telemetry.addData("X Vel mm/s", localizer.velX_mmS);
                telemetry.addData("Y Vel mm/s", localizer.velY_mmS);
                telemetry.addLine("\nPress Gamepad A (Cross) to teleport");

                telemetry.addData("\nRaw X Pod Counts", oq.readSinglePosition_Caching(DEADWHEEL_PORT_X));
                telemetry.addData("Raw Y Pod Counts", oq.readSinglePosition_Caching(DEADWHEEL_PORT_Y));
            }
            else
            {
                badPackets++;
                telemetry.addLine("Data CRC not valid");
            }
            totalPackets++;

            telemetry.addLine(String.format("%d CRC error(s) in %d packets", badPackets, totalPackets));

            telemetry.update();
        }
    }

    long lastWarnFlashTimeMs = 0;
    boolean warnFlash = false;

    void warnIfNotTuned()
    {
        String warnString = "";
        if (IMU_SCALAR == 1.0f)
        {
            warnString += "WARNING: IMU_SCALAR not tuned.<br>";
            warn = true;
        }
        if (X_TICKS_PER_MM == 12.34f  ||  Y_TICKS_PER_MM == 12.34f)
        {
            warnString += "WARNING: TICKS_PER_MM not tuned.<br>";
            warn = true;
        }
        if (TCP_OFFSET_X_MM == 123.4f || TCP_OFFSET_Y_MM == 123.4f)
        {
            warnString += "WARNING: TCP_OFFSET not tuned.<br>";
            warn = true;
        }
        if (warn)
        {
            warnString +="<BR>&nbsp;-&nbsp;Read the code COMMENTS for tuning help.<BR>";

            if (System.currentTimeMillis() - lastWarnFlashTimeMs > 1000)
            {
                lastWarnFlashTimeMs = System.currentTimeMillis();
                warnFlash = !warnFlash;
            }

            telemetry.addLine(String.format("<b><font color='%s' >%s</font></b>",
                                            warnFlash ? "red" : "white", warnString));
        }
    }
}
