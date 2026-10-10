
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name="OctoQuad Advanced", group="OctoQuad")
public class SensorOctoQuadAdv extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        OctoSwerveDrive octoSwerveDrive = new OctoSwerveDrive(octoquad);

        telemetry.addLine("OctoQuad Firmware v" + octoquad.getFirmwareVersion());
        telemetry.addLine("\nPress START to read encoder values");
        telemetry.update();

        waitForStart();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        MovingStatistics avgTime = new MovingStatistics(100);
        ElapsedTime elapsedTime = new ElapsedTime();

        while (opModeIsActive()) {
            telemetry.addData(">", "Press X to Reset Encoders\n");

            if(gamepad1.x) {
                octoquad.resetAllPositions();
            }

            octoSwerveDrive.updateModules();
            octoSwerveDrive.show(telemetry);

            avgTime.add(elapsedTime.nanoseconds());
            elapsedTime.reset();

            telemetry.addData("Loop time", "%.1f mS", avgTime.getMean()/1000000);
            telemetry.update();
        }
    }
}


class OctoSwerveDrive {

    private final OctoQuad octoquad;
    private final List<OctoSwerveModule> allModules = new ArrayList<>();

    public final OctoSwerveModule LeftFront;
    public final OctoSwerveModule RightFront;
    public final OctoSwerveModule LeftBack;
    public final OctoSwerveModule RightBack;

    private final OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();

    public OctoSwerveDrive(OctoQuad octoquad) {
        this.octoquad = octoquad;

        octoquad.resetEverything();

        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);



        allModules.add(LeftFront  = new OctoSwerveModule(octoquad, "LF ",0,0));
        allModules.add(RightFront = new OctoSwerveModule(octoquad, "RF ",1,0));
        allModules.add(LeftBack   = new OctoSwerveModule(octoquad, "LB ",2,0));
        allModules.add(RightBack  = new OctoSwerveModule(octoquad, "RB ",3,0));

        octoquad.saveParametersToFlash();
    }

    public void updateModules() {
        octoquad.readAllEncoderData(encoderDataBlock);
        for (OctoSwerveModule module : allModules) {
            module.updateModule(encoderDataBlock);
        }
    }

    public void show(Telemetry telemetry) {
        telemetry.addData("pos", "   Count     CPS  Degree    DPS");
        for (OctoSwerveModule module : allModules) {
            module.show(telemetry);
        }
    }
}

class OctoSwerveModule {

    public  double driveCounts;
    public  double driveCountsPerSec;
    public  double steerDegrees;
    public  double steerDegreesPerSec;

    private final String   name;
    private final int      channel;
    private final double   angleOffset;

    private static final int    VELOCITY_SAMPLE_INTERVAL_MS = 25;
    private static final double DEGREES_PER_US = (360.0 / 1024.0);
    private static final double VELOCITY_SAMPLES_PER_S = (1000.0 / VELOCITY_SAMPLE_INTERVAL_MS);

    private static final boolean INVERT_DRIVE_ENCODER = false;
    private static final boolean INVERT_STEER_ENCODER = false;

    public OctoSwerveModule (OctoQuad octoquad, String name, int quadChannel, double angleOffset) {
        this.name = name;
        this.channel = quadChannel;
        this.angleOffset = angleOffset;

        octoquad.setSingleEncoderDirection(channel,
             INVERT_DRIVE_ENCODER ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(channel + 4,
             INVERT_STEER_ENCODER ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);

        octoquad.setSingleVelocitySampleInterval(channel, VELOCITY_SAMPLE_INTERVAL_MS);
        octoquad.setSingleVelocitySampleInterval(channel + 4, VELOCITY_SAMPLE_INTERVAL_MS);

        octoquad.setSingleChannelPulseWidthParams (channel + 4,
                                                    new OctoQuad.ChannelPulseWidthParams(1,1024));
    }

    public void updateModule(OctoQuad.EncoderDataBlock encoderDataBlock) {
        driveCounts = encoderDataBlock.positions[channel];
        driveCountsPerSec = encoderDataBlock.velocities[channel] * VELOCITY_SAMPLES_PER_S;

        steerDegrees = AngleUnit.normalizeDegrees(
                        (encoderDataBlock.positions[channel+ 4] * DEGREES_PER_US) - angleOffset);
        steerDegreesPerSec = encoderDataBlock.velocities[channel + 4] *
                        DEGREES_PER_US * VELOCITY_SAMPLES_PER_S;
    }

    public void show(Telemetry telemetry) {
        telemetry.addData(name, "%8.0f %7.0f %7.0f %6.0f",
                                driveCounts, driveCountsPerSec, steerDegrees, steerDegreesPerSec);
    }
}
