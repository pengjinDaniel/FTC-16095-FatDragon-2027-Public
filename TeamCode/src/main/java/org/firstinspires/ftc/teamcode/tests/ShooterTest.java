package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Config
@TeleOp(name = "Shooter Test", group = "TeleOp")
public class ShooterTest extends OpMode {
    public static Alliance alliance = Alliance.BLUE;

    private Shooter shooter;
    private Drive drive;
    private Vision vision;
    private FtcDashboard dashboard;
    private boolean stopLatched;
    private boolean started;
    private double distanceCm = Double.NaN;
    private boolean distanceTargetAccepted;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        shooter = new Shooter(hardwareMap);
        shooter.stop();
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, alliance);
        stopLatched = false;
        started = false;
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {
        updateSensors(gamepad1.right_trigger > TeleOpConstants.triggerThreshold);
        showTelemetry();
    }

    @Override
    public void start() {
        started = true;
    }

    @Override
    public void loop() {
        boolean requested = gamepad1.right_trigger > TeleOpConstants.triggerThreshold;
        if (gamepad1.b) {
            stopLatched = true;
        } else if (!requested) {
            stopLatched = false;
        }
        updateSensors(requested);
        distanceTargetAccepted = false;
        if (stopLatched) {
            shooter.stop();
        } else if (requested) {
            distanceTargetAccepted = shooter.setDistanceTarget(distanceCm);
        } else {
            shooter.idle();
        }
        shooter.periodic();
        showTelemetry();
    }

    private void updateSensors(boolean requested) {
        drive.stop();
        drive.periodic();
        vision.update();
        vision.updateAimTarget(drive, started, stopLatched || gamepad1.b, gamepad1.a || requested);
        distanceCm = vision.hasValidAimingTarget()
                ? DistanceUnit.INCH.toCm(vision.getTargetHorizontalDistanceInches()) : Double.NaN;
    }

    private void showTelemetry() {
        double leftVelocity = shooter.getLeftVelocity();
        double rightVelocity = shooter.getRightVelocity();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("alliance", vision.getAlliance().name());
        packet.put("distanceCm", distanceCm);
        packet.put("targetStatus", vision.getTargetStatus());
        packet.put("calibrationValid", Shooter.areDistanceShotValuesValid());
        packet.put("distanceTargetAccepted", distanceTargetAccepted);
        packet.put("targetVelocityTicksPerSecond", shooter.getTargetVelocity());
        packet.put("controlMode", shooter.getControlMode());
        packet.put("power", shooter.getLeftPower());
        packet.put("rightPower", shooter.getRightPower());
        packet.put("powerLimit", ShooterConstants.maxPower);
        packet.put("pitchPosition", shooter.getPitchPosition());
        packet.put("shooterReady", shooter.isReady());
        packet.put("pitchReady", shooter.isPitchReady());
        packet.put("leftVelocity", leftVelocity);
        packet.put("rightVelocity", rightVelocity);
        packet.put("velocityDifference", Math.abs(leftVelocity - rightVelocity));
        packet.put("stopLatched", stopLatched);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("ShooterConstants: center/wall distance, velocity (ticks/s) and pitch. Same PID control as Blue/Red.");
        telemetry.addLine("Set ShooterTest.alliance before INIT. Drive stays stopped; aim manually.");
        telemetry.addLine("RT: distance-based shot. Release RT: shared standby power. B stops until RT is released.");
        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Target", vision.getSelectedCluster().map(Enum::name).orElse("NONE"));
        telemetry.addData("Distance to CELL", "%.1f cm", distanceCm);
        telemetry.addData("Target status", vision.getTargetStatus());
        telemetry.addData("Calibration status", drive.getVisionCorrectionStatus());
        telemetry.addData("Center/wall calibration", Shooter.areDistanceShotValuesValid() ? "VALID" : "INCOMPLETE OR INVALID");
        telemetry.addData("Distance target accepted", distanceTargetAccepted);
        telemetry.addData("Control mode / target velocity", "%s / %.1f ticks/s",
                shooter.getControlMode(), shooter.getTargetVelocity());
        telemetry.addData("Power / limit", "%.3f / %.3f", shooter.getLeftPower(), ShooterConstants.maxPower);
        telemetry.addData("Pitch position", "%.3f", shooter.getPitchPosition());
        telemetry.addData("Shooter / pitch ready", "%b / %b", shooter.isReady(), shooter.isPitchReady());
        telemetry.addData("Left / right velocity", "%.1f / %.1f ticks/s", leftVelocity, rightVelocity);
        telemetry.addData("Stop latched", stopLatched);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stop();
        }
        if (drive != null) {
            drive.stop();
        }
        if (vision != null) {
            vision.stop();
        }
        CommandScheduler.getInstance().reset();
    }
}
