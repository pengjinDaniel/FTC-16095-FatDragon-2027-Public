package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.drive.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.TagTxVision;

/** Independent TeleOp. Current robot controls with the 12527-2 tx steering loop. */
public abstract class TagTxAim extends CommandOpMode {
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private TagTxVision vision;
    private LegacyTxAimController aim;
    private TagTxShootAssistCommand assist;
    private boolean started, stopLatched;

    protected abstract TagTxVision.Alliance targetAlliance();

    @Override public void initialize() {
        started=false; stopLatched=false;
        drive=new Drive(hardwareMap);
        shooter=new Shooter(hardwareMap);
        intake=new Intake(hardwareMap);
        transit=new Transit(hardwareMap);
        vision=new TagTxVision(hardwareMap,targetAlliance());
        aim=new LegacyTxAimController();
        GamepadEx pad=new GamepadEx(gamepad1);
        assist=new TagTxShootAssistCommand(drive,shooter,intake,transit,vision,aim,pad);
        drive.setDefaultCommand(new DriveCommand(drive,pad));
        intake.setDefaultCommand(new IntakeCommand(intake,transit,pad));
        new Trigger(() -> pad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenActive(drive::resetDriverHeading);
        new Trigger(() -> pad.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(intake::toggleFlower);
        new Trigger(() -> pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>TeleOpConstants.triggerThreshold)
                .whileActiveContinuous(assist);
    }
    @Override public void run() {
        boolean requested=gamepad1.right_trigger>TeleOpConstants.triggerThreshold;
        if (gamepad1.b) stopLatched=true;
        else if (!requested) stopLatched=false;
        if (stopLatched) {
            assist.cancel(); aim.reset(); stopMechanisms(); drive.periodic(); vision.update();
        } else super.run();
        showTelemetry();
    }
    private void showTelemetry() {
        telemetry.addLine("Tag TX Aim " + targetAlliance().label + " | RT: aim/shoot | B: stop | L3: reset forward");
        telemetry.addData("本程序阵营",targetAlliance().label);
        telemetry.addData("当前可见阵营",vision.getVisibleAlliance());
        telemetry.addData("当前瞄准阵营",vision.getTargetAlliance());
        telemetry.addData("允许 Tag（自动选最小偏角）",targetAlliance().tagIds);
        telemetry.addData("Mode", !started ? "INIT" : stopLatched ? "STOPPED" : assist.isActive() ? "TX AIM" : "MANUAL");
        telemetry.addData("Pipeline", TagTxVision.pipelineIndex);
        telemetry.addLine("Camera ID filter must allow: " + targetAlliance().tagIds);
        telemetry.addData("Target", vision.getStatus());
        telemetry.addData("Tag / raw tx", "%d / %.2f deg", vision.getTagId(),vision.getTx());
        telemetry.addData("Filtered / target tx", "%.2f / %.2f deg",aim.getFilteredTx(),aim.getTargetTx());
        telemetry.addData("Distance (in)",vision.getDistance());
        telemetry.addData("P / I / D", "%.4f / %.4f / %.4f",aim.getEffectiveKp(),LegacyTxAimController.kI_alignH,LegacyTxAimController.kD_alignH);
        telemetry.addData("Turn / aligned", "%.3f / %b",assist.getTurnPower(),aim.isLocked());
        telemetry.addData("HIVE (feed interlock)",vision.getTargetHiveState());
        telemetry.addData("Shooter ready / pitch", "%b / %b",shooter.isReady(),shooter.isPitchReady());
        telemetry.addData("Firing",assist.getFiringState());
        telemetry.addData("Pinpoint",drive.getPinpointStatus());
        telemetry.update();
        TelemetryPacket packet=new TelemetryPacket();
        packet.put("txProgramAlliance",targetAlliance().label);
        packet.put("txVisibleAlliance",vision.getVisibleAlliance());
        packet.put("txTargetAlliance",vision.getTargetAlliance());
        packet.put("txTag",vision.getTagId()); packet.put("txRaw",vision.getTx());
        packet.put("txFiltered",aim.getFilteredTx()); packet.put("txTarget",aim.getTargetTx());
        packet.put("txTurn",assist.getTurnPower()); packet.put("txAligned",aim.isLocked());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    private void stopMechanisms() {
        if (drive!=null) drive.stop();
        if (shooter!=null) shooter.stop();
        if (intake!=null) intake.stop();
        if (transit!=null) { transit.stopMotor(); transit.close(); }
    }
    @Override public void runOpMode() throws InterruptedException {
        try {
            initialize();
            while (opModeInInit() && !isStopRequested()) {
                drive.periodic(); vision.update(); showTelemetry(); idle();
            }
            if (isStopRequested()) return;
            waitForStart(); drive.periodic(); drive.resetDriverHeading(); started=true;
            while (opModeIsActive() && !isStopRequested()) run();
        } finally {
            if (assist!=null) assist.cancel();
            reset(); stopMechanisms();
            if (intake!=null) intake.raiseFlower();
            if (vision!=null) vision.stop();
        }
    }
}
