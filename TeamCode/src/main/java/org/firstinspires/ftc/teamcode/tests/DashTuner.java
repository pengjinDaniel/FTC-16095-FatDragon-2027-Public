package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DashboardTuningCommand;
import org.firstinspires.ftc.teamcode.subsystems.tuning.DashboardTuner;

/** Select this OpMode; edit DashboardTuner fields live in FTC Dashboard. */
public class DashTuner extends CommandOpMode {
    private DashboardTuner tuner;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        tuner = new DashboardTuner(hardwareMap);
        tuner.setDefaultCommand(new DashboardTuningCommand(tuner));
        telemetry.addLine("START, then edit Dashboard > Config > DashboardTuner.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (tuner != null) tuner.stop();
        }
    }
}
