
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp (name = "Motor Bulk Reads", group = "Tests")
@Disabled
public class ConceptMotorBulkRead extends LinearOpMode {

    final int       TEST_CYCLES    = 500;

    private DcMotorEx m1, m2, m3, m4;
    private long      e1, e2, e3, e4;
    private double    v1, v2, v3, v4;

    double t1 = 0;
    double t2 = 0;
    double t3 = 0;

    @Override
    public void runOpMode() {

        int cycles;

        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addData(">", "Press START to start tests");
        telemetry.addData(">", "Test results will update for each access method.");
        telemetry.update();
        waitForStart();


        displayCycleTimes("Test 1 of 3 (Wait for completion)");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            e1 = m1.getCurrentPosition();
            e2 = m2.getCurrentPosition();
            e3 = m3.getCurrentPosition();
            e4 = m4.getCurrentPosition();

            v1 = m1.getVelocity();
            v2 = m2.getVelocity();
            v3 = m3.getVelocity();
            v4 = m4.getVelocity();


        }
        t1 = timer.milliseconds() / cycles;
        displayCycleTimes("Test 2 of 3 (Wait for completion)");


        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            e1 = m1.getCurrentPosition();
            e2 = m2.getCurrentPosition();
            e3 = m3.getCurrentPosition();
            e4 = m4.getCurrentPosition();

            v1 = m1.getVelocity();
            v2 = m2.getVelocity();
            v3 = m3.getVelocity();
            v4 = m4.getVelocity();


        }
        t2 = timer.milliseconds() / cycles;
        displayCycleTimes("Test 3 of 3 (Wait for completion)");


        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            e1 = m1.getCurrentPosition();
            e2 = m2.getCurrentPosition();
            e3 = m3.getCurrentPosition();
            e4 = m4.getCurrentPosition();

            v1 = m1.getVelocity();
            v2 = m2.getVelocity();
            v3 = m3.getVelocity();
            v4 = m4.getVelocity();


        }
        t3 = timer.milliseconds() / cycles;
        displayCycleTimes("Complete");

        while (opModeIsActive()) ;
    }

    void displayCycleTimes(String status) {
        telemetry.addData("Testing", status);
        telemetry.addData("Cache = OFF",    "%5.1f mS/cycle", t1);
        telemetry.addData("Cache = AUTO",   "%5.1f mS/cycle", t2);
        telemetry.addData("Cache = MANUAL", "%5.1f mS/cycle", t3);
        telemetry.update();
    }
}

