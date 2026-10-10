
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Concept: Telemetry", group = "Concept")
@Disabled
public class ConceptTelemetry extends LinearOpMode  {
    int poemLine = 0;

    ElapsedTime poemElapsed = new ElapsedTime();

    static final String[] poem = new String[] {

        "Mary had a little lamb,",
        "His fleece was white as snow,",
        "And everywhere that Mary went,",
        "The lamb was sure to go.",
        "",
        "He followed her to school one day,",
        "Which was against the rule,",
        "It made the children laugh and play",
        "To see a lamb at school.",
        "",
        "And so the teacher turned it out,",
        "But still it lingered near,",
        "And waited patiently about,",
        "Till Mary did appear.",
        "",
        "\"Why does the lamb love Mary so?\"",
        "The eager children cry.",
        "\"Why, Mary loves the lamb, you know,\"",
        "The teacher did reply.",
        "",
        ""
    };

    @Override public void runOpMode() {

        ElapsedTime opmodeRunTime = new ElapsedTime();

        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        telemetry.log().setCapacity(6);
        double sPoemInterval = 0.6;

        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }


        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
            });

        opmodeRunTime.reset();
        int loopCount = 1;

        while (opModeIsActive()) {

            if (poemElapsed.seconds() > sPoemInterval) {
                emitPoemLine();
            }

            telemetry.addData("loop count", loopCount);
            telemetry.addData("ms/loop", "%.3f ms", opmodeRunTime.milliseconds() / loopCount);

            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);

            telemetry.update();

            loopCount++;
        }
    }

    void emitPoemLine() {
        telemetry.log().add(poem[poemLine]);
        poemLine = (poemLine+1) % poem.length;
        poemElapsed.reset();
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
