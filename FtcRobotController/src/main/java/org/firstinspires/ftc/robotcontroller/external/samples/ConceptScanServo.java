
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Scan Servo", group = "Concept")
@Disabled
public class ConceptScanServo extends LinearOpMode {

    static final double INCREMENT   = 0.01;
    static final int    CYCLE_MS    =   50;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    Servo   servo;
    double  position = (MAX_POS - MIN_POS) / 2;
    boolean rampUp = true;


    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "left_hand");

        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){

            if (rampUp) {
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;
                }
            }
            else {
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;
                }
            }

            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
