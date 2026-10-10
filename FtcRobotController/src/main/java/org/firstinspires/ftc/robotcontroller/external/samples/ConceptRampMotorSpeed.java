
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: Ramp Motor Speed", group = "Concept")
@Disabled
public class ConceptRampMotorSpeed extends LinearOpMode {

    static final double INCREMENT   = 0.01;
    static final int    CYCLE_MS    =   50;
    static final double MAX_FWD     =  1.0;
    static final double MAX_REV     = -1.0;

    DcMotor motor;
    double  power   = 0;
    boolean rampUp  = true;


    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "left_drive");

        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            if (rampUp) {
                power += INCREMENT ;
                if (power >= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = !rampUp;
                }
            }
            else {
                power -= INCREMENT ;
                if (power <= MAX_REV ) {
                    power = MAX_REV;
                    rampUp = !rampUp;
                }
            }

            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            motor.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }

        motor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
