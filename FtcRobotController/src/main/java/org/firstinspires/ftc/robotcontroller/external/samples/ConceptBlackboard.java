
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Concept: Blackboard", group = "Concept")
@Disabled
public class ConceptBlackboard extends OpMode {
    public static final String TIMES_STARTED_KEY = "Times started";
    public static final String ALLIANCE_KEY = "Alliance";

    @Override
    public void init() {
        Object timesStarted = blackboard.getOrDefault(TIMES_STARTED_KEY, 0);
        blackboard.put(TIMES_STARTED_KEY, (int) timesStarted + 1);

        telemetry.addData("OpMode started times", blackboard.get(TIMES_STARTED_KEY));
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            blackboard.put(ALLIANCE_KEY, "RED");
        } else if (gamepad1.right_bumper) {
            blackboard.put(ALLIANCE_KEY, "BLUE");
        }
        telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
    }
}
