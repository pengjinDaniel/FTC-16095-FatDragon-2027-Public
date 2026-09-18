package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    private final Servo limitServo;
    private TransitState transitState = TransitState.CLOSE;

    public enum TransitState {
        CLOSE(TransitConstants.limitClosed),
        OPEN(TransitConstants.limitOpen);

        final double servoPosition;

        TransitState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public Transit(HardwareMap hardwareMap) {
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
        close();
    }

    public void close() {
        setState(TransitState.CLOSE);
    }

    public void open() {
        setState(TransitState.OPEN);
    }

    public TransitState getState() {
        return transitState;
    }

    @Override
    public void periodic() {
        limitServo.setPosition(transitState.servoPosition);
    }

    private void setState(TransitState state) {
        transitState = state;
        limitServo.setPosition(transitState.servoPosition);
    }
}
