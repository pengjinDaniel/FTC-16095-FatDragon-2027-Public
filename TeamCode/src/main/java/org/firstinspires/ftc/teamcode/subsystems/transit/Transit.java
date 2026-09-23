package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    private final DcMotor transitMotor;
    private final Servo limitServo;
    private TransitState transitState = TransitState.CLOSE;
    private double requestedPower;

    public enum TransitState {
        CLOSE,
        OPEN
    }

    public Transit(HardwareMap hardwareMap) {
        transitMotor = hardwareMap.get(DcMotor.class, TransitConstants.transitMotorName);
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
        transitMotor.setDirection(TransitConstants.transitMotorDirection);
        stopMotor();
        close();
    }

    public void forward() {
        setPower(TransitConstants.forwardPower);
    }

    public void reverse() {
        setPower(TransitConstants.reversePower);
    }

    public void stopMotor() {
        setPower(0.0);
    }

    public double getRequestedPower() {
        return requestedPower;
    }

    public boolean areLimitPositionsCalibrated() {
        return TransitConstants.limitPositionsCalibrated
                && validServoPosition(TransitConstants.limitClosed)
                && validServoPosition(TransitConstants.limitOpen)
                && TransitConstants.limitClosed != TransitConstants.limitOpen;
    }

    public void close() {
        setState(TransitState.CLOSE);
    }

    public void open() {
        if (!areLimitPositionsCalibrated()) {
            close();
            return;
        }
        setState(TransitState.OPEN);
    }

    public TransitState getState() {
        return transitState;
    }

    @Override
    public void periodic() {
        transitMotor.setPower(requestedPower);
        applyLimitPosition();
    }

    private void setState(TransitState state) {
        transitState = state;
        applyLimitPosition();
    }

    private void setPower(double power) {
        requestedPower = power;
        transitMotor.setPower(requestedPower);
    }

    private void applyLimitPosition() {
        if (areLimitPositionsCalibrated()) {
            limitServo.setPosition(transitState == TransitState.CLOSE
                    ? TransitConstants.limitClosed : TransitConstants.limitOpen);
        }
    }

    private static boolean validServoPosition(double position) {
        return Double.isFinite(position) && position >= 0.0 && position <= 1.0;
    }
}
