package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final DistanceSensor colorDistanceSensor;
    private final Servo leftFlowerServo;
    private final Servo rightFlowerServo;
    private double requestedPower;
    private double distanceCm = Double.NaN;
    private int ballCount;
    private boolean ballPresent;
    private boolean countedCurrentPresence;
    private boolean jamRequested;
    private boolean reverseRequested;
    private boolean intakeRequested;
    private boolean shootRequested;
    private FlowerState flowerState = FlowerState.RAISED;

    public enum FlowerState {
        RAISED,
        DEPLOYED
    }

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        colorDistanceSensor = hardwareMap.get(DistanceSensor.class, IntakeConstants.colorSensorName);
        leftFlowerServo = hardwareMap.get(Servo.class, IntakeConstants.leftFlowerServoName);
        rightFlowerServo = hardwareMap.get(Servo.class, IntakeConstants.rightFlowerServoName);
        intakeMotor.setDirection(IntakeConstants.intakeMotorDirection);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
        raiseFlower();
    }

    public void updateInputs(boolean intakeRequested, boolean jamRequested) {
        updateInputs(intakeRequested, jamRequested, false);
    }

    public void updateInputs(boolean intakeRequested, boolean jamRequested, boolean shootRequested) {
        if (this.jamRequested && !jamRequested) {
            clearBallCount();
        }
        this.jamRequested = jamRequested;
        this.intakeRequested = intakeRequested;
        this.shootRequested = shootRequested;
        updateMotorPower();
    }

    public void setReverseRequested(boolean reverseRequested) {
        this.reverseRequested = reverseRequested;
        updateMotorPower();
    }

    public void clearBallCount() {
        ballCount = 0;
        updateMotorPower();
    }

    public void suspend() {
        shootRequested = false;
        setPower(0.0);
    }

    public void stop() {
        jamRequested = false;
        reverseRequested = false;
        intakeRequested = false;
        shootRequested = false;
        setPower(0.0);
    }

    public int getBallCount() {
        return ballCount;
    }

    public double getDistanceCm() {
        return distanceCm;
    }

    public boolean isBallPresent() {
        return ballPresent;
    }

    public boolean areFlowerPositionsCalibrated() {
        return IntakeConstants.flowerPositionsCalibrated
                && validServoPosition(IntakeConstants.leftFlowerRaisedPosition)
                && validServoPosition(IntakeConstants.rightFlowerRaisedPosition)
                && validServoPosition(IntakeConstants.leftFlowerDeployedPosition)
                && validServoPosition(IntakeConstants.rightFlowerDeployedPosition)
                && IntakeConstants.leftFlowerRaisedPosition
                != IntakeConstants.leftFlowerDeployedPosition
                && IntakeConstants.rightFlowerRaisedPosition
                != IntakeConstants.rightFlowerDeployedPosition;
    }

    public FlowerState getFlowerState() {
        return flowerState;
    }

    public void toggleFlower() {
        if (!areFlowerPositionsCalibrated()) {
            return;
        }
        setFlowerState(flowerState == FlowerState.RAISED
                ? FlowerState.DEPLOYED : FlowerState.RAISED);
    }

    public void raiseFlower() {
        setFlowerState(FlowerState.RAISED);
    }

    public double getRequestedPower() {
        return requestedPower;
    }

    @Override
    public void periodic() {
        distanceCm = colorDistanceSensor.getDistance(DistanceUnit.CM);
        boolean validDistance = Double.isFinite(distanceCm) && distanceCm >= 0.0;
        if (validDistance) {
            if (distanceCm <= IntakeConstants.ballEnterDistanceCm) {
                ballPresent = true;
            } else if (ballPresent && distanceCm >= IntakeConstants.ballExitDistanceCm) {
                ballPresent = false;
                countedCurrentPresence = false;
            }
        }
        if (validDistance && ballPresent && !countedCurrentPresence) {
            if (ballCount >= IntakeConstants.maxBallCount) {
                countedCurrentPresence = true;
            } else if (requestedPower > 0.0) {
                ballCount++;
                countedCurrentPresence = true;
            }
        }
        updateMotorPower();
    }

    private void updateMotorPower() {
        if (jamRequested || reverseRequested) {
            setPower(IntakeConstants.jamPower);
        } else if (shootRequested) {
            setPower(IntakeConstants.feedPower);
        } else if (ballCount >= IntakeConstants.maxBallCount) {
            setPower(IntakeConstants.fullReversePower);
        } else if (!Double.isFinite(distanceCm) || distanceCm < 0.0) {
            setPower(0.0);
        } else if (intakeRequested) {
            setPower(IntakeConstants.activePower);
        } else {
            setPower(0.0);
        }
    }

    private void setFlowerState(FlowerState state) {
        flowerState = state;
        if (!areFlowerPositionsCalibrated()) {
            return;
        }
        leftFlowerServo.setPosition(state == FlowerState.RAISED
                ? IntakeConstants.leftFlowerRaisedPosition
                : IntakeConstants.leftFlowerDeployedPosition);
        rightFlowerServo.setPosition(state == FlowerState.RAISED
                ? IntakeConstants.rightFlowerRaisedPosition
                : IntakeConstants.rightFlowerDeployedPosition);
    }

    private static boolean validServoPosition(double position) {
        return Double.isFinite(position) && position >= 0.0 && position <= 1.0;
    }

    private void setPower(double power) {
        requestedPower = power;
        intakeMotor.setPower(requestedPower);
    }
}
