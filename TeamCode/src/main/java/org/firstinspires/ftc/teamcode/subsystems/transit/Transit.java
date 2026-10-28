package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transit extends SubsystemBase {
    private final DcMotor transitMotor;
    private final Servo limitServo;
    private TransitState transitState = TransitState.CLOSE;
    private double requestedPower;
    private final ElapsedTime feedReverseTimer = new ElapsedTime();
    private double feedReverseTimeSeconds;
    private double feedReversePower;
    private FiringState firingState = FiringState.IDLE;

    public enum FiringState {
        IDLE,
        WAITING,
        REVERSING,
        FEEDING
    }

    public enum TransitState {
        CLOSE,
        OPEN
    }

    public Transit(HardwareMap hardwareMap) {
        transitMotor = hardwareMap.get(DcMotor.class, TransitConstants.transitMotorName);
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
        transitMotor.setDirection(TransitConstants.transitMotorDirection);
        transitMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopMotor();
        close();
    }

    public void updateIntakeRequest(boolean intakeRequested, boolean jamRequested) {
        updateIntakeRequest(intakeRequested, jamRequested, false);
    }

    public void updateIntakeRequest(boolean intakeRequested, boolean jamRequested,
                                    boolean intakeReverseRequested) {
        if (firingState != FiringState.IDLE || transitState == TransitState.OPEN) {
            cancelFeeding();
        }
        close();
        if (jamRequested) {
            reverse();
        } else if (intakeReverseRequested) {
            forwardForIntakeReverse();
        } else if (intakeRequested) {
            forward();
        } else {
            stopMotor();
        }
    }

    // Once the gate opens, only explicit cancellation ends the reverse/feed sequence.
    public void requestFeed(boolean readyToOpenLimit) {
        if (firingState != FiringState.REVERSING && firingState != FiringState.FEEDING) {
            double reverseTime = TransitConstants.feedReverseTime;
            double reversePower = TransitConstants.feedReversePower;
            if (!readyToOpenLimit || !areLimitPositionsCalibrated()
                    || !Double.isFinite(reverseTime) || reverseTime < 0.0
                    || !Double.isFinite(reversePower) || reversePower >= 0.0 || reversePower < -1.0) {
                cancelFeeding();
                firingState = FiringState.WAITING;
                return;
            }
            // RT takes ownership before the gate moves, including while LT remains held.
            stopMotor();
            open();
            feedReverseTimeSeconds = reverseTime;
            feedReversePower = reversePower;
            feedReverseTimer.reset();
            firingState = FiringState.REVERSING;
        }
        if (firingState == FiringState.REVERSING) {
            if (feedReverseTimer.seconds() >= feedReverseTimeSeconds) {
                firingState = FiringState.FEEDING;
            } else {
                setPower(feedReversePower);
            }
        }
        if (firingState == FiringState.FEEDING) {
            setPower(TransitConstants.feedPower);
        }
    }

    public void cancelFeeding() {
        stopMotor();
        close();
        firingState = FiringState.IDLE;
        feedReverseTimer.reset();
    }

    public FiringState getFiringState() {
        return firingState;
    }

    public double getFeedReverseTimerProgress() {
        double reverseTime = feedReverseTimeSeconds;
        if (firingState == FiringState.FEEDING) {
            return 1.0;
        }
        return firingState == FiringState.REVERSING
                && Double.isFinite(reverseTime) && reverseTime > 0.0
                ? Math.min(1.0, feedReverseTimer.seconds() / reverseTime) : 0.0;
    }

    public void forward() {
        setPower(TransitConstants.forwardPower);
    }

    public void forwardForIntakeReverse() {
        setPower(TransitConstants.intakeReverseForwardPower);
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
        requestedPower = Double.isFinite(power) ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
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
