package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.PollenChaseController;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.List;

@TeleOp(name = "Limelight Pollen Chase", group = "Test")
public class PollenChaseTest extends LinearOpMode {
    // Robot +X is intake-forward. Camera translation is in cm; positive pitch points upward.
    private static final double CAMERA_FORWARD_CM = -12.715;
    private static final double CAMERA_RIGHT_CM = 0.0; // Assumes LL Right/y is zero; verify on the robot.
    private static final double CAMERA_HEIGHT_CM = 20.7415;
    private static final double CAMERA_PITCH_UP_DEGREES = 50.0;
    private static final double CAMERA_YAW_DEGREES = 180.0;
    // Center to the rear-most robot edge for yaw 180, not center to the lens.
    private static final double APPROACH_EDGE_FROM_CENTER_CM = 17.0;
    private static final double RANGE_SCALE = 1.0;

    private static final int PIPELINE_INDEX = 2;
    private static final double MIN_TARGET_AREA_PERCENT = 0.03;
    private final DcMotor[] motors = new DcMotor[4]; // LF, LB, RF, RB
    private Limelight3A limelight;
    private PollenChaseController chase;
    private String visionStatus = "Waiting for pipeline 2";
    private int actualPipeline = -1;
    private double tx = Double.NaN;
    private double ty = Double.NaN;
    private double frameAgeMs = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        chase = new PollenChaseController(CAMERA_FORWARD_CM, CAMERA_RIGHT_CM, CAMERA_HEIGHT_CM,
                CAMERA_PITCH_UP_DEGREES, CAMERA_YAW_DEGREES, APPROACH_EDGE_FROM_CENTER_CM, RANGE_SCALE);
        try {
            motors[0] = configureMotor(DriveConstants.leftFrontMotorName,
                    DriveConstants.leftFrontMotorDirection);
            motors[1] = configureMotor(DriveConstants.leftBackMotorName,
                    DriveConstants.leftBackMotorDirection);
            motors[2] = configureMotor(DriveConstants.rightFrontMotorName,
                    DriveConstants.rightFrontMotorDirection);
            motors[3] = configureMotor(DriveConstants.rightBackMotorName,
                    DriveConstants.rightBackMotorDirection);
            limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
            limelight.setPollRateHz(50);
            limelight.pipelineSwitch(PIPELINE_INDEX);
            limelight.start();
            telemetry.setMsTransmissionInterval(100);

            while (opModeInInit() && !isStopRequested()) {
                updateVision();
                showTelemetry();
                idle(); // INIT displays measurements; motor power remains zero.
            }
            if (isStopRequested()) return;
            waitForStart();
            chase.reset();

            while (opModeIsActive()) {
                updateVision();
                double forward = chase.getForwardPower();
                double turn = chase.getTurnPower();
                double denominator = Math.max(Math.abs(forward) + Math.abs(turn), 1.0);
                double left = (forward + turn) / denominator;
                double right = (forward - turn) / denominator;
                motors[0].setPower(left);
                motors[1].setPower(left);
                motors[2].setPower(right);
                motors[3].setPower(right);
                showTelemetry();
                idle();
            }
        } finally {
            try {
                for (DcMotor motor : motors) {
                    if (motor != null) motor.setPower(0.0);
                }
            } finally {
                if (limelight != null) limelight.stop();
            }
        }
    }

    private DcMotor configureMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private void updateVision() {
        LLResult result = limelight.getLatestResult();
        boolean valid = false;
        double timestamp = Double.NaN;
        List<List<Double>> corners = null;
        actualPipeline = -1;
        tx = Double.NaN;
        ty = Double.NaN;
        frameAgeMs = Double.NaN;
        if (!limelight.isConnected()) {
            visionStatus = "Limelight disconnected";
        } else if (result == null) {
            visionStatus = "No Limelight result";
        } else {
            actualPipeline = result.getPipelineIndex();
            timestamp = result.getTimestamp();
            tx = result.getTxNC();
            ty = result.getTyNC();
            frameAgeMs = result.getStaleness() + result.getCaptureLatency()
                    + result.getTargetingLatency() + result.getParseLatency();
            if (actualPipeline != PIPELINE_INDEX) {
                visionStatus = "Waiting for pipeline 2";
            } else if (!result.isValid() || result.getColorResults().isEmpty()) {
                visionStatus = "No color target; check pipeline 2";
            } else if (!Double.isFinite(result.getTa())
                    || result.getTa() < MIN_TARGET_AREA_PERCENT || result.getTa() > 100.0) {
                visionStatus = "Target area rejected";
            } else {
                // Keep the contour and principal-pixel angles tied to the same primary target.
                LLResultTypes.ColorResult selected = null;
                double bestAngleDifference = Double.POSITIVE_INFINITY;
                for (LLResultTypes.ColorResult color : result.getColorResults()) {
                    double difference = Math.hypot(color.getTargetXDegreesNoCrosshair() - tx,
                            color.getTargetYDegreesNoCrosshair() - ty);
                    if (difference < bestAngleDifference) {
                        bestAngleDifference = difference;
                        selected = color;
                    }
                }
                if (selected != null) {
                    tx = selected.getTargetXDegreesNoCrosshair();
                    ty = selected.getTargetYDegreesNoCrosshair();
                    corners = selected.getTargetCorners();
                }
                if (corners == null || corners.size() < 4) {
                    visionStatus = "Enable pipeline 2 Output: Send Raw Corners";
                } else {
                    valid = true;
                    visionStatus = "Yellow POLLEN contour";
                }
            }
        }
        chase.update(valid, timestamp, frameAgeMs, tx, ty, corners, System.nanoTime());
    }

    private void showTelemetry() {
        telemetry.addLine("START: automatic REVERSE chase | 3 cm rear-to-ball horizontal gap: stop until restart");
        telemetry.addData("Vision / Pipeline", "%s / %d", visionStatus, actualPipeline);
        telemetry.addData("Chase", chase.getState());
        if (chase.getConfigurationError() != null) {
            telemetry.addData("Camera setup", chase.getConfigurationError());
        }
        telemetry.addData("Pitch up / Yaw (deg)", "%.1f / %.1f",
                CAMERA_PITCH_UP_DEGREES, CAMERA_YAW_DEGREES);
        telemetry.addData("txnc / tync (deg)", "%.2f / %.2f", tx, ty);
        telemetry.addData("Contour width / height (px)", "%.1f / %.1f",
                chase.getContourWidthPixels(), chase.getContourHeightPixels());
        telemetry.addData("Image age (ms)", "%.1f", frameAgeMs);
        telemetry.addData("Rear-to-ball horizontal gap (cm)", "%.2f", chase.getGapCm());
        telemetry.addData("Lens-to-ball center range (cm)", "%.2f", chase.getCameraRangeCm());
        telemetry.addData("Estimated ball height (cm)", "%.2f", chase.getBallHeightCm());
        telemetry.addData("Bearing from approach direction (deg, right +)", "%.2f", chase.getBearingDegrees());
        telemetry.addData("Forward / Turn power", "%.3f / %.3f",
                chase.getForwardPower(), chase.getTurnPower());
        telemetry.update();
    }
}
