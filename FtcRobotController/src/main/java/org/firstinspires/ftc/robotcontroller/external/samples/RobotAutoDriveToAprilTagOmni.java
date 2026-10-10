
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="Omni Drive To AprilTag", group = "Concept")
@Disabled
public class RobotAutoDriveToAprilTagOmni extends LinearOpMode
{
    final double DESIRED_DISTANCE = 30.0;

    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private final boolean USE_WEBCAM = true;
    private final int DESIRED_TAG_ID = -1;
    private final String DESIRED_CLUSTER_NAME = null;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private boolean targetFound  = false;
    private String targetName    = "none";
    private int    targetID      = 0;
    private double targetRange   = 0;
    private double targetBearing = 0;
    private double targetYaw     = 0;

    @Override public void runOpMode()
    {
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;

        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM)
            setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {

                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;

                    if (singleDetection.metadata != null) {
                        if ((DESIRED_TAG_ID < 0) || (singleDetection.id == DESIRED_TAG_ID)) {
                            targetName    = singleDetection.metadata.name;
                            targetID      = singleDetection.id;
                            targetRange   = singleDetection.ftcPose.range;
                            targetBearing = singleDetection.ftcPose.bearing;
                            targetYaw     = singleDetection.ftcPose.yaw;
                            targetFound = true;
                            break;
                        } else {
                            telemetry.addData("Skipping", "Tag ID %d is not desired", singleDetection.id);
                        }
                    } else {
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", singleDetection.id);
                    }
                } else {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                    if (DESIRED_CLUSTER_NAME == null || clusterDet.metadata.shortName.equals(DESIRED_CLUSTER_NAME) ) {
                        targetName    = clusterDet.metadata.shortName;
                        targetID      = -1;
                        targetRange   = clusterDet.ftcPose.range;
                        targetBearing = clusterDet.ftcPose.bearing;
                        targetYaw     = clusterDet.ftcPose.yaw;
                        targetFound = true;
                        break;
                    }
                }
            }

            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", targetID, targetName);
                telemetry.addData("Range",  "%5.1f inches", targetRange);
                telemetry.addData("Bearing","%3.0f degrees", targetBearing);
                telemetry.addData("Yaw","%3.0f degrees", targetYaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            if (gamepad1.left_bumper && targetFound) {

                double  rangeError      = targetRange - DESIRED_DISTANCE;
                double  headingError    = targetBearing;
                double  yawError        = targetYaw;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                drive  = -gamepad1.left_stick_y  / 2.0;
                strafe = -gamepad1.left_stick_x  / 2.0;
                turn   = -gamepad1.right_stick_x / 3.0;
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
