
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Concept: AprilTagMultiPortal", group = "Concept")
@Disabled
public class ConceptAprilTagMultiPortal extends LinearOpMode
{
    VisionPortal portal1;
    VisionPortal portal2;

    AprilTagProcessor aprilTagProcessor1;
    AprilTagProcessor aprilTagProcessor2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        aprilTagProcessor1 = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();

        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(aprilTagProcessor1)
                .build();
        portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setLiveViewContainerId(portal2ViewId)
                .addProcessor(aprilTagProcessor2)
                .build();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Number of tags in Camera 1", aprilTagProcessor1.getDetections().size());
            telemetry.addData("Number of tags in Camera 2", aprilTagProcessor2.getDetections().size());
            telemetry.update();
            sleep(20);
        }
    }
}
