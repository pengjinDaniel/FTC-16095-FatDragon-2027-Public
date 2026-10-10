
package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;


@Disabled
@TeleOp(name = "Concept: Vision Color-Sensor", group = "Concept")
public class ConceptVisionColorSensor extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");



            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                                            result.RGB[0], result.RGB[1], result.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                                            result.HSV[0], result.HSV[1], result.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                                            result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            telemetry.update();

            sleep(20);
        }
    }
}
