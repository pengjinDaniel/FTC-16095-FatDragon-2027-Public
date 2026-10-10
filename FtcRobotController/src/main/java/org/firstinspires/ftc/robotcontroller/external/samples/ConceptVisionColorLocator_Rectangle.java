
package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;


@Disabled
@TeleOp(name = "Concept: Color-Locator (Rectangle)", group = "Concept")
public class ConceptVisionColorLocator_Rectangle extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, blobs);


            telemetry.addLine("Ctr:(X,Y)  Area Dens Aspect Arc  Circ");

            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("(%3d,%3d) %5d %4.2f  %5.2f %3d %5.3f ",
                        (int) boxFit.center.x, (int) boxFit.center.y, b.getContourArea(), b.getDensity(),
                        b.getAspectRatio(), (int) b.getArcLength(), b.getCircularity()));
            }

            telemetry.update();
            sleep(100);
        }
    }
}