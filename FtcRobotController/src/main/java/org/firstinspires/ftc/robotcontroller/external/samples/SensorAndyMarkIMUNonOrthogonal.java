
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot.xyzOrientation;

@TeleOp(name = "Sensor: AndyMark IMU Non-Orthogonal", group = "Sensor")
@Disabled
public class SensorAndyMarkIMUNonOrthogonal extends LinearOpMode
{
    private IMU imu;


    @Override public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");


        double xRotation = 0;
        double yRotation = 0;
        double zRotation = 0;

        Orientation imuRotation = xyzOrientation(xRotation, yRotation, zRotation);

        AndyMarkIMUOrientationOnRobot orientationOnRobot = new AndyMarkIMUOrientationOnRobot(imuRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        while (!isStopRequested()) {
            telemetry.addData("IMU orientation", "X=%.1f,  Y=%.1f,  Z=%.1f \n", xRotation, yRotation, zRotation);

            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
}
