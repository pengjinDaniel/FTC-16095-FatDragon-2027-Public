
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot;
import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot.I2cPortFacingDirection;
import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Sensor: AndyMark IMU Orthogonal", group = "Sensor")
@Disabled
public class SensorAndyMarkIMUOrthogonal extends LinearOpMode
{
    private IMU imu;


    @Override public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");


        LogoFacingDirection logoDirection = LogoFacingDirection.UP;
        I2cPortFacingDirection i2cDirection = I2cPortFacingDirection.FORWARD;

        AndyMarkIMUOrientationOnRobot orientationOnRobot = new AndyMarkIMUOrientationOnRobot(logoDirection, i2cDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        while (!isStopRequested()) {
            telemetry.addData("IMU orientation", "Logo=%s   I2C=%s\n ", logoDirection, i2cDirection);

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
