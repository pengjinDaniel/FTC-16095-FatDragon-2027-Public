package org.firstinspires.ftc.robotcontroller.external.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

@Utility(name = "Test Hardware", description = "Test hardware devices in your robot configuration")
@SuppressWarnings("unused")
public class UtilityTestHardware extends OpMode {
    protected static class Device {
        String configName;
        HardwareDevice hardwareDevice;

        Device(String name, HardwareDevice device) {
            this.configName = name;
            this.hardwareDevice = device;
        }
    }

    int numDevice = 0;
    List<Device> deviceList = new ArrayList<>();

    @Override
    public void init() {
        for (HardwareDevice device : hardwareMap) {
            Set<String> stringSet = hardwareMap.getNamesOf(device);
            for (String name : stringSet) {
                if (!(name.startsWith("Control Hub") || name.startsWith("Expansion Hub"))) {
                    deviceList.add(new Device(name, device));
                }
            }
        }
        telemetry.addData("Num Configurations", deviceList.size());
    }

    @Override
    public void loop() {
        if (deviceList.isEmpty()){
            telemetry.addData("No Devices", "Found");
            return;
        }
        telemetry.addData("Use Dpad Left/Right", "to select device");
        if (gamepad1.dpadRightWasPressed()) {
            numDevice++;
            if (numDevice >= deviceList.size()) {
                numDevice = 0;
            }
        } else if (gamepad1.dpadLeftWasPressed()) {
            numDevice--;
            if (numDevice < 0) {
                numDevice = deviceList.size() - 1;
            }
        }
        try {
            telemetry.addData("Config Name", deviceList.get(numDevice).configName);
            HardwareDevice hardwareDevice = deviceList.get(numDevice).hardwareDevice;
            telemetry.addData("Device Type", hardwareDevice.getDeviceName());
            telemetry.addData("Connection Info", hardwareDevice.getConnectionInfo());
            if (hardwareDevice instanceof DcMotor) {
                testMotor((DcMotor) hardwareDevice);
            } else if (hardwareDevice instanceof CRServo) {
                testCRServo((CRServo) hardwareDevice);
            } else if (hardwareDevice instanceof Servo) {
                testServo((Servo) hardwareDevice);
            } else if (hardwareDevice instanceof ColorSensor) {
                testColorSensor((ColorSensor) hardwareDevice);
                if (hardwareDevice instanceof DistanceSensor) {
                    testDistanceSensor((DistanceSensor) hardwareDevice);
                }
            } else if (hardwareDevice instanceof DistanceSensor) {
                testDistanceSensor((DistanceSensor) hardwareDevice);
            } else if (hardwareDevice instanceof TouchSensor) {
                testTouchSensor((TouchSensor) hardwareDevice);
            } else if (hardwareDevice instanceof IMU) {
                testIMU((IMU) hardwareDevice);
            } else if (hardwareDevice instanceof WebcamName) {
                testWebcam((WebcamName) hardwareDevice);
            } else if (hardwareDevice instanceof AnalogSensor) {
                testAnalogSensor((AnalogSensor) hardwareDevice);
            } else if (hardwareDevice instanceof DigitalChannel) {
                testDigitalChannel((DigitalChannel) hardwareDevice);
            } else {
                telemetry.addData("Testing", "Not supported");
            }
        } catch (Exception e) {
            telemetry.addData("Exception", e.toString());
        }
    }

    void testMotor(DcMotor motor) {
        telemetry.addData("Use Left joystick Y", "to set motor power");
        telemetry.addData("Use Right Bumper", "to set motor power to value");
        double power = -gamepad1.left_stick_y;
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.addData("Value", power);
        if (gamepad1.right_bumper) {
            telemetry.addData("Motor power set to", power);
            motor.setPower(power);
        }
    }

    void testServo(Servo servo) {
        telemetry.addData("Use Left joystick Y", "to select value for servo position");
        telemetry.addData("Use Right Bumper", "to set servo position to value");
        double position = .5 + (-gamepad1.left_stick_y / 2);
        telemetry.addData("Value", position);
        if (gamepad1.right_bumper) {
            telemetry.addData("Servo position set to", position);
            servo.setPosition(position);
        }
    }

    void testCRServo(CRServo crServo) {
        telemetry.addData("Use Left joystick Y", "to select value for CR servo speed");
        telemetry.addData("Use Right Bumper", "to set CR servo speed to value");
        double power = -gamepad1.left_stick_y;
        telemetry.addData("Value", power);
        if (gamepad1.right_bumper) {
            telemetry.addData("CRServo speed set to", power);
            crServo.setPower(power);
        }
    }

    void testColorSensor(ColorSensor colorSensor) {
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
    }

    void testDistanceSensor(DistanceSensor distanceSensor) {
        telemetry.addData("Distance (IN)", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance (CM)", distanceSensor.getDistance(DistanceUnit.CM));
    }

    void testTouchSensor(TouchSensor touchSensor) {
        telemetry.addData("Pressed", touchSensor.isPressed());
    }

    void testIMU(IMU imu) {
        telemetry.addData("Yaw Pitch Roll", imu.getRobotYawPitchRollAngles());
    }

    void testWebcam(WebcamName webcamName) {
        telemetry.addData("isAttached", webcamName.isAttached());
    }

    void testAnalogSensor(AnalogSensor analogSensor) {
        telemetry.addData("Voltage", analogSensor.readRawVoltage());
    }
    void testDigitalChannel(DigitalChannel digitalChannel) {
        telemetry.addData("State", digitalChannel.getState());
    }

}
