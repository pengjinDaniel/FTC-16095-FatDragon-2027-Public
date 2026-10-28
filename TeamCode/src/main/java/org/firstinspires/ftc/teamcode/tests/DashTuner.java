package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DashTuner", group = "Tests")
@Config
public class DashTuner extends OpMode {
    private static final String VERSION = "RAW-20261028-1";

    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static String[] colorSensorName = {"", "", "", ""};
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = new double[4];
    public static int[] slaveTo = {-1, -1, -1, -1};
    public static boolean[] isPositionCloseLoop = new boolean[4];
    public static boolean[] isVelocityCloseLoop = new boolean[4];
    public static double[] kP = new double[4];
    public static double[] kI = new double[4];
    public static double[] kD = new double[4];
    public static double[] kF = new double[4];

    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final Servo[] servos = new Servo[4];
    private final ColorSensor[] colorSensors = new ColorSensor[4];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[4];
    private final String[] boundMotorName = {"", "", "", ""};
    private final String[] boundServoName = {"", "", "", ""};
    private final String[] boundColorSensorName = {"", "", "", ""};
    private final PIDFController[] positionControllers = {
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0)
    };
    private final float[] hsvValues = new float[3];
    private FtcDashboard dashboard;
    private long loopCount;

    @Override
    public void init() {
        loopCount = 0;
        dashboard = FtcDashboard.getInstance();
        bindHardware();
        publish(false);
    }

    @Override
    public void init_loop() {
        bindHardware();
        publish(false);
    }

    @Override
    public void start() {
        bindHardware();
    }

    @Override
    public void loop() {
        loopCount++;
        publish(true);
    }

    @Override
    public void stop() {
        for (int i = 0; i < motors.length; i++) {
            stopMotor(i);
        }
    }

    private void bindHardware() {
        for (int i = 0; i < motors.length; i++) {
            String name = hardwareName(motorName[i]);
            if (!name.equals(boundMotorName[i]) || motors[i] == null) {
                stopMotor(i);
                motors[i] = name.isEmpty() ? null : hardwareMap.get(DcMotorEx.class, name);
                boundMotorName[i] = name;
                if (motors[i] != null) {
                    motors[i].setPower(0.0);
                    motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }

            name = hardwareName(servoName[i]);
            if (!name.equals(boundServoName[i]) || servos[i] == null) {
                servos[i] = name.isEmpty() ? null : hardwareMap.get(Servo.class, name);
                boundServoName[i] = name;
            }

            name = hardwareName(colorSensorName[i]);
            if (!name.equals(boundColorSensorName[i]) || colorSensors[i] == null) {
                colorSensors[i] = name.isEmpty() ? null : hardwareMap.get(ColorSensor.class, name);
                distanceSensors[i] = name.isEmpty() ? null : hardwareMap.tryGet(DistanceSensor.class, name);
                boundColorSensorName[i] = name;
            }
        }
    }

    private void publish(boolean running) {
        TelemetryPacket packet = new TelemetryPacket();
        status(packet, "DashTuner Version", VERSION);
        status(packet, "DashTuner", running ? "RUNNING" : "INIT: press START to apply targets");
        status(packet, "Loop", loopCount);
        telemetry.addLine("Raw slot control: gamepad inputs and production subsystem settings are not used.");
        telemetry.addLine("Set names during INIT; START applies targets. STOP stops all bound motors.");
        telemetry.addLine("Both closed-loop switches off: power; enable only one for position or velocity.");
        for (int i = 0; i < motors.length; i++) {
            String name = hardwareName(motorName[i]);
            status(packet, "motorName " + i, name);
            status(packet, "motorTarget " + i, motorTarget[i]);
            if (!name.equals(boundMotorName[i])) {
                stopMotor(i);
                status(packet, "Motor " + i, "Name changed; stop and reinitialize");
            } else if (motors[i] != null) {
                if (running) {
                    updateMotor(i, packet);
                } else {
                    stopMotor(i);
                    status(packet, "Motor " + i, "Ready: " + boundMotorName[i]);
                }
                status(packet, "currentPosition " + i, motors[i].getCurrentPosition());
                status(packet, "currentVelocity " + i, motors[i].getVelocity());
                status(packet, "currentPower" + i, motors[i].getPower());
            } else if (!name.isEmpty()) {
                status(packet, "Motor " + i, "Not found: " + name);
            }

            name = hardwareName(servoName[i]);
            if (!name.equals(boundServoName[i])) {
                status(packet, "Servo " + i, "Name changed; stop and reinitialize");
            } else if (servos[i] != null) {
                if (hasDuplicate(servos, i)) {
                    status(packet, "Servo " + i, "Hardware is assigned to multiple slots");
                } else if (Double.isFinite(servoTarget[i])) {
                    if (running) {
                        servos[i].setPosition(Range.clip(servoTarget[i], 0.0, 1.0));
                    }
                    status(packet, "Servo " + i, "Ready: " + boundServoName[i]);
                } else {
                    status(packet, "Servo " + i, "Target must be finite");
                }
                status(packet, "servoPosition " + i, servos[i].getPosition());
            } else if (!name.isEmpty()) {
                status(packet, "Servo " + i, "Not found: " + name);
            }

            name = hardwareName(colorSensorName[i]);
            if (!name.equals(boundColorSensorName[i])) {
                status(packet, "Color sensor " + i, "Name changed; stop and reinitialize");
            } else if (colorSensors[i] != null) {
                readColorSensor(i, packet);
            } else if (!name.isEmpty()) {
                status(packet, "Color sensor " + i, "Not found: " + name);
            }
        }
        if (dashboard != null) {
            dashboard.sendTelemetryPacket(packet);
        }
        telemetry.update();
    }

    private void updateMotor(int i, TelemetryPacket packet) {
        if (hasDuplicate(motors, i)) {
            stopMotor(i);
            status(packet, "Motor " + i, "Hardware is assigned to multiple slots");
            return;
        }

        int source = i;
        boolean velocityControl = isVelocityCloseLoop[i];
        boolean positionControl = isPositionCloseLoop[i];
        double target = motorTarget[i];
        if (slaveTo[i] != -1) {
            source = slaveTo[i];
            if (source < 0 || source >= motors.length || source == i
                    || motors[source] == null || hasDuplicate(motors, source)
                    || slaveTo[source] != -1 || !isVelocityCloseLoop[source]
                    || isPositionCloseLoop[source]
                    || !boundMotorName[source].equals(hardwareName(motorName[source]))) {
                stopMotor(i);
                status(packet, "Motor " + i, "Follower requires a bound velocity master (0-3)");
                return;
            }
            velocityControl = true;
            positionControl = false;
            target = -motorTarget[source];
        }

        if ((velocityControl && positionControl) || !Double.isFinite(target)
                || ((velocityControl || positionControl) && (!Double.isFinite(kP[source])
                || !Double.isFinite(kI[source]) || !Double.isFinite(kD[source])
                || !Double.isFinite(kF[source])))) {
            stopMotor(i);
            status(packet, "Motor " + i, "Enable at most one closed loop; target and PIDF must be finite");
            return;
        }

        DcMotor.RunMode runMode = velocityControl
                ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (motors[i].getMode() != runMode) {
            motors[i].setPower(0.0);
            motors[i].setMode(runMode);
            positionControllers[i].reset();
        }

        if (velocityControl) {
            positionControllers[i].reset();
            motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP[source], kI[source], kD[source], kF[source]));
            motors[i].setVelocity(target);
            status(packet, "targetVelocity " + i, target);
        } else if (positionControl) {
            positionControllers[i].setPIDF(kP[i], kI[i], kD[i], kF[i]);
            double power = positionControllers[i].calculate(motors[i].getCurrentPosition(), target);
            if (!Double.isFinite(power)) {
                stopMotor(i);
                status(packet, "Motor " + i, "Position controller output must be finite");
                return;
            }
            motors[i].setPower(Range.clip(power, -1.0, 1.0));
            status(packet, "targetPosition " + i, target);
        } else {
            positionControllers[i].reset();
            motors[i].setPower(Range.clip(target, -1.0, 1.0));
            status(packet, "targetPower " + i, Range.clip(target, -1.0, 1.0));
        }
        String mode = velocityControl ? "VELOCITY" : positionControl ? "POSITION" : "POWER";
        status(packet, "Motor " + i, boundMotorName[i] + ": "
                + (source == i ? mode : "VELOCITY follower of " + source));
    }

    private void readColorSensor(int i, TelemetryPacket packet) {
        int red = colorSensors[i].red();
        int green = colorSensors[i].green();
        int blue = colorSensors[i].blue();
        double scale = 255.0 / Math.max(1, Math.max(red, Math.max(green, blue)));
        Color.RGBToHSV((int) Math.round(red * scale), (int) Math.round(green * scale),
                (int) Math.round(blue * scale), hsvValues);
        packet.put("Alpha" + i, colorSensors[i].alpha());
        packet.put("Red" + i, red);
        packet.put("Green" + i, green);
        packet.put("Blue" + i, blue);
        packet.put("Hue" + i, hsvValues[0]);
        if (distanceSensors[i] != null) {
            packet.put("Distance (cm)" + i, distanceSensors[i].getDistance(DistanceUnit.CM));
        }
        status(packet, "Color sensor " + i, distanceSensors[i] == null
                ? "Color only; distance is unavailable" : "Color and distance");
    }

    private void stopMotor(int i) {
        if (motors[i] != null) {
            motors[i].setPower(0.0);
        }
        positionControllers[i].reset();
    }

    private void status(TelemetryPacket packet, String key, Object value) {
        packet.put(key, value);
        telemetry.addData(key, value);
    }

    private static boolean hasDuplicate(Object[] devices, int index) {
        for (int i = 0; i < devices.length; i++) {
            if (i != index && devices[i] == devices[index]) {
                return true;
            }
        }
        return false;
    }

    private static String hardwareName(String name) {
        return name == null ? "" : name.trim();
    }
}
