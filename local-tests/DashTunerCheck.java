package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.ConstantProvider;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.config.variable.VariableType;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Replays Dashboard array updates and DashTuner motor binding without robot hardware. */
public class DashTunerCheck {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual);
    }

    private static final class Fixture {
        final DashTuner op = new DashTuner();
        final double[] power = {0, 0};
        final DcMotor.RunMode[] mode = {
                DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.RUN_USING_ENCODER
        };
        final Map<String, Object> telemetry = new LinkedHashMap<>();
        final CustomVariable config;

        Fixture() {
            Arrays.fill(DashTuner.motorName, "");
            Arrays.fill(DashTuner.servoName, "");
            Arrays.fill(DashTuner.colorSensorName, "");
            Arrays.fill(DashTuner.motorTarget, 0);
            Arrays.fill(DashTuner.slaveTo, -1);
            Arrays.fill(DashTuner.motorMode, DashTuner.MotorMode.POWER);
            config = ReflectionConfig.createVariableFromClass(DashTuner.class);
            DcMotorEx[] motors = new DcMotorEx[2];
            for (int i = 0; i < motors.length; i++) {
                final int index = i;
                motors[i] = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                        new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                            switch (method.getName()) {
                                case "setPower": power[index] = (double) args[0]; return null;
                                case "getPower": return power[index];
                                case "setMode": mode[index] = (DcMotor.RunMode) args[0]; return null;
                                case "getMode": return mode[index];
                                case "getVelocity": return 0.0;
                                case "getCurrentPosition": return 0;
                                default: throw new AssertionError("Unexpected motor call: " + method);
                            }
                        });
            }
            // SDK lookup accesses Android's AppUtil; simulate only the hardware lookup boundary.
            op.hardwareMap = new HardwareMap(null, null) {
                @Override public <T> T tryGet(Class<? extends T> type, String name) {
                    int index = name.equals("leftShooterMotor") ? 0 : name.equals("rightShooterMotor") ? 1 : -1;
                    return index >= 0 && type.isInstance(motors[index]) ? type.cast(motors[index]) : null;
                }
            };
            op.telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class}, (proxy, method, args) -> {
                        if (method.getName().equals("addData")) telemetry.put((String) args[0], args[1]);
                        return method.getReturnType() == boolean.class ? true : null;
                    });
        }

        void save(String name, double target) {
            CustomVariable names = new CustomVariable();
            names.putVariable("0", new BasicVariable<>(VariableType.STRING, new ConstantProvider<>(name)));
            CustomVariable targets = new CustomVariable();
            targets.putVariable("0", new BasicVariable<>(VariableType.DOUBLE, new ConstantProvider<>(target)));
            CustomVariable diff = new CustomVariable();
            diff.putVariable("motorName", names);
            diff.putVariable("motorTarget", targets);
            config.update(diff); // Same config-tree update used by Dashboard's SAVE_CONFIG handler.
            check(DashTuner.motorName[0].equals(name), "Dashboard writes the motor-name array");
            near(DashTuner.motorTarget[0], target, "Dashboard writes the motor-target array");
        }
    }

    public static void main(String[] args) {
        Fixture beforeInit = new Fixture();
        beforeInit.save("leftShooterMotor", 0.25);
        beforeInit.op.init();
        beforeInit.op.init_loop();
        near(beforeInit.power[0], 0, "INIT keeps configured motor stopped");
        beforeInit.op.start();
        beforeInit.op.loop();
        near(beforeInit.power[0], 0.25, "configuration saved before INIT works");
        check(beforeInit.mode[0] == DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                "POWER mode works with an encoderless motor");
        beforeInit.op.stop();
        near(beforeInit.power[0], 0, "STOP stops a configured motor");

        Fixture atStart = new Fixture();
        atStart.op.init();
        atStart.op.init_loop();
        atStart.save("leftShooterMotor", 0.25);
        atStart.op.start();
        atStart.op.loop();
        near(atStart.power[0], 0.25, "configuration saved after the last INIT callback applies on START");
        atStart.op.stop();

        Fixture running = new Fixture();
        running.op.init();
        running.op.start();
        running.save("leftShooterMotor", 0.25);
        running.op.loop();
        near(running.power[0], 0.25, "configuration saved after START binds the motor");
        check(running.telemetry.get("Motor 0 config").toString().contains("target=0.25"),
                "Driver Station shows the received target");
        check(running.telemetry.get("Motor 0 config").toString().contains("leftShooterMotor"),
                "Driver Station shows the received hardware name");
        check(running.telemetry.get("DashTuner state").toString().startsWith("RUNNING"),
                "Driver Station shows the running state");
        check(running.telemetry.get("DashTuner version").toString().startsWith("DashTuner-"),
                "Driver Station identifies the deployed version");
        running.save("leftShooterMotor", 0.4);
        running.op.loop();
        near(running.power[0], 0.4, "live power updates still apply");
        running.save("rightShooterMotor", 0.3);
        running.op.loop();
        near(running.power[0], 0, "changing the hardware name stops the previous motor");
        near(running.power[1], 0.3, "changing the hardware name binds the new motor");
        running.save("", 0.3);
        running.op.loop();
        near(running.power[1], 0, "clearing the hardware name stops and unbinds the motor");
        running.save("leftShootermotor", 0.25);
        running.op.loop();
        near(running.power[0], 0, "a misspelled hardware name cannot run a different motor");
        check(running.telemetry.get("Motor 0").toString().startsWith("Not found:"),
                "a missing motor has a visible error");
        running.save("leftShooterMotor", 0.25);
        running.op.loop();
        near(running.power[0], 0.25, "correcting a hardware name takes effect while running");
        running.op.stop();
        near(running.power[0], 0, "STOP stops the rebound motor");
        near(running.power[1], 0, "STOP leaves the previous motor stopped");
        System.out.println("PASS DashTunerCheck: " + checks + " assertions");
    }
}
