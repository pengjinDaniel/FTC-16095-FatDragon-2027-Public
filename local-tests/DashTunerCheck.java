package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.ConstantProvider;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.config.variable.VariableType;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.subsystems.tuning.DashboardTuner;

/**
 * Replays Dashboard array updates and the Spinup-style DashboardTuner without robot hardware.
 * Fake devices: leftShooterMotor (no encoder), rightShooterMotor (encoder), leftFrontMotor.
 */
public class DashTunerCheck {
    private static int checks;
    private static final String[] NAMES = {"leftShooterMotor", "rightShooterMotor", "leftFrontMotor"};

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual);
    }

    private static final class Fixture {
        final double[] power = new double[NAMES.length];
        final double[] velocity = new double[NAMES.length];
        final double[] velocityTarget = new double[NAMES.length];
        final DcMotor.RunMode[] mode = new DcMotor.RunMode[NAMES.length];
        final DcMotorSimple.Direction[] direction = new DcMotorSimple.Direction[NAMES.length];
        final DcMotorEx[] motors = new DcMotorEx[NAMES.length];
        final CustomVariable config;
        final DashboardTuner tuner;

        Fixture() {
            Arrays.fill(DashboardTuner.motorName, "");
            Arrays.fill(DashboardTuner.servoName, "");
            Arrays.fill(DashboardTuner.colorSensorName, "");
            Arrays.fill(DashboardTuner.motorTarget, 0);
            Arrays.fill(DashboardTuner.servoTarget, -1);
            Arrays.fill(DashboardTuner.slaveTo, -1);
            Arrays.fill(DashboardTuner.slavePowerScale, 1);
            Arrays.fill(DashboardTuner.isPositionCloseLoop, false);
            Arrays.fill(DashboardTuner.isVelocityCloseLoop, false);
            Arrays.fill(DashboardTuner.kP, 0);
            Arrays.fill(DashboardTuner.kI, 0);
            Arrays.fill(DashboardTuner.kD, 0);
            Arrays.fill(DashboardTuner.kF, 0);
            Arrays.fill(mode, DcMotor.RunMode.RUN_USING_ENCODER);
            config = ReflectionConfig.createVariableFromClass(DashboardTuner.class);
            for (int i = 0; i < motors.length; i++) {
                final int index = i;
                motors[i] = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                        new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                            switch (method.getName()) {
                                case "setPower": power[index] = (double) args[0]; return null;
                                case "getPower": return power[index];
                                case "setMode": mode[index] = (DcMotor.RunMode) args[0]; return null;
                                case "getMode": return mode[index];
                                case "setDirection": direction[index] = (DcMotorSimple.Direction) args[0]; return null;
                                case "getVelocity": return velocity[index];
                                case "setVelocity": velocityTarget[index] = (double) args[0]; return null;
                                case "setPIDFCoefficients": return null;
                                case "getCurrentPosition": return 0;
                                case "equals": return proxy == args[0];
                                case "hashCode": return System.identityHashCode(proxy);
                                default: throw new AssertionError("Unexpected motor call: " + method);
                            }
                        });
            }
            // SDK lookup accesses Android's AppUtil; simulate only the hardware lookup boundary.
            HardwareMap map = new HardwareMap(null, null) {
                @Override public <T> T tryGet(Class<? extends T> type, String name) {
                    int index = Arrays.asList(NAMES).indexOf(name);
                    return index >= 0 && type.isInstance(motors[index]) ? type.cast(motors[index]) : null;
                }
            };
            tuner = new DashboardTuner(map);
        }

        /** Same config-tree update used by Dashboard's SAVE_CONFIG handler. */
        void save(int channel, String name, double target) {
            CustomVariable names = new CustomVariable();
            names.putVariable(String.valueOf(channel),
                    new BasicVariable<>(VariableType.STRING, new ConstantProvider<>(name)));
            CustomVariable targets = new CustomVariable();
            targets.putVariable(String.valueOf(channel),
                    new BasicVariable<>(VariableType.DOUBLE, new ConstantProvider<>(target)));
            CustomVariable diff = new CustomVariable();
            diff.putVariable("motorName", names);
            diff.putVariable("motorTarget", targets);
            config.update(diff);
            check(DashboardTuner.motorName[channel].equals(name), "Dashboard writes the motor-name array");
            near(DashboardTuner.motorTarget[channel], target, "Dashboard writes the motor-target array");
        }
    }

    public static void main(String[] args) {
        // Power mode, live retarget, rename and misspelling.
        Fixture f = new Fixture();
        f.save(0, "leftFrontMotor", 0.25);
        f.tuner.applyTargets();
        near(f.power[2], 0.25, "power mode drives the named motor");
        check(f.direction[2] == DcMotorSimple.Direction.REVERSE, "drive direction matches DriveConstants");
        check(f.mode[2] == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "power mode runs without encoder");
        f.save(0, "leftFrontMotor", 0.4);
        f.tuner.applyTargets();
        near(f.power[2], 0.4, "live power updates apply");
        f.save(0, "rightShooterMotor", 0.3);
        f.tuner.applyTargets();
        near(f.power[2], 0, "changing the hardware name stops the previous motor");
        near(f.power[1], 0.3, "changing the hardware name binds the new motor");
        f.save(0, "leftShootermotor", 0.25);
        f.tuner.applyTargets();
        near(f.power[1], 0, "a misspelled name stops the old motor and runs nothing");

        // Shooter: right encoder leader, left follower copies the leader power.
        Fixture s = new Fixture();
        DashboardTuner.motorName[0] = "rightShooterMotor";
        DashboardTuner.motorName[1] = "leftShooterMotor";
        DashboardTuner.slaveTo[1] = 0;
        DashboardTuner.isVelocityCloseLoop[0] = true;
        DashboardTuner.kP[0] = 0.001;
        DashboardTuner.kF[0] = 0.0003;
        DashboardTuner.motorTarget[0] = 1500;
        s.velocity[1] = 1000;
        s.tuner.applyTargets();
        check(s.direction[0] == DcMotorSimple.Direction.FORWARD
                && s.direction[1] == DcMotorSimple.Direction.REVERSE, "shooter directions match ShooterConstants");
        check(s.power[1] > 0, "leader drives positive power below target");
        near(s.power[0], s.power[1], "encoder-less follower copies leader power");
        near(s.velocityTarget[0], 0, "follower never runs its own velocity loop");

        // Encoder-less leftShooterMotor cannot run a closed loop on its own.
        DashboardTuner.slaveTo[1] = -1;
        DashboardTuner.motorName[0] = "";
        DashboardTuner.isVelocityCloseLoop[1] = true;
        DashboardTuner.kP[1] = 0.001;
        DashboardTuner.motorTarget[1] = 1500;
        s.tuner.applyTargets();
        near(s.power[0], 0, "leftShooterMotor closed loop is blocked");
        near(s.power[1], 0, "unbinding the leader stops it");

        // Position and velocity both selected is rejected instead of falling into power mode.
        DashboardTuner.motorName[1] = "rightShooterMotor";
        DashboardTuner.isPositionCloseLoop[1] = true;
        s.tuner.applyTargets();
        near(s.power[1], 0, "conflicting modes stop the motor");

        DashboardTuner.isPositionCloseLoop[1] = false;
        DashboardTuner.isVelocityCloseLoop[1] = false;
        DashboardTuner.motorTarget[1] = 0.5;
        s.tuner.applyTargets();
        near(s.power[1], 0.5, "back to power mode");
        s.tuner.stop();
        for (double p : s.power) near(p, 0, "STOP zeroes every bound motor");
        Arrays.fill(DashboardTuner.motorName, "");
        System.out.println("PASS DashTunerCheck: " + checks + " assertions");
    }
}
