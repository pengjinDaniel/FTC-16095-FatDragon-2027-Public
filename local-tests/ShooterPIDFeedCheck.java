package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterDistanceCheck;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import sun.misc.Unsafe;

/** Replays RT and D-pad Left through ShooterPID, with and without LT. */
public class ShooterPIDFeedCheck {
    private static Unsafe allocator;
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual);
    }

    private static void set(Object object, Class<?> owner, String name, Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(object, value);
    }

    private static Object helper(String name) throws Exception {
        Method method = ShooterDistanceCheck.class.getDeclaredMethod(name);
        method.setAccessible(true);
        return method.invoke(null);
    }

    private static class TestDrive extends Drive {
        TestDrive() { super(null); }
        @Override public void periodic() {}
        @Override public void moveFieldCentric(double forward, double strafe, double turn) {}
        @Override public void stop() {}
    }

    private static class TestIntake extends Intake {
        boolean requested;
        TestIntake() { super(null); }
        @Override public void periodic() {}
        @Override public void updateInputs(boolean requested, boolean jam) { this.requested = requested; }
        @Override public void stop() { requested = false; }
    }

    private static void replay(boolean lt) throws Exception {
        helper("configure");
        ShooterConstants.targetVelocity = 1500.0;
        ShooterConstants.pitchStartPosition = 0.5;
        Shooter shooter = (Shooter) helper("shooter");
        Field field = ShooterDistanceCheck.class.getDeclaredField("velocities");
        field.setAccessible(true);
        double[] velocities = (double[]) field.get(null);
        velocities[0] = velocities[1] = 1500.0;
        TestDrive drive = (TestDrive) allocator.allocateInstance(TestDrive.class);
        TestIntake intake = (TestIntake) allocator.allocateInstance(TestIntake.class);
        Transit transit = (Transit) allocator.allocateInstance(Transit.class);
        double[] position = {-1.0};
        DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) return null;
                    throw new AssertionError("Unexpected motor call: " + method);
                });
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) {
                        position[0] = (double) args[0]; return null;
                    }
                    throw new AssertionError("Unexpected servo call: " + method);
                });
        set(transit, Transit.class, "transitMotor", motor);
        set(transit, Transit.class, "limitServo", servo);
        set(transit, Transit.class, "feedReverseTimer", new ElapsedTime());
        set(transit, Transit.class, "transitState", Transit.TransitState.CLOSE);
        set(transit, Transit.class, "firingState", Transit.FiringState.IDLE);
        ShooterPID test = (ShooterPID) allocator.allocateInstance(ShooterPID.class);
        set(test, ShooterPID.class, "drive", drive);
        set(test, ShooterPID.class, "intake", intake);
        set(test, ShooterPID.class, "shooter", shooter);
        set(test, ShooterPID.class, "transit", transit);
        set(test, ShooterPID.class, "dashboard", allocator.allocateInstance(FtcDashboard.class));
        test.gamepad1 = (Gamepad) allocator.allocateInstance(Gamepad.class);
        test.telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class}, (proxy, method, args) ->
                        method.getReturnType() == boolean.class ? true : null);
        test.gamepad1.right_trigger = 1.0f;
        test.gamepad1.left_trigger = lt ? 1.0f : 0.0f;
        test.gamepad1.dpad_left = true;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "unready gate stays closed; LT=" + lt);
        check(transit.getFiringState() == Transit.FiringState.WAITING, "request waits for readiness");
        Field status = ShooterPID.class.getDeclaredField("controlStatus");
        status.setAccessible(true);
        check(status.get(test).equals("WAITING: SHOOTER SPEED"), "specific speed-wait status");
        check(intake.requested == lt, "LT changes intake request");
        set(shooter, Shooter.class, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
        set(shooter, Shooter.class, "pitchChangedAtNanos", System.nanoTime() - 1_000_000_000L);
        test.loop();
        near(position[0], TransitConstants.limitOpen, "ready gate opens; LT=" + lt);
        check(transit.getFiringState() == Transit.FiringState.REVERSING, "reverse sequence starts");
        check(status.get(test).equals("REVERSING"), "active sequence status");
        check(shooter.isReady() && shooter.isPitchReady(), "readiness stays valid");
        test.gamepad1.dpad_left = false;
        test.loop();
        test.gamepad1.dpad_left = true;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "second D-pad press cancels request");
        check(transit.getFiringState() == Transit.FiringState.IDLE, "cancel ends feed sequence");
        test.gamepad1.dpad_left = false;
        test.loop();
        test.gamepad1.dpad_left = true;
        test.loop();
        near(position[0], TransitConstants.limitOpen, "next D-pad edge requests feeding again");
        test.gamepad1.right_trigger = 0.0f;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "RT release closes gate");
        test.gamepad1.right_trigger = 1.0f;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "held D-pad does not retrigger after RT release");
    }

    public static void main(String[] args) throws Exception {
        Field field = Unsafe.class.getDeclaredField("theUnsafe");
        field.setAccessible(true);
        allocator = (Unsafe) field.get(null);
        replay(false);
        replay(true);
        System.out.println("PASS ShooterPIDFeedCheck: " + checks + " assertions");
    }
}
