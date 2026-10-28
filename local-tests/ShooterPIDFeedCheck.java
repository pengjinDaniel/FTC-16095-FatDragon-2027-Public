package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.DashboardCore;
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

/** Replays pitch adjustment and automatic RT feeding through ShooterPID, with and without LT. */
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
        boolean shootRequested;
        boolean jamRequested;
        TestIntake() { super(null); }
        @Override public void periodic() {}
        @Override public void updateInputs(boolean requested, boolean jam, boolean shoot) {
            this.requested = requested;
            shootRequested = shoot;
            jamRequested = jam;
        }
        @Override public void stop() { requested = false; shootRequested = false; jamRequested = false; }
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
        FtcDashboard dashboard = (FtcDashboard) allocator.allocateInstance(FtcDashboard.class);
        set(dashboard, FtcDashboard.class, "core", allocator.allocateInstance(DashboardCore.class));
        set(test, ShooterPID.class, "dashboard", dashboard);
        test.gamepad1 = (Gamepad) allocator.allocateInstance(Gamepad.class);
        test.telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class}, (proxy, method, args) ->
                        method.getReturnType() == boolean.class ? true : null);
        test.gamepad1.dpad_right = true;
        test.loop();
        near(shooter.getPitchPosition(), 0.51, "D-pad Right raises pitch without RT");
        test.loop();
        near(shooter.getPitchPosition(), 0.51, "held D-pad Right adjusts only once");
        test.gamepad1.dpad_right = false;
        test.gamepad1.dpad_left = true;
        test.loop();
        near(shooter.getPitchPosition(), 0.5, "D-pad Left lowers pitch without RT");
        test.loop();
        near(shooter.getPitchPosition(), 0.5, "held D-pad Left adjusts only once");
        test.gamepad1.dpad_left = false;
        test.loop();
        shooter.setPitchPosition(ShooterConstants.pitchMaxPosition - 0.005);
        test.gamepad1.dpad_right = true;
        test.loop();
        near(shooter.getPitchPosition(), ShooterConstants.pitchMaxPosition, "pitch obeys upper limit");
        test.gamepad1.dpad_right = false;
        test.loop();
        test.gamepad1.dpad_right = true;
        test.gamepad1.dpad_left = true;
        test.loop();
        near(shooter.getPitchPosition(), ShooterConstants.pitchMaxPosition, "simultaneous edges cancel at limit");
        test.gamepad1.dpad_right = false;
        test.gamepad1.dpad_left = false;
        test.loop();
        shooter.setPitchPosition(ShooterConstants.pitchMinPosition + 0.005);
        test.gamepad1.dpad_left = true;
        test.loop();
        near(shooter.getPitchPosition(), ShooterConstants.pitchMinPosition, "pitch obeys lower limit");
        test.gamepad1.dpad_left = false;
        test.loop();
        test.gamepad1.right_bumper = true;
        test.loop();
        near(position[0], TransitConstants.limitOpen, "RB opens the manual gate without RT");
        test.loop();
        near(position[0], TransitConstants.limitOpen, "held RB toggles the manual gate only once");
        test.gamepad1.right_bumper = false;
        test.loop();
        test.gamepad1.right_bumper = true;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "next RB edge closes the manual gate");
        test.gamepad1.right_bumper = false;
        test.loop();
        shooter.setPitchPosition(0.5);
        test.gamepad1.right_trigger = 1.0f;
        test.gamepad1.left_trigger = lt ? 1.0f : 0.0f;
        test.gamepad1.dpad_right = true;
        test.loop();
        near(shooter.getPitchPosition(), 0.51, "pitch adjusts while RT is held");
        check(transit.getFiringState() == Transit.FiringState.WAITING, "RT requests feed without A");
        near(transit.getRequestedPower(), 0.0, "RT keeps transit stopped until ready");
        test.gamepad1.dpad_right = false;
        test.loop();
        near(shooter.getPitchPosition(), 0.51, "RT does not reset adjusted pitch");
        check(!test.gamepad1.a, "A is never required to start the RT feed sequence");
        near(position[0], TransitConstants.limitClosed, "unready gate stays closed; LT=" + lt);
        check(transit.getFiringState() == Transit.FiringState.WAITING, "request waits for readiness");
        Field status = ShooterPID.class.getDeclaredField("controlStatus");
        status.setAccessible(true);
        check(status.get(test).equals("WAITING: SHOOTER SPEED"), "specific speed-wait status");
        check(intake.requested == lt, "LT changes intake request");
        check(intake.shootRequested, "RT requests intake forward with or without LT");
        set(shooter, Shooter.class, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
        set(shooter, Shooter.class, "pitchChangedAtNanos", System.nanoTime() - 1_000_000_000L);
        test.loop();
        near(position[0], TransitConstants.limitOpen, "ready gate opens; LT=" + lt);
        check(transit.getFiringState() == Transit.FiringState.REVERSING, "reverse sequence starts");
        check(status.get(test).equals("REVERSING"), "active sequence status");
        check(shooter.isReady() && shooter.isPitchReady(), "readiness stays valid");
        set(transit, Transit.class, "feedReverseTimeSeconds", 0.0);
        test.loop();
        check(transit.getFiringState() == Transit.FiringState.FEEDING, "reverse sequence reaches feed");
        near(transit.getRequestedPower(), TransitConstants.feedPower, "RT uses separate shooting feed power");
        test.gamepad1.a = true;
        test.loop();
        check(transit.getFiringState() == Transit.FiringState.FEEDING, "A does not cancel automatic RT feeding");
        test.gamepad1.a = false;
        test.gamepad1.right_bumper = true;
        test.loop();
        near(position[0], TransitConstants.limitOpen, "RT retains gate ownership while RB is pressed");
        near(transit.getRequestedPower(), TransitConstants.feedPower, "RB does not interrupt automatic RT feed");
        test.gamepad1.right_bumper = false;
        test.gamepad1.right_trigger = 0.0f;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "RT release closes gate");
        check(!intake.shootRequested, "RT release clears intake shooting request");
        near(transit.getRequestedPower(), lt ? TransitConstants.forwardPower : 0.0,
                "RT release returns transit to LT power or stop");
        test.gamepad1.left_bumper = true;
        test.loop();
        check(intake.jamRequested, "LB requests intake reverse without RT");
        near(transit.getRequestedPower(), 0.4, "LB runs transit forward without RT");
        test.gamepad1.left_bumper = false;
        test.loop();
        near(transit.getRequestedPower(), lt ? TransitConstants.forwardPower : 0.0,
                "LB release restores LT transit power or stop");
        test.gamepad1.right_trigger = 1.0f;
        test.loop();
        near(position[0], TransitConstants.limitClosed, "RT restart waits for shooter readiness");
        check(transit.getFiringState() == Transit.FiringState.WAITING, "RT restart requests feed without A");
        test.gamepad1.left_bumper = true;
        test.loop();
        check(intake.jamRequested, "LB requests intake reverse with RT");
        near(transit.getRequestedPower(), 0.4, "LB runs transit forward with RT");
        test.gamepad1.b = true;
        test.gamepad1.dpad_right = true;
        test.loop();
        near(shooter.getPitchPosition(), 0.51, "B blocks pitch adjustment");
        check(!intake.shootRequested, "B clears intake shooting request");
        near(transit.getRequestedPower(), 0.0, "B stops transit while LB is held");
        check(!intake.jamRequested, "B clears the LB intake reverse request");
        test.gamepad1.b = false;
        test.gamepad1.left_bumper = false;
        test.gamepad1.dpad_right = false;
        test.loop();
        near(transit.getRequestedPower(), 0.0, "B remains latched while RT stays held");
        check(!shooter.hasTargetVelocity(), "latched stop blocks shooter restart");
        test.gamepad1.right_trigger = 0.0f;
        test.loop();
        test.gamepad1.right_trigger = 1.0f;
        test.loop();
        check(transit.getFiringState() == Transit.FiringState.WAITING, "RT restarts feed request after releasing stop latch");
        set(shooter, Shooter.class, "withinToleranceSinceNanos", System.nanoTime() - 100_000_000L);
        set(shooter, Shooter.class, "pitchChangedAtNanos", System.nanoTime() - 1_000_000_000L);
        test.loop();
        set(transit, Transit.class, "feedReverseTimeSeconds", 0.0);
        test.loop();
        near(transit.getRequestedPower(), TransitConstants.feedPower, "RT resumes feeding after stop recovery");
        test.gamepad1.b = true;
        test.loop();
        near(transit.getRequestedPower(), 0.0, "B cancels an active RT feed");
        near(position[0], TransitConstants.limitClosed, "B closes the active feed gate");
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
