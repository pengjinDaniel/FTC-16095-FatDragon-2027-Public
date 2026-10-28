package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterDistanceCheck;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import sun.misc.Unsafe;

/** Exercises practice/manual aiming with production shooting and feed state machines. */
public class PracticeBlueCheck {
    private static Unsafe allocator;
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-6, message + ": " + actual);
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
        double forward, strafe, turn;
        boolean fieldCentric;
        TestDrive() { super(null); }
        @Override public void moveFieldCentric(double f, double s, double t) {
            forward = f; strafe = s; turn = t; fieldCentric = true;
        }
        @Override public void moveRobot(double f, double s, double t) {
            forward = f; strafe = s; turn = t; fieldCentric = false;
        }
        @Override public void stop() { forward = strafe = turn = 0.0; }
        @Override public Pose2D getPose() {
            throw new AssertionError("Practice must not invoke position hold");
        }
    }

    private static class TestIntake extends Intake {
        int cleared;
        boolean intakeRequested;
        TestIntake() { super(null); }
        @Override public void updateInputs(boolean requested, boolean jam, boolean shoot) {
            intakeRequested = requested;
            super.updateInputs(requested, jam, shoot);
        }
        @Override public void clearBallCount() { cleared++; super.clearBallCount(); }
        @Override public void suspend() { intakeRequested = false; super.suspend(); }
    }

    private static class TestVision extends Vision {
        boolean valid;
        TestVision() { super(null, Alliance.BLUE); }
        @Override public boolean hasValidAimingTarget() { return valid; }
        @Override public double getTargetHorizontalDistanceInches() { return 400.0 / 2.54; }
        @Override public double getTagCenterHorizontalDistanceInches() { return 200.0 / 2.54; }
        @Override public double getYawErrorDegrees() { return 90.0; }
    }

    private static class TestAim extends AutoAimController {
        int calls;
        boolean locked;
        @Override public double calculateTurnPower(boolean valid, double error) { calls++; return 0.123; }
        @Override public boolean isLocked() { return locked; }
        @Override public void reset() { super.reset(); locked = false; }
    }

    private static class TestTimer extends ElapsedTime {
        double time;
        @Override public double seconds() { return time; }
        @Override public void reset() { time = 0.0; }
    }

    private static void ready(Shooter shooter) throws Exception {
        set(shooter, Shooter.class, "ready", true);
        set(shooter, Shooter.class, "pitchChangedAtNanos", System.nanoTime() - 1_000_000_000L);
    }

    public static void main(String[] args) throws Exception {
        Field field = Unsafe.class.getDeclaredField("theUnsafe");
        field.setAccessible(true);
        allocator = (Unsafe) field.get(null);
        PracticeBlue practice = (PracticeBlue) allocator.allocateInstance(PracticeBlue.class);
        check(practice.getAlliance() == Alliance.BLUE, "practice inherits Blue alliance");
        check(!practice.isAutoAimEnabled(), "practice disables automatic aiming");
        check(((Blue) allocator.allocateInstance(Blue.class)).isAutoAimEnabled(), "Blue retains auto aim");
        check(((Red) allocator.allocateInstance(Red.class)).isAutoAimEnabled(), "Red retains auto aim");
        check(PracticeBlue.class.getAnnotation(TeleOp.class).name().equals("Practice Blue"), "menu name");

        helper("configure");
        Shooter shooter = (Shooter) helper("shooter");
        TestDrive drive = (TestDrive) allocator.allocateInstance(TestDrive.class);
        TestIntake intake = (TestIntake) allocator.allocateInstance(TestIntake.class);
        TestVision vision = (TestVision) allocator.allocateInstance(TestVision.class);
        vision.valid = true;
        TestAim aim = new TestAim();
        Gamepad gamepad = (Gamepad) allocator.allocateInstance(Gamepad.class);
        GamepadEx inputs = new GamepadEx(gamepad);
        Transit transit = (Transit) allocator.allocateInstance(Transit.class);
        double[] power = {0.0};
        DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPower")) { power[0] = (double) values[0]; return null; }
                    throw new AssertionError("Unexpected motor call: " + method);
                });
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPosition")) return null;
                    throw new AssertionError("Unexpected servo call: " + method);
                });
        TestTimer timer = new TestTimer();
        set(transit, Transit.class, "transitMotor", motor);
        set(transit, Transit.class, "limitServo", servo);
        set(transit, Transit.class, "feedReverseTimer", timer);
        set(transit, Transit.class, "transitState", Transit.TransitState.CLOSE);
        set(transit, Transit.class, "firingState", Transit.FiringState.IDLE);
        set(intake, Intake.class, "intakeMotor", Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPower")) return null;
                    throw new AssertionError("Unexpected intake motor call: " + method);
                }));
        set(intake, Intake.class, "colorDistanceSensor", Proxy.newProxyInstance(
                DistanceSensor.class.getClassLoader(), new Class<?>[]{DistanceSensor.class},
                (proxy, method, values) -> {
                    if (method.getName().equals("getDistance")) return Double.NaN;
                    throw new AssertionError("Unexpected distance sensor call: " + method);
                }));
        set(intake, Intake.class, "distanceCm", 10.0);
        TransitConstants.limitPositionsCalibrated = true;
        TransitConstants.feedReverseTime = 0.200;
        TransitConstants.feedReversePower = -0.5;
        near(TransitConstants.intakeReverseForwardPower, 0.4, "LB transit forward power");
        intake.updateInputs(true, false);
        transit.updateIntakeRequest(true, false);
        near(intake.getRequestedPower(), IntakeConstants.activePower, "LT runs intake forward");
        near(power[0], TransitConstants.forwardPower, "LT keeps intake transit power");

        IntakeCommand intakeCommand = new IntakeCommand(intake, transit, inputs);
        set(intake, Intake.class, "ballCount", 2);
        gamepad.left_bumper = true;
        intake.setReverseRequested(true);
        intakeCommand.initialize();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "LB reverses intake without LT");
        near(power[0], 0.4, "LB alone runs transit forward");
        intake.periodic();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "LB survives sensor refresh with invalid distance");
        gamepad.left_trigger = 1.0f;
        intakeCommand.execute();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "LB overrides LT intake");
        near(power[0], 0.4, "LB overrides LT transit power");
        gamepad.left_bumper = false;
        intake.setReverseRequested(false);
        set(intake, Intake.class, "distanceCm", 10.0);
        intakeCommand.execute();
        near(intake.getRequestedPower(), IntakeConstants.activePower, "LB release restores LT intake");
        near(power[0], TransitConstants.forwardPower, "LB release restores LT transit power");
        check(intake.getBallCount() == 2, "LB release preserves ball count");
        gamepad.left_trigger = 0.0f;
        intakeCommand.execute();
        near(intake.getRequestedPower(), 0.0, "LB release stops intake when no other request is held");
        near(power[0], 0.0, "LB release stops transit without LT or RT");
        intake.setReverseRequested(true);
        intake.stop();
        intake.periodic();
        near(intake.getRequestedPower(), 0.0, "stop clears LB reverse request");
        set(intake, Intake.class, "distanceCm", 10.0);
        gamepad.right_bumper = true;
        gamepad.left_bumper = true;
        intake.setReverseRequested(true);
        intakeCommand.execute();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "RB reverses intake during normal driving");
        near(power[0], TransitConstants.reversePower, "RB reverses transit during normal driving");
        gamepad.right_bumper = false;
        gamepad.left_bumper = false;
        intake.setReverseRequested(false);
        intakeCommand.execute();
        check(intake.getBallCount() == 0, "RB release retains unjam ball-count reset");
        near(power[0], 0.0, "RB release stops transit without LT");

        ShootAssistCommand command = new ShootAssistCommand(
                drive, shooter, intake, transit, vision, aim, inputs, practice.isAutoAimEnabled());
        command.initialize();
        gamepad.left_stick_y = -0.5f;
        gamepad.left_stick_x = 0.25f;
        gamepad.right_stick_x = -0.4f;
        gamepad.left_trigger = 1.0f;
        command.execute();
        check(drive.fieldCentric, "practice retains normal field-centric driving during RT");
        near(drive.forward, 0.5, "manual forward input");
        near(drive.strafe, 0.25, "manual strafe input");
        near(drive.turn, -0.4, "manual turn input");
        near(shooter.getTargetVelocity(), 1500.0, "distance-adaptive PID target");
        near(shooter.getPitchPosition(), 0.3, "distance-adaptive pitch");
        check(aim.calls == 0, "practice never calculates automatic turn power");
        check(intake.intakeRequested, "LT intake remains available");
        gamepad.left_trigger = 0.0f;
        set(intake, Intake.class, "ballCount", IntakeConstants.maxBallCount);
        command.execute();
        near(intake.getRequestedPower(), IntakeConstants.feedPower, "RT overrides full-bin reverse without LT");
        intake.periodic();
        near(intake.getRequestedPower(), IntakeConstants.feedPower, "RT stays forward after sensor refresh");
        check(!command.canPrepareToFire(), "unready shooter cannot start feeding");
        gamepad.left_bumper = true;
        intake.setReverseRequested(true);
        command.execute();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "LB reverses intake while RT waits");
        near(power[0], 0.4, "LB runs transit forward while RT waits");
        check(transit.getState() == Transit.TransitState.CLOSE, "LB preserves the waiting gate position");
        gamepad.left_bumper = false;
        intake.setReverseRequested(false);
        command.execute();
        near(power[0], 0.0, "LB release restores waiting transit stop");
        ready(shooter);
        command.execute();
        check(command.canPrepareToFire(), "practice can fire without aim lock");
        check(transit.getFiringState() == Transit.FiringState.REVERSING, "shared reverse sequence begins");
        near(power[0], -0.5, "shared reverse power");
        gamepad.left_bumper = true;
        intake.setReverseRequested(true);
        command.execute();
        near(power[0], 0.4, "LB overrides the RT reverse pulse");
        gamepad.left_bumper = false;
        intake.setReverseRequested(false);
        command.execute();
        near(power[0], -0.5, "LB release restores the RT reverse pulse");
        timer.time = 0.201;
        command.execute();
        check(command.canFire(), "shared sequence reaches feeding");
        near(power[0], TransitConstants.feedPower, "shooting uses separate feed power");

        gamepad.left_bumper = true;
        intake.setReverseRequested(true);
        command.execute();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "LB overrides RT intake forward");
        near(power[0], 0.4, "LB overrides shooting transit power");
        transit.periodic();
        near(power[0], 0.4, "LB transit power survives periodic refresh");
        check(command.canFire(), "LB preserves the shooting feed state");
        intake.periodic();
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "LB stays reversed during RT sensor refresh");
        gamepad.left_bumper = false;
        intake.setReverseRequested(false);
        command.execute();
        near(intake.getRequestedPower(), IntakeConstants.feedPower, "LB release restores RT intake forward");
        near(power[0], TransitConstants.feedPower, "LB release restores RT feed power");

        gamepad.left_stick_y = gamepad.left_stick_x = gamepad.right_stick_x = 0.0f;
        command.execute();
        near(drive.forward, 0.0, "neutral stick has no position-hold movement");
        near(drive.turn, 0.0, "neutral stick has no auto turn despite 90-degree aim error");
        vision.valid = false;
        command.execute();
        near(intake.getRequestedPower(), IntakeConstants.feedPower, "RT intake stays forward while aim is invalid");
        check(!shooter.hasTargetVelocity(), "invalid distance cancels PID target");
        check(!command.canFire(), "invalid distance cancels committed feeding");
        check(transit.getState() == Transit.TransitState.CLOSE, "invalid distance closes gate");
        gamepad.right_stick_x = 0.3f;
        command.execute();
        near(drive.turn, 0.3, "manual steering still works without vision");
        gamepad.right_bumper = true;
        gamepad.left_bumper = true;
        intake.setReverseRequested(true);
        command.execute();
        near(power[0], TransitConstants.reversePower, "RB unjam remains available");
        near(intake.getRequestedPower(), IntakeConstants.jamPower, "RB overrides RT intake forward");
        near(shooter.getLeftPower(), 0.0, "RB stops shooter");
        command.end(true);
        near(intake.getRequestedPower(), 0.0, "command cancellation stops intake");
        gamepad.right_bumper = false;
        gamepad.left_bumper = false;
        intake.setReverseRequested(false);
        intake.updateInputs(false, false);
        intake.periodic();
        near(intake.getRequestedPower(), 0.0, "RT release clears shooting override");
        near(power[0], 0.0, "command cancellation stops transit");
        near(drive.turn, 0.0, "command cancellation stops drive");
        check(transit.getState() == Transit.TransitState.CLOSE, "command cancellation closes gate");
        check(intake.cleared > 0, "completed gate cycle clears ball count");

        vision.valid = true;
        gamepad.right_bumper = false;
        gamepad.left_stick_y = -0.5f;
        ShootAssistCommand normal = new ShootAssistCommand(drive, shooter, intake, transit, vision, aim, inputs);
        normal.initialize();
        normal.execute();
        ready(shooter);
        normal.execute();
        check(drive.fieldCentric, "normal Blue/Red keep headless translation during RT");
        near(drive.turn, 0.123, "normal Blue/Red retain automatic turn output");
        check(aim.calls > 0, "normal constructor still calculates auto aim");
        check(!normal.canPrepareToFire(), "normal Blue/Red still require aim lock");
        aim.locked = true;
        normal.execute();
        check(normal.canPrepareToFire(), "normal Blue/Red can fire after aim lock");
        normal.end(true);
        System.out.println("PASS PracticeBlueCheck: " + checks + " assertions");
    }
}
