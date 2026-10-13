// Paused at user request on 2026-10-13. Original source retained below.
// import com.pedropathing.algorithm.Algorithm;
// import com.pedropathing.algorithm.Foresight;
// import com.pedropathing.algorithm.ForesightConfig;
// import com.pedropathing.config.ConfigVar;
// import com.pedropathing.controllers.Controller;
// import com.pedropathing.drivetrain.DrivePowers;
// import com.pedropathing.drivetrain.Drivetrain;
// import com.pedropathing.follower.Follower;
// import com.pedropathing.localization.Localizer;
// import com.pedropathing.localization.MotionState;
// import com.pedropathing.math.Matrix;
// import com.pedropathing.math.Pose;
// import com.pedropathing.math.Vector2D;
// import org.firstinspires.ftc.teamcode.pedro.PedroAlgorithms;
// import java.lang.reflect.Field;
// import java.util.Collections;
// import java.util.Map;
//
// /** Real Pedro 3.0.1 objects; only the physical sensors and motors are simulated. */
// public class PedroRuntimeCheck {
//     static final String[] REQUIRED = {"headingFeedback", "forwardTranslational", "strafeTranslational",
//             "brake", "coast", "linearBrakeCoefficients", "quadraticBrakeCoefficients",
//             "headingBrakeCoefficients", "maxAchievableForwardVelocity", "maxAchievableStrafeVelocity",
//             "naturalForwardDeceleration", "naturalStrafeDeceleration"};
//     static void check(boolean condition, String message) {
//         if (!condition) throw new AssertionError(message);
//     }
//     static ForesightConfig completeExcept(String omitted) {
//         // Synthetic values only for exercising all real ConfigVar validators. Never deployed.
//         return new ForesightConfig(c -> {
//             for (String name : REQUIRED) {
//                 if (name.equals(omitted)) continue;
//                 try {
//                     Field field = ForesightConfig.class.getField(name);
//                     ConfigVar variable = (ConfigVar) field.get(c);
//                     Object value;
//                     if (name.endsWith("Translational") || name.equals("headingFeedback")
//                             || name.equals("brake") || name.equals("coast")) value = Controller.zero;
//                     else if (name.equals("headingBrakeCoefficients")) value = Vector2D.cartesian(1, 1);
//                     else if (name.endsWith("BrakeCoefficients")) value = Matrix.diag(1, 1);
//                     else value = 1.0;
//                     variable.set(value);
//                 } catch (ReflectiveOperationException e) { throw new AssertionError(e); }
//             }
//         });
//     }
//     static class Sensors implements Localizer {
//         MotionState state = MotionState.zero();
//         public void setPose(Pose pose) { state = state.withPose(pose); }
//         public MotionState state() { return state; }
//         public void update() {}
//         public void reset() { state = MotionState.zero(); }
//     }
//     static class Motors implements Drivetrain {
//         DrivePowers last = DrivePowers.zero();
//         public void drive(DrivePowers powers, boolean autonomous) { last = powers; }
//         public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
//         public void stop() { last = DrivePowers.zero(); }
//         public void stop(boolean brake) { stop(); }
//         public Map<String, Object> debug() { return Collections.emptyMap(); }
//         public double interpolateVelocity(double a, double b, double c) { return 0; }
//     }
//     public static void main(String[] args) {
//         boolean reproduced = false;
//         try { new Foresight(new ForesightConfig(c -> {})); }
//         catch (IllegalStateException e) { reproduced = e.getMessage().contains("Config variable has not been set"); }
//         check(reproduced, "must reproduce phone's exception using actual Pedro jar");
//         for (String missing : REQUIRED) {
//             boolean named = false;
//             try { PedroAlgorithms.create(true, completeExcept(missing)); }
//             catch (IllegalArgumentException e) { named = e.getMessage().contains("Foresight." + missing); }
//             check(named, "missing field not diagnosed: " + missing);
//         }
//         boolean rejected = false;
//         try { PedroAlgorithms.create(true, null); }
//         catch (IllegalArgumentException e) { rejected = true; }
//         check(rejected, "missing measured configuration rejected");
//         Algorithm calibrated = PedroAlgorithms.create(true, completeExcept(""));
//         check(calibrated instanceof Foresight, "calibrated configuration constructs real Foresight");
//         calibrated.reset();
//
//         Motors motors = new Motors();
//         Follower follower = new Follower(new Sensors(), motors, PedroAlgorithms.create(false, null));
//         follower.update(.02);
//         check(motors.last.turn() == 0, "idle must stop");
//         follower.manual(.2, -.1, .15); follower.update(.02);
//         check(motors.last.forward() == .2 && motors.last.strafe() == -.1 && motors.last.turn() == .15,
//                 "real follower manual control must work without Foresight dynamics");
//         check(follower.debug() != null, "manual diagnostics must not access missing path configuration");
//         follower.stop(); follower.update(.02);
//         check(motors.last.forward() == 0 && motors.last.turn() == 0, "stop must clear previous manual powers");
//         follower.manual(0, 0, .1); follower.update(.02);
//         follower.hold(new Pose(1, 1, 0));
//         rejected = false;
//         try { follower.update(.02); }
//         catch (IllegalStateException e) { rejected = true; }
//         check(rejected && motors.last.turn() == 0, "unmeasured hold must reject and stop motors");
//         rejected = false;
//         try { follower.algorithm().calculatePath(motors, null, MotionState.zero(), .02); }
//         catch (IllegalStateException e) { rejected = true; }
//         check(rejected, "unmeasured path must reject");
//         System.out.println("PASS: actual Pedro exception reproduced; all 12 required fields audited; manual/idle/stop/debug work; uncalibrated path/hold stop and reject");
//     }
// }
