// Paused at user request on 2026-10-13. Original source retained below.
// package org.firstinspires.ftc.teamcode.tests;
//
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.teamcode.pedro.PedroTuneDrive;
// import org.firstinspires.ftc.teamcode.pedro.PedroDiagnostics;
// import org.firstinspires.ftc.teamcode.subsystems.drive.AimTuneConstants;
// import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
// import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledTurnPid;
// import org.firstinspires.ftc.teamcode.subsystems.drive.AdaptiveTurnPower;
// import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
// import java.io.BufferedWriter;
// import java.io.File;
// import java.io.FileWriter;
// import java.io.IOException;
// import java.util.Locale;
//
// /** Stationary heading-step test: no camera, intake or shooter required. */
// @TeleOp(name = "Pedro Aim Tune MANUAL", group = "Test")
// public class AutoAimPidTuneTest extends OpMode {
//     private static final double[] STEPS = {15, -15, 30, -30};
//     private static final double[] P_SCALES = {.8, 1, 1.2};
//     private static final double[] D_SCALES = {.75, 2};
//     private static final double TIMEOUT = 6, DWELL = .5;
//     private final ScheduledTurnPid pid = new ScheduledTurnPid();
//     private final AdaptiveTurnPower adaptive = new AdaptiveTurnPower();
//     private PedroTuneDrive drive;
//     private BufferedWriter log;
//     private File resultFile;
//     private ScheduledTurnPid.Config config;
//     private ScheduledTurnPid.Config bestConfig;
//     private boolean running, previousA, previousX, signConfirmed;
//     private int candidate, step, samples, crossings, passes;
//     private double target, initialError, previousError, squareSum, overshoot;
//     private double startTime, previousTime, insideSince = -1, settledAt = -1, lastLog;
//     private double startX, startY, scoreSum, bestScore = Double.POSITIVE_INFINITY;
//     private int bestCandidate = -1;
//     private double bestNearP, bestFarP, bestNearD, bestFarD;
//     private double bestCwStatic, bestCcwStatic, referenceCw, referenceCcw;
//     private boolean verifying, candidateChanged;
//     private int configCandidate = -1, adaptiveBoosts, completedTrials;
//     private String status = "Check turn direction: hold RT + D-pad left/right; X confirms";
//     private String path = "No log yet";
//     private boolean autoFinished;
//     private String fault;
//     private final org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence launch =
//             new org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence();
//     protected boolean automaticMode() { return false; }
//     @Override public void start() {
//         if (fault == null && drive != null) {
//             org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence.Config c =
//                     new org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence.Config();
//             c.minimumPower = AimTuneConstants.tuneProbeMinimumPower;
//             c.maximumPower = AimTuneConstants.tuneProbeMaximumPower;
//             c.rampSeconds = AimTuneConstants.tuneProbeRampSeconds;
//             c.timeoutSeconds = AimTuneConstants.tuneProbeTimeoutSeconds;
//             launch.start(getRuntime(), c);
//         }
//     }
//
//     @Override public void init() {
//         try { drive = new PedroTuneDrive(hardwareMap); }
//         catch (RuntimeException e) { latchFault(e); }
//         showFault();
//     }
//     @Override public void init_loop() {
//         if (fault != null) { showFault(); return; }
//         try {
//             drive.periodic();
//             telemetry.addData("Pinpoint", drive.getPinpointStatus());
//             telemetry.addLine("Ready to start heading-only scan; no shooter or camera required");
//             telemetry.update();
//         } catch (RuntimeException e) { latchFault(e); showFault(); }
//     }
//     @Override public void loop() {
//         if (fault != null || drive == null) { showFault(); return; }
//         try { loopOnce(); }
//         catch (RuntimeException e) { latchFault(e); showFault(); }
//     }
//     private void loopOnce() {
//         drive.periodic();
//         double now = getRuntime();
//         double heading = drive.getPose().getHeading(AngleUnit.DEGREES);
//         boolean ready = drive.getPinpointStatus() == GoBildaPinpointDriver.DeviceStatus.READY
//                 && Double.isFinite(heading)
//                 && Double.isFinite(drive.getHeadingSpeedDegreesPerSecond())
//                 && Double.isFinite(drive.getPose().getX(DistanceUnit.INCH))
//                 && Double.isFinite(drive.getPose().getY(DistanceUnit.INCH));
//         boolean held = gamepad1.right_trigger > .5;
//         boolean aEdge = gamepad1.a && !previousA;
//         boolean xEdge = gamepad1.x && !previousX;
//         previousA = gamepad1.a; previousX = gamepad1.x;
//         if (automaticMode()) {
//             automaticLoop(now, heading, ready);
//         } else if (gamepad1.b || (running && (!held || !ready))) {
//             abort("Stopped: B / released RT / Pinpoint not READY");
//         } else if (running) {
//             runTrial(now, heading);
//         } else {
//             drive.stop();
//             if (ready && held && !gamepad1.b) {
//                 if (gamepad1.dpad_left != gamepad1.dpad_right) {
//                     // D-pad LEFT must increase displayed CCW heading, RIGHT must decrease it.
//                     double testPower = AimTuneConstants.tuneManualProbePower;
//                     if (!Double.isFinite(testPower) || testPower <= 0 || testPower > .35) {
//                         abort("Invalid manual probe power; choose 0..0.35"); return;
//                     }
//                     double ccw = gamepad1.dpad_left ? testPower : -testPower;
//                     drive.moveRobot(0, 0, -ccw);
//                     signConfirmed = false;
//                     status = "Release D-pad; X only if LEFT increases heading, RIGHT decreases";
//                 } else if (xEdge) {
//                     signConfirmed = true;
//                     status = "Direction confirmed; hold RT and press A to run 24 trials";
//                 } else if (aEdge && signConfirmed) {
//                     startSession(now, heading);
//                 }
//             }
//         }
//         telemetry.addLine(automaticMode()
//                 ? "AUTO: direction probes -> adaptive scan -> fixed-gain retest -> result. STOP/B cancels."
//                 : "MANUAL: hold RT; D-pad tests sign; X confirms; A starts. B/release RT stops.");
//         telemetry.addData("进度", "%d / 6 参数组；已完成 %d 次试验", Math.min(6, candidate), completedTrials);
//         telemetry.addData("阶段", running ? (verifying ? "固定参数复测（不再加功率）" : "探索：卡住自动补功率") : "等待 / 结束");
//         telemetry.addData("状态", status);
//         telemetry.addData("Pinpoint", drive.getPinpointStatus());
//         if (automaticMode()) telemetry.addData("自检功率 当前 / 峰值", "%.3f / %.3f",
//                 launch.probePower(), launch.peakProbePower());
//         telemetry.addData("Heading / target", "%.2f / %.2f deg", heading, target);
//         telemetry.addData("Candidate / trial", "%d / 6, %d / 4", Math.min(candidate + 1, 6), step + 1);
//         if (config != null) {
//             telemetry.addData("Near P / D", "%.5f / %.5f", config.nearP, config.nearD);
//             telemetry.addData("Far P / D", "%.5f / %.5f", config.farP, config.farD);
//             telemetry.addData("摩擦补偿 CCW / CW", "%.3f / %.3f", config.positiveStaticPower, config.negativeStaticPower);
//             telemetry.addData("本组自动加力次数", adaptiveBoosts);
//         }
//         telemetry.addData("最佳完整参数组", bestCandidate < 0 ? "NONE" : bestCandidate + 1);
//         if (bestCandidate >= 0) {
//             telemetry.addData("近区建议 P / D", "%.5f / %.5f", bestNearP, bestNearD);
//             telemetry.addData("远区建议 P / D", "%.5f / %.5f", bestFarP, bestFarD);
//             telemetry.addData("建议补偿 CW / CCW", "%.3f / %.3f", bestCwStatic, bestCcwStatic);
//         }
//         telemetry.addData("日志位置", path);
//         telemetry.update();
//     }
//
//     private void automaticLoop(double now, double heading, boolean ready) {
//         if (gamepad1.b) { autoFinished = true; abort("Cancelled with B; press Stop then Init to rerun"); return; }
//         if (autoFinished) { drive.stop(); return; }
//         if (running) {
//             if (!ready) { autoFinished = true; abort("Pinpoint lost READY; stopped"); }
//             else { runTrial(now, heading); if (!running) autoFinished = true; }
//             return;
//         }
//         org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence.Action action = launch.update(
//                 now, ready, heading, drive.getPose().getX(DistanceUnit.INCH),
//                 drive.getPose().getY(DistanceUnit.INCH), drive.getHeadingSpeedDegreesPerSecond(),
//                 drive.getTranslationSpeedInchesPerSecond());
//         switch (action) {
//             case PROBE_CCW: status = launch.reason(); drive.moveRobot(0, 0, -launch.probePower()); break;
//             case PROBE_CW: status = launch.reason(); drive.moveRobot(0, 0, launch.probePower()); break;
//             case START_SCAN:
//                 referenceCcw = launch.ccwReferencePower(); referenceCw = launch.cwReferencePower();
//                 drive.stop(); startSession(now, heading); if (!running) autoFinished = true; break;
//             case ABORT: autoFinished = true; abort(launch.reason()); break;
//             default: drive.stop(); status = launch.reason();
//         }
//     }
//
//     private ScheduledTurnPid.Config candidateConfig(int index) {
//         ScheduledTurnPid.Config c = new ScheduledTurnPid.Config();
//         double p = P_SCALES[index / D_SCALES.length], d = D_SCALES[index % D_SCALES.length];
//         c.nearP = AimTuneConstants.autoAimNearKp * p;
//         c.farP = AimTuneConstants.autoAimFarKp * p;
//         c.nearD = AimTuneConstants.autoAimNearKd * d;
//         c.farD = AimTuneConstants.autoAimFarKd * d;
//         // First scan deliberately leaves I at zero, preventing integral windup.
//         c.nearDegrees = AimTuneConstants.autoAimNearErrorDegrees;
//         c.farDegrees = AimTuneConstants.autoAimFarErrorDegrees;
//         c.nearMax = Math.min(.30, AimTuneConstants.tuneNearMaxPower);
//         c.farMax = Math.min(.35, Math.min(AimTuneConstants.tuneFarMaxPower, DriveConstants.autoAimMaxTurnPower));
//         c.positiveStaticPower = Math.min(.25, referenceCcw * .8);
//         c.negativeStaticPower = Math.min(.25, referenceCw * .8);
//         c.stopToleranceDegrees = 1.5;
//         c.slewPerSecond = AimTuneConstants.autoAimSlewPerSecond;
//         return c;
//     }
//
//     private void startSession(double now, double heading) {
//         closeLog(); candidate = step = 0; bestCandidate = -1; bestConfig = null; bestScore = Double.POSITIVE_INFINITY;
//         scoreSum = 0; passes = 0;
//         verifying = candidateChanged = false; configCandidate = -1; completedTrials = adaptiveBoosts = 0;
//         if (!automaticMode()) {
//             referenceCcw = AimTuneConstants.autoAimCcwStaticPower;
//             referenceCw = AimTuneConstants.autoAimCwStaticPower;
//         }
//         try {
//             File dir = new File(AppUtil.FIRST_FOLDER, "pid-tests");
//             if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create log folder");
//             String stamp = Long.toString(System.currentTimeMillis());
//             File csv = new File(dir, "turn-" + stamp + ".csv");
//             resultFile = new File(dir, "turn-" + stamp + "-result.txt");
//             path = csv.getAbsolutePath();
//             log = new BufferedWriter(new FileWriter(csv));
//             log.write("candidate,trial,phase,seconds,target_deg,heading_deg,error_deg,turn_power,near_p,far_p,near_d,far_d,ccw_static,cw_static,angular_speed_deg_s\n");
//             log.write(String.format(Locale.US, "# startup_reference ccw=%.4f cw=%.4f; discovery may adapt, verification freezes all parameters%n", referenceCcw, referenceCw));
//             running = true; beginTrial(now, heading);
//         } catch (IOException e) { abort("Log error: " + e.getMessage()); }
//     }
//
//     private void beginTrial(double now, double heading) {
//         if (configCandidate != candidate) {
//             config = candidateConfig(candidate); configCandidate = candidate;
//             candidateChanged = false; adaptiveBoosts = 0;
//         }
//         if (!config.valid()) { abort("Invalid PID config; check constants"); return; }
//         pid.reset(); adaptive.reset(); target = ScheduledTurnPid.wrapDegrees(heading + STEPS[step]);
//         initialError = previousError = STEPS[step]; samples = crossings = 0;
//         squareSum = overshoot = 0; insideSince = settledAt = -1;
//         startTime = previousTime = now; lastLog = now - .05;
//         startX = drive.getPose().getX(DistanceUnit.INCH);
//         startY = drive.getPose().getY(DistanceUnit.INCH);
//         drive.stop(); status = verifying ? "固定参数复测中" : "探索中；检测卡住后自动增加摩擦补偿";
//     }
//
//     private void runTrial(double now, double heading) {
//         double elapsed = now - startTime, dt = now - previousTime;
//         previousTime = now;
//         if (dt <= 0) { drive.stop(); return; }
//         if (dt > .25) { abort("Control loop stalled >250ms; retry"); return; }
//         double error = ScheduledTurnPid.wrapDegrees(target - heading);
//         double drift = Math.hypot(drive.getPose().getX(DistanceUnit.INCH) - startX,
//                 drive.getPose().getY(DistanceUnit.INCH) - startY);
//         if (Math.abs(error) > Math.abs(initialError) + 10 || drift > 4) {
//             signConfirmed = false; abort("Error grew / translation >4in; verify direction and pods"); return;
//         }
//         double angularSpeed = drive.getHeadingSpeedDegreesPerSecond();
//         if (!verifying && adaptive.update(error, angularSpeed, dt, config)) {
//             candidateChanged = true; adaptiveBoosts++;
//             status = "目标外停住：已自动补功率，继续测试";
//         }
//         double output = pid.calculate(error, dt, config);
//         // Pinpoint CCW heading has opposite sign to Drive.moveRobot turn.
//         drive.moveRobot(0, 0, -output);
//         if (error * initialError < 0) overshoot = Math.max(overshoot, Math.abs(error));
//         if (error * previousError < 0 && Math.abs(error - previousError) > .5) crossings++;
//         previousError = error;
//         samples++; squareSum += error * error;
//         boolean steady = Math.abs(error) <= 1.5 && angularSpeed <= 5;
//         if (steady) {
//             if (insideSince < 0) insideSince = now;
//         } else insideSince = -1;
//         if (insideSince >= 0 && now - insideSince >= DWELL) settledAt = now - startTime;
//         try {
//             if (now - lastLog >= .05) {
//                 log.write(String.format(Locale.US, "%d,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f%n",
//                         candidate + 1, step + 1, verifying ? "verification" : "discovery", elapsed, target, heading, error, -output,
//                         config.nearP, config.farP, config.nearD, config.farD,
//                         config.positiveStaticPower, config.negativeStaticPower, angularSpeed));
//                 lastLog = now;
//             }
//             if (settledAt >= 0 || elapsed >= TIMEOUT) finishTrial(now, heading, error);
//         } catch (IOException e) { abort("Log error: " + e.getMessage()); }
//     }
//
//     private void finishTrial(double now, double heading, double error) throws IOException {
//         drive.stop();
//         boolean passed = settledAt >= 0;
//         if (passed) passes++;
//         double rms = Math.sqrt(squareSum / Math.max(samples, 1));
//         scoreSum += (passed ? settledAt : TIMEOUT + 10) + .2 * overshoot + .1 * rms
//                 + .2 * crossings + .2 * Math.abs(error);
//         log.write(String.format(Locale.US,
//                 "# trial_result candidate=%d trial=%d phase=%s passed=%s settle_s=%.3f overshoot_deg=%.3f rms_deg=%.3f crossings=%d final_error_deg=%.3f%n",
//                 candidate + 1, step + 1, verifying ? "verification" : "discovery", passed, settledAt, overshoot, rms, crossings, error));
//         log.flush();
//         completedTrials++;
//         step++;
//         if (step == STEPS.length) {
//             if (!verifying && candidateChanged) {
//                 // Never recommend a tuple that changed between its four trials.
//                 verifying = true; step = passes = 0; scoreSum = 0;
//                 beginTrial(now, heading); return;
//             }
//             double score = scoreSum / STEPS.length;
//             if (passes == STEPS.length && score < bestScore) {
//                 bestScore = score; bestCandidate = candidate;
//                 bestNearP = config.nearP; bestFarP = config.farP;
//                 bestNearD = config.nearD; bestFarD = config.farD;
//                 bestCcwStatic = config.positiveStaticPower; bestCwStatic = config.negativeStaticPower;
//                 bestConfig = config;
//             }
//             candidate++; step = 0; scoreSum = 0; passes = 0; verifying = false;
//         }
//         if (candidate == P_SCALES.length * D_SCALES.length) {
//             running = false; status = bestCandidate < 0
//                     ? "补偿/功率上限内没有参数组通过；日志已保存，不套用" : "复测通过；建议已保存，不自动套用";
//             saveResult(); closeLog();
//         } else beginTrial(now, heading);
//     }
//
//     private void saveResult() throws IOException {
//         try (BufferedWriter out = new BufferedWriter(new FileWriter(resultFile))) {
//             out.write("Heading-only scan; no camera geometry or shooter calibration.\n");
//             out.write("Limits: near <= .30, far <= .35, adaptive static <= .25; stop within 1.5deg.\n");
//             out.write("Result: " + status + "\n");
//             if (bestCandidate >= 0) out.write(String.format(Locale.US,
//                     "candidate=%d score=%.4f\nnearP=%.6f nearI=0 nearD=%.6f\nfarP=%.6f farI=0 farD=%.6f\ncwStaticPower=%.6f ccwStaticPower=%.6f stopToleranceDegrees=1.5\nnearDegrees=%.4f farDegrees=%.4f nearMax=%.4f farMax=%.4f slewPerSecond=%.4f\n",
//                     bestCandidate + 1, bestScore, bestNearP, bestNearD, bestFarP, bestFarD,
//                     bestCwStatic, bestCcwStatic,
//                     bestConfig.nearDegrees, bestConfig.farDegrees, bestConfig.nearMax, bestConfig.farMax, bestConfig.slewPerSecond));
//             out.write("Retest selected gains both directions with actual camera target before applying.\n");
//         }
//     }
//     private void abort(String reason) {
//         running = false; if (drive != null) drive.stop(); pid.reset(); status = reason;
//         if (resultFile != null) try { saveResult(); } catch (IOException e) { status += "; result save failed"; }
//         closeLog();
//     }
//     private void closeLog() {
//         if (log != null) {
//             try { log.close(); } catch (IOException e) { status = "Log close error: " + e.getMessage(); }
//             log = null;
//         }
//     }
//     private void latchFault(RuntimeException e) {
//         fault = PedroDiagnostics.failure(hardwareMap, e);
//         autoFinished = true; running = false; pid.reset();
//         status = "Stopped: " + fault;
//         if (resultFile != null) try { saveResult(); }
//         catch (IOException saveFailure) { fault += "; result save failed"; }
//         closeLog();
//     }
//     private void showFault() {
//         if (fault == null) return;
//         telemetry.addData("Initialization / runtime fault", fault);
//         telemetry.addLine("Motors stopped. Fix configuration, then Stop -> Init; no automatic restart.");
//         telemetry.update();
//     }
//     @Override public void stop() {
//         if (fault != null) { closeLog(); return; }
//         try {
//             if (running) abort("Interrupted with phone STOP; scan incomplete");
//             else { if (drive != null) drive.stop(); closeLog(); }
//         } catch (RuntimeException e) { latchFault(e); }
//     }
// }
