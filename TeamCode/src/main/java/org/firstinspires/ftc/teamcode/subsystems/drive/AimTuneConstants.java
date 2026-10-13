// Paused at user request on 2026-10-13. Original source retained below.
// package org.firstinspires.ftc.teamcode.subsystems.drive;
// import com.acmerobotics.dashboard.config.Config;
// @Config
// public final class AimTuneConstants {
//     // Fixed four-step robot test passed: turn-1778145377357, candidate 4.
//     // Near/far refer to yaw error. Camera-based aiming still needs field validation.
//     public static volatile boolean autoAimScheduledPidEnabled = true;
//     public static volatile double autoAimNearKp = 0.012;
//     public static volatile double autoAimNearKi = 0.0;
//     public static volatile double autoAimNearKd = 0.002;
//     public static volatile double autoAimFarKp = 0.025;
//     public static volatile double autoAimFarKi = 0.0;
//     public static volatile double autoAimFarKd = 0.003;
//     public static volatile double autoAimNearErrorDegrees = 5.0;
//     public static volatile double autoAimFarErrorDegrees = 20.0;
//     public static volatile double autoAimNearMaxPower = 0.30;
//     public static volatile double autoAimFarMaxPower = 0.35;
//     public static volatile double autoAimTestMaxTurnPower = 0.35;
//     public static volatile double autoAimSlewPerSecond = 1.5;
//     // Automatic direction check, independent of the candidate PID scan. Snapshot at START.
//     public static volatile double tuneProbeMinimumPower = .10;
//     public static volatile double tuneProbeMaximumPower = .25;
//     public static volatile double tuneProbeRampSeconds = 1.0;
//     public static volatile double tuneProbeTimeoutSeconds = 1.8;
//     // Export uses physical CW/CCW directions; the controller performs the sign conversion.
//     public static volatile double autoAimCwStaticPower = 0.091183;
//     public static volatile double autoAimCcwStaticPower = 0.115504;
//     public static volatile double autoAimStopToleranceDegrees = 1.5;
//     public static volatile double tuneNearMaxPower = .30;
//     public static volatile double tuneFarMaxPower = .35;
//     public static volatile double tuneManualProbePower = .25;
//
//     private AimTuneConstants() {}
// }
