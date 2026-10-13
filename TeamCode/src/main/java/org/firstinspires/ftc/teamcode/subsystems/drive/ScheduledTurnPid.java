// Paused at user request on 2026-10-13. Original source retained below.
// package org.firstinspires.ftc.teamcode.subsystems.drive;
//
// /** Angle-error gain scheduling. Units: degrees, seconds, motor power. */
// public final class ScheduledTurnPid {
//     public static final class Config {
//         public double nearP = .012, nearI = 0, nearD = .001;
//         public double farP = .025, farI = 0, farD = .0015;
//         public double nearDegrees = 5, farDegrees = 20;
//         public double nearMax = .18, farMax = .4;
//         public double integralZone = 5, integralLimit = .08;
//         public double derivativeFilterSeconds = .08, slewPerSecond = 1.5;
//         public double positiveStaticPower, negativeStaticPower, stopToleranceDegrees;
//         public boolean valid() {
//             double[] values = {nearP, nearI, nearD, farP, farI, farD,
//                     nearDegrees, farDegrees, nearMax, farMax, integralZone,
//                     integralLimit, derivativeFilterSeconds, slewPerSecond,
//                     positiveStaticPower, negativeStaticPower, stopToleranceDegrees};
//             for (double value : values) if (!Double.isFinite(value) || value < 0) return false;
//             return farDegrees > nearDegrees && nearMax <= farMax && farMax <= 1
//                     && slewPerSecond > 0 && positiveStaticPower <= nearMax
//                     && negativeStaticPower <= nearMax && stopToleranceDegrees <= nearDegrees;
//         }
//     }
//     private double integral, derivative, previousError, output;
//     private boolean initialized;
//     public static double wrapDegrees(double angle) { return Math.IEEEremainder(angle, 360); }
//     public void reset() { integral = derivative = previousError = output = 0; initialized = false; }
//     public double calculate(double errorDegrees, double dt, Config c) {
//         return calculateInternal(errorDegrees, dt, c, false, 0);
//     }
//     public double calculateWithDerivative(double errorDegrees, double dt, Config c, double errorRateDegreesPerSecond) {
//         return calculateInternal(errorDegrees, dt, c, true, errorRateDegreesPerSecond);
//     }
//     private double calculateInternal(double errorDegrees, double dt, Config c, boolean useRate, double errorRate) {
//         if (!Double.isFinite(errorDegrees) || !Double.isFinite(dt)
//                 || dt <= 0 || dt > .25 || c == null || !c.valid() || (useRate && !Double.isFinite(errorRate))) {
//             reset(); return 0;
//         }
//         double error = wrapDegrees(errorDegrees);
//         if (Math.abs(error) <= c.stopToleranceDegrees) { reset(); return 0; }
//         double mix = clip((Math.abs(error) - c.nearDegrees) / (c.farDegrees - c.nearDegrees), 0, 1);
//         mix = mix * mix * (3 - 2 * mix);
//         double p = blend(c.nearP, c.farP, mix), i = blend(c.nearI, c.farI, mix);
//         double d = blend(c.nearD, c.farD, mix), max = blend(c.nearMax, c.farMax, mix);
//         double rawDerivative = useRate ? errorRate : initialized ? wrapDegrees(error - previousError) / dt : 0;
//         derivative += dt / (c.derivativeFilterSeconds + dt) * (rawDerivative - derivative);
//         if (Math.abs(error) > c.integralZone || i == 0) integral = 0;
//         else {
//             double trial = clip(integral + error * dt, -c.integralLimit / i, c.integralLimit / i);
//             double friction = error > 0 ? c.positiveStaticPower : -c.negativeStaticPower;
//             double trialOutput = p * error + i * trial + d * derivative + friction;
//             if (Math.abs(trialOutput) <= max || error * trialOutput < 0) integral = trial;
//         }
//         double friction = error > 0 ? c.positiveStaticPower : -c.negativeStaticPower;
//         double requested = clip(p * error + i * integral + d * derivative + friction, -max, max);
//         output = clip(requested, output - c.slewPerSecond * dt, output + c.slewPerSecond * dt);
//         // A falling scheduled limit always takes precedence over the slew limiter.
//         output = clip(output, -max, max);
//         previousError = error; initialized = true;
//         return output;
//     }
//     private static double blend(double a, double b, double t) { return a + (b - a) * t; }
//     private static double clip(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
// }
