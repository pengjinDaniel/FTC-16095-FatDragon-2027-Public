package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.vision.TagTxVision;

/** Port of Prototype2026-Public-2 MecanumDrivePinpoint.getAlignTurnPower. */
@Config
public class LegacyTxAimController {
    public static double kP_alignH=.025, kI_alignH=0, kD_alignH=.008;
    public static double kP_near=.012, kP_far=.03;
    public static double alignDeadbandNear=3.0, alignDeadbandFar=.5, alignHysteresis=1.5;
    public static double txFilterAlpha=.3, farDistanceThreshold=94;
    // Old robot used 2 degrees. Keep the feature, but do not assume its calibration fits this robot.
    public static double farOffsetDegrees=0;
    private final PIDController pid = new PIDController(kP_alignH, kI_alignH, kD_alignH);
    private boolean initialized, offsetLocked, aligned;
    private int previousTag = -1;
    private double filteredTx, offset, deadband, turn, effectiveKp;

    public double calculateTurnPower(boolean valid, double tx, double distance, int tagId) {
        if (!valid || !TagTxVision.allowed(tagId) || !Double.isFinite(tx) || !validConfig()) {
            reset(); return 0;
        }
        if (tagId != previousTag) { reset(); previousTag = tagId; }
        if (!initialized) { filteredTx=tx; initialized=true; }
        else filteredTx=txFilterAlpha*tx+(1-txFilterAlpha)*filteredTx;
        if (!offsetLocked) {
            boolean far = distance > farDistanceThreshold;
            offset = far ? (tagId == 39 || tagId == 43 ? -farOffsetDegrees : farOffsetDegrees) : 0;
            deadband = far ? alignDeadbandFar : alignDeadbandNear;
            offsetLocked=true;
        }
        if (!Double.isFinite(distance) || distance <= 0) effectiveKp=kP_alignH;
        else if (distance < 40) effectiveKp=kP_near;
        else if (distance > 100) effectiveKp=kP_far;
        else effectiveKp=kP_near+(kP_far-kP_near)*(distance-40)/60.0;
        pid.setPID(effectiveKp,kI_alignH,kD_alignH);
        double error = Math.abs(filteredTx - getTargetTx());
        if (aligned) {
            if (error > deadband+alignHysteresis) aligned=false;
            else { turn=0; return 0; }
        } else if (error < deadband) { aligned=true; turn=0; return 0; }
        turn = -pid.calculate(filteredTx, getTargetTx());
        if (!Double.isFinite(turn)) { reset(); return 0; }
        return Math.max(-1,Math.min(1,turn));
    }
    private boolean validConfig() {
        for (double n : new double[]{kP_alignH,kI_alignH,kD_alignH,kP_near,kP_far,
                alignDeadbandNear,alignDeadbandFar,alignHysteresis,txFilterAlpha,farDistanceThreshold,farOffsetDegrees})
            if (!Double.isFinite(n)) return false;
        return txFilterAlpha >= 0 && txFilterAlpha <= 1 && alignDeadbandNear >= 0
                && alignDeadbandFar >= 0 && alignHysteresis >= 0 && farDistanceThreshold >= 0;
    }
    public void reset() {
        initialized=false; offsetLocked=false; aligned=false; previousTag=-1;
        filteredTx=0; offset=0; turn=0; effectiveKp=0; pid.reset();
    }
    public boolean isLocked() { return aligned; }
    public double getFilteredTx() { return filteredTx; }
    public double getTargetTx() { return -offset; }
    public double getRawTurnPower() { return turn; }
    public double getEffectiveKp() { return effectiveKp; }
}
