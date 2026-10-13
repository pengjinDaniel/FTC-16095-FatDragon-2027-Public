package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.json.JSONArray;
import org.json.JSONObject;

/** Independent 12527-2 style target-angle source; no field-coordinate steering. */
@Config
public class TagTxVision extends SubsystemBase {
    public static int pipelineIndex = 0;
    private final Limelight3A camera;
    private int appliedPipeline = -1;
    private double lastTimestamp = Double.NaN;
    private long lastFrameNanos;
    private Observation observation;
    private String status = "Waiting for Tag 31 / 35 / 39 / 43";

    public TagTxVision(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        camera.setPollRateHz(VisionConstants.pollRateHz);
        camera.start();
    }
    @Override public void periodic() { update(); }
    public void update() {
        observation = null;
        if (pipelineIndex < 0 || pipelineIndex > 9) { status = "Invalid pipeline"; return; }
        if (appliedPipeline != pipelineIndex) {
            if (!camera.pipelineSwitch(pipelineIndex)) { status = "Pipeline switch failed"; return; }
            appliedPipeline = pipelineIndex;
            lastTimestamp = Double.NaN;
        }
        acceptFrame(camera.getLatestResult(), camera.isConnected(), System.nanoTime());
    }
    // Package visibility enables recorded-frame tests without hardware access.
    void acceptFrame(LLResult result, boolean connected, long now) {
        observation = null;
        status = "No usable Tag 31 / 35 / 39 / 43";
        if (!connected || result == null || !result.isValid()) return;
        if (result.getPipelineIndex() != pipelineIndex) { status = "Waiting for correct pipeline"; return; }
        if (result.getStaleness() < 0 || result.getStaleness() > VisionConstants.maxResultAgeMs) {
            status = "Stale camera result"; return;
        }
        double timestamp = result.getTimestamp();
        if (!Double.isFinite(timestamp) || timestamp <= 0) return;
        if (timestamp != lastTimestamp) { lastTimestamp = timestamp; lastFrameNanos = now; }
        if ((now - lastFrameNanos) / 1_000_000L > VisionConstants.maxResultAgeMs) {
            status = "Frozen camera frame"; return;
        }
        observation = select(result);
        if (observation != null) status = "TX aim: Tag " + observation.id;
    }
    public static boolean allowed(int id) { return id == 31 || id == 35 || id == 39 || id == 43; }

    /** Every frame: closest to the crosshair wins, regardless of area/list order/alliance. */
    public static Observation select(LLResult result) {
        if (result == null || !result.isValid() || result.getFiducialResults() == null) return null;
        try {
            JSONArray rawTags = new JSONObject(result.toString()).optJSONArray("Fiducial");
            if (rawTags == null) return null;
            Observation best = null;
            for (FiducialResult tag : result.getFiducialResults()) {
                if (tag == null || !allowed(tag.getFiducialId())) continue;
                int id = tag.getFiducialId(), count = 0;
                JSONObject raw = null;
                for (int i = 0; i < rawTags.length(); i++) {
                    JSONObject candidate = rawTags.optJSONObject(i);
                    if (candidate != null && candidate.optInt("fID", -1) == id) { raw = candidate; count++; }
                }
                if (count != 1 || raw == null || !Double.isFinite(raw.optDouble("tx", Double.NaN))) continue;
                double tx = tag.getTargetXDegrees(); // SAME selected detection as ID and distance.
                double area = tag.getTargetArea() * 100.0;
                if (!Double.isFinite(tx) || Math.abs(tx) > 90 || !Double.isFinite(area)
                        || area < VisionConstants.minimumTargetArea) continue;
                if (best != null && (Math.abs(tx) > Math.abs(best.tx)
                        || (Math.abs(tx) == Math.abs(best.tx) && id > best.id))) continue;
                double distance = Double.NaN;
                if (complete(raw, "t6r_ts")) {
                    Position p = tag.getRobotPoseTargetSpace().getPosition();
                    distance = Math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z) * 39.3701;
                }
                HiveState hive = HiveState.UNKNOWN;
                // Only the feeding interlock uses 3D; turning needs only this tag's tx.
                if (complete(raw, "t6r_fs")) {
                    VisionMath.HiveSolveResult solved = VisionMath.solveHiveFlatPose(
                            tag.getRobotPoseFieldSpace(), VisionConstants.hivePivotHeightInches,
                            VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees);
                    if (solved.correction != null) {
                        double angle = solved.correction.hiveAngleDegrees;
                        double sideX = id == 31 || id == 43 ? -1.0 : 1.0;
                        hive = Math.abs(angle) <= VisionConstants.highHiveTiltThresholdDegrees
                                ? HiveState.MIDDLE : -Math.sin(Math.toRadians(angle))*sideX > 0
                                ? HiveState.HIGH : HiveState.LOW;
                    }
                }
                best = new Observation(id, tx, distance, hive);
            }
            return best;
        } catch (Exception malformed) { return null; }
    }
    private static boolean complete(JSONObject raw, String key) {
        JSONArray values = raw.optJSONArray(key);
        if (values == null || values.length() < 6) return false;
        for (int i = 0; i < 6; i++) if (!Double.isFinite(values.optDouble(i, Double.NaN))) return false;
        return true;
    }
    public static final class Observation {
        public final int id;
        public final double tx, distanceInches;
        public final HiveState hive;
        Observation(int id, double tx, double distance, HiveState hive) {
            this.id=id; this.tx=tx; this.distanceInches=distance; this.hive=hive;
        }
    }
    public boolean hasValidAimingTarget() { return observation != null; }
    public boolean hasValidShootingTarget() { return getTargetHiveState() == HiveState.HIGH; }
    public int getTagId() { return observation == null ? -1 : observation.id; }
    public double getTx() { return observation == null ? Double.NaN : observation.tx; }
    public double getDistance() { return observation == null ? Double.NaN : observation.distanceInches; }
    public HiveState getTargetHiveState() { return observation == null ? HiveState.UNKNOWN : observation.hive; }
    public String getStatus() { return status; }
    public void stop() { observation = null; camera.stop(); }
}
