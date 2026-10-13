package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.List;

/** One fixed inner tag per CELL. Never uses the aggregate botpose as a fallback. */
final class SingleTagLocalization {
    private SingleTagLocalization() { }

    static int tagIdFor(CellCluster cell) {
        if (cell == null) return -1;
        // SDK 12 AprilTagGameDatabase: lateral offsets -6.50, -2.75, +2.75, +6.50 in.
        // The two inner tags are equally close; consistently choose the lower ID.
        switch (cell) {
            case RED_OPPOSITE_AUDIENCE: return 31;
            case RED_AUDIENCE: return 35;
            case BLUE_AUDIENCE: return 39;
            case BLUE_OPPOSITE_AUDIENCE: return 43;
            default: return -1;
        }
    }

    static Selection select(LLResult result, Alliance alliance, CellCluster preferred,
                            double minimumAreaPercent) {
        int expectedId = tagIdFor(preferred);
        if (alliance == null || (preferred != null && !preferred.isForAlliance(alliance))
                || !Double.isFinite(minimumAreaPercent) || minimumAreaPercent < 0.0) {
            return new Selection(expectedId, null, "Invalid single-tag configuration");
        }
        List<LLResultTypes.FiducialResult> tags = result == null ? null : result.getFiducialResults();
        LLResultTypes.FiducialResult chosen = null;
        double bestArea = -1.0;
        if (tags != null) for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null) continue;
            int id = tag.getFiducialId();
            CellCluster cell = CellCluster.forTagId(id).orElse(null);
            if (cell == null || !cell.isForAlliance(alliance) || id != tagIdFor(cell)
                    || (expectedId >= 0 && id != expectedId)) continue;
            double area = tag.getTargetArea() * 100.0;
            if (!Double.isFinite(area) || area < minimumAreaPercent) continue;
            // Before field pose exists, acquire one of this alliance's two anchors.
            // Once the side is known, the caller supplies that CELL; no cross-side fallback.
            if (chosen == null || area > bestArea
                    || (area == bestArea && id < chosen.getFiducialId())) {
                chosen = tag;
                bestArea = area;
            }
        }
        if (chosen == null) return new Selection(expectedId, null, expectedId >= 0
                ? "Waiting for usable Tag " + expectedId
                : "Waiting for inner Tag " + (alliance == Alliance.BLUE ? "39 or 43" : "31 or 35"));
        int id = chosen.getFiducialId();
        int count = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag != null && tag.getFiducialId() == id) count++;
        }
        if (count != 1) return new Selection(id, null, "Duplicate calibration Tag " + id);
        // SDK 12 substitutes six zeros when t6r_fs is missing/short. Check the actual
        // JSON first so an unavailable single-tag pose cannot initialize us at (0,0,0).
        if (!hasCompleteFieldPose(result, id)) {
            return new Selection(id, null, "Tag " + id + " has no complete field pose; check Limelight map/3D");
        }
        return new Selection(id, chosen, "Single Tag " + id + " field pose");
    }

    private static boolean hasCompleteFieldPose(LLResult result, int id) {
        try {
            JSONArray tags = new JSONObject(result.toString()).optJSONArray("Fiducial");
            if (tags == null) return false;
            for (int i = 0; i < tags.length(); i++) {
                JSONObject tag = tags.optJSONObject(i);
                if (tag == null || tag.optInt("fID", -1) != id) continue;
                JSONArray pose = tag.optJSONArray("t6r_fs");
                if (pose == null || pose.length() < 6) return false;
                for (int j = 0; j < 6; j++) {
                    if (!Double.isFinite(pose.optDouble(j, Double.NaN))) return false;
                }
                return true;
            }
        } catch (JSONException e) {
            return false;
        }
        return false;
    }

    static final class Selection {
        final int expectedId;
        final LLResultTypes.FiducialResult tag;
        final String status;
        Selection(int expectedId, LLResultTypes.FiducialResult tag, String status) {
            this.expectedId = expectedId;
            this.tag = tag;
            this.status = status;
        }
        Pose3D pose() { return tag == null ? null : tag.getRobotPoseFieldSpace(); }
    }
}
