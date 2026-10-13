package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.json.*;

/** Uses real SDK JSON parsing so missing pose defaults and aggregate leakage are exercised. */
public class SingleTagLocalizationCheck {
    private static class Frame extends LLResult {
        Frame(JSONObject json) throws JSONException { super(json); }
    }
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static JSONObject tag(int id, double area, double x) throws JSONException {
        return new JSONObject().put("fID", id).put("ta", area)
                .put("t6r_fs", new JSONArray(new double[]{x, .8, .1, 0, 0, 12}));
    }
    private static Frame frame(JSONObject... tags) throws JSONException {
        JSONArray detections = new JSONArray();
        for (JSONObject tag : tags) detections.put(tag);
        return new Frame(new JSONObject().put("v", 1).put("pID", 0).put("ts", 1234)
                .put("Fiducial", detections).put("botpose_tagcount", tags.length)
                // Deliberately incompatible fused pose: single-tag mode must never consume it.
                .put("botpose", new JSONArray(new double[]{9, 9, 9, 30, 30, 170})));
    }
    private static SingleTagLocalization.Selection select(Frame frame, Alliance alliance, CellCluster cell) {
        return SingleTagLocalization.select(frame, alliance, cell, .01);
    }
    public static void main(String[] args) throws Exception {
        int[] expected = {31, 35, 39, 43};
        for (CellCluster cell : CellCluster.values()) {
            int id = expected[cell.getTuningIndex()];
            Alliance alliance = id < 38 ? Alliance.RED : Alliance.BLUE;
            check(SingleTagLocalization.tagIdFor(cell) == id, "fixed inner tag mapping");
            JSONObject anchor = tag(id, .02, 1.25);
            // Larger outer and alternate inner tags cannot win over the designated anchor.
            Frame frame = frame(tag(id - 1, .9, 7), anchor, tag(id + 1, .8, 8), tag(id + 2, .7, 6));
            SingleTagLocalization.Selection chosen = select(frame, alliance, cell);
            check(chosen.tag != null && chosen.tag.getFiducialId() == id, "select only designated ID");
            check(Math.abs(chosen.pose().getPosition().toUnit(DistanceUnit.METER).x - 1.25) < 1e-9,
                    "single fiducial pose used instead of poisoned aggregate");
            check(Math.abs(chosen.pose().getOrientation().getYaw(AngleUnit.DEGREES) - 12) < 1e-9,
                    "heading from the single tag");
            check(select(frame(anchor), alliance, cell).pose() != null, "one visible tag is enough here");
            check(select(frame(tag(id - 1, .9, 2), tag(id + 1, .9, 2)), alliance, cell).pose() == null,
                    "no fallback to another tag in the same CELL");
            check(select(frame(anchor, anchor), alliance, cell).pose() == null, "duplicate anchor rejected");
            check(select(frame(tag(id, .00001, 1)), alliance, cell).pose() == null, "tiny anchor rejected");
            JSONObject missing = tag(id, .02, 1); missing.remove("t6r_fs");
            check(select(frame(missing), alliance, cell).pose() == null,
                    "SDK zero fallback for missing field pose must not initialize localization");
            missing.put("t6r_fs", new JSONArray(new double[]{1, 2}));
            check(select(frame(missing), alliance, cell).pose() == null, "short field pose rejected");
            missing.put("t6r_fs", new JSONArray().put(1).put(2).put(3).put(0).put(0).put("NaN"));
            check(select(frame(missing), alliance, cell).pose() == null, "nonfinite pose rejected");
            check(select(frame(anchor, tag(999, .99, 6)), alliance, cell).pose() != null,
                    "unrelated tag cannot contaminate per-fiducial localization");
        }
        Frame bothBlue = frame(tag(39, .02, 1), tag(43, .03, 2), tag(31, .99, 3));
        check(select(bothBlue, Alliance.BLUE, null).expectedId == 43, "bootstrap only own anchor, largest area");
        check(select(bothBlue, Alliance.BLUE, CellCluster.BLUE_AUDIENCE).expectedId == 39,
                "known side wins over larger opposite-side tag");
        check(select(frame(tag(43, .03, 2)), Alliance.BLUE, CellCluster.BLUE_AUDIENCE).pose() == null,
                "missing preferred-side anchor cannot fall back across sides");
        check(select(frame(tag(31, .99, 3)), Alliance.BLUE, null).pose() == null, "opponent-only frame rejected");
        check(select(frame(tag(43, .02, 2), tag(39, .02, 1)), Alliance.BLUE, null).expectedId == 39,
                "equal-area bootstrap deterministic by ID, independent of detection order");
        check(VisionConstants.minimumBotposeTagCount == 2, "multi-tag default minimum preserved");
        System.out.println("PASS: four fixed inner IDs; real SDK JSON; aggregate poison isolation; one-tag frames; missing/duplicate/short/nonfinite poses; no alternate-ID/side fallback; deterministic bootstrap; multi-tag threshold preserved");
    }
}
