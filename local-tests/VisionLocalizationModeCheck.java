package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.json.*;
import java.lang.reflect.Field;
import sun.misc.Unsafe;

/** Runs production Vision.update on recorded-shape SDK frames, with no robot/network access. */
public class VisionLocalizationModeCheck {
    private static class Frame extends LLResult {
        Frame(JSONObject data) throws JSONException { super(data); }
    }
    private static class Camera extends Limelight3A {
        LLResult frame;
        private Camera() { super(null, null, null); } // Not called: hardware constructor is bypassed below.
        @Override public boolean isConnected() { return true; }
        @Override public LLResult getLatestResult() { return frame; }
    }
    private static void set(Object target, String field, Object value) throws Exception {
        Field f = Vision.class.getDeclaredField(field); f.setAccessible(true); f.set(target, value);
    }
    private static JSONObject tag(int id, double x) throws JSONException {
        return new JSONObject().put("fID", id).put("ta", .03)
                .put("t6r_fs", new JSONArray(new double[]{x, .8, 0, 0, 0, 12}));
    }
    private static Frame frame(boolean anchor, int fusedCount) throws JSONException {
        JSONArray tags = new JSONArray().put(tag(38, 5)).put(tag(40, 6)).put(tag(41, 7));
        if (anchor) tags.put(tag(39, 1.25));
        return new Frame(new JSONObject().put("Fiducial", tags).put("v", 1).put("ts", 1234)
                .put("pID", 0).put("botpose_tagcount", fusedCount)
                .put("botpose", new JSONArray(new double[]{-1, .8, 0, 0, 0, -20})));
    }
    private static void check(boolean value, String message) {
        if (!value) throw new AssertionError(message);
    }
    public static void main(String[] args) throws Exception {
        Field f = Unsafe.class.getDeclaredField("theUnsafe"); f.setAccessible(true);
        Unsafe unsafe = (Unsafe) f.get(null);
        for (Vision.LocalizationMode mode : Vision.LocalizationMode.values()) {
            Camera camera = (Camera) unsafe.allocateInstance(Camera.class);
            Vision vision = (Vision) unsafe.allocateInstance(Vision.class);
            set(vision, "limelight", camera);
            set(vision, "alliance", Alliance.BLUE);
            set(vision, "localizationMode", mode);
            set(vision, "pipelineIndex", 0);
            set(vision, "aimCellSelection", new CellSelectionLatch());
            set(vision, "preferredSingleTagCell", CellCluster.BLUE_AUDIENCE);
            camera.frame = frame(true, 4);
            vision.update();
            check(vision.getCorrectedRobotFieldPose().isPresent(), "both modes accept complete input: " + vision.getPoseStatus());
            double x = vision.getCorrectedRobotFieldPose().get().getX(DistanceUnit.METER);
            check(Math.abs(x - (mode == Vision.LocalizationMode.SINGLE_TAG ? 1.25 : -1)) < 1e-6,
                    "single uses only anchor; multi continues to use aggregate");
            check(vision.getLocalizationTagCount() == (mode == Vision.LocalizationMode.SINGLE_TAG ? 1 : 4),
                    "actual localization source count");
            camera.frame = frame(false, 3);
            vision.update();
            check(vision.getCorrectedRobotFieldPose().isPresent() == (mode == Vision.LocalizationMode.MULTI_TAG),
                    "missing anchor must not fall back to aggregate or other tags");
            if (mode == Vision.LocalizationMode.SINGLE_TAG) {
                check(!vision.getRawRobotFieldPose().isPresent() && vision.getCalibrationTagId() == -1,
                        "no stale single-tag source after loss");
            }
            camera.frame = frame(true, 0);
            vision.update();
            check(vision.getCorrectedRobotFieldPose().isPresent() == (mode == Vision.LocalizationMode.SINGLE_TAG),
                    "single source independent of MT1 count; original multi minimum remains enforced");
        }
        System.out.println("PASS: production Vision.update in both modes; distinct source poses; anchor loss clears source; no aggregate fallback; original MT1 minimum retained");
    }
}
