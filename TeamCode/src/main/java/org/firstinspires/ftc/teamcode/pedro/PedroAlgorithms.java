package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;
import java.lang.reflect.Field;
import java.util.Collections;
import java.util.Map;

/** Manual follower control needs no unmeasured Foresight path dynamics. */
public final class PedroAlgorithms {
    private PedroAlgorithms() {}

    public static Algorithm create(boolean calibrated, ForesightConfig config) {
        if (!calibrated) return new ManualOnly();
        if (config == null) {
            throw new IllegalArgumentException("Pedro paths: measured ForesightConfig is missing");
        }
        validateConfig("Foresight", config);
        return new Foresight(config);
    }

    /** Check every ConfigVar before hardware is touched; name a missing field explicitly. */
    public static void validateConfig(String label, Object config) {
        for (Field field : config.getClass().getFields()) {
            if (!ConfigVar.class.isAssignableFrom(field.getType())) continue;
            try {
                ConfigVar<?> variable = (ConfigVar<?>) field.get(config);
                if (variable == null) throw new IllegalStateException("null ConfigVar");
                variable.get();
            } catch (ReflectiveOperationException | RuntimeException e) {
                throw new IllegalArgumentException(label + "." + field.getName()
                        + " is not configured correctly", e);
            }
        }
    }

    private static final class ManualOnly implements Algorithm {
        private DrivePowers reject(Drivetrain drivetrain) {
            drivetrain.stop();
            throw new IllegalStateException("Pedro paths/hold require measured Foresight calibration; use manual control");
        }
        @Override public DrivePowers calculatePath(Drivetrain d, PathTracker p, MotionState s, double dt) {
            return reject(d);
        }
        @Override public DrivePowers calculateHold(Drivetrain d, Pose p, MotionState s, boolean end, double dt) {
            return reject(d);
        }
        @Override public double completion() { return 0; }
        @Override public Pose closestPose() { return new Pose(0, 0, 0); }
        @Override public Vector2D closestTangent() { return Vector2D.cartesian(0, 0); }
        @Override public Vector2D closestNormal() { return Vector2D.cartesian(0, 0); }
        @Override public double curvature() { return 0; }
        @Override public double remainingDistance() { return 0; }
        @Override public boolean atParametricEnd() { return false; }
        @Override public void reset() {}
        @Override public boolean isBusy() { return false; }
        @Override public Map<String, Object> debug() {
            return Collections.singletonMap("Pedro algorithm", "manual heading tuning; paths not calibrated");
        }
    }
}
