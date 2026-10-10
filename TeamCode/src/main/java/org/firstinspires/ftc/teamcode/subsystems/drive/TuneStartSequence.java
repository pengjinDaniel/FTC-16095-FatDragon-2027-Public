package org.firstinspires.ftc.teamcode.subsystems.drive;

/** Pure startup state machine, exercised on the host before hardware use. */
public final class TuneStartSequence {
    public enum Action { WAIT, PROBE_CCW, PROBE_CW, START_SCAN, ABORT }
    public static final class Config {
        public double minimumPower = .10, maximumPower = .25, rampSeconds = 1.0;
        public double timeoutSeconds = 1.8, minimumTurnDegrees = .75, settleSeconds = .3;
        public boolean valid() {
            double[] values = {minimumPower, maximumPower, rampSeconds, timeoutSeconds,
                    minimumTurnDegrees, settleSeconds};
            for (double value : values) if (!Double.isFinite(value) || value <= 0) return false;
            return minimumPower <= maximumPower && maximumPower <= .35
                    && rampSeconds <= timeoutSeconds && timeoutSeconds <= 3
                    && minimumTurnDegrees <= 3 && settleSeconds <= 2;
        }
    }
    private double started, previous, readySince = -1, probeAt = -1, heading0, x0, y0;
    private double settledSince = -1, detectedAt = -1, probePower, peakProbePower;
    private double ccwReferencePower, cwReferencePower;
    private int direction = 1;
    private Config config = new Config();
    private boolean terminal;
    private String reason = "等待静止，Start 后三秒开始";
    public void start(double now) { start(now, new Config()); }
    public void start(double now, Config c) {
        started = previous = now; readySince = probeAt = settledSince = detectedAt = -1;
        probePower = peakProbePower = 0; terminal = false;
        ccwReferencePower = cwReferencePower = 0; direction = 1;
        config = c;
    }
    public double probePower() { return probePower; }
    public double peakProbePower() { return Math.max(peakProbePower, Math.max(ccwReferencePower, cwReferencePower)); }
    public double ccwReferencePower() { return ccwReferencePower; }
    public double cwReferencePower() { return cwReferencePower; }
    public String reason() { return reason; }
    private Action fail(String why) { reason = why; terminal = true; probePower = 0; return Action.ABORT; }
    public Action update(double now, boolean ready, double heading, double x, double y,
            double angularSpeed, double linearSpeed) {
        if (terminal) return Action.WAIT;
        if (config == null || !config.valid()) return fail("自检功率/时间配置无效，已停止");
        if (!Double.isFinite(now) || now < previous || now - previous > .25)
            return fail("控制循环停顿，已停止");
        previous = now;
        if (!ready || !Double.isFinite(heading) || !Double.isFinite(x) || !Double.isFinite(y)
                || !Double.isFinite(angularSpeed) || !Double.isFinite(linearSpeed)) {
            readySince = -1;
            if (probeAt >= 0 || now - started > 15) return fail("Pinpoint 未就绪，已停止");
            reason = "等待 Pinpoint READY"; return Action.WAIT;
        }
        if (probeAt >= 0) {
            double movement = direction * ScheduledTurnPid.wrapDegrees(heading - heading0);
            if (movement < -1 || Math.abs(movement) > 12 || Math.hypot(x-x0,y-y0) > 4)
                return fail("方向或定位检查失败，请核对电机和定位轮");
            if (detectedAt < 0 && movement >= config.minimumTurnDegrees) {
                detectedAt = now; probePower = 0;
                if (direction > 0) ccwReferencePower = peakProbePower;
                else cwReferencePower = peakProbePower;
            }
            if (detectedAt >= 0) {
                probePower = 0; reason = "已测到转角，停转等待稳定";
                if (Math.abs(angularSpeed) > 2 || Math.abs(linearSpeed) > 1) settledSince = -1;
                else if (settledSince < 0) settledSince = now;
                if (settledSince >= 0 && now - settledSince >= config.settleSeconds) {
                    if (direction > 0) {
                        direction = -1; probeAt = now; heading0 = heading; x0 = x; y0 = y;
                        detectedAt = settledSince = -1;
                        probePower = peakProbePower = config.minimumPower;
                        reason = "正在测顺时针起转功率";
                        return Action.PROBE_CW;
                    }
                    terminal = true; return Action.START_SCAN;
                }
                if (now - detectedAt > 3) return fail("自检停转后未稳定，已停止");
                return Action.WAIT;
            }
            if (now - probeAt >= config.timeoutSeconds) {
                return fail("达到自检功率上限仍未起转；检查电机/定位或调整自检参数");
            }
            probePower = Math.min(config.maximumPower, config.minimumPower
                    + (config.maximumPower - config.minimumPower) * (now - probeAt) / config.rampSeconds);
            peakProbePower = Math.max(peakProbePower, probePower);
            reason = direction > 0 ? "正在测逆时针起转功率" : "正在测顺时针起转功率";
            return direction > 0 ? Action.PROBE_CCW : Action.PROBE_CW;
        }
        if (now - started > 15) return fail("机器人未保持静止，已停止");
        if (Math.abs(angularSpeed) > 2 || Math.abs(linearSpeed) > 1) readySince = -1;
        else if (readySince < 0) readySince = now;
        reason = "等待静止，Start 后三秒开始";
        if (now - started >= 3 && readySince >= 0 && now - readySince >= .5) {
            probeAt = now; heading0 = heading; x0 = x; y0 = y;
            probePower = peakProbePower = config.minimumPower;
            return Action.PROBE_CCW;
        }
        return Action.WAIT;
    }
}
