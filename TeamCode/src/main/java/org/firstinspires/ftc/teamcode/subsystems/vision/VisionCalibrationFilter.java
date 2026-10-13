package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Three fresh frames must agree on the correction after accounting for odometry motion. */
public final class VisionCalibrationFilter {
    public static final long SUCCESS_INTERVAL_NANOS = 150_000_000L;
    private static final int REQUIRED_SAMPLES = 3;
    private final double[][] offsets = new double[REQUIRED_SAMPLES][3];
    private int count;
    private boolean trusted;
    private double lastTimestamp = Double.NaN;
    private long lastFrameNanos = Long.MIN_VALUE;
    private long lastAppliedNanos = Long.MIN_VALUE;
    private String status = "等待可靠视觉位置";

    public boolean update(Pose2D vision, Pose2D odometry, double timestamp, long now,
                          boolean allowCorrection) {
        trusted = false;
        if (!finite(vision) || !finite(odometry) || !Double.isFinite(timestamp) || timestamp <= 0) {
            clear("视觉位置无效，持续尝试"); return false;
        }
        double xyGate = VisionConstants.calibrationSampleSpreadInches;
        double angleGate = VisionConstants.calibrationSampleSpreadDegrees;
        if (!Double.isFinite(xyGate) || xyGate <= 0 || !Double.isFinite(angleGate) || angleGate <= 0) {
            clear("跳变阈值无效"); return false;
        }
        if (lastFrameNanos != Long.MIN_VALUE && now - lastFrameNanos > 200_000_000L) {
            clear("采集间断，重新确认位置");
        }
        double[] offset = {vision.getX(DistanceUnit.INCH)-odometry.getX(DistanceUnit.INCH),
                vision.getY(DistanceUnit.INCH)-odometry.getY(DistanceUnit.INCH),
                wrap(vision.getHeading(AngleUnit.DEGREES)-odometry.getHeading(AngleUnit.DEGREES))};
        for (int i=0; i<count; i++) {
            if (Math.hypot(offset[0]-offsets[i][0],offset[1]-offsets[i][1]) > xyGate
                    || Math.abs(wrap(offset[2]-offsets[i][2])) > angleGate) {
                clear("视觉位置跳变，拒绝校准；持续尝试");
                // Discard this outlier. It cannot immediately become a trusted new baseline.
                lastTimestamp=timestamp; lastFrameNanos=now;
                return false;
            }
        }
        if (timestamp == lastTimestamp) {
            trusted = count == REQUIRED_SAMPLES;
            return false; // Re-reading one physical camera frame is not a new attempt/sample.
        }
        lastTimestamp=timestamp; lastFrameNanos=now;
        if (count == REQUIRED_SAMPLES) {
            for (int i=1; i<count; i++) System.arraycopy(offsets[i],0,offsets[i-1],0,3);
            count--;
        }
        System.arraycopy(offset,0,offsets[count++],0,3);
        trusted = count == REQUIRED_SAMPLES;
        if (!allowCorrection) {
            status="R2按住：暂停位置校准"; return false;
        }
        if (!trusted) {
            status="确认连续稳定位置 " + count + "/" + REQUIRED_SAMPLES; return false;
        }
        if (lastAppliedNanos != Long.MIN_VALUE && now-lastAppliedNanos < SUCCESS_INTERVAL_NANOS) {
            status="上次校准成功，等待150ms间隔"; return false;
        }
        status="位置稳定，正在尝试校准";
        return true;
    }
    public void onApplied(Pose2D before, Pose2D after, long now) {
        double dx=after.getX(DistanceUnit.INCH)-before.getX(DistanceUnit.INCH);
        double dy=after.getY(DistanceUnit.INCH)-before.getY(DistanceUnit.INCH);
        double dh=wrap(after.getHeading(AngleUnit.DEGREES)-before.getHeading(AngleUnit.DEGREES));
        // The vision write itself shifts the odometry reference; do not call that a new jump.
        for (int i=0; i<count; i++) {
            offsets[i][0]-=dx; offsets[i][1]-=dy; offsets[i][2]=wrap(offsets[i][2]-dh);
        }
        lastAppliedNanos=now;
        status="校准成功；150ms后允许下一次";
    }
    public void onWriteRejected() { status="校准未通过，持续尝试（不启动150ms计时）"; }
    public void onWriteRejected(double positionDifferenceInches, double headingDifferenceDegrees) {
        status=String.format(java.util.Locale.US,"校准未通过：与旧定位差 %.0fcm / %.1f°；继续尝试",
                positionDifferenceInches*2.54,headingDifferenceDegrees);
    }
    public void clear(String reason) { count=0; trusted=false; status=reason; }
    public boolean isTrusted() { return trusted; }
    public int getSampleCount() { return count; }
    public String getStatus() { return status; }
    private static double wrap(double value) { return Math.IEEEremainder(value,360.0); }
    private static boolean finite(Pose2D pose) {
        return pose != null && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.DEGREES));
    }
}
