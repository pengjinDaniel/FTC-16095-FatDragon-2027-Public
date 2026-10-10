package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedro.PedroTuneDrive;
import org.firstinspires.ftc.teamcode.pedro.PedroDiagnostics;
import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledAutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.AimTuneConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.AimTargetLatch;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/** Separate opt-in coordinate aim test; does not replace upstream TeleOp or fire a shooter. */
public abstract class PedroCoordinateAimBase extends OpMode {
    private PedroTuneDrive drive;
    private Vision vision;
    private final ScheduledAutoAimController aim = new ScheduledAutoAimController();
    private boolean stopLatched;
    private String fault;
    private double commandedTurn;
    private double controlError = Double.NaN;
    private boolean previousRequested, previousA;
    private final AimTargetLatch target = new AimTargetLatch();
    private BufferedWriter log;
    private double lastLog = -1;
    private String logStatus = "Not started";
    protected abstract Alliance alliance();
    @Override public void init() {
        try {
            drive = new PedroTuneDrive(hardwareMap); vision = new Vision(hardwareMap, alliance());
        } catch (RuntimeException e) { latchFault(e); show(); }
    }
    @Override public void init_loop() {
        if (fault == null) try { updateSensors(!drive.isFieldPoseInitialized()); }
        catch (RuntimeException e) { latchFault(e); }
        show();
    }
    @Override public void start() {
        if (fault != null) return;
        try {
            File dir = new File(AppUtil.FIRST_FOLDER, "pid-tests");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create log folder");
            File csv = new File(dir, "aim-" + System.currentTimeMillis() + ".csv");
            log = new BufferedWriter(new FileWriter(csv)); logStatus = csv.getAbsolutePath();
            log.write("time_s,rt,heading_deg,gyro_deg_s,camera_heading_deg,vision_error_deg,control_error_deg,target_x_in,target_y_in,turn_power,holding,locked,vision_correction,tag_count,frame_age_ms\n");
            log.write("# fixed candidate 4; vision correction only at initialization/RT edge/A edge; D from signed Pinpoint gyro\n");
            log.flush();
        } catch (IOException e) { logStatus = "Log unavailable: " + e.getMessage(); }
    }
    @Override public void loop() {
        if (fault != null) { show(); return; }
        try { loopOnce(); }
        catch (RuntimeException e) { latchFault(e); show(); }
    }
    private void loopOnce() {
        boolean requested = gamepad1.right_trigger > .1;
        boolean requestEdge = requested && !previousRequested;
        boolean aEdge = gamepad1.a && !previousA;
        previousRequested = requested; previousA = gamepad1.a;
        if (gamepad1.b) stopLatched = true;
        else if (!requested) stopLatched = false;
        // Never rewrite the gyro heading continuously while the turn controller is running.
        updateSensors(!drive.isFieldPoseInitialized() || (!stopLatched && (aEdge || requestEdge)));
        boolean poseReady = drive.isFieldPoseInitialized()
                && drive.getPinpointStatus() == com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
        target.update(requested && !stopLatched && poseReady, aEdge,
                vision.hasValidAimingTarget(), vision.getTargetXInches(), vision.getTargetYInches());
        commandedTurn = 0;
        controlError = Double.NaN;
        if (stopLatched) { drive.stop(); aim.reset(); }
        else if (requested) {
            Pose2D p = drive.getPose();
            controlError = target.errorDegrees(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH),
                    p.getHeading(AngleUnit.DEGREES), VisionConstants.shooterYawDegrees);
            double turn = aim.calculateTurnPower(target.isLatched() && poseReady,
                    controlError, drive.getSignedHeadingDegreesPerSecond());
            double limit = AimTuneConstants.autoAimTestMaxTurnPower;
            if (!Double.isFinite(limit) || limit < 0) { drive.stop(); aim.reset(); }
            else {
                commandedTurn = Range.clip(turn,-Math.min(limit,.35),Math.min(limit,.35));
                drive.moveRobot(0,0,commandedTurn);
            }
        } else { aim.reset(); drive.moveRobot(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x); }
        writeLog(requested);
        show();
    }
    private void updateSensors(boolean correct) {
        drive.periodic(); vision.update(); vision.updateFieldTarget(drive,correct);
    }
    private void show() {
        if (fault != null) {
            telemetry.addData("Initialization / runtime fault", fault);
            telemetry.addLine("Motors stopped. Fix configuration, then Stop -> Init.");
            telemetry.update(); return;
        }
        telemetry.addLine("独立Pedro双区间自瞄；RT瞄准，A校正定位，B停；不发射");
        telemetry.addLine("已加载：实测第4组PD及左右摩擦补偿");
        telemetry.addLine("持续瞄准稳定版：RT固定目标，A重新校准");
        telemetry.addData("视觉",vision.getPoseStatus());
        telemetry.addData("Pipeline 实际 / 预期", "%d / %d", vision.getActualPipelineIndex(), vision.getExpectedPipelineIndex());
        telemetry.addData("有效Tag数 / 帧龄", "%d / %d ms", vision.getUsableTagCount(), vision.getPhysicalFrameAgeMs());
        telemetry.addData("目标",vision.getTargetStatus());
        telemetry.addData("控制偏角 / 视觉目标偏角", "%.2f / %.2f", controlError, vision.getYawErrorDegrees());
        telemetry.addData("目标已固定 / 保持", "%s / %s", target.isLatched(), aim.isHolding());
        telemetry.addData("锁定",aim.isLocked());
        telemetry.addData("自瞄转向功率",commandedTurn);
        telemetry.addData("日志",logStatus);
        telemetry.addData("定位",drive.getVisionCorrectionStatus());telemetry.update();
    }
    private void writeLog(boolean requested) {
        if (log == null || getRuntime() - lastLog < .05) return;
        try {
            Pose2D p = drive.getPose();
            double cameraHeading = vision.getCorrectedRobotFieldPose()
                    .map(v -> v.getHeading(AngleUnit.DEGREES)).orElse(Double.NaN);
            log.write(String.format(Locale.US, "%.4f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s,%s,%d,%d%n",
                    getRuntime(), requested, p.getHeading(AngleUnit.DEGREES), drive.getSignedHeadingDegreesPerSecond(),
                    cameraHeading, vision.getYawErrorDegrees(), controlError, target.x(), target.y(), commandedTurn,
                    aim.isHolding(), aim.isLocked(), vision.wasCorrectionApplied(), vision.getUsableTagCount(), vision.getPhysicalFrameAgeMs()));
            lastLog = getRuntime();
            if (!requested) log.flush();
        } catch (IOException e) { logStatus = "Log write failed: " + e.getMessage(); closeLog(); }
    }
    private void closeLog() {
        if (log != null) try { log.close(); }
        catch (IOException e) { logStatus = "Log close failed: " + e.getMessage(); }
        finally { log = null; }
    }
    private void latchFault(RuntimeException e) {
        fault = PedroDiagnostics.failure(hardwareMap, e);
        stopLatched = true; aim.reset();
        target.clear(); closeLog();
        if (vision != null) try { vision.stop(); }
        catch (RuntimeException stopFailure) { fault += "; camera stop failed"; }
    }
    @Override public void stop() {
        try { if(drive!=null)drive.stop(); if(vision!=null)vision.stop(); }
        catch (RuntimeException e) { latchFault(e); }
        finally { target.clear(); closeLog(); }
    }
}
