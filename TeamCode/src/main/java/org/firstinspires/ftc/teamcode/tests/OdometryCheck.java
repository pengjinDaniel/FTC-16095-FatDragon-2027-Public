package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "OD Check", group = "Test")
public class OdometryCheck extends LinearOpMode {
    private static final String LOG_TAG = "ODCheck";
    private static final double FOUR_BAR_TICKS_PER_CM = 198.9436789;
    private static final double SETTLE_SECONDS = 0.35;
    private static final double MAX_TRAVEL_CM = 120.0;
    private static final double MAX_STAGE_ROTATION_DEGREES = 150.0;

    enum Step {
        STILL(0, 0, 0, 5.0),
        FORWARD(0.24, 0, 0, 0.85),
        BACKWARD(-0.24, 0, 0, 0.85),
        LEFT(0, -0.26, 0, 0.85),
        RIGHT(0, 0.26, 0, 0.85),
        CCW(0, 0, -0.26, 1.0),
        FORWARD_AT_ANGLE(0.24, 0, 0, 0.85),
        BACKWARD_AT_ANGLE(-0.24, 0, 0, 0.85),
        CW(0, 0, 0.26, 1.0);

        final double forward, strafe, turn, seconds;

        Step(double forward, double strafe, double turn, double seconds) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.seconds = seconds;
        }
    }

    // Evaluate motion in the robot axes at the start of each step.
    static final class MotionTrace {
        private final Pose2D start;
        double forwardCm, leftCm, headingDegrees;
        double peakForwardCm, peakLeftCm, peakTravelCm, peakHeadingDegrees;

        MotionTrace(Pose2D start) {
            this.start = start;
        }

        void record(Pose2D current) {
            double dx = current.getX(DistanceUnit.CM) - start.getX(DistanceUnit.CM);
            double dy = current.getY(DistanceUnit.CM) - start.getY(DistanceUnit.CM);
            double h = start.getHeading(AngleUnit.RADIANS);
            forwardCm = dx * Math.cos(h) + dy * Math.sin(h);
            leftCm = dy * Math.cos(h) - dx * Math.sin(h);
            headingDegrees = Math.IEEEremainder(
                    current.getHeading(AngleUnit.DEGREES) - start.getHeading(AngleUnit.DEGREES), 360.0);
            peakForwardCm = Math.max(peakForwardCm, Math.abs(forwardCm));
            peakLeftCm = Math.max(peakLeftCm, Math.abs(leftCm));
            peakTravelCm = Math.max(peakTravelCm, Math.hypot(forwardCm, leftCm));
            peakHeadingDegrees = Math.max(peakHeadingDegrees, Math.abs(headingDegrees));
        }

        String issue(Step step, int xTicks, int yTicks) {
            if (step == Step.STILL) {
                if (peakTravelCm > 1.0 || peakHeadingDegrees > 1.0) return "stationary drift";
                if (Math.abs((double) xTicks) > 100 || Math.abs((double) yTicks) > 100) {
                    return "pod counts changed at rest";
                }
                return "";
            }
            if (step.turn != 0.0) {
                if (-Math.signum(step.turn) * headingDegrees < 25.0) return "wrong or weak heading response";
                if (peakTravelCm > 2.5) return "center moved during turn; inspect offsets and pivot";
                return "";
            }
            boolean axial = step.forward != 0.0;
            double progress = axial ? Math.signum(step.forward) * forwardCm
                    : -Math.signum(step.strafe) * leftCm;
            if (progress < 8.0) return "wrong or weak translation on commanded axis";
            double podTravelCm = Math.abs((double) (axial ? xTicks : yTicks)) / FOUR_BAR_TICKS_PER_CM;
            if (podTravelCm < 1.0) return "commanded pod produced few ticks";
            double crossTrack = axial ? peakLeftCm : peakForwardCm;
            if (crossTrack > Math.max(3.0, 0.15 * progress)) return "large cross-axis movement";
            if (peakHeadingDegrees > 3.0) return "heading changed during translation";
            if (Math.abs(progress - podTravelCm) > Math.max(2.0, 0.1 * podTravelCm)) {
                return "reported distance disagrees with 4-bar pod counts";
            }
            return "";
        }
    }

    private final List<String> results = new ArrayList<>();
    private Drive drive;
    private GoBildaPinpointDriver pinpoint;
    private Pose2D origin;
    private Pose2D latest;
    private int samples;
    private int unusualLoopSamples;
    private int minLoopMicros = Integer.MAX_VALUE;
    private int maxLoopMicros;
    private boolean review;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        try {
            drive = new Drive(hardwareMap);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DriveConstants.pinpointName);
            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit() && !isStopRequested()) {
                drive.periodic();
                telemetry.addLine("Level floor with 1.5 m clear on every side. Keep robot still.");
                telemetry.addLine("START runs timed motion. B or STOP aborts.");
                telemetry.addData("Pinpoint", drive.getPinpointReadDiagnostics());
                telemetry.update();
                idle();
            }
            if (isStopRequested()) return;
            waitForStart();
            if (isStopRequested()) return;

            String outcome = runChecks();
            drive.stop();
            RobotLog.ii(LOG_TAG, "%s | %s | %s", outcome, drive.getPinpointReadDiagnostics(),
                    drive.getLastPinpointReadFailure());
            while (opModeIsActive()) {
                telemetry.addData("OD Check", outcome);
                for (String result : results) telemetry.addLine(result);
                telemetry.addData("Pinpoint", drive.getPinpointReadDiagnostics());
                telemetry.addData("Last bad read", drive.getLastPinpointReadFailure());
                telemetry.update();
                sleep(50);
            }
        } finally {
            if (drive != null) drive.stop();
            CommandScheduler.getInstance().reset();
        }
    }

    private String runChecks() {
        ElapsedTime readyTimer = new ElapsedTime();
        do {
            if (!opModeIsActive() || gamepad1.b) return "Aborted before motion";
            drive.periodic();
        } while (drive.getOdometryStatus() != DeviceStatus.READY && readyTimer.seconds() < 3.0);
        if (drive.getOdometryStatus() != DeviceStatus.READY) return "Stopped: Pinpoint did not become READY";

        double offsetX = pinpoint.getXOffset(DistanceUnit.MM);
        double offsetY = pinpoint.getYOffset(DistanceUnit.MM);
        double yawScalar = pinpoint.getYawScalar();
        report(String.format(Locale.US, "Offsets X/Y: %+.1f / %+.1f mm (configured %+.1f / %+.1f); yaw scalar %.4f",
                offsetX, offsetY, DriveConstants.pinpointXOffsetMm, DriveConstants.pinpointYOffsetMm, yawScalar));
        if (pinpoint.getDeviceStatus() != DeviceStatus.READY
                || !offsetMatches(offsetX, DriveConstants.pinpointXOffsetMm)
                || !offsetMatches(offsetY, DriveConstants.pinpointYOffsetMm)
                || !Double.isFinite(yawScalar) || yawScalar <= 0) {
            return "Stopped: invalid Pinpoint configuration readback";
        }
        if (yawScalar < 0.9 || yawScalar > 1.1) {
            review = true;
            report("CHECK: yaw scalar outside the goBILDA diagnostic range");
        }

        String problem = sample();
        if (problem != null) return problem;
        origin = latest;
        for (Step step : Step.values()) {
            problem = runStep(step);
            if (problem != null) return step + " | " + problem;
        }
        MotionTrace closure = new MotionTrace(origin);
        closure.record(latest);
        report(String.format(Locale.US, "Open-loop return to start: %.1f cm / %+.1f deg",
                closure.peakTravelCm, closure.headingDegrees));
        report(String.format(Locale.US, "Device loop time: %d-%d us; %d/%d outside 500-1100 us",
                minLoopMicros, maxLoopMicros, unusualLoopSamples, samples));
        review |= drive.getPinpointBadReadCount() > 0 || unusualLoopSamples > samples * 0.1;
        return review ? "CHECK: review flagged readings" : "PASS: onboard consistency checks";
    }

    private String runStep(Step step) {
        MotionTrace trace = new MotionTrace(latest);
        int startX = pinpoint.getEncoderX();
        int startY = pinpoint.getEncoderY();
        ElapsedTime timer = new ElapsedTime();
        if (!opModeIsActive() || gamepad1.b) return "Aborted before motion";
        boolean powered = step != Step.STILL;
        double endSeconds = step.seconds + (powered ? SETTLE_SECONDS : 0.0);
        double nextUpdateSeconds = 0.0;
        try {
            drive.moveRobot(step.forward, step.strafe, step.turn);
            while (true) {
                double seconds = timer.seconds();
                if (powered && seconds >= step.seconds) {
                    drive.stop();
                    powered = false;
                }
                String problem = sample();
                if (problem != null) return problem;

                trace.record(latest);
                double totalTravel = Math.hypot(
                        latest.getX(DistanceUnit.CM) - origin.getX(DistanceUnit.CM),
                        latest.getY(DistanceUnit.CM) - origin.getY(DistanceUnit.CM));
                if (totalTravel > MAX_TRAVEL_CM || trace.peakTravelCm > MAX_TRAVEL_CM
                        || trace.peakHeadingDegrees > MAX_STAGE_ROTATION_DEGREES) {
                    return "Stopped: reported travel exceeded the test limit";
                }
                if (seconds >= endSeconds) break;
                if (seconds >= nextUpdateSeconds) {
                    nextUpdateSeconds = seconds + 0.1;
                    telemetry.addData("Stage", step);
                    telemetry.addData("Delta (cm / deg)", "F %+.1f / L %+.1f / H %+.1f",
                            trace.forwardCm, trace.leftCm, trace.headingDegrees);
                    telemetry.addData("Pinpoint", drive.getPinpointReadDiagnostics());
                    telemetry.update();
                }
            }
        } finally {
            drive.stop();
        }

        // Raw SDK counts are useful for pod activity even if the estimated pose is wrong.
        int xTicks = pinpoint.getEncoderX() - startX;
        int yTicks = pinpoint.getEncoderY() - startY;
        String issue = trace.issue(step, xTicks, yTicks);
        review |= !issue.isEmpty();
        report(String.format(Locale.US,
                "%s %s | F/L %+.1f/%+.1f cm, H %+.1f deg | peak XY/H %.1f cm/%.1f deg | ticks %+d/%+d%s",
                step, issue.isEmpty() ? "PASS" : "CHECK", trace.forwardCm, trace.leftCm, trace.headingDegrees,
                trace.peakTravelCm, trace.peakHeadingDegrees, xTicks, yTicks,
                issue.isEmpty() ? "" : " | " + issue));
        return null;
    }

    private String sample() {
        if (!opModeIsActive() || gamepad1.b) return "Aborted";
        drive.periodic();
        if (drive.getOdometryStatus() != DeviceStatus.READY) {
            return "Stopped: " + drive.getOdometryStatus();
        }
        latest = drive.getOdometryPose();
        if (latest == null || !Double.isFinite(latest.getX(DistanceUnit.CM))
                || !Double.isFinite(latest.getY(DistanceUnit.CM))
                || !Double.isFinite(latest.getHeading(AngleUnit.DEGREES))
                || !Double.isFinite(pinpoint.getVelX(DistanceUnit.CM))
                || !Double.isFinite(pinpoint.getVelY(DistanceUnit.CM))
                || !Double.isFinite(drive.getHeadingVelocityDegreesPerSecond())) {
            return "Stopped: non-finite Pinpoint pose or velocity";
        }
        int loopMicros = pinpoint.getLoopTime();
        if (loopMicros <= 1) return "Stopped: missing Pinpoint loop time";
        minLoopMicros = Math.min(minLoopMicros, loopMicros);
        maxLoopMicros = Math.max(maxLoopMicros, loopMicros);
        if (loopMicros < 500 || loopMicros > 1100) unusualLoopSamples++;
        samples++;
        return null;
    }

    private static boolean offsetMatches(double measured, double configured) {
        return Double.isFinite(measured) && Math.abs(measured - configured) <= 0.1;
    }

    private void report(String line) {
        results.add(line);
        RobotLog.ii(LOG_TAG, "%s", line);
    }
}
