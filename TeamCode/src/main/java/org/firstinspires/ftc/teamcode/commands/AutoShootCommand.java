package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;

import java.util.function.BooleanSupplier;

/**
 * Runs a stationary shot using the TeleOp controllers and Pedro's field pose.
 * Schedule sequentially after a path, and keep calling follower.update() and
 * CommandScheduler.run(). Vision must also be registered with the scheduler.
 * This command owns the follower until end(); do not run another drive command alongside it.
 */
public class AutoShootCommand extends CommandBase {
    private final Follower follower;
    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final AutoHiveTarget target;
    private final BooleanSupplier localizationReady;
    private final long feedDurationMs;
    private final long timeoutMs;
    private final boolean waitForHigh;
    private final AutoAimController autoAim = new AutoAimController();
    private final PositionHoldController positionHold = new PositionHoldController();

    private long startedNanos;
    private long feedStartedNanos = -1;
    private boolean timedOut;
    private String status = "Not started";

    public AutoShootCommand(Follower follower, Shooter shooter, Intake intake, Transit transit,
                            AutoHiveTarget target, BooleanSupplier localizationReady,
                            long feedDurationMs, long timeoutMs, boolean waitForHigh) {
        if (feedDurationMs <= 0 || timeoutMs < feedDurationMs) {
            throw new IllegalArgumentException("Shot timeout must cover a positive feed duration");
        }
        this.follower = follower;
        this.shooter = shooter;
        this.intake = intake;
        this.transit = transit;
        this.target = target;
        this.localizationReady = localizationReady;
        this.feedDurationMs = feedDurationMs;
        this.timeoutMs = timeoutMs;
        this.waitForHigh = waitForHigh;
        addRequirements(shooter, intake, transit);
    }

    @Override
    public void initialize() {
        startedNanos = System.nanoTime();
        feedStartedNanos = -1;
        timedOut = false;
        status = "Preparing shot";
        autoAim.reset();
        positionHold.reset();
        follower.stop();
        follower.drivetrain.stop();
        transit.cancelFeeding();
        intake.stop();
        // Preserve a distance target already spun up while following the preceding path.
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        if (!localizationReady.getAsBoolean() || pose == null || velocity == null
                || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading()) || !Double.isFinite(velocity.linSpeed())
                || !Double.isFinite(velocity.omega)) {
            stopForInvalidTarget("Waiting for valid localization");
            return;
        }

        target.update(pose);
        double yawError = target.getYawErrorDegrees();
        if (!Double.isFinite(yawError) || !shooter.setDistanceTarget(target.getDistanceCm())) {
            stopForInvalidTarget("Waiting for a valid shot target");
            return;
        }

        double turnPower = autoAim.calculateTurnPower(true, yawError);
        positionHold.update(AutoHiveTarget.toHiveFieldPose(pose), true, velocity.linSpeed());
        // Pedro strafe/turn are left/CCW-positive; TeleOp powers are right/CW-positive.
        DrivePowers powers = new DrivePowers(positionHold.getForwardPower(),
                -positionHold.getStrafePower(), -turnPower);
        follower.manual(powers);
        follower.drivetrain.drive(powers, true);
        intake.updateInputs(false, false, true);

        if (waitForHigh && feedStartedNanos < 0 && target.getState() != HiveState.HIGH) {
            transit.cancelFeeding();
            status = "Waiting for HIGH: " + target.getState();
            return;
        }

        boolean ready = shooter.isReady() && autoAim.isLocked() && shooter.isPitchReady();
        // Transit retains the same committed reverse/feed sequence as TeleOp.
        transit.requestFeed(ready);
        if (transit.getFiringState() == Transit.FiringState.FEEDING) {
            if (feedStartedNanos < 0) feedStartedNanos = System.nanoTime();
            status = "Shooting: " + feedDurationMs + " ms";
        } else if (!autoAim.isLocked()) {
            status = "Aiming";
        } else if (!shooter.isReady()) {
            status = "Waiting for shooter speed";
        } else if (!shooter.isPitchReady()) {
            status = "Waiting for pitch";
        } else {
            status = "Preparing shot: " + transit.getFiringState();
        }
    }

    @Override
    public boolean isFinished() {
        long now = System.nanoTime();
        if (feedStartedNanos >= 0 && (now - feedStartedNanos) / 1_000_000L >= feedDurationMs) {
            return true;
        }
        timedOut = (now - startedNanos) / 1_000_000L >= timeoutMs;
        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        if (transit.getState() == Transit.TransitState.OPEN) intake.clearBallCount();
        transit.cancelFeeding();
        intake.stop();
        shooter.stop();
        autoAim.reset();
        positionHold.reset();
        follower.stop();
        follower.drivetrain.stop();
        status = interrupted ? "Shot interrupted" : timedOut ? "Shot timed out" : "Shot complete";
    }

    public String getStatus() {
        return status;
    }

    public boolean hasTimedOut() {
        return timedOut;
    }

    private void stopForInvalidTarget(String reason) {
        if (transit.getState() == Transit.TransitState.OPEN) intake.clearBallCount();
        transit.cancelFeeding();
        intake.stop();
        shooter.stop();
        autoAim.reset();
        positionHold.reset();
        feedStartedNanos = -1;
        follower.stop();
        follower.drivetrain.stop();
        status = reason;
    }
}
