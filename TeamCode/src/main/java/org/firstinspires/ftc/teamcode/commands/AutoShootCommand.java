package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;

import java.util.function.BooleanSupplier;

public class AutoShootCommand extends CommandBase {
    private final Follower follower;
    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final AutoHiveTarget target;
    private final BooleanSupplier localizationReady;
    private final long feedDurationMs;
    private final long timeoutMs;

    private long startedNanos;
    private long feedStartedNanos = -1;
    private boolean timedOut;
    private String status = "Not started";

    public AutoShootCommand(Follower follower, Shooter shooter, Intake intake, Transit transit,
                            AutoHiveTarget target, BooleanSupplier localizationReady,
                            long feedDurationMs, long timeoutMs) {
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
        addRequirements(shooter, intake, transit);
    }

    @Override
    public void initialize() {
        startedNanos = System.nanoTime();
        feedStartedNanos = -1;
        timedOut = false;
        status = "Preparing shot";
        follower.stop();
        follower.drivetrain.stop();
        transit.cancelFeeding();
        intake.updateShotFeed(false);
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
        if (!shooter.setDistanceTarget(target.getDistanceCm())) {
            stopForInvalidTarget("Waiting for a valid shot target");
            return;
        }

        boolean speedReady = shooter.isReady();
        transit.requestFeed(speedReady);
        Transit.FiringState firingState = transit.getFiringState();
        boolean feeding = firingState == Transit.FiringState.FEEDING;
        intake.updateShotFeed(feeding);
        if (feeding) {
            if (feedStartedNanos < 0) feedStartedNanos = System.nanoTime();
            status = "Shooting: " + feedDurationMs + " ms";
        } else if (firingState == Transit.FiringState.REVERSING) {
            status = "Reversing before feed";
        } else if (!speedReady) {
            status = "Waiting for shooter speed";
        } else {
            status = "Invalid transit gate or reverse settings";
        }
    }

    @Override
    public boolean isFinished() {
        long now = System.nanoTime();
        if (feedStartedNanos >= 0) {
            return (now - feedStartedNanos) / 1_000_000L >= feedDurationMs;
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
        follower.stop();
        follower.drivetrain.stop();
        if (interrupted) {
            status = "Shot interrupted";
        } else if (timedOut) {
            status = "Shot timed out (" + status + ")";
        } else {
            status = "Shot complete";
        }
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
        intake.updateShotFeed(false);
        feedStartedNanos = -1;
        status = reason;
        shooter.stop();
        follower.stop();
        follower.drivetrain.stop();
    }
}
