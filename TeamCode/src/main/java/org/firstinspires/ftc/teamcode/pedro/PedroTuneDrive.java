package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

/** Opt-in Pedro backend. Existing upstream OpModes continue to use original Drive. */
public class PedroTuneDrive extends Drive {
    private Follower follower;
    public PedroTuneDrive(HardwareMap h) {
        super(Constants.validateHardware(h)); // Validate names/offsets before parent initialization.
        try {
            // A previous velocity/position tuner may have left these modes enabled.
            // This test provides its own heading PID and commands raw motor power.
            for (DcMotorEx motor : new DcMotorEx[]{leftFrontMotor, leftBackMotor,
                    rightFrontMotor, rightBackMotor}) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            follower = Constants.create(h);
            // Both localizers read the same Pinpoint. Do not write a possibly invalid
            // pose back into it while resetPosAndIMU is still calibrating.
            stop(); follower.update();
        } catch (RuntimeException e) {
            super.moveRobot(0, 0, 0);
            throw e;
        }
    }
    @Override public void periodic() { super.periodic(); follower.update(); }
    @Override public Pose2D getPose() {
        if (follower == null) return super.getPose();
        Pose p = follower.pose();
        return new Pose2D(DistanceUnit.INCH, p.x(), p.y(), AngleUnit.RADIANS, p.heading());
    }
    @Override public void setPose(Pose2D p) {
        super.setPose(p); // Preserves upstream private fieldPoseInitialized flag and READY test.
        if (follower != null) {
            follower.setPose(new Pose(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH),
                    p.getHeading(AngleUnit.RADIANS))); follower.update();
        }
    }
    @Override public double getTranslationSpeedInchesPerSecond() {
        if (follower == null) return super.getTranslationSpeedInchesPerSecond();
        Velocity v = follower.velocity(); return v == null ? Double.NaN : Math.hypot(v.vx, v.vy);
    }
    public double getHeadingSpeedDegreesPerSecond() {
        return Math.abs(getSignedHeadingDegreesPerSecond());
    }
    public double getSignedHeadingDegreesPerSecond() {
        Velocity v = follower == null ? null : follower.velocity();
        return v == null ? Double.NaN : Math.toDegrees(v.omega);
    }
    @Override public void stop() {
        if (follower == null) { super.moveRobot(0,0,0); return; }
        follower.stop(); follower.drivetrain.stop();
    }
    @Override public void moveRobot(double forward, double strafe, double turn) {
        if (follower == null) { super.moveRobot(forward,strafe,turn); return; }
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            stop(); return;
        }
        follower.manual(forward,-strafe,-turn); follower.update();
    }
    public Follower getFollower() { return follower; }
    public void followPath(com.pedropathing.paths.Path path) {
        if (!Constants.pathFollowingCalibrated || Constants.foresightConfig == null
                || !(follower.algorithm() instanceof com.pedropathing.algorithm.Foresight)) {
            stop(); throw new IllegalStateException("Pedro dynamics not calibrated; recreate follower after calibration");
        }
        follower.follow(path);
    }
}
