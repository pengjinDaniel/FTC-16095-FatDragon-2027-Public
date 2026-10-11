package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Pedro 3.0.1: shared configuration for TeleOp, heading tests and future paths. */
public final class Constants {
    private Constants() {}
    // Path dynamics must be measured separately; do not use defaults as robot calibration.
    public static boolean pathFollowingCalibrated = false;
    // Populate only with this robot's measured path dynamics, then enable the flag above.
    public static ForesightConfig foresightConfig;
    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(DriveConstants.leftFrontMotorName);
            c.backLeftName.set(DriveConstants.leftBackMotorName);
            c.frontRightName.set(DriveConstants.rightFrontMotorName);
            c.backRightName.set(DriveConstants.rightBackMotorName);
            c.frontLeftDirection.set(DriveConstants.leftFrontMotorDirection);
            c.backLeftDirection.set(DriveConstants.leftBackMotorDirection);
            c.frontRightDirection.set(DriveConstants.rightFrontMotorDirection);
            c.backRightDirection.set(DriveConstants.rightBackMotorDirection);
            c.manualBrakeMode.set(true);
        });
    }
    public static PinpointConfig localizerConfig() {
        return new PinpointConfig(c -> {
            c.name.set(DriveConstants.pinpointName);
            // Existing robot measurements are in mm. Set units explicitly, no silent inches.
            c.offsetUnits.set(DistanceUnit.MM);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.xPodOffset.set(DriveConstants.pinpointXOffsetMm);
            c.yPodOffset.set(DriveConstants.pinpointYOffsetMm);
            c.xPodDirection.set(DriveConstants.pinpointForwardPodDirection);
            c.yPodDirection.set(DriveConstants.pinpointStrafePodDirection);
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
        });
    }
    public static HardwareMap validateHardware(HardwareMap h) {
        if (h == null) throw new IllegalArgumentException("HardwareMap is missing");
        if (!Double.isFinite(DriveConstants.pinpointXOffsetMm)
                || !Double.isFinite(DriveConstants.pinpointYOffsetMm)) {
            throw new IllegalArgumentException("Pinpoint pod offsets must be finite (mm)");
        }
        MecanumConfig drive = drivetrainConfig();
        PinpointConfig localizer = localizerConfig();
        PedroAlgorithms.validateConfig("Mecanum", drive);
        PedroAlgorithms.validateConfig("Pinpoint", localizer);
        String[] names = {drive.frontLeftName.get(), drive.backLeftName.get(),
                drive.frontRightName.get(), drive.backRightName.get()};
        DcMotorEx[] motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            if (names[i] == null || names[i].trim().isEmpty()) {
                throw new IllegalArgumentException("Drive motor name " + i + " is empty");
            }
            motors[i] = h.tryGet(DcMotorEx.class, names[i]);
            if (motors[i] == null) throw new IllegalArgumentException("Motor not found: " + names[i]);
            for (int j = 0; j < i; j++) {
                if (motors[i] == motors[j]) {
                    throw new IllegalArgumentException("Two drive slots refer to the same motor: " + names[i]);
                }
            }
        }
        String pinpointName = localizer.name.get();
        if (pinpointName == null || pinpointName.trim().isEmpty()
                || h.tryGet(GoBildaPinpointDriver.class, pinpointName) == null) {
            throw new IllegalArgumentException("Pinpoint not found: " + pinpointName);
        }
        return h;
    }
    public static Follower create(HardwareMap h) {
        // Validate algorithm before any hardware construction so a missing ConfigVar
        // cannot leave a partly constructed drivetrain behind.
        Algorithm algorithm = PedroAlgorithms.create(pathFollowingCalibrated, foresightConfig);
        validateHardware(h);
        return new Follower(new PinpointLocalizer(h, localizerConfig()),
                new Mecanum(h, drivetrainConfig()), algorithm);
    }
}
