package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

import java.util.OptionalDouble;

public class Constants {
    // Replace these blocks with the corresponding AutoTune Java exports after tuning.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
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

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(DriveConstants.pinpointName);
        c.xPodOffset.set(DriveConstants.pinpointXOffsetMm);
        c.yPodOffset.set(DriveConstants.pinpointYOffsetMm);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(DriveConstants.pinpointForwardPodDirection);
        c.yPodDirection.set(DriveConstants.pinpointStrafePodDirection);
    });

    // Paste the entire Foresight Tuner Java export here, replacing this declaration.
    // Its Controller, Matrix, and Vector2D imports are already included above.
    public static ForesightConfig foresightConfig = null;

    public static Foresight createAlgorithm() {
        if (foresightConfig == null) {
            throw new IllegalStateException(
                    "Run AutoTune > Foresight Tuner, then paste its Java export into pedro/Constants.java");
        }
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        Foresight algorithm = createAlgorithm();
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                algorithm);
    }
}
