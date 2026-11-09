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
    // Motor names, directions and pod geometry are shared with the competition Drive.
    // Apply measured hardware changes to DriveConstants; its pod offsets are in millimeters.
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

    // FTC1 measurements exported by the Foresight Tuner.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29362974330914304);
                Controller secondaryTranslationalForward = Controller.proportional(0.10848834422148654);
                Controller primaryTranslationalLateral = Controller.proportional(0.4018130159224158);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14845917274180762);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01367301191849275));
                c.brake.set(Controller.proportionalFeedforward(0.011622060130718837));

                c.headingFeedback.set(Controller.proportional(4.734866377156585));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05806231553416445, 0.005340939042939729));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11655073581304821, 0.04934958786379315));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013526590539923165, 0.002103772477380755));

                c.maxAchievableForwardVelocity.set(81.79608478313465);
                c.maxAchievableStrafeVelocity.set(64.48601849280288);
                c.naturalForwardDeceleration.set(78.37278279390627);
                c.naturalStrafeDeceleration.set(73.14837745654077);
            }
    );

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
