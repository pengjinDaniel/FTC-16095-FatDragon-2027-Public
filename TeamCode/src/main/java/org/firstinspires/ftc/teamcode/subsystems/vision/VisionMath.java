package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Optional;

final class VisionMath {
    private static final double MINIMUM_QUATERNION_MAGNITUDE = 1e-9;

    private VisionMath() {
    }

    static Optional<Pose3D> compose(Pose3D first, Pose3D second) {
        Optional<OpenGLMatrix> firstMatrix = toMatrix(first);
        Optional<OpenGLMatrix> secondMatrix = toMatrix(second);
        if (!firstMatrix.isPresent() || !secondMatrix.isPresent()) {
            return Optional.empty();
        }

        return fromMatrix(firstMatrix.get().multiplied(secondMatrix.get()));
    }

    static Optional<Pose3D> weightedAveragePoses(
            List<Pose3D> poses,
            List<Double> weights
    ) {
        if (poses == null
                || weights == null
                || poses.isEmpty()
                || poses.size() != weights.size()) {
            return Optional.empty();
        }

        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedZ = 0.0;
        double weightedQw = 0.0;
        double weightedQx = 0.0;
        double weightedQy = 0.0;
        double weightedQz = 0.0;
        Quaternion referenceQuaternion = null;

        for (int i = 0; i < poses.size(); i++) {
            double weight = weights.get(i);
            Optional<RobotSpacePosition> position = getRobotSpacePositionInches(poses.get(i));
            Optional<OpenGLMatrix> poseMatrix = toMatrix(poses.get(i));
            if (!Double.isFinite(weight)
                    || weight <= 0.0
                    || !position.isPresent()
                    || !poseMatrix.isPresent()) {
                continue;
            }

            Quaternion quaternion = Quaternion.fromMatrix(poseMatrix.get(), 0);
            double magnitude = quaternion.magnitude();
            if (!Double.isFinite(magnitude) || magnitude <= MINIMUM_QUATERNION_MAGNITUDE) {
                continue;
            }
            quaternion = quaternion.normalized();

            if (referenceQuaternion == null) {
                referenceQuaternion = quaternion;
            }
            double sign = quaternionDot(referenceQuaternion, quaternion) < 0.0 ? -1.0 : 1.0;

            RobotSpacePosition robotSpacePosition = position.get();
            totalWeight += weight;
            weightedX += robotSpacePosition.forwardInches * weight;
            weightedY += robotSpacePosition.lateralInches * weight;
            weightedZ += robotSpacePosition.upInches * weight;
            weightedQw += sign * quaternion.w * weight;
            weightedQx += sign * quaternion.x * weight;
            weightedQy += sign * quaternion.y * weight;
            weightedQz += sign * quaternion.z * weight;
        }

        if (!Double.isFinite(totalWeight) || totalWeight <= 0.0) {
            return Optional.empty();
        }

        double quaternionMagnitude = Math.sqrt(
                weightedQw * weightedQw
                        + weightedQx * weightedQx
                        + weightedQy * weightedQy
                        + weightedQz * weightedQz
        );
        if (!Double.isFinite(quaternionMagnitude)
                || quaternionMagnitude <= MINIMUM_QUATERNION_MAGNITUDE) {
            return Optional.empty();
        }

        Quaternion averageQuaternion = new Quaternion(
                (float) (weightedQw / quaternionMagnitude),
                (float) (weightedQx / quaternionMagnitude),
                (float) (weightedQy / quaternionMagnitude),
                (float) (weightedQz / quaternionMagnitude),
                0
        );
        Orientation orientation = Orientation.getOrientation(
                averageQuaternion.toMatrix(),
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.DEGREES
        );

        return Optional.of(new Pose3D(
                new Position(
                        DistanceUnit.INCH,
                        weightedX / totalWeight,
                        weightedY / totalWeight,
                        weightedZ / totalWeight,
                        0
                ),
                new YawPitchRollAngles(
                        AngleUnit.DEGREES,
                        orientation.firstAngle,
                        orientation.secondAngle,
                        orientation.thirdAngle,
                        0
                )
        ));
    }

    static Optional<RobotSpacePosition> getRobotSpacePositionInches(Pose3D pose) {
        if (pose == null || pose.getPosition() == null || pose.getPosition().unit == null) {
            return Optional.empty();
        }

        // Limelight robot space is target-relative: X forward, Y lateral, and Z up.
        // Distance calculations are sign-independent, so they remain valid across the
        // right-positive/left-positive Y convention change in Limelight OS 2027.
        Position position = pose.getPosition();
        double forward = DistanceUnit.INCH.fromUnit(position.unit, position.x);
        double lateral = DistanceUnit.INCH.fromUnit(position.unit, position.y);
        double up = DistanceUnit.INCH.fromUnit(position.unit, position.z);
        if (!Double.isFinite(forward) || !Double.isFinite(lateral) || !Double.isFinite(up)) {
            return Optional.empty();
        }

        return Optional.of(new RobotSpacePosition(forward, lateral, up));
    }

    static boolean isValidPose(Pose3D pose) {
        return toMatrix(pose).isPresent();
    }

    private static Optional<OpenGLMatrix> toMatrix(Pose3D pose) {
        Optional<RobotSpacePosition> position = getRobotSpacePositionInches(pose);
        if (!position.isPresent() || pose == null || pose.getOrientation() == null) {
            return Optional.empty();
        }

        double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
        double pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
        double roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);
        if (!Double.isFinite(yaw) || !Double.isFinite(pitch) || !Double.isFinite(roll)) {
            return Optional.empty();
        }

        RobotSpacePosition robotSpacePosition = position.get();
        OpenGLMatrix translation = OpenGLMatrix.translation(
                (float) robotSpacePosition.forwardInches,
                (float) robotSpacePosition.lateralInches,
                (float) robotSpacePosition.upInches
        );
        OpenGLMatrix rotation = OpenGLMatrix.rotation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.DEGREES,
                (float) yaw,
                (float) pitch,
                (float) roll
        );
        return Optional.of(translation.multiplied(rotation));
    }

    private static Optional<Pose3D> fromMatrix(OpenGLMatrix matrix) {
        if (matrix == null) {
            return Optional.empty();
        }

        VectorF translation = matrix.getTranslation();
        Orientation orientation = Orientation.getOrientation(
                matrix,
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.DEGREES
        );
        double x = translation.get(0);
        double y = translation.get(1);
        double z = translation.get(2);
        if (!Double.isFinite(x)
                || !Double.isFinite(y)
                || !Double.isFinite(z)
                || !Float.isFinite(orientation.firstAngle)
                || !Float.isFinite(orientation.secondAngle)
                || !Float.isFinite(orientation.thirdAngle)) {
            return Optional.empty();
        }

        return Optional.of(new Pose3D(
                new Position(DistanceUnit.INCH, x, y, z, 0),
                new YawPitchRollAngles(
                        AngleUnit.DEGREES,
                        orientation.firstAngle,
                        orientation.secondAngle,
                        orientation.thirdAngle,
                        0
                )
        ));
    }

    private static double quaternionDot(Quaternion first, Quaternion second) {
        return first.w * second.w
                + first.x * second.x
                + first.y * second.y
                + first.z * second.z;
    }

    static final class RobotSpacePosition {
        final double forwardInches;
        final double lateralInches;
        final double upInches;
        final double range3DInches;
        final double horizontalDistanceInches;

        RobotSpacePosition(double forwardInches, double lateralInches, double upInches) {
            this.forwardInches = forwardInches;
            this.lateralInches = lateralInches;
            this.upInches = upInches;
            horizontalDistanceInches = Math.sqrt(
                    forwardInches * forwardInches + lateralInches * lateralInches
            );
            range3DInches = Math.sqrt(
                    horizontalDistanceInches * horizontalDistanceInches + upInches * upInches
            );
        }
    }
}
