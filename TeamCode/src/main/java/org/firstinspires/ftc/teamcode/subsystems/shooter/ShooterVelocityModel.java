package org.firstinspires.ftc.teamcode.subsystems.shooter;

final class ShooterVelocityModel {
    private ShooterVelocityModel() {
    }

    static double getVelocityForDistance(
            double distanceInches,
            double[] distancePointsInches,
            double[] velocityPointsTicksPerSecond
    ) {
        if (!Double.isFinite(distanceInches)
                || distancePointsInches == null
                || velocityPointsTicksPerSecond == null
                || distancePointsInches.length < 2
                || distancePointsInches.length != velocityPointsTicksPerSecond.length) {
            return Double.NaN;
        }

        for (int i = 0; i < distancePointsInches.length; i++) {
            if (!Double.isFinite(distancePointsInches[i])
                    || !Double.isFinite(velocityPointsTicksPerSecond[i])
                    || velocityPointsTicksPerSecond[i] <= 0.0
                    || (i > 0 && distancePointsInches[i] <= distancePointsInches[i - 1])) {
                return Double.NaN;
            }
        }

        if (distanceInches <= distancePointsInches[0]) {
            return velocityPointsTicksPerSecond[0];
        }
        if (distanceInches >= distancePointsInches[distancePointsInches.length - 1]) {
            return velocityPointsTicksPerSecond[velocityPointsTicksPerSecond.length - 1];
        }

        for (int i = 1; i < distancePointsInches.length; i++) {
            if (distanceInches <= distancePointsInches[i]) {
                double fraction = (distanceInches - distancePointsInches[i - 1])
                        / (distancePointsInches[i] - distancePointsInches[i - 1]);
                return velocityPointsTicksPerSecond[i - 1]
                        + fraction * (
                        velocityPointsTicksPerSecond[i] - velocityPointsTicksPerSecond[i - 1]
                );
            }
        }

        return Double.NaN;
    }
}
