package org.firstinspires.ftc.teamcode.subsystems.shooter;

final class ShooterVelocityModel {
    private ShooterVelocityModel() {
    }

    static double getVelocityForDistance(
            double distanceInches,
            double[] distancePointsInches,
            double[] velocityPointsTicksPerSecond
    ) {
        return interpolate(distanceInches, distancePointsInches,
                velocityPointsTicksPerSecond, Double.MIN_VALUE, Double.POSITIVE_INFINITY);
    }

    static double getPitchForDistance(
            double distanceInches,
            double[] distancePointsInches,
            double[] pitchPoints
    ) {
        return interpolate(distanceInches, distancePointsInches, pitchPoints,
                ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition);
    }

    private static double interpolate(
            double distanceInches,
            double[] distancePointsInches,
            double[] outputPoints,
            double minimumOutput,
            double maximumOutput
    ) {
        if (!Double.isFinite(distanceInches)
                || distancePointsInches == null
                || outputPoints == null
                || distancePointsInches.length < 2
                || distancePointsInches.length != outputPoints.length
                || !Double.isFinite(minimumOutput)
                || Double.isNaN(maximumOutput)
                || minimumOutput > maximumOutput) {
            return Double.NaN;
        }

        for (int i = 0; i < distancePointsInches.length; i++) {
            if (!Double.isFinite(distancePointsInches[i])
                    || !Double.isFinite(outputPoints[i])
                    || outputPoints[i] < minimumOutput
                    || outputPoints[i] > maximumOutput
                    || (i > 0 && distancePointsInches[i] <= distancePointsInches[i - 1])) {
                return Double.NaN;
            }
        }

        if (distanceInches <= distancePointsInches[0]) {
            return outputPoints[0];
        }
        if (distanceInches >= distancePointsInches[distancePointsInches.length - 1]) {
            return outputPoints[outputPoints.length - 1];
        }

        for (int i = 1; i < distancePointsInches.length; i++) {
            if (distanceInches <= distancePointsInches[i]) {
                double fraction = (distanceInches - distancePointsInches[i - 1])
                        / (distancePointsInches[i] - distancePointsInches[i - 1]);
                return outputPoints[i - 1]
                        + fraction * (
                        outputPoints[i] - outputPoints[i - 1]
                );
            }
        }

        return Double.NaN;
    }
}
