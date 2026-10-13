package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Shared field frame for odometry, CELL selection, aim and telemetry.
 * Inches; center (72,72), +X toward blue, +Y away from audience, CCW heading.
 * The 19725 flat-HIVE map has center (0,0), +X toward audience, +Y toward blue.
 * Convert only after the HIVE solve, before writing vision into Pinpoint.
 * Robot-relative tag poses and driver-relative joystick headings are not field poses.
 */
public final class FieldCoordinates {
    public static final double CENTER_INCHES = 72.0;
    private FieldCoordinates() { }

    public static Pose2D fromCenteredHiveMap(Pose2D mapPose) {
        if (mapPose == null) return null;
        return new Pose2D(DistanceUnit.INCH,
                CENTER_INCHES + mapPose.getY(DistanceUnit.INCH),
                CENTER_INCHES - mapPose.getX(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                Math.IEEEremainder(mapPose.getHeading(AngleUnit.DEGREES) - 90.0, 360.0));
    }

    public static boolean isAudienceSide(Pose2D fieldPose) {
        return fieldPose.getY(DistanceUnit.INCH) <= CENTER_INCHES;
    }

    /** Signed distance toward the audience; also used for HIVE high/low geometry. */
    public static double audienceOffsetInches(double fieldYInches) {
        return CENTER_INCHES - fieldYInches;
    }
}
