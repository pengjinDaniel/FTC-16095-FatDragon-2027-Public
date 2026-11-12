package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;

abstract class AllianceAuto extends LinearOpMode {
    private static final double FIELD_CENTER_INCHES = 72.0;

    final Alliance alliance;
    final CellCluster shotCell;
    final PoseFactory poseFactory;

    AllianceAuto(Alliance alliance) {
        this.alliance = alliance;
        boolean blue = alliance == Alliance.BLUE;
        shotCell = blue ? CellCluster.BLUE_AUDIENCE : CellCluster.RED_OPPOSITE_AUDIENCE;

        PoseFactory redPoses = PoseFactory.degrees();
        // Pedro's origin is at the corner of the 144-inch field.
        poseFactory = blue
                ? redPoses.rotateAround(FIELD_CENTER_INCHES, FIELD_CENTER_INCHES, 180.0)
                : redPoses;
    }
}
