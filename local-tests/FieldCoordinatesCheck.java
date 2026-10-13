package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.*;

/** Physical invariants, not just conversion round-trips: bearing, side and HIVE state. */
public class FieldCoordinatesCheck {
    static void check(boolean ok,String why) { if(!ok) throw new AssertionError(why); }
    static Pose2D pose(double x,double y,double heading) {
        return new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES,heading);
    }
    public static void main(String[] args) {
        double[][] mapTargets={{-17.7,-12.75},{17.7,-12.75},{17.7,12.75},{-17.7,12.75}};
        int checks=0;
        for(double x:new double[]{-60,-20,0,20,60}) {
            for(double y:new double[]{-60,-20,0,20,60}) {
                for(double h=-180;h<180;h+=15) {
                    Pose2D map=pose(x,y,h), field=FieldCoordinates.fromCenteredHiveMap(map);
                    // Heading basis and translation must agree (rotation -90 degrees).
                    check(Math.abs(Math.cos(field.getHeading(AngleUnit.RADIANS))-Math.sin(Math.toRadians(h)))<1e-9,"heading axis");
                    for(CellCluster cell:CellCluster.values()) {
                        int i=cell.getTuningIndex();
                        double[] target=VisionConstants.cellTargetFieldXYInches[i];
                        double oldError=VisionMath.coordinateYawErrorDegrees(map,mapTargets[i][0],mapTargets[i][1],180);
                        double newError=VisionMath.coordinateYawErrorDegrees(field,target[0],target[1],180);
                        check(Math.abs(Math.IEEEremainder(newError-oldError,360))<1e-8,"physical aim bearing changed");
                        check(Math.abs(Math.hypot(mapTargets[i][0]-x,mapTargets[i][1]-y)-Math.hypot(target[0]-field.getX(DistanceUnit.INCH),target[1]-field.getY(DistanceUnit.INCH)))<1e-8,"distance changed");
                        for(double tilt:new double[]{-30,30}) {
                            check((-Math.sin(Math.toRadians(tilt))*mapTargets[i][0]>0)==(-Math.sin(Math.toRadians(tilt))*FieldCoordinates.audienceOffsetInches(target[1])>0),"HIGH/LOW changed");
                        }
                        checks++;
                    }
                    for(Alliance alliance:Alliance.values()) {
                        CellCluster selected=CellCluster.forFieldPose(alliance,field).get();
                        check(selected==(alliance==Alliance.BLUE ? (x>=0?CellCluster.BLUE_AUDIENCE:CellCluster.BLUE_OPPOSITE_AUDIENCE) : (x>=0?CellCluster.RED_AUDIENCE:CellCluster.RED_OPPOSITE_AUDIENCE)),"physical half changed");
                    }
                }
            }
        }
        // Travel across audience/non-audience while X stays constant: both red and blue lanes.
        for(Alliance a:Alliance.values()) for(double x:new double[]{20,120}) {
            CellCluster near=CellCluster.forFieldPose(a,pose(x,20,0)).get();
            CellCluster far=CellCluster.forFieldPose(a,pose(x,120,0)).get();
            check(near!=far,"crossing Y=72 must switch even though X never changes sign");
            Pose2D centimeters=new Pose2D(DistanceUnit.CM,x*2.54,20*2.54,AngleUnit.RADIANS,0);
            check(CellCluster.forFieldPose(a,centimeters).get()==near,"cm/in normalization");
        }
        check(!CellCluster.forFieldPose(Alliance.BLUE,pose(Double.NaN,20,0)).isPresent(),"invalid pose");
        System.out.println("PASS: "+checks+" physical bearing/distance/HIVE checks; both alliances, constant-X half crossings, units, heading, invalid pose");
    }
}
