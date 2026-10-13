package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.lang.reflect.Field;
import java.util.EnumMap;
import sun.misc.Unsafe;

/** Real SDK poses, simulated clock and drive; no motors, camera, ADB or network. */
public class VisionCalibrationCheck {
    static Pose2D pose(double x,double heading) { return new Pose2D(DistanceUnit.INCH,x,0,AngleUnit.DEGREES,heading); }
    static void check(boolean ok,String message) { if(!ok) throw new AssertionError(message); }
    static void set(Vision v,String name,Object value) throws Exception {
        Field f=Vision.class.getDeclaredField(name); f.setAccessible(true); f.set(v,value);
    }
    public static class FakeDrive extends Drive {
        Pose2D position; int attempts,writes; boolean initialized,writeSucceeds=true;
        private FakeDrive() { super(null); }
        @Override public Pose2D getPose() { return position; }
        @Override public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() { return GoBildaPinpointDriver.DeviceStatus.READY; }
        @Override public boolean isFieldPoseInitialized() { return initialized; }
        @Override public boolean calibrateFieldPose(Pose2D p,double ts) {
            attempts++; if(!writeSucceeds) return false;
            writes++; position=p; initialized=true; return true;
        }
    }
    static void frame(Vision vision,FakeDrive drive,double timestamp,long now,boolean allowed,double x) throws Exception {
        set(vision,"correctedRobotFieldPose",pose(x,0)); set(vision,"frameTimestamp",timestamp);
        vision.updateCalibration(drive,allowed,now);
    }
    public static void main(String[] args) throws Exception {
        VisionCalibrationFilter f=new VisionCalibrationFilter(); Pose2D zero=pose(0,0), v=pose(4,0);
        check(!f.update(v,zero,1,0,true),"first frame must not initialize alone");
        check(!f.update(v,zero,1,10_000_000,true) && f.getSampleCount()==1,"duplicates are not samples");
        check(!f.update(v,zero,2,20_000_000,true),"two frames still wait");
        check(f.update(v,zero,3,40_000_000,true),"three stable frames permit first attempt immediately");
        f.onWriteRejected();
        check(f.update(v,zero,4,50_000_000,true),"failure retries next fresh frame without 150ms delay");
        f.onApplied(zero,v,50_000_000);
        check(!f.update(v,v,5,199_999_999,true),"149.999ms after SUCCESS still blocked");
        check(f.update(v,v,6,200_000_000,true),"exactly150ms after SUCCESS allowed");
        f.onApplied(v,v,200_000_000);
        check(!f.update(v,v,7,350_000_000,false) && f.isTrusted(),"R2 held trusts vision but never writes");
        check(f.update(v,v,8,351_000_000,true),"release R2 attempts immediately if success interval already elapsed");
        check(!f.update(pose(30,0),v,9,360_000_000,true) && !f.isTrusted(),"large single position jump rejected");
        check(!f.update(v,v,10,370_000_000,true),"after jump must rebuild stability");
        check(!f.update(v,v,11,380_000_000,true),"two stable recovery frames insufficient");
        check(f.update(v,v,12,390_000_000,true),"stable recovery retries without failure delay");
        check(!f.update(pose(4,40),v,13,400_000_000,true),"heading jump rejected");
        f=new VisionCalibrationFilter();
        for(int i=0;i<3;i++) check(f.update(pose(100*i+4,30*i),pose(100*i,30*i),i+1,i*20_000_000L,true)==(i==2),"normal odometry translation/rotation is not a visual jump");
        check(!f.update(null,zero,4,60_000_000,true) && !f.isTrusted(),"lost/invalid pose clears confidence");
        check(!f.update(v,zero,5,70_000_000,true),"new sample after loss starts immediately");
        f=new VisionCalibrationFilter();
        check(!f.update(pose(0,179),zero,1,0,true),"wrap frame1");
        check(!f.update(pose(0,-179),zero,2,20_000_000,true),"wrap frame2");
        check(f.update(pose(0,180),zero,3,40_000_000,true),"heading wrap not jump");
        check(!f.update(pose(0,180),zero,4,300_000_000,true),"long gap clears old stability");
        Field uf=Unsafe.class.getDeclaredField("theUnsafe"); uf.setAccessible(true); Unsafe u=(Unsafe)uf.get(null);
        Vision vision=(Vision)u.allocateInstance(Vision.class); FakeDrive drive=(FakeDrive)u.allocateInstance(FakeDrive.class);
        drive.position=zero; drive.writeSucceeds=false;
        set(vision,"calibrationFilter",new VisionCalibrationFilter());
        for(int i=1;i<=3;i++) frame(vision,drive,i,i*20_000_000L,true,4);
        check(drive.attempts==1 && drive.writes==0,"production wiring attempts stable frame, actual write fails");
        drive.writeSucceeds=true; frame(vision,drive,4,80_000_000L,true,4);
        check(drive.attempts==2 && drive.writes==1,"production retries failed write next frame");
        frame(vision,drive,5,100_000_000L,true,4);
        check(drive.writes==1,"production obeys successful-write cooldown");
        frame(vision,drive,6,200_000_000L,false,4); frame(vision,drive,7,240_000_000L,false,4);
        check(drive.writes==1 && vision.isCorrectedPoseAccepted(),"held R2 never writes but shooting confidence survives");
        frame(vision,drive,8,250_000_000L,true,4);
        check(drive.writes==2,"release R2 writes when ready");
        // Full production target geometry/firing validity while corrections are paused.
        set(vision,"alliance",Alliance.BLUE); set(vision,"aimCellSelection",new CellSelectionLatch());
        EnumMap<CellCluster,HiveState> states=new EnumMap<>(CellCluster.class);
        for(CellCluster cell:CellCluster.values()) states.put(cell,HiveState.HIGH);
        set(vision,"observedClusterStates",states);
        set(vision,"calibrationFilter",new VisionCalibrationFilter());
        long now=System.nanoTime();
        for(int i=0;i<3;i++) frame(vision,drive,100+i,now-20_000_000L+i*5_000_000L,false,4);
        vision.updateFieldTarget(drive,false);
        check(vision.hasValidAimingTarget() && vision.hasValidShootingTarget(),"full aim/shoot remains usable on trusted HIGH while R2 held");
        check(drive.writes==2,"geometry/feed check never recalibrates held R2");
        // Verify production selection, including releasing and re-pressing R2 across halves.
        for (Alliance alliance : Alliance.values()) {
            set(vision,"alliance",alliance);
            vision.setAimSelectionHeld(false);
            drive.position=pose(-40,0); vision.updateFieldTarget(drive,false);
            CellCluster left=CellCluster.forFieldX(alliance,-40).get();
            CellCluster right=CellCluster.forFieldX(alliance,40).get();
            check(vision.getSelectedCluster().get()==left,"negative half selects its own CELL");
            vision.setAimSelectionHeld(true); vision.updateFieldTarget(drive,false);
            drive.position=pose(40,0); vision.updateFieldTarget(drive,false);
            check(vision.getSelectedCluster().get()==left,"held R2 intentionally preserves selected CELL");
            vision.setAimSelectionHeld(false); vision.updateFieldTarget(drive,false);
            check(vision.getSelectedCluster().get()==right,"release R2 on positive half selects other CELL");
            vision.setAimSelectionHeld(true); vision.updateFieldTarget(drive,false);
            check(vision.getSelectedCluster().get()==right,"re-press R2 retains newly selected half");
        }
        System.out.println("PASS: three fresh consistent frames, success-only150ms cooldown, continuous failed-write retry, R2 inhibits every write, normal odometry motion, position/heading jumps, wrap, loss/gaps, confidence and production aiming/firing while held; both alliances switch CELL after release across field halves.");
    }
}
