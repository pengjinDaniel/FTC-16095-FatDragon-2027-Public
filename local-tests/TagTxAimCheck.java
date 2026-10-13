package org.firstinspires.ftc.teamcode.subsystems.vision;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.subsystems.drive.LegacyTxAimController;
import org.json.*;
import sun.misc.Unsafe;
import java.lang.reflect.Field;

/** Real SDK frames and controller; no camera, ADB, or motors. */
public class TagTxAimCheck {
    static class Frame extends LLResult {
        Frame(JSONObject data) throws JSONException { super(data); }
        @Override public long getStaleness() { return 0; }
    }
    static JSONObject tag(int id, double tx) throws Exception {
        return new JSONObject().put("fID",id).put("tx",tx).put("ta",.1)
            .put("t6r_ts",new JSONArray(new double[]{0,0,2,0,0,0}));
    }
    static Frame frame(int pipeline, double timestamp, JSONObject... tags) throws Exception {
        JSONArray list=new JSONArray(); for(JSONObject tag:tags) list.put(tag);
        return new Frame(new JSONObject().put("v",1).put("pID",pipeline).put("ts",timestamp)
            .put("tx",80).put("Fiducial",list));
    }
    static void check(boolean ok,String msg) { if(!ok) throw new AssertionError(msg); }
    static void near(double a,double b,String msg) { check(Math.abs(a-b)<1e-6,msg+": "+a); }
    public static void main(String[] args) throws Exception {
        TagTxVision.Observation a=TagTxVision.select(frame(0,1,tag(31,10),tag(35,-3),tag(39,5),tag(43,-8),tag(30,0)));
        check(a.id==35,"smallest absolute tx of allowed four, not list first or disallowed center");
        near(a.tx,-3,"per-tag tx instead of aggregate tx"); near(a.distanceInches,78.7402,"same-tag distance");
        check(a.hive==HiveState.UNKNOWN,"tx does not require a 3D field pose; missing 3D cannot permit feeding");
        check(TagTxVision.select(frame(0,2,tag(35,-3),tag(43,1))).id==43,"switch immediately when another allowed tag is closer");
        check(TagTxVision.select(frame(0,3,tag(43,2),tag(31,-2))).id==31,"deterministic equal-angle tie");
        for(int id:new int[]{31,35,39,43}) check(TagTxVision.select(frame(0,4,tag(id,4))).id==id,"each of four works alone");
        check(TagTxVision.select(frame(0,5,tag(30,0),tag(32,1),tag(38,2)))==null,"reject other IDs");
        check(TagTxVision.select(frame(0,6,tag(31,3),tag(31,4)))==null,"duplicate selected ID rejected");
        JSONObject missing=tag(39,0); missing.remove("tx");
        check(TagTxVision.select(frame(0,7,missing))==null,"missing tx must not become aligned zero");
        Frame mixed=frame(0,8,tag(31,10),tag(35,-3),tag(39,1),tag(43,-2));
        check(TagTxVision.select(mixed,TagTxVision.Alliance.RED).id==35,"red ignores closer blue");
        check(TagTxVision.select(mixed,TagTxVision.Alliance.BLUE).id==39,"blue selects closest blue");
        check(TagTxVision.select(frame(0,9,tag(31,0),tag(35,1)),TagTxVision.Alliance.BLUE)==null,"blue never falls back to red");
        check(TagTxVision.select(frame(0,10,tag(39,0),tag(43,1)),TagTxVision.Alliance.RED)==null,"red never falls back to blue");
        check(TagTxVision.select(frame(0,11,tag(39,4),tag(43,-1)),TagTxVision.Alliance.BLUE).id==43,"same-alliance live switch");
        Field u=Unsafe.class.getDeclaredField("theUnsafe"); u.setAccessible(true);
        TagTxVision vision=(TagTxVision)((Unsafe)u.get(null)).allocateInstance(TagTxVision.class);
        Field mode=TagTxVision.class.getDeclaredField("alliance"); mode.setAccessible(true);
        mode.set(vision,TagTxVision.Alliance.RED);
        vision.acceptFrame(mixed,true,800_000_000L);
        check(vision.getTagId()==35 && vision.getTargetAlliance().equals("红方"),"live red selection and output");
        check(vision.getVisibleAlliance().equals("红方 + 蓝方"),"report both visible colors despite selection filter");
        vision.acceptFrame(frame(0,12,tag(39,0)),true,900_000_000L);
        check(!vision.hasValidAimingTarget() && vision.getVisibleAlliance().equals("蓝方"),"opponent visible but never aimed");
        mode.set(vision,TagTxVision.Alliance.BLUE);
        Frame fresh=frame(0,100,tag(39,5));
        vision.acceptFrame(fresh,true,1_000_000_000L); check(vision.hasValidAimingTarget(),"fresh tx works without field position");
        vision.acceptFrame(fresh,true,1_201_000_000L); check(!vision.hasValidAimingTarget(),"frozen physical frame stops aim"); check(vision.getVisibleAlliance().equals("无可用目标"),"stale visibility cleared");
        vision.acceptFrame(frame(1,101,tag(39,5)),true,1_202_000_000L); check(!vision.hasValidAimingTarget(),"wrong pipeline rejected");
        vision.acceptFrame(frame(0,102,tag(39,5)),false,1_203_000_000L); check(!vision.hasValidAimingTarget(),"camera disconnect stops aim");
        double oldD=LegacyTxAimController.kD_alignH, oldAlpha=LegacyTxAimController.txFilterAlpha;
        try {
            LegacyTxAimController.kD_alignH=0; LegacyTxAimController.txFilterAlpha=1;
            LegacyTxAimController pid=new LegacyTxAimController();
            near(pid.calculateTurnPower(true,10,30,31),.12,"positive tx commands positive clockwise power, near gain");
            pid.reset(); near(pid.calculateTurnPower(true,-10,120,35),-.3,"negative tx commands left, far gain");
            pid.reset(); near(pid.calculateTurnPower(true,10,70,39),.21,"old distance interpolation");
            pid.reset(); near(pid.calculateTurnPower(true,2,30,39),0,"near deadband"); check(pid.isLocked(),"lock");
            near(pid.calculateTurnPower(true,4,30,39),0,"hysteresis stays aligned");
            check(pid.calculateTurnPower(true,5,30,39)>0 && !pid.isLocked(),"leaves hysteresis");
            LegacyTxAimController.txFilterAlpha=.3;
            pid.calculateTurnPower(true,20,30,39);
            check(pid.calculateTurnPower(true,-10,30,43)<0,"tag switch resets old filter and PID");
            near(pid.getFilteredTx(),-10,"no old target blended into new tag");
            near(pid.calculateTurnPower(false,0,30,43),0,"lost target stops"); check(!pid.isLocked(),"lost target clears lock");
        } finally { LegacyTxAimController.kD_alignH=oldD; LegacyTxAimController.txFilterAlpha=oldAlpha; }
        System.out.println("PASS: alliance filtering, visible-color telemetry, no opponent fallback; four tags; minimum absolute per-tag tx; immediate switching; poisoned global tx isolation; invalid/missing/duplicate tags; frozen/disconnected/wrong-pipeline rejection; old PID direction/distance/filter/deadband/hysteresis; target-switch reset and lost-target stop.");
    }
}
