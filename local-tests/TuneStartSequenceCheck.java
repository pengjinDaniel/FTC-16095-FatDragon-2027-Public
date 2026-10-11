import org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence;
import static org.firstinspires.ftc.teamcode.subsystems.drive.TuneStartSequence.Action.*;
public class TuneStartSequenceCheck {
    static void check(boolean ok, String why) { if (!ok) throw new AssertionError(why); }
    static TuneStartSequence armed() {
        TuneStartSequence s=new TuneStartSequence(); s.start(0);
        for(int i=1;i<=150;i++)s.update(i*.02,true,0,0,0,0,0);
        return s;
    }
    public static void main(String[] args) {
        TuneStartSequence s=armed();
        check(s.update(3.02,true,1,0,0,5,0)==WAIT && s.probePower()==0,"CCW motion stops immediately");
        check(s.ccwReferencePower()>=.1,"CCW reference captured");
        int cwStarts=0,scanStarts=0;
        for(int i=152;i<=169;i++)if(s.update(i*.02,true,1,0,0,0,0)==PROBE_CW)cwStarts++;
        check(cwStarts>0 && s.cwReferencePower()==0,"must probe other direction before scan");
        check(s.update(3.4,true,0,0,0,5,0)==WAIT && s.probePower()==0,"CW motion stops immediately");
        for(int i=171;i<=190;i++)if(s.update(i*.02,true,0,0,0,0,0)==START_SCAN)scanStarts++;
        check(scanStarts==1 && s.cwReferencePower()>=.1,"both references plus one fixed scan start");
        check(s.update(3.82,true,0,0,0,0,0)==WAIT,"no automatic rerun");
        s=armed();check(s.update(3.02,true,-2,0,0,2,0)==ABORT,"wrong CCW sign abort");
        s=armed();check(s.update(3.02,false,0,0,0,0,0)==ABORT,"sensor lost abort");
        s=armed();check(s.update(3.02,true,1,5,0,0,0)==ABORT,"drift abort");
        s=armed();check(s.update(3.3,true,1,0,0,0,0)==ABORT,"loop stall abort");
        s=new TuneStartSequence();s.start(0);
        for(int i=1;i<=750;i++)check(s.update(i*.02,true,0,0,0,3,0)==WAIT,"moving must wait");
        check(s.update(15.02,true,0,0,0,3,0)==ABORT,"stationary timeout");
        s=armed();double previousPower=s.probePower();
        for(int i=151;i<=225;i++){
            check(s.update(i*.02,true,0,0,0,0,0)==PROBE_CCW,"CCW probe must ramp");
            check(s.probePower()>=previousPower && s.probePower()<=.25,"bounded monotonic ramp");
            previousPower=s.probePower();
        }
        check(s.peakProbePower()==.25,"maximum probe power reached");
        for(int i=226;i<240;i++)s.update(i*.02,true,0,0,0,0,0);
        check(s.update(4.82,true,0,0,0,0,0)==ABORT && s.probePower()==0,"no turn times out at ceiling");
        s=armed();s.update(3.02,true,1,0,0,0,0);
        for(int i=152;i<=169;i++)s.update(i*.02,true,1,0,0,0,0);
        check(s.update(3.4,true,3,0,0,0,0)==ABORT,"wrong CW sign abort");
        TuneStartSequence.Config bad=new TuneStartSequence.Config();bad.maximumPower=.9;
        s=new TuneStartSequence();s.start(0,bad);
        check(s.update(.02,true,0,0,0,0,0)==ABORT,"unsafe probe settings reject");
        System.out.println("PASS: two-direction probe/reference, bounded ramp, settling, one-shot scan, wrong sign, sensor loss, drift, timeout, loop stall");
    }
}
