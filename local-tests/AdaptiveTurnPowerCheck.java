// Paused at user request on 2026-10-13. Original source retained below.
// import org.firstinspires.ftc.teamcode.subsystems.drive.AdaptiveTurnPower;
// import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledTurnPid;
//
// public class AdaptiveTurnPowerCheck {
//     static void check(boolean condition, String message) {
//         if (!condition) throw new AssertionError(message);
//     }
//     public static void main(String[] args) {
//         // Actual failed-trial endpoint errors from turn-1778143209756.csv.
//         double[] stalledErrors = {9.8217,-8.1242,8.7714,-8.3438,2.4879,-1.7989,
//                 6.4562,-5.9255,-1.8486,1.8784,6.2125,-5.8748,-1.7045,1.7985,
//                 3.9633,-3.3461,-1.5714,4.0276,-3.1798,-2.5676};
//         for (double error : stalledErrors) {
//             ScheduledTurnPid.Config c=new ScheduledTurnPid.Config();
//             c.nearMax=.30;c.farMax=.35;c.stopToleranceDegrees=1.5;
//             c.positiveStaticPower=c.negativeStaticPower=.205*.8;
//             AdaptiveTurnPower adaptive=new AdaptiveTurnPower();
//             ScheduledTurnPid pid=new ScheduledTurnPid();
//             boolean recoveredEffort=false;
//             for(int i=0;i<150;i++) {
//                 adaptive.update(error,0,.02,c);
//                 double power=pid.calculate(error,.02,c);
//                 check(Math.abs(power)<=.35,"power ceiling must hold");
//                 check(Math.signum(power)==Math.signum(error),"compensation direction");
//                 if(Math.abs(power)>=.205)recoveredEffort=true;
//             }
//             check(recoveredEffort,"stalled endpoint must regain observed startup effort: "+error);
//             check(c.positiveStaticPower<=.25 && c.negativeStaticPower<=.25,"static ceiling");
//             check(pid.calculate(1,.02,c)==0,"entering tolerance stops immediately");
//         }
//         ScheduledTurnPid.Config c=new ScheduledTurnPid.Config();c.nearMax=.30;c.farMax=.35;c.stopToleranceDegrees=1.5;
//         AdaptiveTurnPower a=new AdaptiveTurnPower();
//         for(int i=0;i<100;i++)check(!a.update(10,5,.02,c),"moving robot must not receive stall boost");
//         check(c.positiveStaticPower==0,"no unnecessary boost while moving");
//         for(int i=0;i<100;i++)check(!a.update(1,0,.02,c),"within tolerance must not boost");
//         check(!a.update(Double.NaN,0,.02,c),"invalid feedback must not boost");
//         for(int i=0;i<1000;i++)a.update(10,0,.02,c);
//         check(c.positiveStaticPower==.25 && c.negativeStaticPower==0,"bounded, separate direction compensation");
//         System.out.println("PASS: all 20 observed stationary endpoints regain startup effort within bounds; direction, tolerance stop, no boost while moving, invalid feedback");
//     }
// }
