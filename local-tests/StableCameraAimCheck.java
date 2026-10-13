// Paused at user request on 2026-10-13. Original source retained below.
// import org.firstinspires.ftc.teamcode.subsystems.drive.AimTargetLatch;
// import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledAutoAimController;
// import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledTurnPid;
// import org.firstinspires.ftc.teamcode.subsystems.drive.AimTuneConstants;
//
// public class StableCameraAimCheck {
//     static void check(boolean condition, String message) {
//         if (!condition) throw new AssertionError(message);
//     }
//     public static void main(String[] args) {
//         AimTargetLatch target = new AimTargetLatch();
//         check(target.update(true,false,true,17.7,12.75),"trigger must capture target");
//         double first=target.errorDegrees(.01,0,0,180);
//         target.update(true,false,true,-17.7,12.75);
//         double second=target.errorDegrees(-.01,0,0,180);
//         check(target.x()==17.7 && Math.abs(first-second)<.1,"field-half jitter must not switch physical CELL");
//         target.update(true,false,false,Double.NaN,Double.NaN);
//         check(target.isLatched(),"established coordinate aim survives camera dropout while gyro pose is valid");
//         check(Double.isNaN(target.errorDegrees(0,0,Double.NaN,180)),"invalid gyro pose rejected");
//         target.update(true,true,true,-17.7,12.75);
//         check(target.x()==-17.7,"explicit A refresh may select new target");
//         target.update(false,false,true,17.7,12.75);
//         check(!target.isLatched(),"trigger release clears target");
//
//         ScheduledAutoAimController controller = new ScheduledAutoAimController();
//         check(controller.calculateTurnPower(true,1,0)==0 && controller.isHolding(),"inside tolerance enters hold");
//         for(int i=0;i<200;i++) {
//             double noise=(i%2==0)?2.2:-2.2;
//             check(controller.calculateTurnPower(true,noise,0)==0,"boundary noise must not alternate friction kicks");
//         }
//         check(controller.calculateTurnPower(true,3,0)<0 && !controller.isHolding(),"real deviation releases hold");
//         check(controller.calculateTurnPower(false,3,0)==0 && !controller.isHolding(),"invalid target clears hold");
//         check(controller.calculateTurnPower(true,3,Double.NaN)==0,"invalid gyro rate must stop");
//         double oldSlew=AimTuneConstants.autoAimSlewPerSecond;AimTuneConstants.autoAimSlewPerSecond=100;
//         double ordinary=new ScheduledAutoAimController().calculateTurnPower(true,3,0);
//         double braking=new ScheduledAutoAimController().calculateTurnPower(true,3,40);
//         check(ordinary<0 && braking>ordinary,"CCW gyro rate must provide opposite CW braking");
//         AimTuneConstants.autoAimSlewPerSecond=oldSlew;
//         ScheduledTurnPid.Config c=new ScheduledTurnPid.Config();
//         c.nearP=c.farP=0;c.slewPerSecond=100;
//         ScheduledTurnPid pid=new ScheduledTurnPid();
//         check(pid.calculateWithDerivative(40,.02,c,0)==0,"stationary gyro has zero D");
//         check(pid.calculateWithDerivative(-40,.02,c,0)==0,"target jump must not inject derivative kick");
//         System.out.println("PASS: latched CELL across boundary jitter, explicit refresh/release, 200 noisy hold samples, gyro braking/sign/fault, no target-jump derivative kick");
//     }
// }
