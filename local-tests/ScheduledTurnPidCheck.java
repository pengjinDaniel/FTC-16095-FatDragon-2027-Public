import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledTurnPid;
import org.firstinspires.ftc.teamcode.subsystems.drive.AimGeometry;
public class ScheduledTurnPidCheck {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    public static void main(String[] args) {
        check(Math.abs(AimGeometry.clockwiseError(-10, 0, 180)) < 1e-9, "rear target must align rear shooter");
        check(AimGeometry.clockwiseError(-10, 1, 180) < 0, "rear-right target requires CCW turn");
        check(AimGeometry.clockwiseError(-10, -1, 180) > 0, "rear-left target requires CW turn");
        check(Math.abs(AimGeometry.clockwiseError(10, 0, 0)) < 1e-9, "front shooter compatibility");
        check(Math.abs(AimGeometry.desiredRobotHeadingRadians(-10, 0, 180)) < 1e-9, "field rear target needs zero robot heading");
        check(Double.isNaN(AimGeometry.clockwiseError(0, 0, 180)), "zero range rejected");
        ScheduledTurnPid pid = new ScheduledTurnPid();
        ScheduledTurnPid.Config c = new ScheduledTurnPid.Config();
        c.slewPerSecond = 100;
        double far = pid.calculate(30, .02, c);
        pid.reset(); double near = pid.calculate(3, .02, c);
        check(far > near && far <= c.farMax && near <= c.nearMax, "scheduled limits");
        pid.reset(); check(pid.calculate(-30, .02, c) == -far, "symmetric signs");
        check(pid.calculate(Double.NaN, .02, c) == 0, "NaN must stop");
        check(pid.calculate(30, .3, c) == 0, "stalled loop must stop");
        c.farDegrees = c.nearDegrees;
        check(pid.calculate(30, .02, c) == 0, "bad thresholds must stop");
        c = new ScheduledTurnPid.Config();
        check(Math.abs(pid.calculate(30, .02, c)) <= .030001, "slew limit");
        pid.reset(); c.slewPerSecond = 100;
        pid.calculate(179, .02, c);
        check(pid.calculate(-179, .02, c) < 0, "wrap derivative must not spike wrong direction");
        pid.reset(); double below = pid.calculate(4.999, .02, c);
        pid.reset(); double above = pid.calculate(5.001, .02, c);
        check(Math.abs(above-below) < .001, "smooth near boundary");
        for (double pScale : new double[]{.8,1,1.2}) for (double dScale : new double[]{.5,1}) {
            c = new ScheduledTurnPid.Config();
            c.nearP *= pScale; c.farP *= pScale; c.nearD *= dScale; c.farD *= dScale;
            for (double target : new double[]{15,-15,30,-30}) {
                pid.reset(); double angle=0, speed=0;
                for (int n=0;n<600;n++) {
                    double u=pid.calculate(target-angle,.02,c);
                    check(Double.isFinite(u) && Math.abs(u)<=c.farMax+1e-9,"finite bounded output");
                    // Synthetic turn plant only; not a robot model or real calibration.
                    speed += .02/.12*(u*180-speed); angle += speed*.02;
                }
                check(Math.abs(target-angle)<1.5,"synthetic convergence");
            }
        }
        System.out.println("PASS: scheduling, signs, invalid inputs, slew, wrap, continuity, 24 synthetic trials");
    }
}
