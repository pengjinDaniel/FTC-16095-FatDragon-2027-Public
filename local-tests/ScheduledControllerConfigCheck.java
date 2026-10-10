import org.firstinspires.ftc.teamcode.subsystems.drive.AimTuneConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.ScheduledAutoAimController;

public class ScheduledControllerConfigCheck {
    static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
    public static void main(String[] args) {
        double slew = AimTuneConstants.autoAimSlewPerSecond;
        AimTuneConstants.autoAimSlewPerSecond = 100;
        double ccw = new ScheduledAutoAimController().calculateTurnPower(true, 3);
        double cw = new ScheduledAutoAimController().calculateTurnPower(true, -3);
        check(Math.abs(ccw + (.012 * 3 + .115504)) < 1e-6,
                "exported physical CCW compensation must map to negative Drive power");
        check(Math.abs(cw - (.012 * 3 + .091183)) < 1e-6,
                "exported physical CW compensation must map to positive Drive power");
        check(new ScheduledAutoAimController().calculateTurnPower(true, 1) == 0,
                "measured stop tolerance must stop camera aiming");
        check(Math.abs(new ScheduledAutoAimController().calculateTurnPower(true, 90)) <= .35,
                "camera controller must preserve measured far power ceiling");
        AimTuneConstants.autoAimSlewPerSecond = slew;
        ScheduledAutoAimController controller = new ScheduledAutoAimController();
        check(controller.calculateTurnPower(true, 20) < 0, "CCW error requires CW-negative power");
        check(controller.calculateTurnPower(false, 20) == 0 && !controller.isLocked(), "invalid target clears output/lock");
        double enter = DriveConstants.aimLockToleranceDegrees;
        DriveConstants.aimLockToleranceDegrees = Double.NaN;
        check(controller.calculateTurnPower(true, 20) == 0 && !controller.isLocked(), "NaN lock tolerance must stop");
        DriveConstants.aimLockToleranceDegrees = enter;
        double exit = DriveConstants.aimUnlockToleranceDegrees;
        DriveConstants.aimUnlockToleranceDegrees = enter - 1;
        check(controller.calculateTurnPower(true, 20) == 0, "reversed lock thresholds must stop");
        DriveConstants.aimUnlockToleranceDegrees = exit;
        double stable = DriveConstants.aimLockStableTimeSeconds;
        DriveConstants.aimLockStableTimeSeconds = -1;
        check(controller.calculateTurnPower(true, 20) == 0, "negative dwell must stop");
        DriveConstants.aimLockStableTimeSeconds = stable;
        double p = AimTuneConstants.autoAimFarKp;
        AimTuneConstants.autoAimFarKp = Double.POSITIVE_INFINITY;
        check(controller.calculateTurnPower(true, 20) == 0 && !controller.isLocked(), "invalid scheduled gain must stop");
        AimTuneConstants.autoAimFarKp = p;
        AimTuneConstants.autoAimScheduledPidEnabled = false;
        double legacyP = DriveConstants.autoAimKp;
        DriveConstants.autoAimKp = Double.NaN;
        check(controller.calculateTurnPower(true, 20) == 0 && !controller.isLocked(), "invalid legacy gain must stop");
        DriveConstants.autoAimKp = legacyP;
        check(controller.calculateTurnPower(true, 20) < 0, "upstream negative-P radian convention remains supported");
        AimTuneConstants.autoAimScheduledPidEnabled = true;
        System.out.println("PASS: actual FTCLib/SDK controller; valid signs, invalid targets, missing/invalid gains and lock settings stop without false lock");
    }
}
