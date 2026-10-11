import com.pedropathing.config.ConfigVar;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PedroAlgorithms;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.reflect.Field;

public class PedroConfigCheck {
    static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
    static int readAll(Object config) throws Exception {
        int count = 0;
        for (Field f : config.getClass().getFields()) {
            if (ConfigVar.class.isAssignableFrom(f.getType())) {
                ((ConfigVar<?>) f.get(config)).get(); count++;
            }
        }
        return count;
    }
    public static void main(String[] args) throws Exception {
        MecanumConfig drive = Constants.drivetrainConfig();
        PinpointConfig pinpoint = Constants.localizerConfig();
        PedroAlgorithms.validateConfig("Mecanum", drive);
        PedroAlgorithms.validateConfig("Pinpoint", pinpoint);
        int variables = readAll(drive) + readAll(pinpoint);
        check(drive.frontLeftName.get().equals(DriveConstants.leftFrontMotorName), "front-left name preserved");
        check(drive.frontLeftDirection.get() == DriveConstants.leftFrontMotorDirection, "motor direction preserved");
        check(pinpoint.offsetUnits.get() == DistanceUnit.MM, "pod offset units must remain mm");
        check(pinpoint.globalDistanceUnit.get() == DistanceUnit.INCH, "Pedro distances must be inches");
        check(pinpoint.xPodOffset.get() == -96 && pinpoint.yPodOffset.get() == -74, "measured offsets preserved");
        check(Constants.foresightConfig == null && !Constants.pathFollowingCalibrated,
                "no pretend path calibration deployed");
        check(PedroAlgorithms.create(false, Constants.foresightConfig) != null, "actual production config starts manual algorithm");
        boolean rejected = false;
        try { PedroAlgorithms.validateConfig("Mecanum", new MecanumConfig(c -> {})); }
        catch (IllegalArgumentException e) { rejected = e.getMessage().contains("Mecanum.frontLeftName"); }
        check(rejected, "missing motor ConfigVar must name field");
        rejected = false;
        try { PedroAlgorithms.validateConfig("Pinpoint", new PinpointConfig(c -> {})); }
        catch (IllegalArgumentException e) { rejected = e.getMessage().contains("Pinpoint.name"); }
        check(rejected, "missing Pinpoint ConfigVar must name field");
        System.out.println("PASS: " + variables + " actual Mecanum/Pinpoint ConfigVars resolved; units, measured offsets and directions preserved; missing fields diagnosed");
    }
}
