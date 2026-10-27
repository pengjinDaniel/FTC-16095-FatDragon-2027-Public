# Pedro 3 autonomous setup

This package uses Pedro revhub/core 3.0.2 and tuning 1.0.1. The four procedures are unchanged copies from [official Quickstart revision 9dc8978](https://github.com/Pedro-Pathing/Quickstart/tree/9dc89787898fab6b5cd9095df38489a14bf51a95/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures), under the repository's FIRST license.

1. Deploy the 2027 project through Android Studio.
2. Connect to the Robot Controller Wi-Fi and open [AutoTune](http://192.168.43.1:10158).
3. Run **Mecanum Tuner**, then **Pinpoint Tuner**. Replace the corresponding complete declarations in `Constants.java` with their Java exports, then rebuild and deploy. The initial motor names, directions, and Pinpoint settings come from the current robot's `DriveConstants`.
4. Run **Foresight Tuner**. Replace `foresightConfig = null` with its complete Java export. The required imports are already present. Rebuild and deploy.
5. Run **Tests**, especially Localization, Line, Curve, and Hold. Then select autonomous **test** in the **Auto** group.

AutoTune procedures launch their measurement OpModes through the web interface. Follow the procedure's Start/Stop prompts with room for its specified motions. The exports do not edit the source automatically.

The autonomous preserves all seven supplied paths. Its 90-degree starting pose and 270-degree path-1 heading are preserved exactly; verify the starting orientation before running. Missing Foresight configuration keeps the autonomous stationary with an INIT message.

The route uses the 0–144-inch Pedro frame. `AutoHiveTarget` translates it by (-72, -72) into the existing custom HIVE fmap frame, with the same axes and heading. Red uses Limelight pipeline 1 and the red flat-HIVE fmap. This route targets the red opposite-audience CELL (tags 30–33); vision never resets Pedro's localizer.

Path 2 and its 500 ms endpoint wait use the same intake/transit requests as LT. Path 5 also uses LT throughout. Paths 1, 4, and 6 are followed by 900 ms of forward feeding, after shooter/Pitch readiness and the existing transit reverse pulse. Only path 4 waits for a fresh HIGH observation. LOW, MIDDLE, and UNKNOWN keep it waiting until HIGH or OpMode stop. Once forward feeding starts, its 900 ms shot is committed, matching the existing transit behavior.

The host regression in `local-tests/TestAutoCheck.java` exercises the real FTCLib command groups, transit timers, and HIVE solver with synthetic sensors. It does not validate physical tracking, shooting calibration, or the uploaded Limelight configuration.
