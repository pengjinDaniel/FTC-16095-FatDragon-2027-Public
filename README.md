# FTC 16095 FatDragon — 2026–2027 BIOBUZZ

Official code repository for FTC Team 16095 FatDragon during the 2026–2027 BIOBUZZ season.

## Current Setup

- FTC SDK 12.0
- Java
- Android Studio
- FTCLib Command-Based framework

## Project Structure

- `subsystems/` — robot subsystems
- `commands/` — command-based robot actions
- `tests/` — test OpModes

## Current Progress

- [x] Basic mecanum drivetrain
- [x] Command-based drivetrain control
- [x] Pinpoint integration (odometry accuracy and position hold need robot validation)
- [x] Limelight field localization and CELL aiming (field validation pending)
- [x] Intake, transit, shooter, and flower controls (sensor and mechanism tuning needed)

## Controls

| Control | Competition TeleOp | Limelight Aim Test Blue / Red |
| --- | --- | --- |
| Left stick | Robot-relative translation, including while RT is held | Same |
| Right stick X | Manual turn when RT is released | Same |
| RT | Aim at the CELL in the robot's field half, prepare the shooter, and feed when ready | Aim only |
| Release left stick while holding RT | Pinpoint holds the current position when position hold is calibrated | Same |
| LT | Intake | Unused |
| LB | Reverse intake and transit to clear a jam | Unused |
| RB | Toggle the flower | Unused |
| A | Unused | Apply a fresh corrected vision pose to Pinpoint |
| B | Correct Pinpoint from vision | Stop until B and RT are released |

Movement and position error do not directly block feeding. A valid HIGH target,
aim lock, shooter readiness, pitch settling, and calibrated limit-servo positions
are still required.

## Field Coordinate CELL Aiming

Blue uses Limelight pipeline 0; red uses pipeline 1. Both pipelines use their
alliance-only BIOBUZZ_HiveFlat fmap. The map is a flat-HIVE reference, not a HIGH
CELL map. Full 3D targeting must be enabled and Snap Robot to Floor must be No.

`Vision` reads MegaTag1 `getBotpose()` and corrects the flat-map observation for HIVE
rotation about the field Y axis. The correction uses the robot-on-floor constraint
and checks the corrected robot-up vector against a level tolerance. Invalid frames,
mixed-alliance tags, wrong pipelines, stale results, fewer than two usable tags,
and failed corrections never overwrite Pinpoint. No uncorrected MT1 fallback is used.

The field origin is the center of the field. +X is the audience half; -X is the
opposite half. Runtime distances are inches and headings are radians internally.
The user selected Team 19725's target points (X = +/-17.7 in, Y = +/-12.75 in) and
pivot height (43.95 in). These are reference values pending field validation.

Tests initialize Pinpoint from the first valid corrected vision pose during INIT,
without a multi-frame initialization filter. TeleOp initializes from its first valid
corrected frame after START. This follows the China Championship single-observation
calibration approach; its turret transforms and autoEndPose are not copied. RT applies
new valid vision corrections and aims using Pinpoint. At other times the tests leave
Pinpoint tracking intact so odometry drift can be compared against vision. After
localization, losing sight of Tags does not stop coordinate aiming; shooting still
requires a current corrected observation accepted by the localization layer for the
selected HIGH CELL. Moving and turning do not disable vision corrections. A new
physical frame must arrive within 200 ms; repeated HTTP replies do not refresh this
camera-frame deadline. Pinpoint faults
invalidate field initialization. `DriveConstants.pinpointConfigurationConfirmed`
must be true only after the wheel type, pod offsets and directions are confirmed.

First localization accepts a valid corrected pose directly. Subsequent corrections
use initial bring-up gates of 12 in position difference and 30 deg wrapped heading
difference. Three consecutive rejected new usable vision frames force the latest
corrected pose to overwrite Pinpoint. Duplicate frames do not advance this count;
invalid or stale observations reset the sequence and cannot force an overwrite.
These gates are software test defaults, not measured localization accuracy. Telemetry
shows raw/solve/accepted flags, both tilt roots and their level errors, MT1 quality,
frame age, correction differences, rejection count, and forced-overwrite status.

CELL selection follows the robot's field X half, with X = 0 assigned to the audience
half. The desired robot heading is `atan2(targetY-robotY, targetX-robotX)` minus the
rear-facing shooter's 180-degree offset. The wrapped heading error feeds the
Prototype2026 radian PIDF gains. Translation remains robot-relative.

## Aim Test Bring-Up

Use `Limelight Aim Test Blue` or `Limelight Aim Test Red`. These OpModes initialize
only the drivetrain, Pinpoint and Limelight; they do not run the shooter or feeders.
The automatic test turn is limited to 0.15 power. B stops motion until RT is released.

1. During INIT, confirm expected/actual pipeline, at least two usable MT1 tags,
   corrected pose status, Pinpoint READY, and Field Pose Initialized. Compare corrected
   X/Y/heading against several measured robot positions. Raw MT1 XYZ and HIVE angle
   remain visible for flat/tilted comparisons.
2. START with RT released. Push or drive along known distances and rotate to compare
   Pinpoint against vision. A applies a fresh vision calibration; no correction is
   applied automatically during this odometry check.
3. Move into each field X half and confirm the selected CELL and target XY. Check both
   alliances. Keep RT released while checking selection.
4. From a small heading error, hold RT briefly and verify the error decreases. Stop
   with B if the turn direction is wrong. Tune sign/gains only after confirming axes.
5. Repeat with the HIVE flat and tilted in both directions. Geometrically invalid
   and stale observations must not move Pinpoint; HIGH/LOW states must match the
   physical CELL positions. An outlier within the usable corrected-pose stream is
   skipped twice and overwritten on the third rejected new frame, per user policy.

Source: Team19725_BIOBUZZ `MT1Localizer.java`, `OneMap_MT1Localizer.md`, and
`HypParams.java`; Prototype2026-Public `VisionMecanumDrive.getAlignTurnPower()`.

## Notes

This repository is under active development for the 2026–2027 FTC season.

Shooter velocity control and automatic firing remain disabled until the 2027-specific
PIDF, readiness thresholds, distance tables, and servo positions are measured and
calibrated, and the camera pose and selected field coordinates are verified on the robot.
Autonomous routines are not implemented yet.
