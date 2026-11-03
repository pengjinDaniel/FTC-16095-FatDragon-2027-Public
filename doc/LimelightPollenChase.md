# Limelight pipeline 2 handheld POLLEN chase test

## Behavior

Select **Limelight Pollen Chase** in the **Test** group. INIT displays
measurements without driving. START automatically follows a visible yellow
POLLEN held in the camera's view. At an estimated **3 cm horizontal gap from
the robot's approach-side edge to the nearest ball surface**, it stops and
stays stopped until the OpMode is restarted.

For the reported **camera yaw 180 degrees**, the approach side is the **rear**
of the robot and approaching produces **reverse drive power**. The robot keeps
the rear camera pointed at the ball. No gamepad input, intake, shooter,
Pinpoint, AprilTag localization, search rotation, or blind coasting is used.

Lost targets, wrong pipeline, missing/clipped contours, data older than 100 ms,
and frozen physical frames stop the motors. Three distinct valid frames are
required before driving or resuming after target loss.

## Mount values and distance reference

The constants at the top of `PollenChaseTest.java` now use **upward-positive
pitch**, matching the user's installation:

| Constant | Value / meaning |
| --- | --- |
| `CAMERA_FORWARD_CM` | **-12.715**, converted from LL x = -0.127150 m. Robot +X is intake-forward. |
| `CAMERA_RIGHT_CM` | **0.0**, an assumption to verify against LL Right/y. |
| `CAMERA_HEIGHT_CM` | **20.7415**, converted from LL z = 0.207415 m. |
| `CAMERA_PITCH_UP_DEGREES` | **+50.0**; upward is positive. |
| `CAMERA_YAW_DEGREES` | **180.0**; the camera faces toward the rear. This test supports yaw 0 or 180 and an upright, unrolled image. |
| `APPROACH_EDGE_FROM_CENTER_CM` | **17.0**, the user-reported robot-center to rear-most edge distance, in cm. It is not the 12.715 cm lens offset. |
| `RANGE_SCALE` | **1.0** initially; a correction multiplier for measured size-based range. |

The distance setpoint is a **horizontal chassis-to-ball gap**, not the slant
range from the lens to the ball. The telemetry shows both separately.
A ball held higher can have a much greater camera range while having the same
3 cm horizontal gap.

Correct LL x/z/pitch/yaw installation values in pipeline 0/1 can remain as they
are. This test has its own constants and does not read its extrinsics from
the AprilTag localization output. No physical downward tilt is required for
this handheld-ball test. The ball must remain fully visible throughout the
approach; lower it as needed near the robot if it approaches the top of view.

## Ranging without assuming the ball is on the floor

The POLLEN diameter is approximately **7.1 cm**. The code obtains the selected
yellow target's raw contour corners, measures its pixel width and height, and
uses the angular silhouette of a known-size sphere to estimate camera range.
It then applies the camera's **+50-degree upward pitch**, **180-degree yaw**,
and translation to obtain the ball position relative to the chassis.

There is **no fixed target-height plane**. The ball's height is an estimated
output, not an assumed 3.55 cm value. The initial focal lengths come from the
Limelight 3A's nominal full-frame 54.5-degree horizontal and 42-degree vertical
field of view. Keep resolution at 640 x 480 and zoom at 1x.

Both projected diameters should give similar range estimates; inconsistent,
tiny, nonfinite or image-clipped contours are rejected. When the dimensions
differ moderately, the nearer estimate is used. Color-only detection still
cannot identify every yellow object semantically. Use actual POLLEN, expose
its whole outline, and keep fingers and yellow background objects from
obscuring or merging into the contour.

Nominal optics, lens distortion, imperfect ball shape, contour thresholding,
and braking all affect the final gap. Check the distance with a ruler before
a moving test. If range needs a scale correction, start with `RANGE_SCALE=1`,
place the ball at a known **lens-to-ball-center** distance, and set:

```text
RANGE_SCALE = measuredLensToBallCenterCm / reportedLensToBallCenterCm
```

Keep camera settings fixed afterward and verify at another distance.
When measuring to the nearest ball surface along the line of sight, add
3.55 cm to obtain the ball-center distance.

## Limelight web interface: pipeline 2

Open `http://limelight.local:5801` for configuration. Create/edit pipeline
**index 2**; do not change existing pipeline 0/1 settings.

| Setting | Initial value or action |
| --- | --- |
| Pipeline name | `Pollen Chase` |
| Type | Color / Retroreflective color thresholding. |
| Resolution / zoom | **640 x 480 / full frame / 1x**; image upright. Pixel-size ranging requires this exact resolution. |
| Exposure / gain / white balance | Fix values that show the yellow ball clearly without washing out its outline. Retune HSV after changing camera settings. |
| Hue | Start **20-40** on a 0-179 scale (about 40-80 degrees on a 0-360 scale). |
| Saturation | Start **80-255** on a 0-255 scale. |
| Value | Start **60-255** on a 0-255 scale. |
| Hue inversion | Off. |
| View while tuning | Threshold view. Keep one whole yellow ball contour and reject the background/red/blue balls. |
| Erosion / dilation | Start off. Use only slight adjustments; strong morphology changes apparent size and therefore range. |
| Target area | Start **0.03%-30%**. |
| Aspect ratio | Start **0.7-1.4**. |
| Fullness | Start **30%-100%**; POLLEN has holes. |
| Direction filter | Disabled/unrestricted. |
| Target grouping | Single / no grouping. Never merge multiple balls for size-based ranging. |
| Sort mode | Largest area / largest target. Use one ball for the first test. |
| Output targeting region | **Center** of the bounding rectangle. |
| Output: Send Raw Corners | **Yes / enabled. Required by this version.** Missing corners prevent movement. |
| Contour simplification | Low enough to preserve the ball outline and at least four points. |
| Crosshair | Neutral/centered. The code uses principal-pixel angles. |
| Compute 3D / SolvePnP | Off; no field map, Python script or neural model is needed. |

These thresholds are starting values. Use the eyedropper/Add/Subtract tools
with real POLLEN under the actual lighting. Raw corners are read through FTC's
HTTP SDK; "Send JSON over NT" is not required.

## First robot check

1. Confirm the entered **17 cm center-to-rear-edge** measurement and camera Right/y.
2. Configure pipeline 2, especially **640 x 480**, **1x**, **Center targeting**
   and **Send Raw Corners = Yes**.
3. INIT: verify contour dimensions, estimated ball height, camera range, and
   rear-to-ball horizontal gap against measurements at multiple held heights.
   INIT never applies the computed motor powers.
4. With wheels lifted, START and verify a centered ball commands reverse
   wheel motion. A ball moved to the rear camera's image-right should turn
   the chassis so the rear points toward it.
5. On a clear level floor, hold a whole POLLEN in view and START. Keep the ball
   visible as the robot gets close; if it leaves the frame, the robot stops.
   Measure the actual stopped gap, and restart the OpMode for another chase.

Initial limits are **0.50 translation** and **0.18 rotation**, with a **0.07**
friction floor. Only `PollenChaseController` contains the chase gains.
The 3 cm value is a setpoint requiring physical validation, not a guarantee
of zero stopping error. Limelight pipeline 2 is used throughout this test;
HIVE localization is not running simultaneously in this OpMode.

## Local verification

```sh
pollen_check_dir=$(mktemp -d /tmp/pollen-chase-check.XXXXXX)
javac -d "$pollen_check_dir" \
  TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/vision/PollenChaseController.java \
  local-tests/PollenChaseCheck.java
java -cp "$pollen_check_dir" org.firstinspires.ftc.teamcode.subsystems.vision.PollenChaseCheck
./gradlew --offline :TeamCode:assembleDebug
```

The host check renders perspective silhouettes from known 3D sphere positions
to verify arbitrary held heights, upward pitch, yaw-180 reversing, steering
signs, lateral camera offsets, the 3 cm stop latch, invalid contours and
lost/stale/frozen frames. It does not validate actual segmentation or braking.

## References

- [Limelight JSON results and raw contour corners](https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification)
- [Limelight contour filtering and output targeting](https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/contour-filtering-and-sorting)
- [Limelight 3A optics](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [Hive-Vision camera-relative chase reference](https://github.com/hivevisionftc/Hive-Vision/blob/59f65a4373f5817370361d751294ae7a47088181/ftc_ball_chase_lib/final/BallChaseController.java)
- [Limelight author's Chief Delphi color-ball tracking guidance](https://www.chiefdelphi.com/t/limelight-2022-1-critical-fix-first-cargo-tracking-improvement-python-updates/401351)
