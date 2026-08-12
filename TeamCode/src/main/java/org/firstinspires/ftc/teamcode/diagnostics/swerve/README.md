# Swerve Bring-Up Dashboard

A driver-station tool for wiring, diagnosing and calibrating the coaxial swerve drivetrain,
served from the Control Hub the same way FTC Dashboard is.

## Using it from another computer

The dashboard is served by the Robot Controller, not by your laptop, so **there is nothing to
install to use it**. Anyone can:

1. Join the Control Hub's WiFi (SSID usually `FTC-####`)
2. Open `http://192.168.43.1:8080/swerve`

That works from any laptop, phone or tablet with a browser, with no Android Studio, no SDK and no
checkout. Several people can have it open at once.

Only someone **deploying new code** needs the toolchain:

1. Clone the repo and open it in Android Studio (which writes the root `local.properties` itself).
   The included Pedro build needs one too; `settings.gradle` generates it automatically from the
   root, so a fresh clone builds without hand-editing anything.
2. Connect to the hub: `adb connect 192.168.43.1:5555`
3. `./gradlew :TeamCode:installDebug` — or Android Studio's **Run** button.

Use **Run**, not **Rebuild Project**: rebuild asks every module for `testClasses`, which drags an
npm install and a Svelte build through `tools/pedro-visualizer` for a change that only touches
TeamCode.

## Running without a Driver Station

A DS is normally required twice: to author the hardware configuration, and to start OpModes.
Neither is actually necessary.

1. **Write the config from this page.** `SwerveWebApp` registers on the Robot Controller's web
   server at *app startup*, not from an OpMode, so `http://192.168.43.1:8080/swerve` is reachable
   even with no configuration at all. The **Robot configuration** section at the top generates the
   XML, writes it with the SDK's own `RobotConfigFileManager`, and activates it.

   The XML is parsed back with the SDK's `ReadXMLFileHandler` *before* activation, so a malformed
   config is refused rather than left active and broken. It is written as a new file, so an
   existing configuration is never overwritten.

2. **Restart the robot** with the **Restart robot app** button next to the configuration section.
   The hardware map is only rebuilt on restart, so a new config does nothing until you do this.

   There is no restart control on the Robot Controller's own Manage page
   (`/manage.html` — and note the extension, since `/manage` returns HTTP 500 `Mime type unknown`).
   Restarting the robot is a Driver Station menu item, which is precisely what a team without a DS
   cannot reach; hence the button here, which calls `AppUtil.restartApp`.

   The equivalent over adb, if you prefer:

   ```bash
   adb shell am force-stop com.qualcomm.ftcrobotcontroller
   adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
   ```

3. **Start the OpMode from FTC Dashboard.** `http://192.168.43.1:8080/dash` lists OpModes and can
   init/start/stop them. The dependency is already in `TeamCode/build.gradle`.

Prefer to configure by hand instead? **Copy XML** gives you the same file to
`adb push … /sdcard/FIRST/`. Note that pushing the file alone is not enough — the Robot Controller
tracks the active configuration separately, which is what the Write & activate button sets for you.

An Android emulator is *not* a good fallback: the DS app needs to join the Control Hub's WiFi
access point or use WiFi Direct, and an emulator NATs through the host and can do neither. If you
want a real DS, any spare Android phone with the Driver Station app from the Play Store works.

## Opening it

1. Deploy the app and select the **`Swerve Bring-Up`** OpMode (Diagnostics group) on the driver
   station. Do **not** press START yet.
2. Join a laptop to the robot's WiFi and open **<http://192.168.43.1:8080/swerve>**.

The dashboard is fully live during INIT: encoder voltages and angles update, and you can inspect
wiring. **Nothing moves until START is pressed** — motion commands are refused before then.

Everything also works from gamepad 1 if the network is down:

| Control | Action |
| --- | --- |
| dpad up / down | select pod |
| A | pulse the selected drive motor |
| B | run the wiring scan |
| X | capture forward zero for the selected pod |
| Y | run the encoder sweep |
| bumpers | jog the selected pod |
| START (button) | stop everything |

> **Put the robot on blocks before running any routine.** Every scan moves the drivetrain.

## Live swerve view

A top-down diagram of the robot, forward pointing up, sitting above the guided steps. Each pod is
drawn at its real position from `podX`/`podY`, showing:

- **Actual** (solid blue) — where the wheel is really pointing, derived from the encoder by
  inverting `CoaxialPod.adjustThetaForEncoder`.
- **Commanded** (dashed yellow) — where the kinematics are telling it to point. Only present while
  a drive or PID step is active; with `IGNORE_ANGLE_CHANGES` an idle drivetrain commands nothing.
- **Tracking error** — a hatched wedge spanning the two angles, labelled in degrees, turning red
  past 10°. Error is never signalled by color alone.
- Drive power as an arrow along the wheel axis, and the chassis translation/rotation command in
  recessive ink at the center.

This is the fastest way to spot a bad pod: forward should point every wheel up, strafe should point
them all sideways, and rotation should form the classic X. A pod whose solid wheel does not sit on
its dashed target is not tracking — suspect its zero, its encoder pairing, or its PIDF.

The two series colors were validated for colorblind separation against the page surface
(ΔE 27+, well clear of the ΔE 8 floor), and are distinguished by fill style as well as hue.

## Recommended order

Work top to bottom through the guided steps on the page.

### 1. Identify wiring & pair encoders

Turns each pod one at a time while watching all four analog channels, and determines two things
that are otherwise guesswork:

- **Which encoder belongs to which pod.** The Axon MINI+ feedback wires are spliced two per
  analog port, so a swapped pair is easy to create and nearly invisible later — the robot simply
  drives wrong. A pod whose encoder is remapped shows a red `remapped → seN` badge.
- **Servo direction.** `CoaxialPod`'s turn PID only converges if positive servo power makes the
  encoder reading *increase*. That relationship is measured, not assumed.

The scan reports problems rather than silently guessing: a pod whose encoder never responds, two
pods claiming the same channel, or an ambiguous match all appear as warnings.

### 2. Measure encoder range

Spins each pod for four seconds and records the true min/max voltage of the Axon output. These
become `analogMinVoltage` / `analogMaxVoltage`.

### 3. Set rotation convention

The one value that cannot be measured from inside the robot: whether a rising encoder reading means
counter-clockwise rotation viewed from above. Spin a pod, watch it, answer once. This sets
`encoderReversed`, and is normally the same for all four pods.

### 4. Capture forward zero

Servos go limp whenever no routine is running, so pods can be turned by hand. Point every wheel
straight forward with the bevel gears facing the same way, then capture. This sets
`angleOffsetRad`. Use the nudge buttons for a pod that is stiff.

### 5. Tune the turn PIDF

Steps the selected pod through the **real `CoaxialPod` control path**, so what you tune here
transfers directly to competition code with no reimplementation to drift out of sync. The chart
plots heading error in degrees against time, with a ±2° band.

Raise `kF` until the pod just overcomes stiction, then `kP` for speed, then `kD` to damp overshoot.
Pods are rebuilt on every coefficient change, so there is no recompile between attempts.

### 6. Verify kinematics

Drives a real Pedro `Swerve` object at low power. Forward should point all wheels forward; rotate
should form the classic X. If one wheel drives backward against the others, flip that pod's
`drive REV` flag on its card.

Drive commands are watchdogged: if the browser tab closes, the laptop sleeps or WiFi drops, the
robot stops within 400 ms.

### 7. Export

**Generate constants** produces a paste-ready block for `SwerveDrivetrainConstants.java` matching
the existing file's style and imports. The generated source is verified to compile.

## Where the hardware configuration lives

There are two copies, and they serve different purposes.

**`TeamCode/src/main/res/xml/swerve_bringup.xml` — the baseline, shipped in the APK.**
The SDK discovers any `res/xml` file rooted at `<Robot type="FirstInspires-FTC">` and offers it as a
read-only configuration. That makes it the recovery path: it is version controlled with the code,
reinstalls with the app, and cannot be lost by wiping the hub. Activate it with **Use built-in
config** on the dashboard, then restart. Read-only is the point — it is the known-good fallback.

**`/sdcard/FIRST/SwerveBringUp.xml` on the hub — the editable working copy.**
Created by **Write & activate** when you need to change ports. It lives only on that hub.

If you change ports in the dashboard and want the change to stick for the team, copy the XML back
into `res/xml/swerve_bringup.xml` and commit it. Otherwise the next person to flash a hub gets the
old ports.

## Persistence

Calibration auto-saves to `FIRST/swerve_bringup_cal.txt` on the hub and reloads when the OpMode
restarts, so an interrupted session is not lost. Note this is the *tool's* state — it does not
change robot behavior until you export the constants into `SwerveDrivetrainConstants.java`.

## Files

| File | Role |
| --- | --- |
| `SwerveBringUp.java` | The OpMode: hardware access, routines, state machine |
| `SwerveWebApp.java` | Registers `/swerve` routes on the RC's web server |
| `dashboard.html` | The UI, bundled into the APK as a classpath resource |
| `PodCal.java` | Per-pod calibration, and conversion to a real `CoaxialPod` |
| `SwerveBench.java` | Thread-safe hand-off between the OpMode and web threads |
| `SwerveExport.java` | Renders calibration back into Java source |

Wiring work is done against raw `DcMotorEx` / `CRServo` / `AnalogInput` devices so the tool still
functions when the constants are wrong or unknown — which is the situation it exists to fix.
