Things to do when the season starts:

1 - Get the config off the driver hub and into the main codebase for 0 risk of losing the config in a wipe.
- Steps to get the config
  1. Wire up the swerve pods and build the hardware config the normal way, through the Driver
     Station's "Configure Robot" wizard. Name it something lowercase/numbers/underscores only,
     e.g. `swerve_config` (that name is what shows up in the config picker).
  2. Pull the resulting XML off the Control Hub. Either:
     - `adb pull /sdcard/FIRST/swerve_config.xml` (connect via `adb connect 192.168.43.1:5555`
       while on the hub's Program & Manage wifi network), or
     - browse to it through the Control Hub's web manager at `192.168.43.1:8080`, or
     - a file manager app if going through the Driver Hub tablet directly.
  3. Drop that file into `TeamCode/src/main/res/xml/swerve_config.xml` in this repo (same
     mechanism already used for `teamwebcamcalibrations.xml` in that folder).
  4. Commit it to git.
  5. From then on, any deploy (`Run -> Team Code` in Android Studio) auto-installs that XML back
     onto whatever hub is connected, at `/sdcard/FIRST/swerve_config.xml`. A wiped or swapped
     Control Hub gets the exact same config back the next time code is pushed to it, no need to
     redo the wizard.
  6. After a wipe, double check the config shows as selected on the Driver Station (pick it once
     from "Configure Robot" if it isn't auto-selected) before trusting it.
  - Sanity check the pulled file against `pedroPathing/Constants.java`: it should declare motors
    named `sm0`-`sm3`, continuous rotation servos `ss0`-`ss3`, analog inputs `se0`-`se3`, and an
    I2C device named `pinpoint` (not on I2C port 0, that's the Control Hub's built-in IMU).

2 - Ensure wiring is the same as @swervewiring.png
