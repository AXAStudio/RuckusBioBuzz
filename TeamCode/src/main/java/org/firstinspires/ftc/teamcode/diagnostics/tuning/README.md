# Mechanism Tuner

Diagnostic only — never ships. Tunes everything except the drivetrain (that is Swerve Bring-Up).

1. Join the robot WiFi, open **http://192.168.43.1:8080/tune**.
2. On the Driver Station run **Mechanism Tuner** (Diagnostics group), INIT, START.
   Nothing moves before START; everything stops if no browser or host script has talked to the
   robot for 30 s.

| Tab | What it does |
|---|---|
| Overview | Status, message history from Claude, test banner |
| Flywheels | Live velocity chart with the at-speed band, spin / open loop, hub velocity PIDF, step test, synthetic kick, feed-forward sweep, shot/dip event table, recorder (`/tune/rec.csv`), Claude prompt |
| Turrets & servos | Turret center / reversed / gear ratio against the exact `turret.java` formula; gate and scoopula positions with Mark buttons |
| Color sensor | Live raw RGB, what the shipped `intake.dominant()` rule calls it, intake run through the shipped `intake` class, labelled samples and a threshold suggestion (`/tune/color.csv`) |
| Swerve | Whether Swerve Bring-Up is running, link to `/swerve`, Claude prompt |
| Export | The values to ship as declarations for `systems/*.java` |

Starting values are read from the shipped statics at init. Intake thresholds are applied to the
shipped statics for the run so the shipped intake logic uses them, and restored on STOP. Nothing
reaches competition code except through `tools/mechtune/mechtune.py export --write`.

**Operator banners.** `GET /tune/notify?text=...&level=action|info|warn&await=ack|none|shot:pollen|shot:nectar`
posts a note; `/tune/notes` lists them; `/tune/ack?id=N&reply=...&resolution=done|skipped` answers one.
Registered at app start, so they work under any OpMode. `notify.js` renders them on any page that
includes it (this one and `/swerve`). `await=shot:<wheel>` notes close themselves when that
flywheel's dip detector logs a shot. Host side: `mechtune.py ask` / `say` / `clear`.

Files: `MechanismTuner.java` (OpMode), `TuningHub.java` (state, command queue, notes),
`TuningWebApp.java` (routes), `FlyRecorder.java` (6000-sample trace, does not wrap),
`tuning.html`, `notify.js`. Procedure for a Claude session: `MECHANISM_TUNING_TASK.md`.
