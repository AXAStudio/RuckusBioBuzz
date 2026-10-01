# Task: tune the swerve through Swerve Bring-Up, validate through Pedro, ship `Constants.java`

Read `CLAUDE.md` first, then `ROBOT_CONTROL.md` (how to connect, the gates, the commands). Rules
1, 2, 3, 5, 6, 8 and 11 of `CLAUDE.md` §8 apply unchanged and nothing here overrides them. The
question protocol, OPS REQUEST batching, `RUN_STATE.md`, ABORT and pre-flight are as written in
`SWERVE_TASK.md` ("Run this end to end", "Safety") - follow them here too.

**The deliverable is `TeamCode/.../pedroPathing/Constants.java`**, committed, deployed, and
accepted by Pedro on the installed build - not a dashboard state, not a table of good numbers.

---

## The one rule this task exists for: Pedro validates, the tool searches

The bring-up tool's own tests are **search tools, not validation**. They run control laws that
competition code never runs:

| Tool test | What it actually exercises | Why it does not validate |
|---|---|---|
| `pidStep`, `pidStepAll`, `autoTune`, `perpod_tune.py`, `kssweep.py` | one pod's turn loop, robot still | No path follower, no drive load, no Foresight demand pattern. The rolling-plant discovery (2026-08-14) is exactly a gain set that passed every static test and limit-cycled while driving |
| `headingStep`, `headingGoto`, `setHeadingPidf`, `headtune.py` | the TOOL's heading hold: PIDF with kD and a sign feed-forward | Foresight's heading controller is **P only**, called twice per update with different errors. A kP tuned with a kD under it does not transfer |
| `drive`, `robot.py drive`, `drivecapture.py`, dashboard sticks | the tool's own `Swerve` built with ITS config (`IGNORE_ANGLE_CHANGES` unless xLock, `manualBrakeMode` off, its epsilon) | Not the shipped `SwerveConfig`, not the follower, and the fence clamp is in the loop |

**Every gain is accepted only on `tools/swervetune/pedrocheck.py` results** - the real
`Follower` + `Foresight` + shipped `SwerveConfig`, driving holds, turns, lines, curves and the
actual auto paths, with pod tracking, path error and heading error recorded at loop rate:

- `--pods live` - the follower runs the tool's live pod calibration. Use it to validate pod
  gains/zeros **before** exporting. This is the pre-deploy check.
- `--pods shipped` - the follower runs `Constants.java` as installed. This is **acceptance**. It
  counts only when `/state` shows `pedro.gainsShipped` and `pedro.staticsShipped` true and
  `RUN_STATE.md` `on_robot_build == last_commit`.

A result from a tool test may justify *trying* a value. It never justifies *shipping* one. If you
catch yourself writing "settles in 0.5 s, so ship it" about a `pidStep` or `headingGoto` number,
stop and run `pedrocheck.py`.

---

## Survey 0 (ask first, take defaults, keep working)

1. Field: full BIOBUZZ field available (field fence) / practice area only (marked box)?
   DEFAULT: practice area. Paths in stage 8 need the full field.
2. Robot footprint for the fence, L x W in: DEFAULT 18 x 18 (FTC max - safe, costs room). The
   dashboard draws a 12.2 x 11.5 chassis; that figure is unverified.
3. `pedrocheck.py` thresholds (`THRESHOLDS` in the script; all guesses but criterion 7/8/9's
   origins): hold settle 1.5 s / residual 0.5 in / overshoot 1.0 in; turn settle 1.0 s /
   overshoot 5 deg / residual 1.0 deg; path max cross-track 2.0 in, max heading err 3.0 deg, end
   error 1.0 in. DEFAULT: as listed.
4. Path power for validation: DEFAULT 0.5, and the auto's real power in stage 8.
5. Robot time and number of deploys available. DEFAULT: 3 deploys (stages 5, 6, 7).
6. Is the Foresight Tuner (Pedro AutoTune, stage 6) to be run by the operator now? DEFAULT yes.

---

## Stages - in this order (later stages assume earlier ones)

Each stage: **search** with the tool, **validate** with Pedro, **record** n / spread / volts /
surface in `FINDINGS.md`, **commit** per finding.

### Stage 0 - session start
`robot.py check`; pre-flight OPS REQUEST; confirm the installed build (`on_robot_build`) and that
`robot.py constants` reports "already matches" - if it does not, the tool's calibration file and
`Constants.java` disagree, and you must decide which is true **before** tuning anything.

### Stage 1 - calibration (tool only; affects everything downstream)
Wire scan, zeros, analog ranges, drive directions, encoder names. Then the crawl crab check
(drive straight at crawl, measure the odometry crab angle, `zeroTrim`, re-measure) - a common
zero bias is invisible to every pod-relative test.
*Validate:* crab angle < 1 deg at crawl (n >= 3 each way).

### Stage 2 - odometry (Pinpoint offsets and directions)
The odometry starts from Constants.java (`pinpointXPodOffset`/`YPodOffset` = pod distance from the
robot centre, `pinpointX/YPodReversed`) - every OpMode, including the bring-up tool, configures
the Pinpoint from there. Iterate with `odoConfig xPodOffset= yPodOffset= xPodReversed=
yPodReversed=` (same names), or have the operator run Pedro's Pinpoint Tuner and import its output
with `robot.py pinpoint FILE`. **`odoConfig` resets the pose and clears the fence** - do this
stage before arming any fence, and re-arm after. Until the values are exported, `/state`
`odo.shipped` is false and `errors[]` says so.
*Validate:* paired +/-45 deg rotations orbit <= 0.6 in per 45 deg; a taped straight line reads
within 1% of its measured length.

### Stage 3 - arm the fence
Field available: `fieldFence` from an operator-measured pose (ROBOT_CONTROL.md 3.4.1, robot.py
needs `--pose-from-operator`). A walled or taped area of known size: `borderFence width= height=`
(3.4.2). Else the marked box. `robot.py check` must exit 0.

### Stage 4 - pod turn gains
*Search* with step tests on blocks and tiles. *Validate* every candidate with
`pedrocheck.py suite --pods live` (A/B: `--arms` with `setPidf` per arm, randomised and
interleaved by the script). Read **pod tracking while Pedro drives** (`pod*_err_p95_deg`, flips),
plus the path metrics. n >= 10 per arm before claiming a difference (rule 2). `setPidf` writes
the tool's calibration, so after an A/B the LAST arm's gains are live - set the winner explicitly
and re-read `/state` pods before exporting.

### Stage 5 - export, deploy, accept (deploy #1)
`robot.py constants` (dry run, read every changed line and every WARNING) -> `--write` ->
`./gradlew :TeamCode:assembleDebug` -> commit with evidence -> OPS REQUEST: install, start
Swerve Bring-Up -> re-check fence (`robot.py check`) -> `robot.py constants` must say
"already matches" -> `pedrocheck.py suite --pods shipped` = the baseline for stage 6.

### Stage 6 - Foresight constants (deploy #2)
The velocities, brake coefficients, coast/brake kV, heading kP and the primary/secondary
translational kP come from **Pedro's Foresight Tuner** (AutoTune, `pedroPathing/Tuning.java`,
port 10158), which the **operator** runs. Put in the OPS REQUEST:
- it drives up to 48 in (36 in braking) with **no bring-up fence** - clear space, hand on STOP;
- it is a different OpMode: it builds a Follower, which re-origins the Pinpoint, so the fence
  is discarded when Swerve Bring-Up comes back - re-arm it (stage 3);
- reply with the tuner's whole results list or code block.

Save the reply to a file, `robot.py foresight FILE` (dry run) -> `--write` (it also zeroes the
carried translational kI/kD: the tuner fits P-only) -> build -> commit -> deploy ->
`pedrocheck.py suite --pods shipped`, compared against the stage 5 baseline.
**The Foresight Tuner is identification, not validation** - its numbers ship only if this
comparison holds up.

### Stage 7 - follower refinement (deploy #3 if anything changes)
A/B follower gains through Pedro: `pedrocheck.py suite --pods live --arms arms.json` with
`pedroPidf` arms (`hp` heading kP, `tp`/`tfp`/`tfs`/`tsp`/`tss` translational, `ti`/`td`,
`fzpa`/`lzpa`). Never tune heading with `setHeadingPidf`/`headingGoto` for Foresight. Export,
deploy, accept with `--pods shipped`.

### Stage 8 - the real paths
On the field fence: `pedrocheck.py path auto/visualizerAutos/<auto>.pp --line <each line>
--pods shipped` at the auto's power, n >= 3 per line. Branch lines need `--start`. Compare
durations with the visualizer's estimate (the pedro-path MCP `review_path`).
Only when stages 6-8 pass: set `FORESIGHT_MEASURED = true` **by hand**, commit it with the
`pedrocheck.jsonl` rows that justify it, deploy, and have the operator run the actual auto
OpMode once, supervised - autos have no fence.

### Stage 9 - what Claude cannot validate
The competition `TeleOp` (`follower.manual()` from the sticks) has no HTTP path and no recorder.
Ask the operator to drive it after the final deploy and report anything that differs from the
bench; do not claim it validated.

---

## Before every `--write` (shipped change) and every acceptance run

- [ ] Every value being written has a `pedrocheck.py` result behind it, cited in the commit.
- [ ] `robot.py constants` WARNINGS read and handled: tool-only settings (mixer `setMixer`,
      schedule, servo slew, kI, positional) do NOT ship through Constants - making them ship is
      a vendored change (`third_party/PedroPathing`) and must be flagged as such (CLAUDE.md).
- [ ] After deploy: `robot.py constants` -> "already matches"; `/state` `pedro.gainsShipped`
      and `staticsShipped` true; fence re-checked.
- [ ] The tool restores follower gains, mixer and schedule to the installed build at init, stop
      and `pedroReset`. Statics still live in the app process between OpModes - if the tool
      crashed rather than stopped, restart the app before running an auto.

## Done means
`Constants.java` committed and installed; the stage 5/6/7 acceptance runs and stage 8 paths in
`current_runs/pedrocheck.jsonl` and summarised in `FINDINGS.md` with n, spread, volts, surface;
`FORESIGHT_MEASURED` set or a stated reason it is not; every miss with its number and cause.
