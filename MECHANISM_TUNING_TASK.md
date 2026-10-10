# Task: tune the mechanisms through Mechanism Tuner, ship `systems/*.java`

Read `CLAUDE.md` first. Rules 1, 2, 3, 5, 6, 8 and 11 of `CLAUDE.md` §8 apply unchanged and
nothing here overrides them. This file is only the *how* for everything that is not the
drivetrain; the drivetrain is `TUNING_TASK.md`.

| Thing | Where |
|---|---|
| OpMode | `Mechanism Tuner` (Diagnostics group) — `diagnostics/tuning/MechanismTuner.java`. **Diagnostic, never ships** |
| Dashboard | `http://192.168.43.1:8080/tune` (tabs: Overview, Flywheels, Turrets & servos, Color sensor, Swerve, Export) |
| Front door | `tools/mechtune/mechtune.py` (stdlib only, run from anywhere). `MECHTUNE_HOST=ip:port` overrides |
| Trial log | `tools/mechtune/trials.jsonl` (every step / kick / shot / ff / A-B arm, with volts and PIDF) |
| Traces | `mechtune.py pull LABEL` → `tools/mechtune/runs/LABEL.csv` |
| Deliverable | values spliced into `systems/flywheel.java`, `intake.java`, `turret.java`, `gate.java`, `scoopula.java` by `mechtune.py export --write` — a **shipped** change |

---

## Talking to the operator: banners, not chat

The operator is at the robot, not at the laptop. Every request goes to the dashboard:

```
python tools/mechtune/mechtune.py ask "Swap to a fresh battery, then reply with the volts"
```

posts a banner with a beep on `/tune` **and** `/swerve` (both load `/tune/notify.js`), mirrors
it onto Driver Station telemetry while Mechanism Tuner runs, blocks until the operator presses
Done / Can't, and prints their typed reply. Exit 0 = done, 8 = skipped, 9 = timed out.
`say "..."` posts an info-only note. Batch requests the way `SWERVE_TASK.md` batches OPS
REQUESTs: one banner with everything, not five.

Shots are special: `mechtune.py shots` posts "Feed ONE POLLEN into the POLLEN turret now (shot
i/n)" with `await=shot:pollen`, and the banner **closes itself** when the flywheel's dip detector
sees the shot, so the operator never has to touch the laptop with a piece in hand.

The notes board works with no OpMode running, so `ask "Start Mechanism Tuner and press START"`
is how a session begins.

## What the tool can and cannot see

- **Flywheels are the hub's own velocity PIDF** (`RUN_USING_ENCODER`, `setVelocity` in ticks/s),
  exactly as `systems/flywheel.java` runs them. The OpMode only *samples* velocity (bulk read, one
  sample per loop) — the controller runs on the hub. Loop rate limits measurement resolution, not
  control. Report `loopHzTrue` (= 1/mean dt) with every timing number.
- **"Back at speed" uses the shipped definition**: `|vel − target| ≤ AT_SPEED_FRACTION × target`.
  That is the condition `shooter.java` waits on before opening the gate, so recovery time is
  directly "how long until the next shot can fire".
- **`kick` is synthetic**: `setMotorDisable()` for N ms then re-enable and re-issue the target.
  It is repeatable and costs no operator time, so it is the **search** tool. Whether the hub
  keeps its integrator across a disable is undocumented. A ball loads the wheel differently.
  **Gains are accepted only on real shots** (`shots`, or `ab --test shots`) — the same
  search-vs-validate split `TUNING_TASK.md` enforces with `pedrocheck.py`.
- The tool never builds the drivetrain. Nothing here can move the robot.

## Safety (not overridable)

- Nothing moves before START. Motion commands in INIT come back `REFUSED`.
- Every spinning command needs `--confirmed-clear`, which you may pass only after an `ask`
  in this session in which the operator confirmed hands, pieces and cables are clear of both
  flywheels. Ask again after any battery swap or physical change.
- Velocity is capped at 3000 ticks/s in the OpMode. Everything stops if no client has read
  `/tune/state` or sent a command for 30 s (`mechtune.py` keeps it alive while it waits).
- On any error or Ctrl-C, `mechtune.py` sends `flyOff` / `stopAll`. The Driver Station STOP is
  the real emergency stop.

---

## Part A — Flywheel PIDF

### Survey A (ask once with `ask`, take the defaults, keep working)

1. **Operating velocity.** `modules/shootingRegression` currently returns `(int) distance` (inches)
   as ticks/s — a placeholder, so it does not tell you the real shooting speed. DEFAULT: after the
   ff sweep, test at **70% of the measured max velocity** for each wheel, and also at the target
   the operator names if they know one. Record which.
2. **Zone entry step.** Out of zone the shipped wheel idles at `IDLE_FRACTION` (0.5) × target and
   steps to full on entry. DEFAULT: the step test is **0.5 T → T**, which is the step competition
   actually sees; a 0 → T spin-up is recorded once for reference.
3. **Pieces available** for real shots, and how many shots the operator will feed. DEFAULT: 5 per
   wheel for baseline, 10 per arm for the acceptance A/B.
4. **Acceptance.** DEFAULT: candidate beats shipped on real-shot `recoverMs` with the 95% CI of
   (candidate − shipped) entirely below 0, with step overshoot ≤ 5% and steady-state error std ≤
   one third of the band. All are guesses; say so in the commit.

### Stages, in order

1. **Pre-flight.** `mechtune.py check`. One `ask`: confirm the robot is powered on a charged battery
   (reply volts), both flywheels clear, no pieces staged, gate closed. Record volts on every result
   (CLAUDE.md rule 7); `trials.jsonl` gets them automatically from the hub.
2. **Direction.** `mechtune.py cmd flyPower wheel=pollen p=0.15` then `ask "Is the POLLEN wheel
   spinning in the shooting direction?"`. Fix with `cmd flyDir wheel=pollen reversed=1`. Same for
   nectar. Then `off`. A reversed wheel makes every later number meaningless.
3. **Feed-forward.** `mechtune.py ff pollen --confirmed-clear` (and nectar). Takes ~12 s each.
   Gives `maxVelFit` and `fFromFit = 32767 / maxVelFit` (FIRST's hub velocity PIDF guide). Run it
   twice; if the two F values differ by more than 3%, the battery or the wheel is not steady —
   find out why before going on. Note the volts: F is not voltage-compensated on the hub.
4. **Baseline the shipped gains** (`flywheel.java` values are loaded at OpMode init, so the
   tool starts on them). Smoke-test one of each first (rule 2), then:
   (T = the test target from Survey A) `step pollen --from 0.5T --to T --n 5`, `kick pollen --target T --n 10`,
   `shots pollen --target T --n 5`. Same for nectar. Pull one trace (`pull baseline-pollen`).
5. **Search** with `pidf WHEEL P I D F`, `step` and `kick`. Start from FIRST's recipe —
   F from stage 3, P = 0.1 × F, I = 0.1 × P, D = 0 — and move one coefficient at a time
   (rule 4). Watch overshoot and steady-state ripple on steps, recovery on kicks. If ripple grows
   as P rises, back off. Write down every candidate you try, including losers.
6. **Validate on real shots.** `mechtune.py ab pollen --a "<shipped>" --b "<candidate>" --test shots
   --target T --n 10 --confirmed-clear`. Randomised interleaved blocks; prints mean, sd, 95% CI,
   Welch t and p. That output is the evidence. If the CI straddles 0, the candidate is not shown to
   be better — do not ship it on a point estimate.
7. **Band.** Only if the data supports it: if steady-state ripple at the accepted gains is well
   inside `AT_SPEED_FRACTION`, a tighter band means more consistent exit velocity per shot; if
   ripple keeps leaving the band at rest, the band is too tight and the gate will chatter.
   `cmd atSpeed frac=0.04`. Changing the band changes what "recovered" means, so re-run the
   baseline arm under the new band before comparing.
8. **Ship.** With the accepted values live in the tool:
   `mechtune.py export --only flywheel` (dry run, read the diff) → `--write` → `gradlew
   :TeamCode:assembleDebug` → commit **one value per commit** with the A/B output in the message
   (rule 11 authorship, no Co-Authored-By trailer). Log the session in `FINDINGS.md`.
9. **Confirm what is installed.** After the operator deploys and restarts Mechanism Tuner, its
   PIDF at init is the shipped static, so `mechtune.py state` must show the exported values. If it
   does not, the APK on the robot is not the commit you think.

## Part B — Intake color sensor

The shipped rule (`intake.dominant`): a channel is "seen" when it is above its threshold **and**
above `DOMINANCE ×` each of the other two channels. Wrong-alliance → reverse the intake.

1. `ask` the operator to hold each item at the sensor the way it arrives during intake, and
   capture with `mechtune.py color sample LABEL --n 40 --ask "..."`. Labels: `RED`, `BLUE`,
   `EMPTY`, plus one label per other thing the sensor can see (the other game piece, a half-in
   piece, tile, a hand). At least two captures per label at different positions.
2. `mechtune.py color show` — the per-label table, how many reads of each label the *current*
   thresholds call RED / BLUE, and a suggestion (grid search over threshold × dominance on all
   samples, centred in the tied-best region).
3. A suggestion with any wrong calls on its own training samples is not good enough: capture more,
   or report the overlap. Check the margin: if a label's min/max sits within ~10% of the threshold,
   ambient light will move it across.
4. `mechtune.py color apply`, then put the intake in WATCH (`cmd intake mode=WATCH`) with each
   alliance and `ask` the operator to present wrong-alliance and right-alliance pieces: reject
   must fire on wrong only. That live check is the acceptance.
5. `color csv` (archive), `export --only intake --write`, build, commit with the confusion counts.

## Part C — Turrets, gate, scoopula (operator-driven; Claude assists)

Turrets are positional servos: there is no PID. What can be wrong is the angle → position map in
`turret.java`: `pos = center ± deg / (SERVO_RANGE_DEG × grB / grA)`.

1. **Center**: nudge until the turret points straight forward, `turretCal which=pollen here=1`.
2. **Direction**: `turretDeg which=pollen deg=45` must turn the turret to the robot's left
   (counter-clockwise from above, the field-angle convention `turret.aimFrom` uses). Otherwise
   flip `reversed`.
3. **Ratio**: point at +90 and −90 and ask the operator to measure the actual angles. If the
   measured span is S for a commanded 180°, the true range is `range × 180 / S`; adjust `grB`
   (or `grA`) to match and re-measure. n ≥ 3 measurements per side.
4. **Gate / scoopula**: `gate.openPos/closePos` and `scoopula.scoopPos/unscoopPos` are **0 / 0**
   in shipped code. The operator moves each with the slider and presses Mark; `gatePulse ms=250`
   then checks the gate feeds one staged piece.
5. `export --only turret,gate,scoopula --write`, build, commit.
