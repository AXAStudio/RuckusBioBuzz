# One-servo positional A/B — bench plan

Reflash one servo to Servo Mode, fit it to one pod, and score it against the other three still in
CR mode. Same conditions, same scripts where they apply. Reversible: one unit at risk, and
`CoaxialPod` stays live as the fallback.

**Sequence matters.** Positional A/B first, mechanical pass after. Lubricating first changes
breakaway on the test pod and confounds the comparison — and if positional mode works, the
breakaway targets stop mattering.

## Programmer settings

Needs the **archive-generation programmer** (MAX+/MINI+/MICRO+). The MK2 unit is not compatible
downward. Confirm the label reads `MINI+` and not `MINI MK2`.

| setting | value | why |
|---|---|---|
| Mode | Servo (load the Servo Mode firmware file) | this is a firmware download, not a parameter |
| Servo Angle | **~200°** | 180° is all the flip logic needs. Narrower is better: at 1 µs over 600–2400 µs that is 1800 steps, so 200° gives 0.11°/step against 0.197° at 355°, and it shrinks the 1 µs deadband in pod-angle terms by the same ratio |
| Sensitivity (deadband) | **start mid-range, sweep** | do NOT max it. 1 µs deadband into a stiction-heavy 1:1 pod is how the internal loop hunts forever |
| Soft Start | **on** | limits how hard the pod snaps on enable. Not a substitute for `initFromEncoder()` — both |
| Inversion | as needed | match the existing pod direction convention |
| Overload Protection | note the setting, test it | see below |
| Dampening Factor | **leave alone** | factory-tuned per the vendor docs. Last resort only, and log it if changed |
| PWM Power | leave at default initially | a cap interacts with breakaway; change one thing at a time |

## Travel window placement

Programmed travel ~200°, giving 180° of working range plus 20° of overlap. In the overlap both
representations of a heading are reachable, so the changeover becomes a hysteresis band rather
than a forced instant.

Place the window so the seam sits at **wheel heading ≈20°**. Measured pod occupancy on this
drivetrain: forward 90°, strafe 0/180°, X-lock 46.5° and 133.5°. 20° avoids all four and is clear
of pure strafe. This is chosen from four drive-test headings, not match telemetry — if we want it
placed on data, log pod-heading occupancy through a practice match and move it.

**Put the encoder's wrap in the same place.** The Axon's analog output is non-monotonic across its
wrap; it produced a spurious 1452 °/s reading during slew characterisation. Both dead zones in one
unreachable spot costs nothing and removes both.

## Calibration

Two numbers, `rawDegAtPos0` and `rawDegAtPos1`: the raw encoder angle at servo position 0.0 and
1.0. Within the window the servo shaft and pod are 1:1 and the encoder is on that shaft, so the
relationship is a straight line and its endpoints are the whole calibration.

Command 0.0, wait for it to stop, read `pods[i].rawDeg`; repeat at 1.0. Verify midpoint: commanding
0.5 should read within a degree of the average.

## Overload protection test

A pod scrubbing on carpet, stalling into a silent power cut, is a steering failure with no error
message — the SDK gets nothing back from the servo. Detection has to be `getSlipDeg()`, which is
why the encoder stays wired.

Two people, robot on blocks, recorder running:

1. `recStart`, command a step, let it settle.
2. Hold the pod firmly against rotation for ~5 s. Record throughout.
3. Release. Keep recording for 5 s.
4. `recStop`, pull the trace.

Report: how long until it stops pushing (slip large and stable), whether it recovers unaided on
release, whether it needs a power cycle, and whether the analog output stays valid throughout.
Repeat at two Sensitivity settings — deadband and overload interact.

If recovery is not automatic, the follower needs a slip watchdog: `getSlipDeg()` above a threshold
for longer than a settle time is a fault worth surfacing to the driver.

## Criterion 8 for positional mode: holding current

"Servo power at rest, RMS, below measured deadband" is a CR quantity with no analogue — there is no
commanded power. Reusing post-settle peak-to-peak would be criterion 6 wearing a hat, and it misses
the failure that matters: an internal loop working hard to hold station shows as current long
before it shows as motion.

**Measured on the battery current channel, not the servo rail.** The rail reads nothing useful here
— the turn servos are fed from the servo power module and the hub supplies only their PWM. With all
four holding, the rail moved −0 → 10 mA against a 44 mA sd, while the battery channel moved
189 → 205 mA against 12, a 5.6σ effect. `crit8_current.py` measures it, with the three pods not
under test PWM-disabled so only one thing changes state.

**Pass condition:** holding current minus idle current indistinguishable from zero within noise.
Stated relatively because the old 0.025 is a CR number that means nothing in position mode.

**CR baseline, measured 2026-08-12 at 12.74 V, 30 s per condition, n≈128 samples each:**

| pod | hold − idle | verdict |
|---|---|---|
| 0 | −1 ± 2 mA | pass |
| 1 | +2 ± 2 mA | pass |
| 2 | **+4 ± 2 mA** | elevated at 2σ |
| 3 | +3 ± 2 mA | pass |

Pod 2 being the only elevated one is *consistent with* it also having the worst residual (1.06°)
and the worst post-settle peak-to-peak (1.24°). It is a pre-registered directional test rather than
a fishing result, but it is 2σ off a single 30 s window, so it should not carry weight until it is
re-run with longer integration. Treat it as a hypothesis, not a finding.

Standard error is ~2 mA against a per-servo effect of a few mA, so the long dwell is load-bearing;
do not shorten it. Keep the audible check as a qualitative gate, not the measurement. If a future
setup puts the servos on hub power, the rail channel becomes the better instrument and this should
be revisited.

## A/B design: within-pod, on pod 0

Scoring one reflashed pod against fleet aggregates would be invalid — the per-pod residual spread
(0.76 / 0.73 / 1.06 / 0.52°) is larger than the effect being looked for, so which pod gets
reflashed would determine the answer.

**Reflash pod 0.** Its residual, 0.76°, is the closest of the four to the fleet mean of 0.77°, and
it is clean on criterion 8 at −1 ± 2 mA, so any improvement is attributable to the mode change
rather than to having fixed an outlier. Avoid pod 2 (worst on both, would flatter positional
through regression to the mean) and pod 3 (0.52°, unrepresentatively good). If a different pod is
markedly easier to reach, take it — the within-pod design is what makes the comparison valid, so
accessibility is a fair tiebreak.

**Procedure, all in one session on one pack:**

1. Re-run pod 0's CR baseline immediately before touching it: `ploose.py` restricted to pod 0 at
   the interim gains, plus `crit8_current.py 0`. Same scripts, same conditions.
2. Reflash and refit.
3. Re-run exactly those two. Score positional against pod 0's own numbers from step 1, never
   against the fleet.

**No refit confound.** The programmer connects through the servo's existing cable — the servo
stays in the pod and only the lead moves off the servo power module. Nothing mechanical is
disturbed, so no remove-and-refit control run is needed.

**Port type must change too.** A servo reflashed to Servo Mode needs its port redeclared: the SDK
builds a different device class per port type and asking for a `Servo` on a CR port throws.
Activate `swerve_positional_p0.xml` (`GET /swerve/config?builtin=swerve_positional_p0`) and restart
the robot. `swerve_bringup.xml` goes back when the A/B is done.

## What to measure

Everything that does not assume power-in/rate-out still applies:

- `trials.py` / `phase4_gate.py` — step scoring is pod-type agnostic, it reads the encoder
- residual at t=3 s, settle to ±2.0°, post-settle p-p, rings, P(loose)
- run the positional pod and the three CR pods in the same `pidStepAll`, so the comparison is
  simultaneous and immune to battery drift

These assume CR and will not work on the reflashed pod: `wireScan`, `sweep`, `spinServo`, `nudge`,
`rawServo`, `PodAutoTuner`, `phase2_plant.py`, `phase2_delay*.py`, `phase4_pulsecal.py`. Keep the
reflashed pod out of them, or run them on the other three only.

## Targets

Beat the CR interim on the same block: residual at t=3 s **0.77 ± 0.55° mean, 2.36° max**, settle to
±2.0° **0.647 s**, P(loose) **0/60**. Sub-degree worst case is what would close criterion 5, and
0.11°/step says it is reachable if the internal loop's deadband under load cooperates.
