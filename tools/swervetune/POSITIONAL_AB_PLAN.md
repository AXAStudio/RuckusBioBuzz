# One-servo positional A/B — bench plan

Reflash one servo to Servo Mode, fit it to one pod, and score it against the other three still in
CR mode. Same conditions, same scripts where they apply. Reversible: one unit at risk, and
`CoaxialPod` stays live as the fallback.

**Sequence matters.** Positional A/B first, mechanical pass after. Lubricating first changes
breakaway on the test pod and confounds the comparison — and if positional mode works, the
breakaway targets stop mattering.

## Programmer settings

Servos are **Axon MINI+**, and the archive-generation programmer is confirmed — these exact
units were flashed into CR mode with it during the build, so step 1 reverses an operation already
performed once rather than attempting a new one.

| setting | value | why |
|---|---|---|
| Mode | Servo (load the Servo Mode firmware file) | this is a firmware download, not a parameter |
| Servo Angle | **194** (= 270°) | The programmer scale is 0–255 mapping to 0–355°, so 194 = 270°. Width buys hysteresis (90° here) and nothing else — traverse frequency is one per 180° of demand sweep at any width. The cap is the encoder: the band occupies that much of its 360°, and the remainder must hide the non-monotonic wrap. 270° leaves 90° of cover; 355° would leave 5°. Resolution 0.15°/step, 7x finer than the 1.0° target |
| Sensitivity (deadband) | **start mid-range, sweep** | do NOT max it. 1 µs deadband into a stiction-heavy 1:1 pod is how the internal loop hunts forever |
| Soft Start | **on** | limits how hard the pod snaps on enable. Not a substitute for `initFromEncoder()` — both |
| Inversion | as needed | match the existing pod direction convention |
| Overload Protection | note the setting, test it | see below |
| Dampening Factor | **leave alone** | factory-tuned per the vendor docs. Last resort only, and log it if changed |
| PWM Power | leave at default initially | a cap interacts with breakaway; change one thing at a time |

## Travel window placement

190° of travel: 180° of headings plus a 10° overlap arc. Both representations of a heading are
reachable inside that arc, so the changeover is a hysteresis band rather than a forced instant.

**The whole arc must clear the dwell set, not just one end** — the traverse happens somewhere in it.
Dwell headings mod 180 in the tool wheel frame (forward = 90), X-locks from
`atan2(146.42, ±154.24) = ±43.51°`:

| dwell | tool frame | from forward |
|---|---|---|
| forward | 90.00° | f |
| X-lock RF/LB | 133.51° | f + 43.51° |
| strafe | 0.00°/180° | f + 90° |
| X-lock LF/RB | 46.49° | f + 136.49° |

Gaps are unequal — forward↔X-lock 43.51°, strafe↔X-lock 46.49° — so the arc goes in a
strafe↔X-lock gap. Centred there:

**Overlap arc = tool frame 18.25° → 28.25° (f + 108.25° → f + 118.25°).**
Clearances: strafe 18.25°, X-lock LF/RB 18.24°, forward 61.75°, X-lock RF/LB 64.74°.

Balanced rather than biased toward X-lock: at 18° the clearance is ~20x the residual and has
stopped binding, so an even split is more robust to calibration error either way.

**Put the encoder's wrap in the same place.** The Axon's analog output is non-monotonic across its
wrap; it produced a spurious 1452 °/s reading during slew characterisation. Both dead zones in one
unreachable spot costs nothing and removes both.

`seam.py` computes all of this from the measured endpoints and reports the best available tooth
shift if the placement is poor.

## End-stop clamp — mandatory before any step

Position mode adds a hazard CR mode did not have: the travel ends are **hard stops**, and commanding
past one stalls the servo into its overload cutout. That is a silent steering failure — the encoder
just shows a pod that stopped tracking, and nothing is reported.

1. **Clamp.** `setClampMarginDeg(3.0)` holds every command 3° inside each programmed endpoint.
   190° − 6° = a 184° commandable band.
2. **Coverage proof.** Representations of a heading form a lattice 180° apart, so *any* interval
   ≥180° wide contains one; 184° works with 4° of slack. `verifyCoverage(0.25)` checks the
   implementation against the real calibration by sweeping the whole heading circle, and
   `rebuildPods()` refuses to build a positional pod that fails. Published as `posCoverage`.
3. **Clamp probe.** `GET /swerve/cmd?action=probeClamp&over=30` asks for 30° past each end and
   reports the positions actually written. **Run before any step response** — a clamp that has never
   been exercised is an assumption.

`noCandidateFault` surfaces the case where no representation lands in band, rather than letting the
pod quietly stop tracking.

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

**Procedure — note the double flash, it is deliberate:**

Where the travel band lands in encoder space is a property of the reflashed servo, so it cannot be
measured until after the first flash. If it needs re-clocking, that is a mechanical disturbance
which must not fall between the baseline and the test. Reflashing is non-invasive — the programmer
connects through the servo's existing cable — so flashing twice costs nothing and keeps the
mechanical state identical across the comparison.

**Steps 1–2 are the most dangerous moment in the whole sequence** and need someone at the bench.
The band is unmeasured, so the position mapping is fiction, and a position-mode servo drives to
whatever it is told the instant it has power. Three things stand in the way:

- `PositionalPod` **writes nothing to the servo while uncalibrated**. `initFromEncoder()` and
  `move()` both return early. The only path that can move an uncalibrated pod is `calGoto`.
- `calGoto` **walks** to a target position in 0.02 steps with a 0.15 s dwell, checking after each
  that the encoder actually followed. Two consecutive steps under 1° of movement and it stops and
  says so. The pod's mechanical range may be smaller than the servo's programmed travel, and
  nothing else would notice the difference between measuring a limit and grinding into one.
- Coverage proof and `probeClamp` run **after the first calibration too**, not only before the
  test. A band that has been measured but never verified is still unverified.

Expect the pod to move when PWM is first enabled: with nothing commanded the servo drives to its
default position, roughly mid-travel. Soft Start limits how hard. Hands clear, robot on blocks.

1. Flash pod 0 to Servo Mode. Activate `swerve_positional_p0.xml`, restart.
2. **Flag before enabling the servo.** Then `calGoto` outward from mid-travel to each end,
   `calMark` each one. `posCalibrated` goes true once the span exceeds 100°.
3. **Gate: coverage + clamp.** `posCoverage` must be positive, and `probeClamp` must write two
   positions strictly inside 0 and 1. Do not proceed otherwise.
4. `seam.py --pod 0`. If clearance is poor: re-clock, re-zero, re-calibrate, repeat from 2.
5. `bandgate.py --save 0` — records where this flash put the band.
6. **Flash back to CR.** Restore `swerve_bringup.xml`, restart.
7. CR baseline: `ploose.py --pod 0` and `crit8_current.py 0`.
8. Flash to Servo Mode again, same settings. Positional config, restart, re-calibrate.
9. **Hard gate: `bandgate.py --check 0`.** Endpoints must match the first flash within **2.0°**
   and the span within **1.0°**. It exits non-zero and says stop if not — a band that does not
   reproduce means the first flash's calibration does not describe the second, the seam analysis
   is void, and every number downstream would be scored against a wrong mapping. That failure is
   silent and would look like a result.
10. Coverage + `probeClamp` again.
11. Positional run: exactly the two scripts from step 7. Score against pod 0's own numbers.
12. Overload procedure — two-person, flag first.

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
