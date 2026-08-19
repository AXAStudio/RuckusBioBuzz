# Positional steering mode — built, then shelved

**Decision, 2026-08-12: the drivetrain is continuous-rotation only. This work is kept for the
record, not for use.** It is disabled at the source (`PodCal.POSITIONAL_ENABLED = false`) and
cannot be reached without deliberately editing that constant.

Read this before reviving it. It is not a list of bugs to fix — the design was sound and most of
it worked. It is a list of what the approach costs, which is what the decision turned on.

## What it was for

Every measured limit on the CR pod traces to one physical fact: the steering path has no creep
regime. Static friction breaks to kinetic and the pod goes from stopped to 9–80 °/s with nothing
in between, so any external controller with the authority to correct a small error commits to
degrees of travel before feedback — 39 ms transport plus a 42 ms velocity lag — can act on it.
Six software approaches failed against that. Running the servo's own internal position loop makes
breakaway the servo's problem, at a rate our 20 ms PWM frame can never match.

## What was built

- `PositionalPod` — a `SwervePod` implementation using `Servo` rather than `CRServo`. Sits
  alongside `CoaxialPod`; `FollowerBuilder.swerveDrivetrain` takes a varargs of the interface, so
  a drivetrain can mix them. **No Pedro changes were needed.**
- Travel-window and seam analysis, `seam.py`
- End-stop clamp, reflective coverage proof, `probeClamp`
- Guarded endpoint calibration: `calHome`, `calGoto` with stall-hold, `calMark`
- `swerve_positional_p0.xml` and per-resource config activation
- Hardened boot encoder read (median of 5, refuses on disagreement)
- `bandgate.py` — band reproducibility across reflashes

## What we learned, and it was promising

Pod 0 was flashed, calibrated and homed successfully. Endpoints measured cleanly at
`rawDeg 301.66` (position 0.0) and `51.53` (position 1.0), span **250.1°**, and the encoder wrap
fell outside the band, so no re-clocking was needed.

The single move we observed in detail — `calHome`, 98° — landed **with no overshoot** and then held
to **0.34° peak-to-peak, sd 0.067°**. For comparison, CR at the interim gains gives 0.77° mean
residual with a 2.36° worst case. On that one data point the servo's internal loop absorbs a
stiction break far better than anything we achieved externally.

**We never got a scored comparison.** No step trials, no A/B against CR, criterion 8 never measured
in positional mode, overload behaviour never tested. This was shelved on cost, not on a result.

## What it costs — the reason it was shelved

**Traverse cost, unavoidable.** Bounded travel breaks the flip guarantee. A sweeping demand forces
a 180° traverse — about **260 ms with the wheel pointed the wrong way** — once per 180° of demand
sweep. That frequency is **independent of window width**: position tracks demand one for one, each
traverse removes exactly 180° of accumulated position, and position is bounded, so over a sweep of
D degrees the count is D/180 whatever the width. Widening buys hysteresis only. CR mode has no
equivalent — it can rotate continuously and never traverse.

**Endpoint stall.** `calHome` showed stick-slip *inside the servo's own loop*: a dead stall at 72.5°
of travel for 0.38 s — 40+ consecutive identical encoder samples — then a 25.4° break at ~390 °/s.
The servo does not escape stiction; it handles the break better. A 0.38 s stall on every move would
be a serious latency cost, and we never measured whether it recurs on smaller steps.

**Operational surface, on a robot students maintain.** This is what actually decided it:

| burden | why it persists |
| --- | --- |
| Firmware flash per servo | Physical, one servo at a time, needs the archive-generation programmer. A replacement servo arrives in CR mode and will not work until flashed |
| Port type in the config XML | `<Servo>` not `<ContinuousRotationServo>`. Wrong type and the pod does not build |
| Two-point endpoint calibration per servo | Guarded walk to both ends; must be redone after any reflash or horn disturbance |
| `calHome` before any calibration | One uncontrolled move of up to half the travel, every time |
| Seam placement tied to chassis geometry | The dwell set comes from `atan2(dtLength, ±dtWidth)`; change the chassis and it must be re-derived |
| Encoder wrap must stay outside the band | Constrains horn clocking; a bad boot read drives the pod a long way under torque |
| Band reproducibility across flashes | Verified by `bandgate.py`; an unreproducible band silently invalidates the calibration |

None of that is hard once. All of it is permanent, and all of it is a way for a future team to have
a drivetrain that does not work for a reason no one remembers.

## If you revive it

Set `PodCal.POSITIONAL_ENABLED = true`, then follow `POSITIONAL_AB_PLAN.md`, which is still
accurate. The one-pod within-pod A/B design in it is sound and is the right way to get the evidence
this work never obtained: reflash one pod, baseline it against itself in CR first, and score with
`ploose.py --pod N` and `crit8_current.py N`.

Do not skip the coverage proof or `probeClamp`. During this work the coverage proof caught a
half-finished calibration that would otherwise have been used, and the end stops are hard — driving
into one stalls the servo into its overload cutout with nothing reported.
