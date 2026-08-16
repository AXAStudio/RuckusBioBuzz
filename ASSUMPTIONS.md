# Assumptions taken this session

Every default taken from Survey 0 without an answer, plus anything else assumed rather than
measured. Each says how to overturn it.

## From Survey 0 (defaults taken, work proceeded)

| # | Item | Assumed | Consequence if wrong |
|---|---|---|---|
| Q1 | Robot time / surface | >2 h on FTC tiles, robot on the floor. All drive commands still gated behind a confirmed OPS REQUEST. | Captures get rescheduled; nothing else changes. |
| Q2 | Drive path under study | Both: reproduce through the dashboard, confirm on `DriveTeleOp`. | If dashboard-only, the shipped-path findings (§2.3) stay unverified. |
| Q3 | Criterion 7, heading error translating | < 3.0° | Threshold only. |
| Q4 | Criterion 8, heading error at rest | < 1.0° | Threshold only. |
| Q5 | Criterion 9, cross-track | < 2.0 in | Threshold only; drives how much clearance the Task 3 path needs. |
| Q6 | Editing vendored Pedro | Yes — `third_party/PedroPathing` is treated as our source, as prior commits already do. | Fixes in `Swerve.java` / `CustomDrivetrain.java` would have to be worked around. |
| Q7 | Lubrication pass | **Self-answered from the code, not assumed.** It happened before 2026-08-13; gains were re-fitted after it. CLAUDE.md's "pending" is stale. | — |
| Q8 | Recorder columns | Yes, add columns and report the cost. Added `p{i}_ctgt`; `tgt` kept so the archive stays comparable. | Revert the column; `steerqual.py` falls back to `tgt`. |
| Q9 | Task 3 path shape | Closed loop that returns to start, so it can be repeated for statistics. | Path gets redesigned; the profile and validation method do not change. |
| Q10 | Heading interpolation | Tangential on traverses, constant on the approach; a constant-heading variant also generated because it isolates Task 2. | Path variant selection only. |

## Assumed rather than measured

1. **`String.format` is the bulk of the 36.7 ms publish cost.** Arithmetic fit (~1000 calls ×
   ~35 µs), not yet a measurement. The deployed per-section timers and the `setFastFmt` A/B
   settle it. Until then it stays a hypothesis in FINDINGS.md.
2. **Teleop centripetal correction is dead code and cannot NPE** (FINDINGS §2.5). Derived from
   `Vector` not overriding `equals`. Predicted safe; unverified on hardware. First
   `DriveTeleOp` run is the test, and the operator has been warned.
3. **The saved box survives a redeploy.** `swerve_field_box.txt` is reloaded at init and the
   Pinpoint pose is not reset by configuration. `/state` is re-read to confirm `box.valid`
   before any motion, so this assumption is checked rather than trusted.
4. **`p{i}_ctgt` costs nothing measurable in the loop.** One field read and a normalise per pod
   per loop. Will be confirmed against the loop-dt distribution before/after.
5. **The archived runs were taken on FTC tiles.** The runs carry battery voltage but not
   surface. Inferred from the session logs that produced them; if any were on blocks the
   friction-dependent numbers (pod error, path ratio) would not compare.
6. **`mydrive-001` is representative of human driving.** One 71.9 s session, one driver. The
   attribution percentages in FINDINGS §2.2 are from that run; the mechanisms are structural,
   but their shares are not a population estimate.
