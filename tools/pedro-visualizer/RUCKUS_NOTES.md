# Ruckus Bio Buzz Pedro Visualizer

This is a local copy of the Pedro Pathing visualizer:

https://github.com/Pedro-Pathing/Visualizer

Local customization:

- The export menu includes `TeamCode Auto`.
- That exporter generates an FTC OpMode in `org.firstinspires.ftc.teamcode.auto`.
- Generated autos use this repo's `PathStep` helper for the start and endpoint poses.
- The sequence editor supports `Path`, `Wait`, and `Event` items. `Add Event` creates a timed `Shoot` event by default.
- The control panel supports named pose variables that can be assigned to the start pose or path endpoints.
- Endpoints assigned to pose variables still allow editable heading mode, linear start heading, and heading curve while the pose controls the final position/heading.
- Each path has a `Path Speed` scale from `0.05` to `1.0`; TeamCode export passes it to PedroPathing as the per-path max power.
- Each path can define parallel event markers. TeamCode export turns them into PedroPathing parametric, temporal, or pose callbacks so mechanisms can start while the path is still running, with optional timed finish handling.
- The control panel has a telemetry readout that follows playback, showing the current path state, pose, path progress, path speed, active parallel events, and the next queued event marker.
- Visualization settings can show event pins on the field, per-segment path length/time labels, a 30-second autonomous countdown overlay, an optional blue-to-red velocity gradient based on the trapezoidal motion profile, and estimated swerve module angles on robot previews.
- Undo/redo history keeps recent recovery snapshots in localStorage across reloads.
- Browser project files are stored in IndexedDB, with one-time migration from the old localStorage file blob.
- Linear heading paths include an editable heading curve graph. Values above `1.0` shift more of the turn toward the end of the path.
- The control panel includes `Mirror X` and `Mirror Y` actions for switching alliances by flipping path coordinates and headings on either field axis.
- The TeamCode exporter validates generated autos, can download a `.java` file, and can save directly to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/` while running from Vite. Direct saves run `:TeamCode:compileDebugJavaWithJavac` and restore the previous file if the generated Java does not compile.

Run locally:

```powershell
npm install
npm run dev -- --host 127.0.0.1 --port 5173
```
