# Ruckus Bio Buzz Pedro Visualizer

This is a local copy of the Pedro Pathing visualizer:

https://github.com/Pedro-Pathing/Visualizer

Local customization:

- The export menu includes `TeamCode Auto`.
- That exporter generates an FTC OpMode in `org.firstinspires.ftc.teamcode.auto`.
- Generated autos use this repo's `PathStep` helper for the start and endpoint poses.
- The sequence editor supports `Path`, `Wait`, and `Event` items. `Add Event` creates a timed `Shoot` event by default.
- The control panel supports named pose variables that can be assigned to the start pose or path endpoints.
- Each path has a `Path Speed` scale from `0.05` to `1.0`; TeamCode export passes it to PedroPathing as the per-path max power.
- Linear heading paths include an editable heading curve graph. Values above `1.0` shift more of the turn toward the end of the path.
- The TeamCode exporter validates generated autos, can download a `.java` file, and can save directly to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/` while running from Vite.

Run locally:

```powershell
npm install
npm run dev -- --host 127.0.0.1 --port 5173
```
