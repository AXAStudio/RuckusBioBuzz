# Ruckus Bio Buzz Pedro Visualizer

This is a local copy of the Pedro Pathing visualizer:

https://github.com/Pedro-Pathing/Visualizer

Local customization:

- The export menu includes `TeamCode Auto`.
- That exporter generates an FTC OpMode in `org.firstinspires.ftc.teamcode.auto`.
- Generated autos use this repo's `PathStep` helper for the start and endpoint poses.
- The sequence editor supports `Path`, `Wait`, and `Event` items. `Add Event` creates a timed `Shoot` event by default.

Run locally:

```powershell
npm install
npm run dev -- --host 127.0.0.1 --port 5173
```
