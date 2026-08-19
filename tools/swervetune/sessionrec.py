"""Record a human driving session continuously, then find the wobble.

Chains recorder chunks while the driver drives (the buffer holds ~45-75 s at loop rate and
stops when full, so this restarts it the moment each chunk fills and pulls the CSV to runs/).
Each chunk now carries heading, hold target, field pose and the applied drive commands
alongside the per-pod data, all synchronized at loop rate.

    python sessionrec.py <label>          # record until Ctrl-C (or touch STOPFILE)
    python sessionreport.py <label>       # then analyze the whole session

Chunk gaps of ~1 s occur while the hub renders each CSV; wobble is a sustained behaviour, so
a session loses nothing that matters.
"""

from __future__ import annotations

import os
import sys
import time

from swervebench import Bench

STOP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "STOP_SESSION")


def main(label):
    b = Bench(require_live=False)
    if os.path.exists(STOP):
        os.remove(STOP)
    print(f"session '{label}': recording. Drive! Stop with Ctrl-C or: touch {STOP}")
    chunk = 0
    try:
        while not os.path.exists(STOP):
            chunk += 1
            tag = f"{label}-{chunk:03d}"
            b.cmd("recStart", label=tag)

            # Wait for the run identity before trusting anything about `recording` -
            # /state serves the pre-start snapshot until the command drains.
            t0 = time.time()
            started = False
            while time.time() - t0 < 5:
                time.sleep(0.25)
                rec = b.state().get("rec") or {}
                if rec.get("label") == tag:
                    started = True
                    break
            if not started:
                print(f"  {tag}: recStart never took effect; retrying", flush=True)
                continue

            t0 = time.time()
            while time.time() - t0 < 90:
                if os.path.exists(STOP):
                    break
                rec = b.state().get("rec") or {}
                if rec.get("label") != tag or not rec.get("recording", True):
                    break
                time.sleep(0.5)

            b.cmd("recStop")
            time.sleep(0.3)
            csv_text = b.rec_csv()
            from swervebench import _archive
            path = _archive(csv_text, {"label": tag})
            n = csv_text.count("\n") - 2
            st = b.state()
            print(f"  {tag}: {n} samples saved ({path})  V {st['voltage']:.2f}", flush=True)
    except KeyboardInterrupt:
        pass
    b.cmd("recStop")
    print(f"session '{label}' done: {chunk} chunks in runs/.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "drive")
