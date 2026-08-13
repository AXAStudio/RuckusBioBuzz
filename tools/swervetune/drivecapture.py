"""Continuous pod capture while a human drives.

The recorder holds 3000 samples and stops when full rather than wrapping, so at ~90 Hz a
single run is about 33 seconds. That is deliberate - a ring that overwrote the interesting
part would be worse than a short trace. To cover a whole driving session this restarts the
recorder as soon as each chunk fills, and writes every chunk to runs/ as it lands.

There is a gap of roughly a second between chunks while the hub renders 3000 rows of CSV.
Shake is a sustained behaviour rather than a one-off event, so a periodic one-second hole is
acceptable here in a way it would not be for step-response work.

Stops when runs/DRIVE_STOP appears, or on Ctrl-C.

    python drivecapture.py <session-label>
"""

import gzip
import os
import sys
import time

from swervebench import Bench, parse_csv

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")
STOP = os.path.join(RUNS, "DRIVE_STOP")

# The hub's WiFi drops for a second or two fairly often, and a driving session is exactly when
# nobody wants to be told the capture died twenty seconds ago. Every call goes through a retry:
# a transient drop costs a sample, not the session.
RETRY_S = 25.0


def retry(fn, *a, **kw):
    """Runs fn until it succeeds or RETRY_S elapses. Returns None if it never does."""
    t0 = time.time()
    while time.time() - t0 < RETRY_S:
        try:
            return fn(*a, **kw)
        except Exception:  # noqa: BLE001 - any transport failure is worth another go
            if os.path.exists(STOP):
                return None
            time.sleep(0.5)
    return None


def main() -> int:
    label = sys.argv[1] if len(sys.argv) > 1 else "drive"
    os.makedirs(RUNS, exist_ok=True)
    if os.path.exists(STOP):
        os.remove(STOP)

    b = retry(Bench)
    if b is None:
        print("no link to the robot", flush=True)
        return 1

    st = retry(b.state) or {}
    if st.get("mode") != "DRIVE":
        print(f"warning: mode is {st.get('mode')}, not DRIVE", flush=True)
    print(f"session {label}  |  V {st['voltage']:.2f}  |  "
          f"kS {st['pods'][0]['ks']}  kP {st['pods'][0]['kp']}  kD {st['pods'][0]['kd']}",
          flush=True)
    print(f"stop with: touch {STOP}", flush=True)

    chunk = 0
    try:
        while not os.path.exists(STOP):
            chunk += 1
            tag = f"{label}-{chunk:03d}"
            if retry(b.cmd, "recStart", label=tag) is None and not os.path.exists(STOP):
                print(f"  {tag}: link down through recStart; retrying", flush=True)
                continue

            # Commands are queued and drained on the OpMode loop, and /state serves the last
            # published snapshot. Reading it straight after recStart returns the pre-start
            # snapshot, where recording is still false from the previous recStop - which reads
            # identically to "this chunk already finished". Wait for the label to come back
            # before believing anything about recording, so run identity settles the ambiguity
            # rather than timing.
            t0 = time.time()
            started_ok = False
            while time.time() - t0 < 5:
                time.sleep(0.25)
                st = retry(b.state)
                rec = (st or {}).get("rec") or {}
                if rec.get("label") == tag:
                    started_ok = True
                    break
            if not started_ok:
                print(f"  {tag}: recStart never took effect; retrying", flush=True)
                continue

            # Poll rather than sleeping a fixed 33 s: loop rate varies with what the pods are
            # doing, so the fill time is not a constant and a fixed wait would either clip the
            # chunk or idle after it filled.
            t0 = time.time()
            while time.time() - t0 < 90:
                if os.path.exists(STOP):
                    break
                st = retry(b.state)
                if st is None:
                    break
                rec = st.get("rec") or {}
                if rec.get("label") != tag or not rec.get("recording", True):
                    break
                time.sleep(0.5)

            retry(b.cmd, "recStop")
            time.sleep(0.2)

            csv_text = retry(b.rec_csv)
            if csv_text is None:
                print(f"  {tag}: pull failed after retries; continuing", flush=True)
                continue

            try:
                cols = parse_csv(csv_text)
            except Exception as exc:  # noqa: BLE001
                print(f"  {tag}: unparseable ({exc}); continuing", flush=True)
                continue

            n = len(cols.get("t", []))
            if n == 0:
                print(f"  {tag}: empty; continuing", flush=True)
                continue

            stamp = time.strftime("%Y%m%d-%H%M%S")
            path = os.path.join(RUNS, f"{stamp}_{tag}.csv.gz")
            with gzip.open(path, "wt", newline="") as fh:
                fh.write(csv_text)

            span = cols["t"][-1] - cols["t"][0]
            # 1/mean(dt), not mean(loopHz): loopHz is instantaneous and averaging it reads ~1.8x
            # optimistic.
            dts = [x for x in cols.get("dt", []) if x == x and x > 0]
            volts = [x for x in cols.get("volts", []) if x == x]
            print(f"  {tag}: {n} samples, {span:.1f}s, "
                  f"{(len(dts) / sum(dts) if dts else float('nan')):.0f} Hz, "
                  f"{(sum(volts) / len(volts) if volts else float('nan')):.2f} V "
                  f"-> {os.path.basename(path)}", flush=True)
    except KeyboardInterrupt:
        print("interrupted", flush=True)
    finally:
        retry(b.cmd, "recStop")

    print(f"done: {chunk} chunks", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
