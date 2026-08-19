"""Full simulated-joystick verification of DRIVE mode: every direction, both rotations, combos.

For each of 8 robot-frame translation directions: command it like a held stick, measure via
odometry (a) the angle between commanded and actual motion in the robot frame, (b) heading
drift while translating (the soft heading lock's report card). Then pure rotation both ways,
then translate+rotate combos. Every leg is bounds-checked against the box BEFORE it runs, with
a recenter when room runs short - the robot never moves toward a wall it cannot stop for.

    python drivecheck.py
"""

from __future__ import annotations

import math
import time

from swervebench import Bench
from boxdrive import Driver, wrap180

POWER = 0.28
LEG_S = 1.2
MARGIN = 9.0


def main():
    b = Bench()
    d = Driver(b)
    box = d  # inset bounds already in Driver

    def pose():
        st = b.state()
        return st["pose"]["x"], st["pose"]["y"], math.radians(st["heading"]["deg"]), st

    def recenter():
        cx = (d.x0 + d.x1) / 2
        cy = (d.y0 + d.y1) / 2
        d.drive_to(cx, cy, 0.3, 6.0)
        d.cmd(0, 0, 0)
        time.sleep(1.0)

    def room_ok(x, y, fdir_x, fdir_y, dist):
        tx, ty = x + fdir_x * dist, y + fdir_y * dist
        return (d.x0 - 2 <= tx <= d.x1 + 2) and (d.y0 - 2 <= ty <= d.y1 + 2)

    def leg(f, s, t, dur):
        t0 = time.time()
        while time.time() - t0 < dur:
            d.cmd(f, s, t)
            time.sleep(0.07)
        d.cmd(0, 0, 0)
        time.sleep(0.55)

    dirs = [("fwd", 1, 0), ("fwd-left", 0.707, 0.707), ("left", 0, 1),
            ("back-left", -0.707, 0.707), ("back", -1, 0), ("back-right", -0.707, -0.707),
            ("right", 0, -1), ("fwd-right", 0.707, -0.707)]

    print(f"{'direction':<11} {'dist':>5} {'dir err':>8} {'hdg drift':>9}  verdict")
    worst_dir = 0.0
    worst_hdg = 0.0
    for (name, f, s) in dirs:
        x, y, h, _ = pose()
        # expected field direction of this robot-frame command (f fwd, s left)
        ex = f * math.cos(h) - s * math.sin(h)
        ey = f * math.sin(h) + s * math.cos(h)
        need = 12.0
        if not room_ok(x, y, ex, ey, need):
            recenter()
            x, y, h, _ = pose()
            ex = f * math.cos(h) - s * math.sin(h)
            ey = f * math.sin(h) + s * math.cos(h)
            if not room_ok(x, y, ex, ey, need):
                print(f"{name:<11}  SKIP - no room even from centre")
                continue
        t0 = time.time()
        while time.time() - t0 < LEG_S / 2:
            d.cmd(f * POWER, s * POWER, 0)
            time.sleep(0.07)
        xm, ym, hm, _ = pose()
        while time.time() - t0 < LEG_S:
            d.cmd(f * POWER, s * POWER, 0)
            time.sleep(0.07)
        d.cmd(0, 0, 0)
        time.sleep(0.55)
        x1, y1, h1, _ = pose()
        dx, dy = x1 - xm, y1 - ym
        dist = math.hypot(dx, dy)
        if dist < 2:
            print(f"{name:<11} {dist:>5.1f}  did not move enough to judge")
            continue
        derr = math.degrees(math.atan2(ey * dx - ex * dy, ex * dx + ey * dy))
        hdrift = abs(wrap180(math.degrees(h1 - h)))
        worst_dir = max(worst_dir, abs(derr))
        worst_hdg = max(worst_hdg, hdrift)
        ok = "OK" if abs(derr) < 6 and hdrift < 2 else "CHECK"
        print(f"{name:<11} {dist:>5.1f} {derr:>+7.1f}d {hdrift:>8.2f}d  {ok}")
        # return leg to roughly the start
        leg(-f * POWER, -s * POWER, 0, LEG_S)

    # pure rotation both ways
    for (name, t) in (("rotate CCW", 0.3), ("rotate CW", -0.3)):
        x, y, h, _ = pose()
        leg(0, 0, t, 1.2)
        x1, y1, h1, _ = pose()
        dh = wrap180(math.degrees(h1 - h))
        slide = math.hypot(x1 - x, y1 - y)
        ok = "OK" if (dh > 15 if t > 0 else dh < -15) and slide < 3 else "CHECK"
        print(f"{name:<11} swept {dh:+6.1f}d  slide {slide:4.1f} in  {ok}")

    # combo: forward + rotate (arc) both ways
    for (name, t) in (("arc CCW", 0.25), ("arc CW", -0.25)):
        recenter()
        x, y, h, _ = pose()
        leg(POWER, 0, t, 1.0)
        x1, y1, h1, _ = pose()
        dh = wrap180(math.degrees(h1 - h))
        dist = math.hypot(x1 - x, y1 - y)
        ok = "OK" if dist > 4 and (dh > 8 if t > 0 else dh < -8) else "CHECK"
        print(f"{name:<11} moved {dist:4.1f} in, swept {dh:+6.1f}d  {ok}")

    recenter()
    b.cmd("stop")
    print(f"\nworst direction error {worst_dir:.1f} deg, worst heading drift {worst_hdg:.2f} deg")


if __name__ == "__main__":
    main()
