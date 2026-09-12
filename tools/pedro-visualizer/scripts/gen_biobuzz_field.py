#!/usr/bin/env python3
"""Render public/fields/biobuzz.webp - the FTC BIOBUZZ (2026-2027) FIELD.

Run:  python scripts/gen_biobuzz_field.py [out.webp]

WHERE THE NUMBERS COME FROM
---------------------------
Every dimension below is either quoted from the BIOBUZZ Competition Manual V1
(2026-2027 FIRST Tech Challenge, "BIOBUZZ presented by RTX") or measured off
its orthographic top view, Figure 9-2, with the 6 x 6 TILE grid as the ruler.
Manual section 9.1 puts a +/- 1 in. tolerance on every nominal dimension it
prints, so nothing here is claimed tighter than that.

  quoted from the manual
    Sec 9.2   FIELD ~144 x 144 in., 36 interlocking TILES
    Sec 9.3   LOADING ZONE ~23 in. wide x 11 in. deep, tape + adjoining wall
    Sec 9.3   GARDEN ~23 in. x 2 in., opposite corners, [2] 1 in. tape strips
              (Fig 9-3: both "23 in." spans are "Width set by TILE Seams")
    Fig 9-8   Frame width 49.46 in., frame depth 38.95 in.
    Fig 9-9   HIVE: CELL depth 12.04 in., CELL gap 18.84 in., total 42.91 in.
    Fig 9-10  HIVE center-to-center 25.5 in.; HIVE tilt 30 deg
    Fig 9-11  CELL opening 20 in. wide x 14 in. tall (peaked "house" outline)
    Fig 9-12  FLOWER top ring opening 4.0 in. dia
    Sec 9.8   POLLEN 2.8 in. dia (yellow), NECTAR 3.6 in. dia (red/blue)
    Fig 10-2  staging: 4 POLLEN per FLOWER, 4 POLLEN per GARDEN, 3 NECTAR in
              each ALLIANCE's upward-tilted CELL (pre-loads and ALLIANCE AREA
              NECTAR sit outside the FIELD, so they are not drawn)

  measured off Figure 9-2 (top view), anchored to the TILE seams
    FLOWER centres sit ON a TILE seam: 2 seams from one corner along each wall,
      going round the FIELD - top wall at 2T, right wall at 2T, bottom at 4T,
      left at 4T (T = one TILE).  The FIELD has 180 deg rotational symmetry.
    LOADING ZONE -RED spans TILE row A5 (one TILE, left wall); -BLUE is its
      180 deg rotation (right wall, F2).
    GARDEN -RED is the bottom-left corner TILE (A1); -BLUE the top-right (F6).
    FLOWER top plate reaches ~4.5 in. into the FIELD, centred ~2.7 in. off the
      wall.
    RED CELL peaks point away from the audience, BLUE CELL peaks toward it -
      each HIVE is one rigid body tilted 30 deg, so both of its CELLS lean the
      same way. The upward-tilted CELL (the one holding 3 NECTAR at staging) is
      the audience-side CELL for RED and the far-side CELL for BLUE.

FRAME AND SCALE
---------------
The visualizer stretches the field image over 0..141.5 in. in both axes
(FIELD_SIZE in src/config/defaults.ts), and decode.webp/intothedeep.webp put
the six TILE seams at exactly 1/6..5/6 of the image. So the image spans the
TILE surface edge to edge: one TILE = 141.5/6 = 23.583 in. of visualizer
coordinate, and absolute sizes (FLOWER, HIVE, balls) use that same scale.
TILE-referenced features are placed on seams, not on nominal 24 in. multiples.

Orientation matches Figure 9-2: audience at the BOTTOM of the image, RED
ALLIANCE on the LEFT. In visualizer coordinates (y up from the bottom-left)
that makes x = 0 the red wall and y = 0 the audience wall.

Palette and line weights are sampled from decode.webp so the four fields read
as one set: #2c2c2c tiles, black seams, flat fills, heavy black outlines.
"""

import math
import sys
from PIL import Image, ImageDraw

# ---------------------------------------------------------------- frame -----
OUT = sys.argv[1] if len(sys.argv) > 1 else "public/fields/biobuzz.webp"
PX = 4096                      # match decode.webp
SS = 2                         # supersample factor
FIELD_IN = 141.5               # visualizer FIELD_SIZE
S = PX * SS / FIELD_IN         # px per inch on the supersampled canvas
T = FIELD_IN / 6.0             # TILE pitch, 23.583 in
C = FIELD_IN / 2.0             # field centre, 70.75 in

# ---------------------------------------------------------------- colors ----
TILE = (44, 44, 44)            # #2c2c2c
SEAM = (0, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREY = (125, 125, 125)         # #7d7d7d structure
LGREY = (217, 217, 217)        # #d9d9d9 panels
RED = (237, 29, 38)            # #ed1d26
BLUE = (47, 49, 146)           # #2f3192
GOLD = (214, 158, 46)          # FLOWER rings
PANEL = (240, 228, 140)        # BIOBUZZ logo panels
POLLEN = (252, 209, 22)        # yellow POLLEN
DARK = (26, 26, 26)            # openings

# outline weights, inches
W_SEAM = 0.18
W_EDGE = 0.30
W_THIN = 0.22
W_OUT = 0.40


def p(v):
    """inches -> supersampled pixels"""
    return v * S


img = Image.new("RGB", (PX * SS, PX * SS), TILE)
d = ImageDraw.Draw(img)


def line(pts, fill, w, joint="curve"):
    d.line([(p(x), p(y)) for x, y in pts], fill=fill, width=max(1, int(p(w))),
           joint=joint)


def poly(pts, fill=None, outline=None, w=W_OUT, close=True):
    xy = [(p(x), p(y)) for x, y in pts]
    if fill is not None:
        d.polygon(xy, fill=fill)
    if outline is not None:
        d.line(xy + ([xy[0]] if close else []), fill=outline,
               width=max(1, int(p(w))), joint="curve")


def rect(x0, y0, x1, y1, fill=None, outline=None, w=W_OUT):
    poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], fill, outline, w)


def circle(cx, cy, dia, fill=None, outline=None, w=W_OUT):
    r = dia / 2.0
    box = [p(cx - r), p(cy - r), p(cx + r), p(cy + r)]
    d.ellipse(box, fill=fill, outline=outline,
              width=max(1, int(p(w))) if outline else 0)


def ball(cx, cy, dia, fill, dimple):
    """A SCORING ELEMENT, drawn like decode.webp's artifacts: flat fill, heavy
    black outline, ring of dimples (the balls are Gopher ResisDent, Sec 9.8)."""
    circle(cx, cy, dia, fill=BLACK)
    circle(cx, cy, dia * 0.80, fill=fill)
    dd = dia * 0.13
    circle(cx, cy, dd, fill=dimple)
    for i in range(6):
        a = math.radians(i * 60 + 15)
        circle(cx + math.cos(a) * dia * 0.27, cy + math.sin(a) * dia * 0.27,
               dd, fill=dimple)


# ------------------------------------------------------------ TILE floor ----
for i in range(1, 6):
    line([(i * T, 0), (i * T, FIELD_IN)], SEAM, W_SEAM, joint=None)
    line([(0, i * T), (FIELD_IN, i * T)], SEAM, W_SEAM, joint=None)



# --------------------------------------------------------- tape: zones ------
TAPE_W = 2.0        # Sec 9.3 allows 1 in. or 2 in. gaffers tape
LZ_DEPTH = 11.0     # Sec 9.3 / Fig 9-3


def loading_zone(side, y0, y1, color):
    """LOADING ZONE: ~23 in. (one TILE) of wall x 11 in. deep, three tape runs
    closed by the perimeter wall (Fig 9-3)."""
    if side == "left":
        x0, x1 = 0.0, LZ_DEPTH
        rect(x0, y0, x1, y0 + TAPE_W, fill=color)
        rect(x0, y1 - TAPE_W, x1, y1, fill=color)
        rect(x1 - TAPE_W, y0, x1, y1, fill=color)
    else:
        x0, x1 = FIELD_IN - LZ_DEPTH, FIELD_IN
        rect(x0, y0, x1, y0 + TAPE_W, fill=color)
        rect(x0, y1 - TAPE_W, x1, y1, fill=color)
        rect(x0, y0, x0 + TAPE_W, y1, fill=color)


# RED LOADING ZONE: left wall, TILE A5 (one TILE down from the far wall).
loading_zone("left", T, 2 * T, RED)
# BLUE LOADING ZONE: the 180 deg rotation of it.
loading_zone("right", 4 * T, 5 * T, BLUE)


def garden(x0, x1, y0, y1, color, pollen_from_left):
    """GARDEN: ~23 in. (one corner TILE) x 2 in., [2] 1 in. tape strips
    (Fig 9-3), staged with 4 POLLEN against the wall (Fig 10-2)."""
    rect(x0, y0, x1, y1, fill=color)
    line([(x0, (y0 + y1) / 2), (x1, (y0 + y1) / 2)], BLACK, 0.10, joint=None)
    cy = (y0 + 1.5) if y0 < FIELD_IN / 2 else (y1 - 1.5)
    for i in range(4):
        cx = (x0 + 1.7 + i * 2.9) if pollen_from_left else (x1 - 1.7 - i * 2.9)
        ball(cx, cy, 2.8, POLLEN, GOLD)


# GARDEN -RED: bottom-left corner TILE (A1), audience/red corner.
garden(0, T, FIELD_IN - TAPE_W, FIELD_IN, RED, True)
# GARDEN -BLUE: top-right corner TILE (F6).
garden(5 * T, FIELD_IN, 0, TAPE_W, BLUE, False)


# ---------------------------------------------------------------- FLOWER ----
FL_RING = 5.4       # outer dia of the top ring plate, measured off Fig 9-2
FL_OPEN = 4.0       # Sec 9.7: top ring opening 4.0 in. dia
FL_OFF = 2.7        # ring centre, inches off the wall
FL_MOUNT = 6.4      # wall bracket, along the wall
FL_MOUNTD = 1.4


def flower(wall, along):
    """One of the four FLOWERS, bolted to the perimeter wall (Sec 9.7), with
    the topmost of its 4 staged POLLEN showing in the opening (Fig 10-2)."""
    if wall == "top":
        cx, cy = along, FL_OFF
        rect(cx - FL_MOUNT / 2, 0, cx + FL_MOUNT / 2, FL_MOUNTD,
             fill=GREY, outline=BLACK, w=W_THIN)
    elif wall == "bottom":
        cx, cy = along, FIELD_IN - FL_OFF
        rect(cx - FL_MOUNT / 2, FIELD_IN - FL_MOUNTD, cx + FL_MOUNT / 2,
             FIELD_IN, fill=GREY, outline=BLACK, w=W_THIN)
    elif wall == "left":
        cx, cy = FL_OFF, along
        rect(0, cy - FL_MOUNT / 2, FL_MOUNTD, cy + FL_MOUNT / 2,
             fill=GREY, outline=BLACK, w=W_THIN)
    else:
        cx, cy = FIELD_IN - FL_OFF, along
        rect(FIELD_IN - FL_MOUNTD, cy - FL_MOUNT / 2, FIELD_IN,
             cy + FL_MOUNT / 2, fill=GREY, outline=BLACK, w=W_THIN)
    circle(cx, cy, FL_RING, fill=GOLD, outline=BLACK, w=W_THIN)
    circle(cx, cy, FL_OPEN, fill=DARK, outline=BLACK, w=W_THIN)
    ball(cx, cy, 2.8, POLLEN, GOLD)


flower("top", 2 * T)        # seam 2 TILES from the red corner
flower("right", 2 * T)
flower("bottom", 4 * T)
flower("left", 4 * T)


# ----------------------------------------------------------- HIVE Structure --
FRAME_W = 49.46             # Fig 9-8
FRAME_D = 38.95             # Fig 9-8
HIVE_SEP = 25.5             # Fig 9-10, HIVE centre to centre
TILT = math.radians(30.0)   # Fig 9-10
CELL_W = 20.0               # Fig 9-11
CELL_H = 14.0               # Fig 9-11
CELL_D = 12.04              # Fig 9-9
CELL_PITCH = 18.84 + CELL_D  # Fig 9-9: gap + one CELL = centre to centre

# Top-view projection of one 30-deg-tilted CELL, and of the CELL pair.
#   cell_dy: 18.1 in. from the CAD dimensions; Fig 9-2 measures 17.6 - inside
#            the manual's own +/- 1 in.
#   pair_dy: the CAD pitch projects to 26.7 in., but Fig 9-2 measures the two
#            CELLS 26.9 in. apart AND the whole pair carried 2.4 in. off the
#            pivot, toward the way that HIVE leans (RED toward the far wall,
#            BLUE toward the audience - they lean opposite ways, which is what
#            makes one CELL of each ALLIANCE face up). The figure is the only
#            source for that offset, so the measured pair is what is drawn.
cell_dy = CELL_H * math.cos(TILT) + CELL_D * math.sin(TILT)   # ~18.1 in
pair_dy = 26.86                                               # Fig 9-2
pair_lean = 2.41                                              # Fig 9-2

# Fig 9-2 measures the rails ~2.0 in. wide with 49.46 in. between their OUTER
# faces, so the frame width is outside-to-outside and each rail runs inward
# from it. rail_x is the rail centreline.
RAIL_W = 2.0
rail_x = (C - FRAME_W / 2 + RAIL_W / 2, C + FRAME_W / 2 - RAIL_W / 2)
pivot_x = (C - HIVE_SEP / 2, C + HIVE_SEP / 2)
rail_y0, rail_y1 = C - FRAME_D / 2, C + FRAME_D / 2

# Frame: two triangles standing on the TILES, leaning in to meet the crossbar.
# Seen from above each one is a base rail plus two legs running to its pivot.
for rx, px_ in zip(rail_x, pivot_x):
    line([(rx, rail_y0), (px_, C)], LGREY, 1.1)
    line([(rx, rail_y1), (px_, C)], LGREY, 1.1)
for rx, px_ in zip(rail_x, pivot_x):
    rect(rx - RAIL_W / 2, rail_y0, rx + RAIL_W / 2, rail_y1, fill=GREY,
         outline=BLACK, w=W_THIN)

# Crossbar joining the two apexes, carrying both pivots (43.95 in. up).
rect(pivot_x[0], C - 1.5, pivot_x[1], C + 1.5, fill=WHITE, outline=BLACK,
     w=W_THIN)
# BIOBUZZ panels, one on each face of the frame.
rect(pivot_x[0] + 1.0, C - 4.4, pivot_x[1] - 1.0, C - 1.9, fill=PANEL,
     outline=BLACK, w=W_THIN)
rect(pivot_x[0] + 1.0, C + 1.9, pivot_x[1] - 1.0, C + 4.4, fill=PANEL,
     outline=BLACK, w=W_THIN)


def cell(cx, cy, color, peak_up):
    """One CELL, top view: the peaked 20 in. opening of Fig 9-11 foreshortened
    by the 30 deg HIVE tilt. peak_up points the roof away from the audience."""
    hw = CELL_W / 2
    hd = cell_dy / 2
    roof = cell_dy * 0.34
    s = 1 if peak_up else -1   # image y grows toward the audience
    shoulder = cy - s * (hd - roof)
    outer = [(cx - hw, cy + s * hd), (cx + hw, cy + s * hd),
             (cx + hw, shoulder), (cx, cy - s * hd), (cx - hw, shoulder)]
    poly(outer, fill=LGREY)
    poly(outer, outline=color, w=1.5)
    poly(outer, outline=BLACK, w=W_THIN)
    # inner rim of the CELL, the second chevron visible from above
    k = 0.62
    inner = [(cx - hw * k, cy + s * hd * 0.15),
             (cx + hw * k, cy + s * hd * 0.15),
             (cx + hw * k, cy - s * (hd - roof) * 0.55),
             (cx, cy - s * hd * 0.66),
             (cx - hw * k, cy - s * (hd - roof) * 0.55)]
    poly(inner, outline=color, w=1.1)
    poly(inner, outline=BLACK, w=0.15)


for hx, color, peak_up in ((pivot_x[0], RED, True), (pivot_x[1], BLUE, False)):
    lean = -pair_lean if peak_up else pair_lean
    cell(hx, C + lean - pair_dy / 2, color, peak_up)
    cell(hx, C + lean + pair_dy / 2, color, peak_up)

# Pivot blocks last, so they sit over the CELL arms.
for px_ in pivot_x:
    rect(px_ - 1.6, C - 1.6, px_ + 1.6, C + 1.6, fill=(60, 60, 60),
         outline=BLACK, w=W_THIN)

# Staging (Fig 10-2): 3 NECTAR in each ALLIANCE's upward-tilted CELL. RED's
# upward CELL is the audience-side one, BLUE's is the far-side one.
for hx, cy, color in ((pivot_x[0], C - pair_lean + pair_dy / 2, RED),
                      (pivot_x[1], C + pair_lean - pair_dy / 2, BLUE)):
    for i in (-1, 0, 1):
        ball(hx + i * 3.8, cy, 3.6, color, BLACK)

# FIELD perimeter wall. Drawn last: the image edge is the inside face of the
# wall, and tape and FLOWERS run right up to it.
e = W_EDGE / 2
d.rectangle([p(e) - 1, p(e) - 1, p(FIELD_IN - e), p(FIELD_IN - e)],
            outline=BLACK, width=max(1, int(p(W_EDGE))))

# ---------------------------------------------------------------- output ----
img = img.resize((PX, PX), Image.LANCZOS)
img.save(OUT, "WEBP", lossless=True, quality=100, method=6)
print("wrote", OUT)
