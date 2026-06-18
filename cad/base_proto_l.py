"""base_proto_l.py — L-shaped proto of the NEW base plate, for test-fitting the mounts
without printing the full ~210x87mm deck.

The new deck's printed-part mounts are two groups, perpendicular in layout:
  - the 2 bearing-upright mounts (front, on the centreline: FB_X/RB_X at y=±INSERT_Y),
    M3 clearance + counterbore from below
  - the vertical feeder-bracket mounts (rear, offset out at y=FEEDER_BR_HOLE_Y:
    x=FEEDER_X±FEEDER_BR_INS_X), same M3 clearance + counterbore

We bound just those with an L:
  - a long ARM along X at the feeder line (y≈FEEDER_BR_HOLE_Y) reaching the far feeder hole
  - a front BLOCK around the bearing mounts, dropping down to the centreline,
that share a corner so it prints as one piece. The front edge stays at BASE_X0 so the
uprights/gear seat exactly as on the full deck. Holes come straight from base.py so the
proto's spacing matches production.

    py/bin/python cad/base_proto_l.py   ->  build/base_proto_l.stl
"""
import os
from build123d import *
from base import (zcyl, BASE_TH, BASE_X0, FB_X, RB_X, INSERT_Y, CLEAR_D, CBORE_D, CBORE_H,
                  FEEDER_X, FEEDER_BR_INS_X, FEEDER_BR_HOLE_Y)

M = 6.0                          # material margin around holes (Ø6 cbore -> ~3mm wall)
ARM_HW = 8.0                     # half-width (Y) of the feeder arm
YB = INSERT_Y + M                # 18 — the bearing block reaches y = ±YB
YF = FEEDER_BR_HOLE_Y            # 41 — feeder mount line


def _slab(x0, x1, ya, yb):
    y0, y1 = min(ya, yb), max(ya, yb)
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, -BASE_TH / 2) * Box(x1 - x0, y1 - y0, BASE_TH)


def proto_plate():
    far_x = FEEDER_X + FEEDER_BR_INS_X + M          # past the far feeder hole
    bear_x = RB_X + M                               # past the rear bearing mount
    sgn = 1.0 if YF >= 0 else -1.0                  # feeder edge side (+Y or −Y)
    # feeder ARM (long, along X) + bearing BLOCK (front, reaching from the opposite side past
    # the bearings to overlap the arm by 2mm), fused into an L
    plate = _slab(BASE_X0, far_x, YF - ARM_HW, YF + ARM_HW)          # arm at the feeder line
    plate += _slab(BASE_X0, bear_x, -sgn * YB, (YF - sgn * ARM_HW) + sgn * 2)   # block -> arm

    cuts = []
    for ux in (FB_X, RB_X):                                          # bearing mounts
        for sy in (-INSERT_Y, INSERT_Y):
            cuts.append(zcyl(CLEAR_D, ux, sy, -BASE_TH - 1, BASE_TH + 2))
            cuts.append(zcyl(CBORE_D, ux, sy, -BASE_TH - 0.01, CBORE_H))
    for sx in (FEEDER_BR_INS_X, -FEEDER_BR_INS_X):                   # feeder-bracket mounts
        hx = FEEDER_X + sx
        cuts.append(zcyl(CLEAR_D, hx, FEEDER_BR_HOLE_Y, -BASE_TH - 1, BASE_TH + 2))
        cuts.append(zcyl(CBORE_D, hx, FEEDER_BR_HOLE_Y, -BASE_TH - 0.01, CBORE_H))
    holes = cuts[0]
    for c in cuts[1:]:
        holes += c
    return plate - holes


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = proto_plate()
    export_stl(p, "build/base_proto_l.stl")
    import trimesh
    m = trimesh.load("build/base_proto_l.stl")
    bb = p.bounding_box()
    print("base_proto_l bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          f" 2 bearing + 2 feeder-bracket mounts  bodies:{len(m.split(only_watertight=False))}"
          f"  watertight:{m.is_watertight}")
