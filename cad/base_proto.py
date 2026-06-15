"""base_proto.py — a short/narrow FRONT section of the base plate for fast
prototyping of the gear + uprights (so you don't reprint the full ~210mm plate).

A rectangle bounding only the kept features + clearance:
  - the 4 upright mount holes (M3 clearance + counterbore from below)
  - the 2 FRONT (nose-end) feeder mounting holes
  - the Ø24 feeder nose-boss hole
Rear edge ~3mm behind the boss; sides ~3mm outside the front feeder holes; the
front edge stays at the production edge (BASE_X0) so the gear + front upright
mount exactly as they do on the full base.

    py/bin/python cad/base_proto.py   ->  build/base_proto.stl
"""
import os
from build123d import *
from base import (zcyl, BASE_TH, BASE_X0, FB_X, RB_X, INSERT_Y, CLEAR_D, CBORE_D,
                  CBORE_H, FEEDER_X, FEEDER_BOLT_SPAN, FEEDER_BOLT_W2, FEEDER_HOLE,
                  FEEDER_NOSE_X, FEEDER_NOSE_HOLE)

CLEAR = 3.0          # material behind the boss and beside the front feeder holes
FEED_FRONT_X = FEEDER_X - FEEDER_BOLT_SPAN / 2     # the nose-end feeder bolt line


def proto_plate():
    x0 = BASE_X0
    x1 = FEEDER_NOSE_X + FEEDER_NOSE_HOLE / 2 + CLEAR        # 3mm behind the nose boss
    yhalf = FEEDER_BOLT_W2 / 2 + FEEDER_HOLE / 2 + CLEAR     # 3mm beside the front feeder holes
    plate = Pos((x0 + x1) / 2, 0, -BASE_TH / 2) * Box(x1 - x0, 2 * yhalf, BASE_TH)
    holes = zcyl(FEEDER_NOSE_HOLE, FEEDER_NOSE_X, 0, -BASE_TH - 1, BASE_TH + 2)
    for hy in (FEEDER_BOLT_W2 / 2, -FEEDER_BOLT_W2 / 2):     # 2 front (nose-end) feeder bolts
        holes += zcyl(FEEDER_HOLE, FEED_FRONT_X, hy, -BASE_TH - 1, BASE_TH + 2)
    for ux in (FB_X, RB_X):                                  # upright mounts: clearance + cbore from below
        for sy in (-INSERT_Y, INSERT_Y):
            holes += zcyl(CLEAR_D, ux, sy, -BASE_TH - 1, BASE_TH + 2)
            holes += zcyl(CBORE_D, ux, sy, -BASE_TH - 0.01, CBORE_H)
    return plate - holes


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = proto_plate()
    export_stl(p, "build/base_proto.stl")
    bb = p.bounding_box()
    print("base_proto bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          " (full base is 210 x 80)")
