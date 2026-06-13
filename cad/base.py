"""base.py — base for the cantilevered rotating-head architecture (build123d).

Port of base.scad with real solids: one fused body, filleted gussets, true
involute fixed gear. Frame: X = wire axis (head at -X, feeder at +X), Z = up,
base top at z=0, wire axis at z=AXIS_Z.

Run:  py/bin/python cad/base.py   ->  build/base.stl + build/base.step
"""
import os
from build123d import *
from gears import spur_gear

# ── wire axis / feed tube ───────────────────────────────────────────
AXIS_Z = 15.0          # wire axis above the base top; = 21mm above the deck (6mm plate).
                       # Lowered so the feeder sits flat on the base (its wire output is
                       # ~15mm above its mounting face) and the feed tube lines up with it.
                       # The Ø63 fixed gear dips well below the base -> hangs off the
                       # front overhang (machine is cantilevered / on a stand).
TUBE_BORE = 10.0

# ── fixed gear (head's rotation pinion meshes its outside) ──────────
FG_MODULE, FG_TEETH, FG_W = 1.5, 40, 8.0
FG_X = 0.0                       # gear front face

# ── 608 bearings for the passive feed tube ──────────────────────────
BRG_OD, BRG_W, BRG_BORE, BRG_LIP = 22.0, 7.0, 8.0, 2.0
FB_X, RB_X = 16.0, 72.0
UP_T, UP_W = 8.0, BRG_OD + 14
UP_H = AXIS_Z + BRG_OD / 2 + 6

# ── feeder (1KGSSJ-B) ───────────────────────────────────────────────
# Positioned so the feeder NOSE sits ~14mm in front of the rear bearing upright
# (its output tube then reaches ~35mm further forward into the spindle).
FEEDER_L = 122.0
FEEDER_CLEAR = 14.0                          # gap from rear upright rear-face to feeder nose
FEEDER_X = (RB_X + UP_T / 2) + FEEDER_CLEAR + FEEDER_L / 2   # nose at upright + 14mm
FEEDER_BOLT_SPAN, FEEDER_BOLT_W1, FEEDER_BOLT_W2 = 104.5, 47.3, 27.3
FEEDER_HOLE, FEEDER_MOTOR_HOLE = 5.3, 56.0
FEEDER_MOTOR_X = FEEDER_X + FEEDER_BOLT_SPAN / 2 - 32
FEEDER_NOSE_HOLE = 24.0
FEEDER_NOSE_X = FEEDER_X - FEEDER_BOLT_SPAN / 2 + 16   # 16mm BEHIND the front bolt line (inboard)
FEEDER_HOLES = [
    (FEEDER_X - FEEDER_BOLT_SPAN / 2,  FEEDER_BOLT_W2 / 2),
    (FEEDER_X - FEEDER_BOLT_SPAN / 2, -FEEDER_BOLT_W2 / 2),
    (FEEDER_X + FEEDER_BOLT_SPAN / 2,  FEEDER_BOLT_W1 / 2),
    (FEEDER_X + FEEDER_BOLT_SPAN / 2, -FEEDER_BOLT_W1 / 2),
]

# ── base plate ──────────────────────────────────────────────────────
BASE_W, BASE_TH = 80.0, 6.0
BASE_X0 = FG_X + FG_W + 2
BASE_X1 = FEEDER_X + FEEDER_L / 2 + 8


def xcyl(d, x0, x1, y=0.0, z=AXIS_Z):
    """Solid cylinder Ø d along +X from x0 to x1, centred at (y, z)."""
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(
        d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def zcyl(d, x, y, z0, h):
    """Solid cylinder Ø d along +Z from z0, at (x, y)."""
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def upright(x):
    body = Pos(x, 0, (UP_H) / 2 - 1) * Box(UP_T, UP_W, UP_H + 2)
    seat = Pos(x - UP_T / 2, 0, AXIS_Z) * Rot(0, 90, 0) * Cylinder(
        BRG_OD / 2, BRG_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    thru = xcyl(BRG_OD - 2 * BRG_LIP, x - UP_T / 2 - 0.5, x + UP_T / 2 + 0.5)
    return body - seat - thru


def fixed_gear_unit():
    gear = (Pos(FG_X, 0, AXIS_Z) * Rot(0, 90, 0)
            * spur_gear(FG_TEETH, FG_MODULE, FG_W, bore=TUBE_BORE))
    # backing annulus: gear -> front upright, around the bearing seat
    annulus = (xcyl(42, FG_X + FG_W - 0.1, FB_X - UP_T / 2 + 1)
               - xcyl(BRG_OD + 3, FG_X - 1, FB_X + 1))
    return gear + annulus


def base_plate():
    plate = Pos((BASE_X0 + BASE_X1) / 2, 0, -BASE_TH / 2) * Box(
        BASE_X1 - BASE_X0, BASE_W, BASE_TH)
    holes = zcyl(FEEDER_MOTOR_HOLE, FEEDER_MOTOR_X, 0, -BASE_TH - 1, BASE_TH + 2)
    holes += zcyl(FEEDER_NOSE_HOLE, FEEDER_NOSE_X, 0, -BASE_TH - 1, BASE_TH + 2)
    for (hx, hy) in FEEDER_HOLES:
        holes += zcyl(FEEDER_HOLE, hx, hy, -BASE_TH - 1, BASE_TH + 2)
    for bx in (BASE_X0 + 12, RB_X - 4):
        for by in (-BASE_W / 2 + 6, BASE_W / 2 - 6):
            holes += zcyl(5, bx, by, -BASE_TH - 1, BASE_TH + 2)
    return plate - holes


def build_base():
    # frame is one fused solid; the fixed gear is kept as a separate touching solid
    # (it connects to the front upright via its backing annulus). Fusing the
    # many-faced involute gear into the frame is slow AND drops the teeth in OCC,
    # so it rides along as a Compound child — still one printable object.
    frame = base_plate() + upright(FB_X) + upright(RB_X)
    return Compound(children=[frame, fixed_gear_unit()])


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    base = build_base()
    export_stl(base, "build/base.stl")
    export_step(base, "build/base.step")
    bb = base.bounding_box()
    print("base volume", round(base.volume, 1),
          "bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)])
