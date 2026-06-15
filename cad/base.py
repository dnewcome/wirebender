"""base.py — base for the cantilevered rotating-head architecture (build123d).

Now built as SEPARATELY PRINTED parts joined with M3 heat-set inserts:
    base_plate   the flat deck (feeder + benchtop + upright mount holes)
    upright x2   bolt UP from under the plate into inserts in their bottom faces
    gear         bolts onto the FRONT upright's face (coaxial with the bearing)

Frame: X = wire axis (head at -X, feeder at +X), Z = up, base top at z=0, wire
axis at z=AXIS_Z. base.py emits the full assembly (build/base.stl, used by the
sim) AND each part on its own (build/base_plate.stl, upright_front/rear.stl,
gear.stl) ready to slice.

Run:  py/bin/python cad/base.py
"""
import os
import math
from build123d import *
from gears import spur_gear

# ── wire axis / feed tube ───────────────────────────────────────────
AXIS_Z = 35.0          # wire axis = bearing-bore CENTRE, measured above the deck top (z=0).
                       # The feeder sits FLAT on the deck and its output axis is 35mm above
                       # its mounting face, so the feed tube lines up at z=35. At 35 the Ø63
                       # fixed gear clears the deck (bottom ~3.5mm above z=0).
TUBE_BORE = 10.0

# ── fixed gear (head's rotation pinion meshes its outside) ──────────
FG_MODULE, FG_TEETH, FG_W = 1.5, 40, 8.0
FG_X = 0.0                       # gear front face

# ── 608 bearings for the passive feed tube ──────────────────────────
BRG_OD, BRG_W, BRG_BORE, BRG_LIP = 22.0, 7.0, 8.0, 2.0
FB_X, RB_X = 16.0, 72.0
UP_T, UP_W = 8.0, BRG_OD + 14
UP_H = AXIS_Z + BRG_OD / 2 + 6

# ── fasteners (M3 heat-set inserts; uprights from below, gear from the front) ──
INSERT_D, INSERT_H = 4.6, 5.0    # heat-set insert hole (M3 brass, larger ~5mm-OD family)
CLEAR_D = 3.4                    # M3 screw clearance
CBORE_D, CBORE_H = 6.0, 3.0      # socket-head counterbore (upright bottom mounts)
CSK_D = 6.5                      # M3 countersunk (90° flat) head Ø — the gear screws
NUT_AF, NUT_T = 5.8, 2.6         # M3 hex-nut trap (across-flats, depth) on the upright back —
                                 # gear screws go THROUGH the upright into these (no inserts
                                 # near the bearing seat, which heat-set inserts were deforming)
INSERT_Y = 12.0                  # upright bottom-bolt spread (±Y)
R_GEAR_BOLT = 14.0               # gear bolt-circle radius (outside the Ø22 bearing seat)
GEAR_BOLT_ANG = (180, 60, -60)   # 3 bolts: one up, two lower
SPACER_OD, SPACER_H = 30.0, 4.0  # separate spacer ring: spans the gap to the upright face
SPACER_BORE = 14.0               # clears the bearing inner race when bolted down (not just the tube)

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


# ── uprights ────────────────────────────────────────────────────────


def upright(x, front=False):
    """A bearing upright, printed on its own. Bottom face flat on the plate (z=0)
    with two heat-set inserts (screws up from under the plate). The FRONT upright
    also gets the gear bolt-circle inserts on its front face."""
    body = Pos(x, 0, UP_H / 2) * Box(UP_T, UP_W, UP_H)        # bottom at z=0
    seat = Pos(x - UP_T / 2, 0, AXIS_Z) * Rot(0, 90, 0) * Cylinder(
        BRG_OD / 2, BRG_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    thru = xcyl(BRG_OD - 2 * BRG_LIP, x - UP_T / 2 - 0.5, x + UP_T / 2 + 0.5)
    part = body - seat - thru
    for sy in (-INSERT_Y, INSERT_Y):                          # bottom-face inserts
        part -= Pos(x, sy, 0) * Cylinder(INSERT_D / 2, INSERT_H,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))
    if front:                                                 # gear mount: through holes + nut traps
        xf, xb = x - UP_T / 2, x + UP_T / 2                    # front / back faces
        for (Y, Z) in _gear_bolts_world():
            part -= xcyl(CLEAR_D, xf - 0.5, xb + 0.5, y=Y, z=Z)            # clearance all the way through
            part -= (Pos(xb, Y, Z) * Rot(0, -90, 0)                       # captive hex-nut trap on the back
                     * extrude(RegularPolygon(NUT_AF / 2 / math.cos(math.radians(30)), 6), NUT_T))
    return part


# ── fixed gear (separate print, bolts to the front upright) ─────────


def _gear_bolts_native():
    """Bolt-circle positions in the gear's own (printed, axis=+Z) XY plane."""
    return [(R_GEAR_BOLT * math.cos(math.radians(a)),
             R_GEAR_BOLT * math.sin(math.radians(a))) for a in GEAR_BOLT_ANG]


def _gear_bolts_world():
    """Same bolts mapped to the upright front face (Y, Z), via the assembly
    placement Pos(0,0,AXIS_Z)*Rot(0,90,0): native (xn,yn) -> (yn, AXIS_Z - xn)."""
    return [(yn, AXIS_Z - xn) for (xn, yn) in _gear_bolts_native()]


def gear_part():
    """The fixed gear as a standalone print: just the toothed disk with three
    COUNTERSUNK (90° flat-head) bolt holes — no hub. Native orientation
    (axis = +Z, countersinks open on the z=0 front face), ready to slice."""
    part = spur_gear(FG_TEETH, FG_MODULE, FG_W, bore=TUBE_BORE)       # z = 0..FG_W
    for (bx, by) in _gear_bolts_native():
        part -= Pos(bx, by, -1) * Cylinder(CLEAR_D / 2, FG_W + 2,
                                           align=(Align.CENTER, Align.CENTER, Align.MIN))
        part -= Pos(bx, by, -0.5) * Cone(CSK_D / 2 + 0.5, 0, CSK_D / 2 + 0.5,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))
    return part


def gear_spacer():
    """Spacer ring (was the gear's rear hub), printed separately. Sits between the
    gear back and the front upright; bolt holes are plain screw clearance."""
    ring = Cylinder(SPACER_OD / 2, SPACER_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ring -= Pos(0, 0, -1) * Cylinder(SPACER_BORE / 2, SPACER_H + 2,
                                     align=(Align.CENTER, Align.CENTER, Align.MIN))
    for (bx, by) in _gear_bolts_native():
        ring -= Pos(bx, by, -1) * Cylinder(CLEAR_D / 2, SPACER_H + 2,
                                           align=(Align.CENTER, Align.CENTER, Align.MIN))
    return ring


def gear_in_place():
    """The gear positioned on the wire axis at the front (axis along +X)."""
    return Pos(0, 0, AXIS_Z) * Rot(0, 90, 0) * gear_part()


def spacer_in_place():
    """The spacer ring between the gear back (z=FG_W) and the upright face."""
    return Pos(0, 0, AXIS_Z) * Rot(0, 90, 0) * Pos(0, 0, FG_W) * gear_spacer()


# ── base plate ──────────────────────────────────────────────────────


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
    # upright mounts: clearance + counterbore from UNDER the plate, under each upright
    for ux in (FB_X, RB_X):
        for sy in (-INSERT_Y, INSERT_Y):
            holes += zcyl(CLEAR_D, ux, sy, -BASE_TH - 1, BASE_TH + 2)
            holes += zcyl(CBORE_D, ux, sy, -BASE_TH - 0.01, CBORE_H)
    return plate - holes


def build_base():
    """Full assembly as a Compound (touching parts, not fused) — the sim uses this."""
    return Compound(children=[base_plate(),
                              upright(FB_X, front=True),
                              upright(RB_X),
                              gear_in_place(),
                              spacer_in_place()])


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    base = build_base()
    parts = {
        "base.stl": base,                              # full assembly (sim uses this)
        "base_plate.stl": base_plate(),
        "upright_front.stl": upright(FB_X, front=True),
        "upright_rear.stl": upright(RB_X),
        "gear.stl": gear_part(),                       # native +Z, print-ready
        "gear_spacer.stl": gear_spacer(),
    }
    for fn, p in parts.items():
        export_stl(p, f"build/{fn}")
    export_step(base, "build/base.step")
    bb = base.bounding_box()
    print("base assembly bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          "| parts:", ", ".join(k for k in parts if k != "base.stl"))
