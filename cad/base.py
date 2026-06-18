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
AXIS_Z = 30.0          # wire axis = bearing-bore CENTRE, above the deck top (z=0). The feeder
                       # now stands vertically on feeder_bracket with its output on this axis
                       # (feeder_bracket.CENTER_Z tracks AXIS_Z). Lowered 35->30 in this
                       # orientation, dropping the bearing uprights 5mm. NOTE: the fixed gear is
                       # coaxial here — see the clearance print at the bottom of this file.
TUBE_BORE = 10.0

# ── fixed gear (head's rotation pinion meshes its outside) ──────────
FG_MODULE, FG_TEETH, FG_W = 1.5, 40, 8.0
FG_X = 0.0                       # gear front face

# ── 608 bearings for the passive feed tube ──────────────────────────
BRG_OD, BRG_W, BRG_BORE, BRG_LIP = 22.0, 7.0, 8.0, 2.0
FB_X, RB_X = 16.0, 72.0
UP_T, UP_W = 8.0, BRG_OD + 14
UP_H = AXIS_Z + BRG_OD / 2 + 6

# ── front-upright stiffening foot (REARWARD only; the deck front edge + the fixed gear
#    hanging in front block any −X foot). Turns the thin blade into a braced L-bracket. ──
FRONT_FOOT_BACK = 16.0           # foot reaches this far behind the blade back face (+X)
FRONT_FOOT_T = 6.0               # foot slab thickness (>= INSERT_H so it holds the M3 inserts)
FRONT_FOOT_BOLT_X = FB_X + UP_T / 2 + FRONT_FOOT_BACK - 5.0   # rear foot insert X (=31)
GUSSET_T = 4.0                   # gusset rib thickness (Y)
GUSSET_RISE = 22.0               # gusset height up the blade back (stays below the bearing bore)
GUSSET_Y = 12.0                  # gussets at ±this (over the bolt lines)

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

# ── vertical feeder bracket (feeder_bracket.py) placement on the deck ──
# The feeder now stands vertically against feeder_bracket; the bracket bolts to the deck
# like an upright. These define the deck-side mount (single source — feeder_bracket imports
# them). The bracket runs along X centred on FEEDER_X; its mounting FACE sits FEEDER_BR_FACE_Y
# off the centreline on the −Y edge and its body is FEEDER_BR_T thick, so the bottom-edge inserts
# (and thus the deck holes) sit FEEDER_BR_HOLE_Y out — a little past the face by the half-thickness.
FEEDER_BR_FACE_Y = -35.0                                 # mounting-face offset from the wire axis (−Y edge)
FEEDER_BR_T = 12.0                                       # bracket plate thickness
FEEDER_BR_INS_X = 58.0                                   # bottom-edge mount spacing (x = ±this)
FEEDER_BR_HOLE_Y = FEEDER_BR_FACE_Y - FEEDER_BR_T / 2    # −41 — deck holes a half-thickness further out (−Y)

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


def _front_foot(x):
    """Rearward foot slab fused to the bottom of the front blade (z = 0..FRONT_FOOT_T)."""
    x0, x1 = x - UP_T / 2, x + UP_T / 2 + FRONT_FOOT_BACK
    return Pos((x0 + x1) / 2, 0, FRONT_FOOT_T / 2) * Box(x1 - x0, UP_W, FRONT_FOOT_T)


def _gusset(x, yc):
    """A right-triangle rib (thickness GUSSET_T at y=yc) tying the blade back face up to
    the foot top: vertical leg up the blade, horizontal leg out along the foot."""
    x0 = x + UP_T / 2                              # blade back face
    x1 = x0 + FRONT_FOOT_BACK                      # foot rear edge
    z0, z1 = FRONT_FOOT_T, FRONT_FOOT_T + GUSSET_RISE
    with BuildPart() as bp:
        with BuildSketch(Plane.XZ):                # local (a,b) -> global (x=a, z=b)
            with BuildLine():
                Polyline((x0, z0), (x0, z1), (x1, z0), close=True)
            make_face()
        extrude(amount=GUSSET_T / 2, both=True)    # centred on y=0, thickness GUSSET_T
    return Pos(0, yc, 0) * bp.part


def upright(x, front=False):
    """A bearing upright, printed on its own. Bottom face flat on the plate (z=0)
    with two heat-set inserts (screws up from under the plate). The FRONT upright also
    gets a rearward stiffening foot + gussets (anti-tip) and the gear bolt-circle holes."""
    part = Pos(x, 0, UP_H / 2) * Box(UP_T, UP_W, UP_H)        # blade, bottom at z=0
    if front:                                                 # add the foot + gussets BEFORE drilling
        part += _front_foot(x)
        for gy in (-GUSSET_Y, GUSSET_Y):
            part += _gusset(x, gy)
    # bearing seat + through bore cut AFTER the gussets so the bore stays clear at the blade back
    part -= Pos(x - UP_T / 2, 0, AXIS_Z) * Rot(0, 90, 0) * Cylinder(
        BRG_OD / 2, BRG_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= xcyl(BRG_OD - 2 * BRG_LIP, x - UP_T / 2 - 0.5, x + UP_T / 2 + 0.5)
    for sy in (-INSERT_Y, INSERT_Y):                          # blade bottom-face inserts
        part -= Pos(x, sy, 0) * Cylinder(INSERT_D / 2, INSERT_H,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))
    if front:
        for sy in (-INSERT_Y, INSERT_Y):                      # rear foot inserts
            part -= Pos(FRONT_FOOT_BOLT_X, sy, 0) * Cylinder(INSERT_D / 2, INSERT_H,
                                                             align=(Align.CENTER, Align.CENTER, Align.MIN))
        xf, xb = x - UP_T / 2, x + UP_T / 2                    # gear mount: plain clearance through holes
        for (Y, Z) in _gear_bolts_world():
            # just a through hole — the captive hex-nut traps were dropped because the lowered
            # axis (AXIS_Z 35->30) brought the gear bolt circle into the bearing seat; fasten
            # with a loose nut/washer on the back face instead.
            part -= xcyl(CLEAR_D, xf - 0.5, xb + 0.5, y=Y, z=Z)
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
    # extension so the deck reaches under the vertical feeder bracket's mount holes
    # (they sit at y=FEEDER_BR_HOLE_Y, just past the plate's ∓BASE_W/2 edge); sign follows the edge
    sgn = 1.0 if FEEDER_BR_HOLE_Y >= 0 else -1.0
    ext_in = sgn * (BASE_W / 2 - 2)                 # overlap the deck edge on the bracket side
    ext_out = FEEDER_BR_HOLE_Y + sgn * 6            # 6mm past the holes
    plate += Pos(FEEDER_X, (ext_in + ext_out) / 2, -BASE_TH / 2) * Box(
        2 * (FEEDER_BR_INS_X + 8), abs(ext_out - ext_in), BASE_TH)

    cuts = []
    # benchtop mounting holes
    for bx in (BASE_X0 + 12, RB_X - 4):
        for by in (-BASE_W / 2 + 6, BASE_W / 2 - 6):
            cuts.append(zcyl(5, bx, by, -BASE_TH - 1, BASE_TH + 2))
    # upright mounts: clearance + counterbore from UNDER the plate, under each upright
    for ux in (FB_X, RB_X):
        for sy in (-INSERT_Y, INSERT_Y):
            cuts.append(zcyl(CLEAR_D, ux, sy, -BASE_TH - 1, BASE_TH + 2))
            cuts.append(zcyl(CBORE_D, ux, sy, -BASE_TH - 0.01, CBORE_H))
    # front-upright rearward foot: 2 extra mounts at the foot rear (widens the X bolt base)
    for sy in (-INSERT_Y, INSERT_Y):
        cuts.append(zcyl(CLEAR_D, FRONT_FOOT_BOLT_X, sy, -BASE_TH - 1, BASE_TH + 2))
        cuts.append(zcyl(CBORE_D, FRONT_FOOT_BOLT_X, sy, -BASE_TH - 0.01, CBORE_H))
    # vertical feeder bracket: 2 counterbored mounts (matches the bracket's bottom-edge inserts)
    for sx in (FEEDER_BR_INS_X, -FEEDER_BR_INS_X):
        hx = FEEDER_X + sx
        cuts.append(zcyl(CLEAR_D, hx, FEEDER_BR_HOLE_Y, -BASE_TH - 1, BASE_TH + 2))
        cuts.append(zcyl(CBORE_D, hx, FEEDER_BR_HOLE_Y, -BASE_TH - 0.01, CBORE_H))
    holes = cuts[0]
    for c in cuts[1:]:
        holes += c
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
    # fixed gear is coaxial on the wire axis; its OD dips to gear_bottom in z. That only fouls
    # the deck if the deck is actually UNDER it in x — but the gear (x=FG_X..FG_X+FG_W) sits in
    # front of the deck's front edge (BASE_X0 = FG_X+FG_W+2), so a radial dip hangs over empty air.
    gear_bottom = AXIS_Z - FG_MODULE * (FG_TEETH + 2) / 2
    deck_under_gear = BASE_X0 < FG_X + FG_W
    note = (f"DIPS {-gear_bottom:.1f}mm below the deck and the deck runs under it — needs a clearance cut"
            if gear_bottom < 0 and deck_under_gear
            else f"dips to z={gear_bottom:.1f} but clears (deck starts at x={BASE_X0:.0f}, in front of the gear)"
            if gear_bottom < 0 else "clears")
    print(f"fixed gear bottom z={gear_bottom:.1f} (deck top=0): {note}")
