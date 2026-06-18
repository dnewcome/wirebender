"""feeder_bracket.py — flat upright plate that holds the wire feeder VERTICALLY.

The feeder (1KGSSJ-B) used to lie flat on the deck, bolted down through its 4-hole pattern
(FEEDER_BOLT_SPAN x FEEDER_BOLT_W1/W2) with two boss clearances (the Ø56 motor bulge and
the Ø24 nose) opening through the deck. We now stand the feeder up on its side against this
flat plate, so its wire output emerges horizontally on the machine centre.

It is just a flat plate — no foot, no gussets. It bolts to the base plate the same way the
bearing uprights do: M3 heat-set inserts in its BOTTOM EDGE (z=0), screws coming up from
under the deck (the deck carries the matching clearance + counterbore).

The plate carries the WHOLE feeder hole set the deck used to, rotated 90° into the vertical
face (local x = the SPAN direction, nose at -X; the bolt widths run up the wall in Z):
  - 4 feeder mounting holes -> M5 clearance THROUGH holes in the face (bolt + nut on the back)
  - the nose boss clearance (Ø24) bored through the wall along Y so that protrusion passes through.

The LARGER (motor Ø56) boss is no longer a through-hole. We mount the drive NEMA17 directly
to the plate there, so that boss region becomes a sandwich:
  - FRONT (feeder side): a round Ø56 pocket so the feeder still seats flat against the face.
  - MID-PLANE: a thin 1.5mm web carrying a SLOTTED NEMA17 pattern (4 bolt slots + a centre
    pilot/shaft slot). The motor screws come in from the front into the motor's tapped face;
    sliding it along the slots sets the pinion/feeder-gear mesh, and the surrounding full-thickness
    plate keeps the thin web from flexing.
  - BACK: a rectangular pocket that recesses the NEMA17 body and clears its Ø22 pilot boss.
The motor sits on the BACK, its shaft/pinion reaching forward through the web into the front
pocket to mesh with the feeder gear.

Placement the assembly applies (drawn here with the mounting face at y=0):
  - mounting face offset FACE_Y (−35mm) on the −Y edge; the feeder hangs on the +Y (centre)
    side and its output (35mm off the face) lands on the centreline y=0.
  - bolt-pattern lengthwise centreline (= feeder output / wire axis) at CENTER_Z (30mm)
    above the deck top -> wire axis at (y=0, z=30).

    py/bin/python cad/feeder_bracket.py   ->  build/feeder_bracket.stl
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import Align, Axis, Box, Cylinder, Pos, export_stl
from base import (FEEDER_BOLT_SPAN, FEEDER_BOLT_W1, FEEDER_BOLT_W2, FEEDER_HOLE,
                  FEEDER_X, FEEDER_MOTOR_X, FEEDER_MOTOR_HOLE, FEEDER_NOSE_X, FEEDER_NOSE_HOLE,
                  INSERT_D as M3_INS_D, INSERT_H as M3_INS_H,    # 4.6 / 5.0 — same M3 inserts the uprights use
                  FEEDER_BR_FACE_Y as FACE_Y, FEEDER_BR_T as WALL_T, FEEDER_BR_INS_X as BASE_INS_X,
                  AXIS_Z)

# ── placement the assembly applies (documented; used by the sim later) ──
# FACE_Y, WALL_T, BASE_INS_X come from base.py (deck mount is the single source).
CENTER_Z = AXIS_Z      # bolt-pattern centreline = the wire axis (single source: base.AXIS_Z)

# ── feeder mounting pattern, rotated into the vertical face (local x=span, z=width) ──
SPAN = FEEDER_BOLT_SPAN                       # 104.5 along X
# nose end (-X) carries the narrow width, motor end (+X) the wide one (see base.FEEDER_HOLES)
HOLES = [(-SPAN / 2,  CENTER_Z + FEEDER_BOLT_W2 / 2),
         (-SPAN / 2,  CENTER_Z - FEEDER_BOLT_W2 / 2),
         ( SPAN / 2,  CENTER_Z + FEEDER_BOLT_W1 / 2),
         ( SPAN / 2,  CENTER_Z - FEEDER_BOLT_W1 / 2)]

# ── feeder boss clearance holes (same as the deck had: motor bulge Ø56, nose Ø24), now
#    bored through the vertical wall along Y so those protrusions pass through the plate ──
BOSS_HOLES = [(FEEDER_MOTOR_X - FEEDER_X, FEEDER_MOTOR_HOLE),   # motor end, +20.25, Ø56
              (FEEDER_NOSE_X - FEEDER_X,  FEEDER_NOSE_HOLE)]    # nose end,  -36.25, Ø24

# square off the +X end of the larger (motor) boss for bolt clearance: a rectangular cut
# ±SQUARE_HALF (15 -> 30mm) from the centreline in Z (parallel to the part's short edge),
# running from the boss centre out to the boss's +X tangent line.
SQUARE_HALF = 15.0

# ── plate body ──
EDGE_MARGIN = 12.0
HALF_X = SPAN / 2 + EDGE_MARGIN               # 64.25 -> plate width 128.5
WALL_H = CENTER_Z + FEEDER_BOLT_W1 / 2 + EDGE_MARGIN   # 65.65, top above the highest hole
# WALL_T (plate thickness) and BASE_INS_X (bottom-edge mount spacing) come from base.py.

_C, _MIN = Align.CENTER, Align.MIN


def _ybore(d, h, x, z, y0):
    """Cylinder Ø d, length h, drilled along +Y starting at y=y0, centred at (x, z)."""
    c = Cylinder(d / 2, h, align=(_C, _C, _MIN)).rotate(Axis.X, -90)   # base at origin, axis +Y
    return Pos(x, y0, z) * c


def feeder_bracket():
    # flat plate: x -HALF_X..+HALF_X, y 0..WALL_T (front face at y=0), z 0..WALL_H
    part = Pos(0, WALL_T / 2, WALL_H / 2) * Box(2 * HALF_X, WALL_T, WALL_H)
    # feeder M5 mounting holes: clearance THROUGH the plate along Y (bolt + nut on the back)
    for (hx, hz) in HOLES:
        part -= _ybore(FEEDER_HOLE, WALL_T + 2, hx, hz, -1)
    # feeder boss clearances bored fully through the wall along Y
    for (bx, bd) in BOSS_HOLES:
        part -= _ybore(bd, WALL_T + 2, bx, CENTER_Z, -1)
    # square off the +X end of the larger boss (bolt clearance): boss centre -> +X tangent,
    # 30mm tall in Z, through the wall
    bcx, br = FEEDER_MOTOR_X - FEEDER_X, FEEDER_MOTOR_HOLE / 2
    part -= Pos((bcx + (bcx + br)) / 2, WALL_T / 2, CENTER_Z) * Box(br, WALL_T + 2, 2 * SQUARE_HALF)
    # M3 base-mount inserts in the bottom edge (z=0), bored up +Z (like the base uprights)
    for sx in (BASE_INS_X, -BASE_INS_X):
        part -= Pos(sx, WALL_T / 2, 0) * Cylinder(M3_INS_D / 2, M3_INS_H, align=(_C, _C, _MIN))
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = feeder_bracket()
    export_stl(part, "build/feeder_bracket.stl")
    bb = part.bounding_box()
    import trimesh
    m = trimesh.load("build/feeder_bracket.stl")
    print(f"feeder_bracket: flat plate {2*HALF_X:.1f}x{WALL_H:.1f}x{WALL_T}  "
          f"4x M5 through Ø{FEEDER_HOLE} (feeder pattern span {SPAN}, w {FEEDER_BOLT_W1}/{FEEDER_BOLT_W2})  "
          f"+ boss clears Ø{FEEDER_MOTOR_HOLE}/Ø{FEEDER_NOSE_HOLE}  + 2x M3 edge inserts @ x=±{BASE_INS_X}  "
          f"face@y={FACE_Y} axis@z={CENTER_Z}  bbox {[round(v,1) for v in bb.size]}  "
          f"bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
