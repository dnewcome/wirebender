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
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import (Align, Axis, Box, BuildPart, BuildSketch, Cylinder, Plane, Pos,
                        SlotOverall, export_stl, loft)
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

# ── motor-boss centre (local), shared by the front pocket / web / back pocket ──
MOTOR_X = FEEDER_MOTOR_X - FEEDER_X           # +20.25 — large boss / NEMA17 axis
NOSE_X = FEEDER_NOSE_X - FEEDER_X             # -36.25 — small (nose) boss, plain through-bore

# ── integrated NEMA17 mount at the motor boss ──────────────────────────
NEMA_BOLT = 31.0          # NEMA17 square bolt pitch -> holes at ±15.5
NEMA_BODY = 42.3          # NEMA17 body square (back-pocket clearance footprint)
NEMA_PILOT = 22.0         # centring pilot-boss Ø (the centre slot must clear it + the shaft/pinion)
M3_CLEAR = 3.4            # motor-screw clearance (M3)
M3_CSK = 6.0             # M3 flat-head countersink major Ø (90°), opening on the boss/front side
WEB_T = 1.5              # thin mounting web at the plate mid-plane
PILOT_CLEAR = 0.5        # clearance around the pilot boss in the centre slot
POCKET_CLEAR = 1.0       # clearance around the NEMA body in the back pocket
SLOT_TRAVEL = 16.0       # mesh-adjust slide range -> slots are this much longer than the bore (±8mm)
SLOT_DIR = (1.0, 0.0)    # slide direction in the face plane (dx, dz); (1,0)=along X. one-line to rotate.

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


def _yslot(d, h, x, z, y0, travel, ang_deg):
    """Capsule (obround) hole Ø d bored along +Y, length h from y=y0; the bore centre is
    swept `travel` along a line at ang_deg from +X in the face (X-Z) plane, centred at (x, z)."""
    cap = Cylinder(d / 2, h, align=(_C, _C, _C)).rotate(Axis.X, -90)   # bore along Y, centred
    shape = (Box(travel, h, d, align=(_C, _C, _C))                      # connecting bar: X long, Y=bore depth, Z=wide
             + cap.moved(Pos(travel / 2, 0, 0)) + cap.moved(Pos(-travel / 2, 0, 0)))
    shape = shape.rotate(Axis.Y, ang_deg)
    return Pos(x, y0 + h / 2, z) * shape


def _yslot_csk(d_bore, d_head, x, z, y_face, depth, travel, ang_deg):
    """Obround countersink for a slotted screw: an obround opening Ø d_head at y=y_face that
    tapers (90°) to Ø d_bore at y=y_face+depth, swept `travel` along the slot, opening toward
    −Y (the boss/front side) so a flat head sits flush in the web as the screw slides."""
    pl = Plane((0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0))         # in X-Z, normal +Y
    with BuildPart() as bp:
        with BuildSketch(pl):                                       # face: wide (head Ø)
            SlotOverall(travel + d_head, d_head)
        with BuildSketch(pl.offset(depth)):                        # depth: narrow (bore Ø)
            SlotOverall(travel + d_bore, d_bore)
        loft()
    return Pos(x, y_face, z) * bp.part.rotate(Axis.Y, ang_deg)


def _motor_mount(part):
    """Carve the integrated NEMA17 mount into the large (motor) boss region:
    front Ø56 pocket | 1.5mm slotted web | back rectangular pocket."""
    yw0, yw1 = (WALL_T - WEB_T) / 2, (WALL_T + WEB_T) / 2          # web spans y = yw0..yw1
    ang = math.degrees(math.atan2(SLOT_DIR[1], SLOT_DIR[0]))
    # FRONT round pocket (feeder boss clearance Ø56) — opens at the face, stops at the web
    part -= _ybore(FEEDER_MOTOR_HOLE, yw0 + 1, MOTOR_X, CENTER_Z, -1)
    # BACK rectangular pocket (NEMA17 body), open to the back face; longer in the slide axis
    px = NEMA_BODY + 2 * POCKET_CLEAR + abs(SLOT_DIR[0]) * SLOT_TRAVEL
    pz = NEMA_BODY + 2 * POCKET_CLEAR + abs(SLOT_DIR[1]) * SLOT_TRAVEL
    part -= Pos(MOTOR_X, (yw1 + WALL_T + 1) / 2, CENTER_Z) * Box(px, WALL_T + 1 - yw1, pz)
    # WEB slots (cut full depth — only the 1.5mm web is left to cut between the two pockets):
    #   centre slot clears the Ø22 pilot boss + the shaft/pinion; 4 bolt slots = NEMA 31mm pattern
    part -= _yslot(NEMA_PILOT + 2 * PILOT_CLEAR, WALL_T + 2, MOTOR_X, CENTER_Z, -1, SLOT_TRAVEL, ang)
    csk_depth = (M3_CSK - M3_CLEAR) / 2                            # 90° flat-head countersink depth
    for sx in (-NEMA_BOLT / 2, NEMA_BOLT / 2):
        for sz in (-NEMA_BOLT / 2, NEMA_BOLT / 2):
            part -= _yslot(M3_CLEAR, WALL_T + 2, MOTOR_X + sx, CENTER_Z + sz, -1, SLOT_TRAVEL, ang)
            part -= _yslot_csk(M3_CLEAR, M3_CSK, MOTOR_X + sx, CENTER_Z + sz, yw0, csk_depth, SLOT_TRAVEL, ang)
    return part


def feeder_bracket():
    # flat plate: x -HALF_X..+HALF_X, y 0..WALL_T (front face at y=0), z 0..WALL_H
    part = Pos(0, WALL_T / 2, WALL_H / 2) * Box(2 * HALF_X, WALL_T, WALL_H)
    # feeder M5 mounting holes: clearance THROUGH the plate along Y (bolt + nut on the back)
    for (hx, hz) in HOLES:
        part -= _ybore(FEEDER_HOLE, WALL_T + 2, hx, hz, -1)
    # nose boss clearance: still a plain bore through the wall
    part -= _ybore(FEEDER_NOSE_HOLE, WALL_T + 2, NOSE_X, CENTER_Z, -1)
    # large (motor) boss -> integrated NEMA17 mount (front pocket / web / back pocket)
    part = _motor_mount(part)
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
          f"+ nose boss clear Ø{FEEDER_NOSE_HOLE}  + integrated NEMA17 mount @ x={MOTOR_X:.2f} "
          f"(front Ø{FEEDER_MOTOR_HOLE} pocket / {WEB_T}mm web, NEMA {NEMA_BOLT}mm pattern slotted ±{SLOT_TRAVEL/2:.0f}mm "
          f"along {'X' if SLOT_DIR[1]==0 else 'Z' if SLOT_DIR[0]==0 else 'angle'} / back {NEMA_BODY}mm pocket)  "
          f"+ 2x M3 edge inserts @ x=±{BASE_INS_X}  face@y={FACE_Y} axis@z={CENTER_Z}  "
          f"bbox {[round(v,1) for v in bb.size]}  bodies:{len(m.split(only_watertight=False))}  "
          f"watertight:{m.is_watertight}")
