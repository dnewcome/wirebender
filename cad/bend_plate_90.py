"""bend_plate_90.py — bend_plate variant: boss extrudes out, then turns 90° UP into a riser
carrying a CROSS-AXIS (Y) adjustment slot.

Same start as bend_plate.py (the parametric cyclo NEMA base + a coplanar flange extension off
the +X edge), but instead of ending in an outward insert face, the boss bends 90° up into a
vertical riser tab. The tab has ONE horizontal slot running in the cross (Y) direction, long
enough to hold both bolts (at the ±BMNT_SLOT_Y spacing) plus ±SLOT_TRAVEL of slide.

Bolted to rothead's vertical bending plate — whose slots run in Z — the two perpendicular
slots give the bend actuator full 2-axis (Y + Z) positioning: Z sets the disk height/reach,
Y sets the mandrel-center-to-wire offset. Through-bolts + nuts (no inserts), so both slots
can slide.

The base is now a parametric B-rep, so the union/cut is done with OCC booleans (build123d)
— no trimesh+manifold, no vendor mesh.

    py/bin/python cad/bend_plate_90.py   ->  build/bend_plate_90.stl
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import Align, Box, Cone, Cylinder, Pos, Rot, export_stl
import rothead as R
from cyclo_base import cyclo_base, SQ, PLATE_T, NEMA_BOLT, NEMA_CLEAR, NEMA_CSK

M3_CLEAR = R.M3_CLEAR                 # 3.4 — through-bolt clearance
BOLT_SP = 2 * R.BMNT_SLOT_Y          # 16 — bolt spacing in Y (matches rothead's 2 slots)
BOSS_OUT = 17.0                      # how far the horizontal arm reaches past the plate edge
                                     #   (was 11; +6mm total to push the riser out so the head clears)
BOSS_BOND = 3.0                      # overlap back into the plate so the arm fuses to the wall
RISER_T = 6.0                        # riser tab thickness (X) — the mounting plate
RISER_H = 22.0                       # riser tab height (Z) above the flange plane
SLOT_TRAVEL = 8.0                    # ± cross-axis (Y) slide beyond the bolt span (full width gives room)
SLOT_HALF = BOLT_SP / 2 + SLOT_TRAVEL  # 16 — Y half-length of the slot
SLOT_Z = 11.0                        # slot centre height up the riser from the flange plane

_C, _MIN = Align.CENTER, Align.MIN
CSK_DEPTH = (NEMA_CSK - NEMA_CLEAR) / 2   # 90° flat-head countersink depth (matches cyclo_base)


def _yslot(half, d, x0, x1, zc):
    """Clearance slot bored along +X (x0..x1), elongated ±half in Y, Ø d, centred at z=zc."""
    L = x1 - x0
    s = Pos((x0 + x1) / 2, 0, zc) * Box(L, 2 * half, d)
    for sy in (half, -half):
        s += Pos((x0 + x1) / 2, sy, zc) * Rot(0, 90, 0) * Cylinder(d / 2, L, align=(_C, _C, _C))
    return s


def _csk(x, y, ztop):
    """Re-cut a NEMA flat-head hole the full-width arm covered: through-clearance up to
    z=ztop + the 90° countersink cone opening upward there (same geometry as cyclo_base)."""
    thru = Pos(x, y, -1) * Cylinder(NEMA_CLEAR / 2, ztop + 2, align=(_C, _C, _MIN))
    cone = Pos(x, y, ztop - CSK_DEPTH) * Cone(NEMA_CLEAR / 2, NEMA_CSK / 2, CSK_DEPTH, align=(_C, _C, _MIN))
    return thru + cone


def bend_plate_90():
    part = cyclo_base()                       # NEMA face z=0, Ø30 boss up, centred XY
    edge_x = SQ / 2                           # +X plate edge (21)
    full_w = SQ                               # full NEMA plate width — square off the +X-end and span it all
    flange = PLATE_T                          # flat flange-band height (4.85)
    face_x = edge_x + BOSS_OUT

    # horizontal arm: full-width coplanar flange extension off the +X edge (z 0..flange);
    # fills the corner bevels at that end, clears the motor below and the round body above.
    pad_x0 = edge_x - BOSS_BOND
    part += Pos((pad_x0 + face_x) / 2, 0, flange / 2) * Box(face_x - pad_x0, full_w, flange)
    # vertical riser: the 90° bend-up tab at the outer end, full width, rising from the flange
    # plane (+2 overlaps down into the arm so they fuse cleanly).
    part += Pos(face_x - RISER_T / 2, 0, flange - 2 + (RISER_H + 2) / 2) * Box(RISER_T, full_w, RISER_H + 2)

    # one cross-axis (Y) slot bored through the riser along X
    part -= _yslot(SLOT_HALF, M3_CLEAR, face_x - RISER_T - 1, face_x + 1, flange + SLOT_Z)
    # re-cut the 2 +X NEMA bolt-hole countersinks the full-width arm partially covered
    for hy in (NEMA_BOLT, -NEMA_BOLT):
        part -= _csk(NEMA_BOLT, hy, flange)
    return part, flange


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part, flange = bend_plate_90()
    export_stl(part, "build/bend_plate_90.stl")
    import trimesh
    m = trimesh.load("build/bend_plate_90.stl")
    bb = part.bounding_box()
    print(f"bend_plate_90 (cyclo base + out+up riser): {[round(v, 1) for v in bb.size]}"
          f"  riser {RISER_T}x{RISER_H} above flange z={flange}"
          f"  cross-axis Y-slot ±{SLOT_HALF} (bolts ±{BOLT_SP / 2}, travel ±{SLOT_TRAVEL})"
          f"  bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
