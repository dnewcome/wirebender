"""bend_plate.py — the cyclo NEMA base PLATE with a side BOSS + heat-set inserts.

Start from the parametric cyclo NEMA base (cyclo_base.cyclo_base(), vendor-independent)
and EXTRUDE A BOSS off its +X side. Two M3 heat-set inserts are drilled into the boss's
outer FACE (horizontal, blind back) — NOT through the plate's broad face.

The boss is a COPLANAR extension of the flat flange band only (z = 0 .. flange top), so it
clears BOTH the motor mounted on the flat (NEMA) face underneath and the round cyclo body /
output boss raised on the other face. It bolts to rothead.bend_piece's vertical slotted
face: the inserts sit at the slot spacing (2*BMNT_SLOT_Y), and the slots give the
±BMNT_SLOT mandrel-height adjustment.

The base is now a parametric B-rep, so the union/cut is done with OCC booleans (build123d)
— no trimesh+manifold, no vendor mesh.

    py/bin/python cad/bend_plate.py   ->  build/bend_plate.stl
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import Align, Box, Cylinder, Pos, Rot, export_stl
import rothead as R
from cyclo_base import cyclo_base, SQ, PLATE_T

INSERT_D, INSERT_H = R.IFACE_INS_D, R.IFACE_INS_H   # 4.6 hole, 5 deep
BOLT_SP = 2 * R.BMNT_SLOT_Y    # 16 — match bend_piece's 2 slots (y = ±8)
BOSS_OUT = 11.0                # how far the boss mounting face sits beyond the plate edge
                               #   (was 8.0; +3mm to push the insert face clear of the turned-down end cap)
BOSS_W = BOLT_SP + 12          # 28 — boss width (Y): the 2 inserts at ±8 + margin
BOSS_BOND = 3.0                # overlap back into the plate so the boss fuses to the wall

_C, _MIN = Align.CENTER, Align.MIN


def _xcyl(d, x0, x1, y, z):
    """Solid cylinder Ø d along +X from x0 to x1 at (y, z)."""
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, x1 - x0, align=(_C, _C, _MIN))


def bend_plate():
    part = cyclo_base()                       # NEMA face z=0, Ø30 boss up, centred XY
    edge_x = SQ / 2                           # +X plate edge (21)
    flange = PLATE_T                          # flat flange-band height (4.85)
    face_x = edge_x + BOSS_OUT
    lip_z = INSERT_D + 3.0                    # protruding-lip height (insert + wall)
    # pad: coplanar flange extension across the whole boss footprint (z 0..flange) —
    # flush with the flange where it bonds, clears the motor (under the flat face) and
    # the round body (above the flange), both within the plate footprint.
    pad_x0 = edge_x - BOSS_BOND
    part += Pos((pad_x0 + face_x) / 2, 0, flange / 2) * Box(face_x - pad_x0, BOSS_W, flange)
    # lip: the part PROTRUDING past the plate edge is clear of both the motor and the
    # round body, so give it real depth for the M3 insert. Its TOP is flush at z=flange
    # (nothing proud of the output side, so the round body can grow), and it extends
    # DOWN toward the motor side for insert wall.
    zc = flange - lip_z / 2
    part += Pos((edge_x + face_x) / 2, 0, zc) * Box(BOSS_OUT, BOSS_W, lip_z)
    # 2 M3 heat-set inserts drilled -X into the lip FACE (horizontal, blind back)
    for s in (1, -1):
        part -= _xcyl(INSERT_D, face_x - INSERT_H, face_x + 0.25, s * BOLT_SP / 2, zc)
    return part, flange, lip_z


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part, flange, lip_z = bend_plate()
    export_stl(part, "build/bend_plate.stl")
    import trimesh
    m = trimesh.load("build/bend_plate.stl")
    bb = part.bounding_box()
    print("bend_plate (parametric cyclo base + side boss):", [round(v, 1) for v in bb.size],
          f"| pad flush in the flange band (z=0..{flange})",
          f"| lip top flush at z={flange} (output side clear), extends down to "
          f"z={round(flange - lip_z, 1)} (motor side) holding the 2 M3 inserts @ {BOLT_SP}mm",
          f"| bodies:{len(m.split(only_watertight=False))} watertight:{m.is_watertight}")
