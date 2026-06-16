"""bend_plate.py — printable cyclo mounting plate with a boss + heat-set inserts.

The wirebender analog of 3dprint-robot/angle_mount, but with the inserts on the
PLATE instead of the cyclo body: a totally flat plate that carries the bend
actuator (pancake + cycloid, NEMA17 pattern + central output bore) with 2 M3
heat-set inserts ORTHOGONAL to the plate face (axial, blind back — no raised
boss). It bolts to rothead.bend_piece's vertical slotted face — the inserts match
the slot spacing (±BMNT_SLOT_Y), and the slots give the ±BMNT_SLOT height
adjustment for different mandrels.

    py/bin/python cad/bend_plate.py   ->  build/bend_plate.stl
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import *
import rothead as R
from parts import NEMA_BOLT

PLATE_SZ = R.CYC_SQ            # 42 — NEMA17 / cyclo footprint
T = 7.0                       # flat plate thickness (= insert depth + ~2mm backing)
BORE_D = R.CYC_OUT_D          # 30 — clears the cycloidal output boss
INSERT_D, INSERT_H = R.IFACE_INS_D, R.IFACE_INS_H   # 4.6 hole, 5 deep
M3_CLEAR = R.M3_CLEAR         # 3.4 — NEMA mount clearance
BOLT_SP = 2 * R.BMNT_SLOT_Y   # 16 — match bend_piece's 2 slots (y = ±8)
INSERT_Y = PLATE_SZ / 2 - 6   # insert row near the +Y edge (clear of bore + NEMA holes)


def _zhole(d, x, y, z0, h):
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def bend_plate():
    p = Pos(0, 0, T / 2) * Box(PLATE_SZ, PLATE_SZ, T)             # totally flat plate, z 0..T
    # central output bore + 4 NEMA17 mount holes (the bend actuator bolts on below)
    p -= _zhole(BORE_D, 0, 0, -1, T + 2)
    for sx in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
        for sy in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
            p -= _zhole(M3_CLEAR, sx, sy, -1, T + 2)
    # 2 M3 heat-set inserts ORTHOGONAL to the flat face (axial), melted in from the
    # top, blind back — at the bend_piece slot spacing
    for sx in (BOLT_SP / 2, -BOLT_SP / 2):
        p -= _zhole(INSERT_D, sx, INSERT_Y, T - INSERT_H, INSERT_H + 0.1)
    return p


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = bend_plate()
    export_stl(part, "build/bend_plate.stl")
    bb = part.bounding_box()
    print("bend_plate:", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          f"| Ø{BORE_D} bore, NEMA {NEMA_BOLT}, 2x M3 inserts @ {BOLT_SP}mm (orthogonal to face)")
