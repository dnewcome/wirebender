"""nema_template.py — thin NEMA17 drilling/marking template.

A flat plate the size of a NEMA17 face (42.3mm square, chamfered corners) with the 4-bolt
pattern (31mm square) and the Ø22 pilot bore. Lay it on the housing, mark or drill straight
through the holes, and you get the motor pattern transferred exactly — no blind holes.

Holes are M3-clearance (Ø3.4) so a marker tip or a drill passes; the Ø22 centre marks the
shaft/pilot bore. Printed thin so it's quick and sits flat.

    py/bin/python cad/nema_template.py   ->  build/nema_template.stl
"""
import os

from build123d import Align, Cylinder, Pos, Rectangle, chamfer, export_stl, extrude

NEMA = 42.3            # NEMA17 face (square)
NEMA_CHAMF = 6.0       # corner chamfer
NEMA_BOLT = 31.0       # bolt pattern (holes at ±15.5)
PILOT_D = 22.0         # centering pilot bore
THICK = 1.6            # thin template
HOLE_D = 3.4           # bolt-hole marks (M3 clearance / passes a drill or marker)

_C, _MIN = Align.CENTER, Align.MIN


def nema_template():
    sk = Rectangle(NEMA, NEMA)
    sk = chamfer(sk.vertices(), NEMA_CHAMF)          # NEMA17 chamfered corners
    plate = extrude(sk, THICK)
    plate -= Pos(0, 0, -1) * Cylinder(PILOT_D / 2, THICK + 2, align=(_C, _C, _MIN))   # pilot bore
    for x in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
        for y in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
            plate -= Pos(x, y, -1) * Cylinder(HOLE_D / 2, THICK + 2, align=(_C, _C, _MIN))
    return plate


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = nema_template()
    export_stl(part, "build/nema_template.stl")
    import trimesh
    m = trimesh.load("build/nema_template.stl")
    print(f"nema_template: {NEMA}mm NEMA17 face x {THICK}  4x Ø{HOLE_D} @ {NEMA_BOLT}sq  pilot Ø{PILOT_D}"
          f"  bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
