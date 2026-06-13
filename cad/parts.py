"""parts.py — reusable models of bought parts (ghosts), build123d.

nema17(depth, ...) -> a NEMA17 stepper: 42.3mm chamfered-corner body, Ø22 pilot
boss + Ø5 shaft on the front face, 4× M3 on the 31mm square. Front (shaft) face at
z=0, body extends -Z, shaft/pilot extend +Z. Pass depth=24 for a pancake.
"""
from build123d import *

NEMA = 42.3
NEMA_CHAMF = 6.0
NEMA_BOLT = 31.0
PILOT_D, PILOT_H = 22.0, 2.0
SHAFT_D = 5.0


def nema17(depth=34.0, shaft_len=20.0, pilot=True):
    sk = Rectangle(NEMA, NEMA)
    sk = chamfer(sk.vertices(), NEMA_CHAMF)        # cut the 4 corners
    m = extrude(sk, -depth)                          # body behind the face (z<0)
    if pilot:
        m += Cylinder(PILOT_D / 2, PILOT_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    m += Pos(0, 0, PILOT_H if pilot else 0) * Cylinder(
        SHAFT_D / 2, shaft_len, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
        for y in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
            m -= Pos(x, y, 0) * Cylinder(1.6, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return m


if __name__ == "__main__":
    export_stl(nema17(depth=24, shaft_len=18), "/tmp/nema_pancake.stl")
    print("pancake bbox:", [round(v, 1) for v in nema17(depth=24).bounding_box().size])
