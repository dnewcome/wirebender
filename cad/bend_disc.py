"""bend_disc.py — the bend-pin disc that bolts onto the rotating arbor_mount posts.

A flat 2mm disc whose OD matches the arbor's overall width across its mounting ears
(2*BOLT_R + POST_OD). It bolts to the four arbor posts (the M3 inserts), so it carries
the same 4-bolt pattern; those mounting holes are countersunk on the OUTER face for M3
flat-head screws.

On the OPPOSITE (inner / arbor-facing) face are two countersunk 3mm holes for the
bending pins: one on the disc centre (the mandrel) and one 10mm radially out (the bend
pin). Both are countersunk from the inner face so flat-head pins/screws seat there and
the pin shafts protrude out the outer face toward the wire.

    py/bin/python cad/bend_disc.py   ->  build/bend_disc.stl
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import (Align, Cone, Cylinder, Pos, export_stl)
import arbor_mount as AM

T = 2.0                              # disc thickness
OD = 2 * AM.BOLT_R + AM.POST_OD      # overall Ø = across the arbor ears (60mm)
BOLT_R = AM.BOLT_R                   # mounting bolt circle (matches the arbor posts)
ANGLES = AM.ANGLES

MNT_CLEAR = 3.4                      # M3 clearance for the 4 mounting screws
PIN_D = 3.0                          # bending-pin holes (centre mandrel + bend pin)
PIN_OFFSET = 10.0                    # bend pin centre, radially out from the disc centre
PIN_ANGLE = 0.0                      # direction of the bend-pin offset (+X)
CSK_OD = 6.0                         # countersink major Ø (M3 flat head)

_C, _MIN = Align.CENTER, Align.MIN


def _csk_hole(part, x, y, r_hole, top):
    """Cut a through hole at (x,y) plus a 90° countersink opening at the top (z=T) or
    bottom (z=0) face of the T-thick disc (disc base at z=0)."""
    r_head = CSK_OD / 2
    depth = r_head - r_hole                      # 90° countersink (45° wall)
    part -= Pos(x, y, -1) * Cylinder(r_hole, T + 2, align=(_C, _C, _MIN))
    if top:                                      # wide at the top face (z=T), narrowing down
        part -= Pos(x, y, T - depth) * Cone(r_hole, r_head, depth, align=(_C, _C, _MIN))
    else:                                        # wide at the bottom face (z=0), narrowing up
        part -= Pos(x, y, 0) * Cone(r_head, r_hole, depth, align=(_C, _C, _MIN))
    return part


def bend_disc():
    part = Cylinder(OD / 2, T, align=(_C, _C, _MIN))         # disc, base at z=0
    # 4 mounting holes on the arbor bolt circle, countersunk on the OUTER face (top)
    for a in ANGLES:
        x, y = BOLT_R * math.cos(math.radians(a)), BOLT_R * math.sin(math.radians(a))
        part = _csk_hole(part, x, y, MNT_CLEAR / 2, top=True)
    # 2 bend-pin holes, countersunk on the INNER face (bottom): centre + 1mm offset
    part = _csk_hole(part, 0.0, 0.0, PIN_D / 2, top=False)
    ox = PIN_OFFSET * math.cos(math.radians(PIN_ANGLE))
    oy = PIN_OFFSET * math.sin(math.radians(PIN_ANGLE))
    part = _csk_hole(part, ox, oy, PIN_D / 2, top=False)
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = bend_disc()
    export_stl(part, "build/bend_disc.stl")
    bb = part.bounding_box()
    print(f"bend_disc: Ø{OD:.1f} x {T}mm  4x M3 csk @ r{BOLT_R} (outer face)  "
          f"2x Ø{PIN_D} csk @ centre +{PIN_OFFSET}mm (inner face)  "
          f"bbox {[round(v,1) for v in bb.size]}")
