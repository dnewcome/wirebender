"""bend_disc.py — the bend-pin disc that bolts onto the rotating arbor_mount posts.

A flat 3mm disc whose OD matches the arbor's overall width across its mounting ears
(2*BOLT_R + POST_OD). It bolts to the four arbor posts (the M3 heat-set inserts), so it
carries the same 4-bolt pattern; those mounting holes are countersunk on the OUTER face
for M3 flat-head screws.

On the OPPOSITE (inner / arbor-facing) face are two countersunk holes for the bending
pins: the MANDREL on the disc centre (the wire wraps this) and the BEND PIN PIN_OFFSET
radially out (this sweeps the wire around the mandrel). Both are countersunk from the
inner face so flat-head pins/screws seat there and the pin shafts protrude out the outer
face toward the wire.

DESIGN NOTES — planned variants (not built yet):
  * ROLLER BEND PIN: a future version replaces the fixed outer bend pin with a larger
    ROLLER (turning on a shoulder screw / small bearing) so the wire rolls instead of
    dragging as it wraps — less marring, lower friction, more usable bending force. The
    outer pin can be relatively large/stout for most jobs.
  * OUTER-PIN RADIUS (PIN_OFFSET) is the per-job knob — it sets the minimum bend LENGTH.
    We'll ship several plate versions with the bend pin at different radii: smaller wire
    can run the pin closer in (tighter/shorter features), heavier wire needs it further
    out. PIN_OFFSET is the parameter to sweep across those versions.
  * MANDREL Ø (MANDREL_D) sets the minimum bend RADIUS (the wire wraps it:
    r ≈ MANDREL_D/2 + wire/2). It changes per target bend radius and is FLOORED by the
    wire size — the mandrel pin must be large/strong enough not to deflect under the
    bending force of the chosen wire. So the mandrel is the strength-constrained pin; the
    outer pin is the geometry/throughput pin.

    py/bin/python cad/bend_disc.py   ->  build/bend_disc.stl
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import (Align, Cone, Cylinder, Pos, export_stl)
import arbor_mount as AM

T = 3.0                              # disc thickness (was 2.0; thicker for the M4 csk + stiffness)
OD = 2 * AM.BOLT_R + AM.POST_OD      # overall Ø = across the arbor ears (62mm)
BOLT_R = AM.BOLT_R                   # mounting bolt circle (matches the arbor posts)
ANGLES = AM.ANGLES

MNT_CLEAR = 3.4                      # M3 clearance for the 4 mounting screws (into the arbor M3 inserts)
MNT_CSK = 6.0                        # M3 flat-head countersink major Ø
MANDREL_D = 3.0                      # centre mandrel pin Ø (wraps the wire; see DESIGN NOTES — varies w/ bend radius + wire)
PIN_D = 3.0                          # outer bend pin Ø (the sweeper; sim PIN_D)
PIN_OFFSET = 10.0                    # bend pin centre, radially out from the disc centre (sim PIN_SWEEP_R)
PIN_ANGLE = 0.0                      # direction of the bend-pin offset (+X)
PIN_CSK = 6.0                        # pin flat-head countersink major Ø (M3-class pins/screws)

_C, _MIN = Align.CENTER, Align.MIN


def _csk_hole(part, x, y, r_hole, csk_od, top):
    """Cut a through hole at (x,y) plus a 90° countersink opening at the top (z=T) or
    bottom (z=0) face of the T-thick disc (disc base at z=0)."""
    r_head = csk_od / 2
    depth = r_head - r_hole                      # 90° countersink (45° wall)
    part -= Pos(x, y, -1) * Cylinder(r_hole, T + 2, align=(_C, _C, _MIN))
    if top:                                      # wide at the top face (z=T), narrowing down
        part -= Pos(x, y, T - depth) * Cone(r_hole, r_head, depth, align=(_C, _C, _MIN))
    else:                                        # wide at the bottom face (z=0), narrowing up
        part -= Pos(x, y, 0) * Cone(r_head, r_hole, depth, align=(_C, _C, _MIN))
    return part


def bend_disc():
    part = Cylinder(OD / 2, T, align=(_C, _C, _MIN))         # disc, base at z=0
    # 4 mounting holes on the arbor bolt circle, M4 countersunk on the OUTER face (top)
    for a in ANGLES:
        x, y = BOLT_R * math.cos(math.radians(a)), BOLT_R * math.sin(math.radians(a))
        part = _csk_hole(part, x, y, MNT_CLEAR / 2, MNT_CSK, top=True)
    # 2 bend-pin holes, countersunk on the INNER face (bottom): centre mandrel + outer bend pin
    part = _csk_hole(part, 0.0, 0.0, MANDREL_D / 2, PIN_CSK, top=False)
    ox = PIN_OFFSET * math.cos(math.radians(PIN_ANGLE))
    oy = PIN_OFFSET * math.sin(math.radians(PIN_ANGLE))
    part = _csk_hole(part, ox, oy, PIN_D / 2, PIN_CSK, top=False)
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = bend_disc()
    export_stl(part, "build/bend_disc.stl")
    import trimesh
    m = trimesh.load("build/bend_disc.stl")
    bb = part.bounding_box()
    print(f"bend_disc: Ø{OD:.1f} x {T}mm  4x M3 csk @ r{BOLT_R} (outer face)  "
          f"mandrel Ø{MANDREL_D} @ centre + bend pin Ø{PIN_D} @ {PIN_OFFSET}mm (inner face)  "
          f"bbox {[round(v, 1) for v in bb.size]}  bodies:{len(m.split(only_watertight=False))}  "
          f"watertight:{m.is_watertight}")
