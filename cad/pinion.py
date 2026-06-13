"""pinion.py — stepper pinion for the head-rotation drive (build123d).

The rotation stepper (NEMA17, 5mm D-shaft) drives this 12T pinion, which meshes
the fixed 40T base gear (40/12 = 3.33:1). Module and face width are imported from
base.py so the pinion stays locked to that gear. The Ø5 bore has a flat for the
D-shaft, and a radial M3 set screw clamps onto that flat.

    py/bin/python cad/pinion.py   ->  build/pinion.stl
"""
import os
from build123d import *
from gears import spur_gear
from base import FG_MODULE, FG_W           # lock module + face width to the base gear

PIN_TEETH = 12
PIN_W = FG_W                               # 8.0 — same face width as the base gear
SHAFT_D = 5.0                              # NEMA17 shaft
FLAT_DEPTH = 0.5                           # D-shaft flat (5mm -> 4.5mm across the flat)
HUB_OD, HUB_H = 12.0, 6.0                  # set-screw boss above the gear
GRUB_D = 2.7                               # self-tapping M3 set screw (threads into plastic)


def pinion():
    gear = spur_gear(PIN_TEETH, FG_MODULE, PIN_W, bore=0)          # bore cut below (D-bore)
    hub = Pos(0, 0, PIN_W) * Cylinder(HUB_OD / 2, HUB_H,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = gear + hub
    H = PIN_W + HUB_H

    # D-bore: Ø5 through the whole part, with a flat at +Y for the shaft flat
    bore = Cylinder(SHAFT_D / 2, H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(0, SHAFT_D / 2, -1) * Box(SHAFT_D + 2, 2 * FLAT_DEPTH, H + 4,
                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= Pos(0, 0, -1) * bore

    # radial M3 set screw through the hub wall, landing on the flat
    part -= (Pos(0, HUB_OD / 2 + 1, PIN_W + HUB_H / 2) * Rot(90, 0, 0)
             * Cylinder(GRUB_D / 2, HUB_OD / 2 + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = pinion()
    export_stl(p, "build/pinion.stl")
    bb = p.bounding_box()
    print(f"pinion {PIN_TEETH}T m{FG_MODULE} w{PIN_W}  bore Ø{SHAFT_D} (flat {FLAT_DEPTH})  "
          f"hub Ø{HUB_OD}x{HUB_H}  set screw M3"
          f"  bbox {[round(v,1) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}")
