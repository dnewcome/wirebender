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
from base import FG_MODULE, FG_W, INSERT_D, INSERT_H   # lock to the base gear; reuse M3 insert spec

PIN_TEETH = 12
PIN_W = FG_W                               # 8.0 — same face width as the base gear
SHAFT_D = 5.0                              # NEMA17 shaft (round Ø)
SHAFT_FLAT = 4.5                           # across the D-flat
BORE_CLEAR = 0.4                           # diametral slip-fit clearance (printed bore comes out tight)
BORE_D = SHAFT_D + BORE_CLEAR              # 5.4 round bore
FLAT_Y = (SHAFT_FLAT + BORE_CLEAR) / 2     # flat-face position from centre (clears the shaft flat)
HUB_OD, HUB_H = 16.0, 8.0                  # boss big enough for a radial M3 heat-set insert
SET_CLEAR_D = 3.2                          # M3 screw clearance from the insert to the shaft


def pinion():
    gear = spur_gear(PIN_TEETH, FG_MODULE, PIN_W, bore=0)          # bore cut below (D-bore)
    hub = Pos(0, 0, PIN_W) * Cylinder(HUB_OD / 2, HUB_H,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = gear + hub
    H = PIN_W + HUB_H

    # D-bore: round Ø BORE_D through the whole part, flat at +Y for the shaft flat
    bore = Cylinder(BORE_D / 2, H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(0, FLAT_Y, -1) * Box(BORE_D + 4, (BORE_D / 2 - FLAT_Y) + 2, H + 4,
                                     align=(Align.CENTER, Align.MIN, Align.MIN))
    part -= Pos(0, 0, -1) * bore

    # radial set screw: M3 heat-set insert pocket from the OD, then clearance to the flat
    ssz, yo = PIN_W + HUB_H / 2, HUB_OD / 2 + 1
    part -= (Pos(0, yo, ssz) * Rot(90, 0, 0)
             * Cylinder(INSERT_D / 2, INSERT_H + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    part -= (Pos(0, yo, ssz) * Rot(90, 0, 0)
             * Cylinder(SET_CLEAR_D / 2, HUB_OD / 2 + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return part


def _watertight_export(part, path):
    """Export, then repair the mesh. gggears' 12T undercut tessellates with a few
    non-manifold faces (the 40T gear is fine); this keeps the undercut geometry but
    makes the printable STL watertight."""
    import trimesh
    export_stl(part, path)
    g = trimesh.load(path)
    if not g.is_watertight:
        g.merge_vertices(); trimesh.repair.fix_winding(g); trimesh.repair.fill_holes(g)
        g.export(path)
    return g.is_watertight


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = pinion()
    wt = _watertight_export(p, "build/pinion.stl")
    print("  watertight:", wt)
    bb = p.bounding_box()
    print(f"pinion {PIN_TEETH}T m{FG_MODULE} w{PIN_W}  bore Ø{BORE_D} (flat @{FLAT_Y:.2f})  "
          f"hub Ø{HUB_OD}x{HUB_H}  set screw: M3 heat-set Ø{INSERT_D}x{INSERT_H}"
          f"  bbox {[round(v,1) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}")
