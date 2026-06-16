"""feeder_gear.py — printable stepper gear for the feeder conversion (build123d).

11-tooth, module-0.8 spur gear on a NEMA17 5mm D-shaft, with a radial M3 set screw
onto the flat. Replaces the stock brushed-motor pinion when converting the feeder
to a stepper (with 2 reduction stages removed, since the stepper is slower and far
higher-torque than the stock can motor). Major (tip) Ø ≈ m*(N+2) = 10.4mm; pitch
Ø = m*N = 8.8mm.

    py/bin/python cad/feeder_gear.py   ->  build/feeder_gear.stl
"""
import os
from build123d import *
from gears import spur_gear
from base import INSERT_D, INSERT_H        # reuse the M3 heat-set insert spec

TEETH = 11
MODULE = 0.8
WIDTH = 6.0                                # face width (match the feeder gear it meshes)
SHAFT_D = 5.0                              # NEMA17 shaft (round Ø)
SHAFT_FLAT = 4.5                           # across the D-flat
BORE_CLEAR = 0.4                           # diametral slip-fit clearance (printed bore comes out tight)
BORE_D = SHAFT_D + BORE_CLEAR              # 5.4 round bore
FLAT_Y = (SHAFT_FLAT + BORE_CLEAR) / 2     # flat-face position from centre (clears the shaft flat)
HUB_OD, HUB_H = 16.0, 6.0                  # boss big enough for a radial M3 heat-set insert
SET_CLEAR_D = 3.2                          # M3 screw clearance from the insert to the shaft flat


def feeder_gear():
    gear = spur_gear(TEETH, MODULE, WIDTH, bore=0)                 # bore cut below (D-bore)
    hub = Pos(0, 0, WIDTH) * Cylinder(HUB_OD / 2, HUB_H,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = gear + hub
    H = WIDTH + HUB_H

    # D-bore: round Ø BORE_D through the whole part, flat at +Y for the shaft flat
    bore = Cylinder(BORE_D / 2, H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(0, FLAT_Y, -1) * Box(BORE_D + 4, (BORE_D / 2 - FLAT_Y) + 2, H + 4,
                                     align=(Align.CENTER, Align.MIN, Align.MIN))
    part -= Pos(0, 0, -1) * bore

    # radial set screw: M3 heat-set insert pocket from the OD, then clearance to the flat
    ssz, yo = WIDTH + HUB_H / 2, HUB_OD / 2 + 1
    part -= (Pos(0, yo, ssz) * Rot(90, 0, 0)
             * Cylinder(INSERT_D / 2, INSERT_H + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    part -= (Pos(0, yo, ssz) * Rot(90, 0, 0)
             * Cylinder(SET_CLEAR_D / 2, HUB_OD / 2 + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return part


def _watertight_export(part, path):
    """Export, then repair: gggears' low-tooth undercut tessellates with a few
    non-manifold faces; keep the undercut geometry but make the STL watertight."""
    import trimesh
    export_stl(part, path)
    g = trimesh.load(path)
    if not g.is_watertight:
        g.merge_vertices(); trimesh.repair.fix_winding(g); trimesh.repair.fill_holes(g)
        g.export(path)
    return g.is_watertight


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = feeder_gear()
    wt = _watertight_export(p, "build/feeder_gear.stl")
    bb = p.bounding_box()
    print(f"feeder_gear {TEETH}T m{MODULE} w{WIDTH}  major Ø{MODULE*(TEETH+2):.1f}  pitch Ø{MODULE*TEETH:.1f}"
          f"  bore Ø{BORE_D} (flat @{FLAT_Y:.2f})  hub Ø{HUB_OD}x{HUB_H}  M3 set screw"
          f"  bbox {[round(v,1) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}  watertight: {wt}")
