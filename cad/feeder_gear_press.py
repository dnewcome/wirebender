"""feeder_gear_press.py — collar-less PRESS-FIT feeder stepper pinion (build123d).

Same 11-tooth m0.8 gear as feeder_gear.py, but with NO hub/collar, NO set screw, and NO
heat-set insert — it presses straight onto the NEMA17 5mm D-shaft. Use this when there isn't
axial room for the set-screw boss. Anti-rotation comes from the D-flat (so the joint only has
to resist axial pull-off, not torque); the interference fit holds it on. A small lead-in
chamfer at the shaft-entry face starts the press.

Overall height is just the gear face width (no hub) — half the height of feeder_gear.

    py/bin/python cad/feeder_gear_press.py   ->  build/feeder_gear_press.stl
"""
import os
from build123d import *
from gears import spur_gear

TEETH = 11
MODULE = 0.8
WIDTH = 6.0               # face width (meshes the feeder gear); also the full press engagement
SHAFT_D = 5.0             # NEMA17 round shaft Ø
SHAFT_FLAT = 4.5          # across the D-flat
# PRESS_FIT is THE tuning knob. feeder_gear.py's proven SLIP fit is +0.4 diametral; this press
# bore is tighter. +0.2 = a firm press on a typical FDM printer (~0.2mm interference vs the slip
# fit). Go +0.1/-0.0 for a tighter grip, +0.3 if it cracks or won't seat — printer-dependent.
PRESS_FIT = 0.2
BORE_D = SHAFT_D + PRESS_FIT
FLAT_Y = (SHAFT_FLAT + PRESS_FIT) / 2     # flat-face position from centre
LEAD_IN = 0.6             # 45° chamfer at the entry mouth to start the press


def feeder_gear_press():
    part = spur_gear(TEETH, MODULE, WIDTH, bore=0)            # bore cut below (press-fit D-bore)
    # press-fit D-bore through the gear: round Ø BORE_D with a flat at +Y for the shaft flat
    bore = Cylinder(BORE_D / 2, WIDTH + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(0, FLAT_Y, -1) * Box(BORE_D + 4, (BORE_D / 2 - FLAT_Y) + 2, WIDTH + 4,
                                     align=(Align.CENTER, Align.MIN, Align.MIN))
    part -= Pos(0, 0, -1) * bore
    # lead-in chamfer at the bottom (shaft-entry) face: wide at z=0, narrowing to the bore
    part -= Cone(BORE_D / 2 + LEAD_IN, BORE_D / 2, LEAD_IN,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    return part


def _watertight_export(part, path):
    """gggears' low-tooth undercut tessellates with a few non-manifold faces; keep the
    undercut geometry but make the STL watertight."""
    import trimesh
    export_stl(part, path)
    g = trimesh.load(path)
    if not g.is_watertight:
        g.merge_vertices(); trimesh.repair.fix_winding(g); trimesh.repair.fill_holes(g)
        g.export(path)
    return g.is_watertight


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = feeder_gear_press()
    wt = _watertight_export(p, "build/feeder_gear_press.stl")
    bb = p.bounding_box()
    print(f"feeder_gear_press {TEETH}T m{MODULE} w{WIDTH}  major Ø{MODULE*(TEETH+2):.1f}"
          f"  press D-bore Ø{BORE_D} (flat @{FLAT_Y:.2f}, +{PRESS_FIT} vs shaft)  no hub/set screw"
          f"  bbox {[round(v,1) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}  watertight: {wt}")
