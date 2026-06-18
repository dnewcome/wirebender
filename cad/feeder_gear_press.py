"""feeder_gear_press.py — PRESS-FIT feeder stepper pinion (build123d).

Same 11-tooth m0.8 gear as feeder_gear.py, but with NO set screw and NO heat-set insert — it
presses straight onto the NEMA17 5mm D-shaft. The interference fit holds it axially; the D-flat
carries the torque. A small lead-in chamfer at the shaft-entry face starts the press.

It slipped feeding heavy wire because the flat wasn't actually engaging: the old bore flat sat
0.35mm proud of the real shaft flat (placed at (SHAFT_FLAT+fit)/2 instead of relative to the
true shaft-flat radius), so torque rode on the round press alone and span. Two changes:
  1. The bore flat is now placed FLAT off the true shaft-flat position so it BEARS (the press
     bore Ø is unchanged — that fit was correct).
  2. More engagement: an 8mm toothed body + a 4mm collar (at the gear's major Ø) = 12mm of
     D-bore gripping the shaft (was 6mm). The collar is a plain extension, no insert/boss.

    py/bin/python cad/feeder_gear_press.py   ->  build/feeder_gear_press.stl
"""
import os
from build123d import *
from gears import spur_gear

TEETH = 11
MODULE = 0.8
WIDTH = 8.0               # toothed face width (was 6.0); meshes the feeder gear
COLLAR_H = 4.0            # plain collar on top -> 12mm total D-bore engagement
MAJOR_D = MODULE * (TEETH + 2)   # gear major (tip) Ø, 10.4mm
COLLAR_OD = MAJOR_D + 0.6        # 11.0 — just proud of the tips; EXACTLY = tip Ø makes the collar
#                                  edge tangent to every tooth tip -> degenerate, non-manifold boolean.
SHAFT_D = 5.0             # NEMA17 round shaft Ø
SHAFT_FLAT = 4.5          # across the D-flat (flat face -> opposite round side)
# PRESS_FIT is THE tuning knob. feeder_gear.py's proven SLIP fit is +0.4 diametral; this press
# bore is tighter. +0.2 = a firm press on a typical FDM printer (~0.2mm interference vs the slip
# fit). Go +0.1/-0.0 for a tighter grip, +0.3 if it cracks or won't seat — printer-dependent.
PRESS_FIT = 0.2
BORE_D = SHAFT_D + PRESS_FIT
SHAFT_FLAT_Y = SHAFT_FLAT - SHAFT_D / 2   # 2.0 — true shaft-flat distance from the axis
FLAT_Y = SHAFT_FLAT_Y + PRESS_FIT / 2     # 2.1 — bore flat just proud of the shaft flat, same fit as the round
LEAD_IN = 0.6             # 45° chamfer at the entry mouth to start the press


def feeder_gear_press():
    gear = spur_gear(TEETH, MODULE, WIDTH, bore=0)           # bore cut below (press-fit D-bore)
    collar = Pos(0, 0, WIDTH) * Cylinder(COLLAR_OD / 2, COLLAR_H,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = gear + collar
    H = WIDTH + COLLAR_H
    # press-fit D-bore through the whole part: round Ø BORE_D with a flat at +Y for the shaft flat
    bore = Cylinder(BORE_D / 2, H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(0, FLAT_Y, -1) * Box(BORE_D + 4, (BORE_D / 2 - FLAT_Y) + 2, H + 4,
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
    print(f"feeder_gear_press {TEETH}T m{MODULE} w{WIDTH} + collar Ø{COLLAR_OD:.1f}x{COLLAR_H}  major Ø{MAJOR_D:.1f}"
          f"  press D-bore Ø{BORE_D} flat@{FLAT_Y:.2f} (+{PRESS_FIT} vs shaft, flat bears)  engagement {WIDTH+COLLAR_H}mm"
          f"  bbox {[round(v,1) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}  watertight: {wt}")
