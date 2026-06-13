"""bend_endcap.py — custom cycloidal output plate with the bending die built in.

Takes the Sweep Dynamics 'End_Cap' (the rotating output) out of the purchased
STEP and adds a bending PIN that sticks up from the output face. As the cycloidal
output rotates, the pin sweeps the wire around a (separate) fixed die at the wire
exit — single-pin bending. PIN_R (the pin's radius from the output centre) is the
min-bend-spacing / tightness knob.

Vendor geometry is NOT committed — regenerate locally:
    py/bin/python cad/bend_endcap.py   ->  build/bend_endcap.stl   (gitignored)
"""
import os
from build123d import *

CYCLO_STEP = os.path.expanduser(
    "~/Downloads/cad-files/sweepdynamics/micro-cycloidal/20-1 Micro Cycloidal.step")

PIN_D = 4.0        # bend-pin diameter
PIN_H = 9.0        # how far it sticks up
PIN_R = 13.0       # pin centre radius from the output axis (smaller = tighter/closer bends)
PIN_ANGLE = 0.0    # placed between two bolt holes


def bend_endcap():
    s = import_step(CYCLO_STEP)
    cap = [c for c in s.children if c.label == "End_Cap"][0]
    # work in the cap's own frame, output face up (+Z)
    bb = cap.bounding_box()
    cap = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * cap
    top = cap.bounding_box().max.Z
    import math
    px, py = PIN_R * math.cos(math.radians(PIN_ANGLE)), PIN_R * math.sin(math.radians(PIN_ANGLE))
    pin = Pos(px, py, top - 0.5) * Cylinder(PIN_D / 2, PIN_H + 0.5,
                                            align=(Align.CENTER, Align.CENTER, Align.MIN))
    return cap + pin


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = bend_endcap()
    export_stl(part, "build/bend_endcap.stl")
    bb = part.bounding_box()
    print("custom end cap bbox:", [round(v, 1) for v in bb.size], "pin_R", PIN_R)
