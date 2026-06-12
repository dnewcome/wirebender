"""Involute spur gears for build123d, via the gggears library (proper involute
profiles with undercut + root fillets, not the hand-rolled triangles).

Dependency (install into the project venv):
    py/bin/pip install "git+https://github.com/GarryBGoode/gggears"
It imports as `py_gearworks`.

spur_gear(teeth, module, thickness, pressure_angle, bore) -> build123d Part,
centred on the origin, axis +Z, spanning z = 0..thickness — same contract as
before, so cad/base.py and cad/rothead.py are unchanged.
"""
import numpy as np
import py_gearworks as gg
from build123d import Pos, Cylinder


def spur_gear(teeth, module, thickness, pressure_angle=20.0, bore=0.0):
    g = gg.SpurGear(
        number_of_teeth=int(teeth),
        module=float(module),
        height=float(thickness),
        pressure_angle=np.radians(pressure_angle),
        enable_undercut=True,       # clean roots on low-tooth pinions (e.g. 12T)
        root_fillet=0.15,           # rounded tooth roots (strength + printability)
        tip_fillet=0.05,
    )
    part = g.build_part()           # build123d Part, z = 0..thickness
    if bore > 0:
        part = part - Pos(0, 0, thickness / 2) * Cylinder(bore / 2, thickness + 2)
    return part


if __name__ == "__main__":
    from build123d import export_stl
    for t, b in ((40, 10), (12, 5)):
        g = spur_gear(t, 1.5, 8, bore=b)
        export_stl(g, f"/tmp/gear{t}.stl")
        print(f"{t}T m1.5 -> volume {round(g.volume, 1)}")
