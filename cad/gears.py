"""Involute spur gears for build123d (no native gear primitive).

spur_gear(teeth, module, thickness, pressure_angle, bore) -> build123d Solid,
centred on the origin with its axis along +Z. Tooth flanks are true involutes;
the tooth is centred on +X and patterned with PolarLocations.
"""
import numpy as np
from build123d import (BuildSketch, Polygon, Circle, PolarLocations, add,
                       extrude, Cylinder)


def _involute_flank(rb, r0, ra, n=7):
    """Involute points from radius r0 out to ra (flank on the +angle side)."""
    def pt(r):
        t = np.sqrt(max((r / rb) ** 2 - 1.0, 0.0))
        return np.array([rb * (np.cos(t) + t * np.sin(t)),
                         rb * (np.sin(t) - t * np.cos(t))])
    return np.array([pt(r) for r in np.linspace(r0, ra, n)])


def spur_gear(teeth, module, thickness, pressure_angle=20.0, bore=0.0):
    z, m = int(teeth), float(module)
    pa = np.radians(pressure_angle)
    rp = m * z / 2.0             # pitch radius
    rb = rp * np.cos(pa)         # base radius
    ra = rp + m                  # tip (addendum) radius
    rd = rp - 1.25 * m           # root (dedendum) radius
    r0 = max(rb, rd)

    flank = _involute_flank(rb, r0, ra)
    # centre the tooth: the flank should cross the pitch circle at +pi/(2z)
    tp = np.sqrt((rp / rb) ** 2 - 1.0)
    th_p = np.arctan2(rb * (np.sin(tp) - tp * np.cos(tp)),
                      rb * (np.cos(tp) + tp * np.sin(tp)))
    off = np.pi / (2 * z) - th_p
    c, s = np.cos(off), np.sin(off)
    R = np.array([[c, -s], [s, c]])
    f1 = (R @ flank.T).T            # upper flank (root -> tip)
    f2 = f1 * np.array([1.0, -1.0])  # lower flank (mirror about X)

    # extend each flank radially DOWN into the root disk so the tooth fuses to it
    # (the involute only starts at rb, which can sit just outside rd -> floating teeth)
    def to_root(p):
        s = (rd - 0.5) / np.hypot(p[0], p[1])
        return (p[0] * s, p[1] * s)

    # closed tooth: root1, up flank1, across tip, down flank2, root2
    profile = ([to_root(f1[0])] + [tuple(p) for p in f1]
               + [tuple(p) for p in f2[::-1]] + [to_root(f2[0])])

    with BuildSketch() as sk:
        add(Circle(rd))                       # solid root disk
        with PolarLocations(0, z):
            add(Polygon(*profile, align=None))  # teeth fused on top
    gear = extrude(sk.sketch, thickness)
    if bore > 0:
        gear -= Cylinder(bore / 2, thickness + 2)
    return gear


if __name__ == "__main__":
    from build123d import export_stl
    g = spur_gear(40, 1.5, 8, bore=10)
    export_stl(g, "/tmp/gear40.stl")
    print("40T m1.5: volume", round(g.volume, 1))
