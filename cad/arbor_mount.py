"""arbor_mount.py — parametric cyclo ring (housing) + a fused 4-bolt post pattern.

The whole part is 3D-printed. We take the parametric Housing (housing.py) and fuse on a
4-bolt mounting pattern: four posts that stick OUT radially past the ring OD and stand
PROUD of the part's top face, each tied back to the ring by a short rib and carrying an
M4 heat-set insert (open at the top). A bending plate bolts down onto the four posts.

Everything here is clean build123d B-rep geometry — the housing is a parametric solid
(no vendor STEP/STL), so the posts/ribs fuse with ordinary OCC booleans (this is the
fully-parametric replacement for the old trimesh+manifold-on-the-vendor-mesh approach).

    py/bin/python cad/arbor_mount.py   ->  build/arbor_mount.stl
"""
import math
import os
import sys

from build123d import Align, Box, Cylinder, Pos, Rot, export_stl

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from housing import HEIGHT, OD, housing    # parametric cyclo Housing (no vendor dependency)

BOLT_R = 26.0           # bolt-circle radius — posts stick out radially past the ring OD (~20.5)
POST_OD = 10.0          # post Ø (was 8.0; +2mm for the M4 insert wall + to grow the disc OD so the
                        #   Ø8 M4 flat-heads clear the disc rim — bend_disc OD = 2*BOLT_R + POST_OD)
PROUD = 4.0             # posts stand this far above the part's top face (2mm base + 2mm extra clearance)
POST_DROP = 4.0         # post also drops this far below the face (body for the insert + rib bond)
RIB_IN = 20.5           # rib spans (RIB_IN-1)..(BOLT_R+1); inner reach r19.5 welds ~1.5mm into the
                        #   Ø42 wall (solid overlap so OCC fuses it) yet clears the Ø37 bore (r<18.5)
RIB_W = 9.0             # rib width (tangential)
INSERT_D = 5.6          # M4 heat-set insert bore, run as a THROUGH hole so the iron can't bottom out
ANGLES = (45, 135, 225, 315)      # 4-bolt square pattern, off the feature axes

_C = Align.CENTER


def arbor_mount():
    face = HEIGHT                               # top face of the ring
    top = face + PROUD                          # post tops, PROUD above the face
    zc = (top + face - POST_DROP) / 2           # post/rib mid-Z
    post_h = PROUD + POST_DROP
    rib_len = BOLT_R - RIB_IN + 2               # rib reaches from the ring wall out to the post
    rib_rmid = (RIB_IN + BOLT_R) / 2

    # The posts only reach the ring through their ribs, so the pieces are not all
    # pairwise-overlapping — incremental `+` would leave a disjoint ShapeList. Fuse the
    # whole set in one OCC boolean (everything is transitively connected -> one solid).
    adds, holes = [housing()], []
    for a in ANGLES:
        ar = math.radians(a)
        c, s = math.cos(ar), math.sin(ar)
        adds.append(Pos(BOLT_R * c, BOLT_R * s, zc) * Cylinder(POST_OD / 2, post_h))          # post
        adds.append(Pos(rib_rmid * c, rib_rmid * s, zc) * Rot(0, 0, a) * Box(rib_len, RIB_W, post_h))  # rib
        holes.append(Pos(BOLT_R * c, BOLT_R * s, zc)
                     * Cylinder(INSERT_D / 2, post_h + 2, align=(_C, _C, _C)))                 # insert (through)
    # Relief: a cylinder of the part OD sitting on the top face, subtracted so the proud
    # posts/ribs get a curved inner wall at the ring OD (r=OD/2). This clears the overhang
    # the ribs would otherwise leave inside the ring, so the end_cap drops in and seats flush.
    holes.append(Pos(0, 0, face) * Cylinder(OD / 2, PROUD + 2, align=(_C, _C, Align.MIN)))
    part = adds[0].fuse(*adds[1:]).cut(*holes)
    return part, face


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part, face = arbor_mount()
    export_stl(part, "build/arbor_mount.stl")
    import trimesh
    m = trimesh.load("build/arbor_mount.stl")
    b = m.bounds
    print(f"arbor_mount: ring + 4-bolt pattern  bbox {[round(v,1) for v in (b[1]-b[0])]}"
          f"  bolt-circle Ø{2*BOLT_R}  posts {PROUD}mm proud of face z={face:.1f}"
          f"  M4 inserts  bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
