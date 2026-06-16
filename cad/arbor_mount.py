"""arbor_mount.py — vendor cyclo ring (Housing_v1) + a fused 4-bolt post pattern.

The whole part is 3D-printed. We take the vendor Housing_v1 geometry and fuse on a
4-bolt mounting pattern: four posts that stick OUT radially past the ring OD and
stand 2mm PROUD of the part's top face, each tied back to the ring by a short rib
and carrying an M3 heat-set insert (open at the top). A bending plate bolts down
onto the four posts.

Robustness/speed notes (this part fought us for many iterations):
  * The union is done with trimesh + manifold3d (like bend_plate.py), NOT OCC —
    OCC intermittently refuses to merge overlapping solids fused onto the imported
    vendor B-rep. Manifold always merges overlapping meshes.
  * The vendor mesh is CACHED (build/_housing.stl). The slow ~200s STEP import runs
    only once, to seed the cache; after that iterations are seconds.

    py/bin/python cad/arbor_mount.py   ->  build/arbor_mount.stl
"""
import math
import os
import sys

import trimesh
from trimesh.transformations import rotation_matrix as _rotm, translation_matrix as _trm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from housing import housing_mesh          # vendor Housing_v1 extracted from the assembly STEP

BOLT_R = 26.0           # bolt-circle radius — posts stick out radially past the ring OD (~20.5)
POST_OD = 8.0           # post Ø
PROUD = 4.0             # posts stand this far above the part's top face (2mm base + 2mm extra clearance)
POST_DROP = 4.0         # post also drops this far below the face (body for the insert + rib bond)
RIB_IN = 21.75          # rib inner reach: into the wall (r17.2..20.5) but clear of the bore (r<18.5)
RIB_W = 9.0             # rib width (tangential)
INSERT_D, INSERT_H = 4.6, 5.0     # M3 heat-set insert, open at the post top
ANGLES = (45, 135, 225, 315)      # 4-bolt square pattern, off the feature axes


def _cyl(radius, h, x, y, zc):
    return trimesh.creation.cylinder(radius=radius, height=h, sections=72,
                                     transform=_trm([x, y, zc]))


def _radial_box(lx, ly, lz, r_mid, ang, zc):
    c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    T = _trm([r_mid * c, r_mid * s, zc]) @ _rotm(math.radians(ang), [0, 0, 1])
    return trimesh.creation.box([lx, ly, lz], transform=T)


def arbor_mount():
    h = housing_mesh()
    face = h.bounds[1][2]                       # top face of the ring
    top = face + PROUD                          # post tops, 2mm proud
    zc = (top + face - POST_DROP) / 2           # post/rib mid-Z
    post_h = PROUD + POST_DROP

    adds, holes = [h], []
    for a in ANGLES:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        x, y = BOLT_R * c, BOLT_R * s
        adds.append(_cyl(POST_OD / 2, post_h, x, y, zc))                 # post (out at BOLT_R)
        adds.append(_radial_box(BOLT_R - RIB_IN + 2, RIB_W, post_h,      # rib: ring wall -> post
                                (RIB_IN + BOLT_R) / 2, a, zc))
        holes.append(_cyl(INSERT_D / 2, INSERT_H + 0.2, x, y, top - (INSERT_H + 0.2) / 2 + 0.05))

    part = trimesh.boolean.union(adds, engine="manifold")
    part = trimesh.boolean.difference([part] + holes, engine="manifold")
    return part, face


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part, face = arbor_mount()
    if not part.is_watertight:
        part.merge_vertices(); trimesh.repair.fix_winding(part); trimesh.repair.fill_holes(part)
    part.export("build/arbor_mount.stl")
    b = part.bounds
    print(f"arbor_mount: ring + 4-bolt pattern  bbox {[round(v,1) for v in (b[1]-b[0])]}"
          f"  bolt-circle Ø{2*BOLT_R}  posts {PROUD}mm proud of face z={face:.1f}"
          f"  M3 inserts  bodies:{len(part.split(only_watertight=False))}  watertight:{part.is_watertight}")
