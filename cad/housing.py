"""housing.py — vendor cyclo Housing_v1 (the rotating internal-gear ring), extracted clean.

The Housing is the rotating output ring of the micro-cycloidal drive: an Ø42 shell whose
bore is lined by a polar array of ~40 axial cycloidal pin lobes (the internal "gear"),
running between z≈4 and z≈14 with chamfered ends.

Unlike the End_Cap, the vendor Housing_v1 solid tessellates CLEANLY (watertight, single
body), and its exact B-spline lobe profile IS the functional gear surface — far more
accurate than any circular-pin idealization. So we take the body straight from the vendor
assembly STEP rather than re-deriving it. arbor_mount.py fuses the 4-bolt post pattern
onto this body.

The (slow) assembly-STEP import runs once and is cached to build/_housing.stl; after that
loads are instant. Delete that cache to force a fresh extraction from the STEP.

    py/bin/python cad/housing.py   ->  build/housing.stl
"""
import os

import trimesh

CYCLO_STEP = os.path.expanduser(
    "~/Downloads/cad-files/sweepdynamics/micro-cycloidal/20-1 Micro Cycloidal.step")
CACHE = "build/_housing.stl"        # cached vendor Housing mesh (seeded from the STEP once)


def housing_mesh():
    """Vendor Housing_v1, centred (axis at origin, base at z=0). Cached after first import."""
    if not os.path.exists(CACHE):
        if not os.path.exists(CYCLO_STEP):
            raise SystemExit(f"need {CACHE} or the vendor STEP {CYCLO_STEP}")
        from build123d import export_stl, import_step          # slow path, runs once
        s = import_step(CYCLO_STEP)
        h = [c for c in s.children if c.label == "Housing_v1"][0]
        export_stl(h, CACHE)
    m = trimesh.load(CACHE)
    c = m.bounds.mean(0)
    m.apply_translation([-c[0], -c[1], -m.bounds[0][2]])        # axis -> origin, base -> z0
    return m


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    m = housing_mesh()
    m.export("build/housing.stl")
    b = m.bounds
    print(f"housing: vendor Housing_v1 (cyclo ring)  bbox {[round(v,1) for v in (b[1]-b[0])]}  "
          f"bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
