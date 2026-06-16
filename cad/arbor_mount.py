"""arbor_mount.py — a version of the cyclo actuator BODY (Housing_v1) with 4
standoff posts + M3 heat-set inserts, to mount a bending plate onto it.

The housing is a 42x42 can, solid only at its corners (the centre is the cycloid
mechanism), so 4 posts stand off the OUTPUT-end face at the corners, each with an
M3 heat-set insert at its top; the bending plate bolts down onto the post tops.

Imports the vendor Housing_v1 from the cyclo STEP (slow ~200s; B-rep, so build123d
booleans cleanly). Vendor geometry is NOT committed.

    py/bin/python cad/arbor_mount.py   ->  build/arbor_mount.stl
"""
import os
from build123d import *

CYCLO_STEP = os.path.expanduser(
    "~/Downloads/cad-files/sweepdynamics/micro-cycloidal/20-1 Micro Cycloidal.step")

POST_SQ = 16.0         # posts at (±POST_SQ, ±POST_SQ) -> a 32mm square, on the corner walls
POST_OD = 7.0          # standoff post Ø
POST_H = 6.0           # standoff height (gap from the output face to the bending plate)
INSERT_D, INSERT_H = 4.6, 5.0      # M3 heat-set insert (in from the post top)


def arbor_mount():
    if not os.path.exists(CYCLO_STEP):
        raise SystemExit(f"cyclo STEP missing: {CYCLO_STEP} (vendor, not redistributed)")
    s = import_step(CYCLO_STEP)
    h = [c for c in s.children if c.label == "Housing_v1"][0]
    bb = h.bounding_box()
    h = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * h       # centre XY; output end up (+Z)
    top = h.bounding_box().max.Z
    for sx in (POST_SQ, -POST_SQ):
        for sy in (POST_SQ, -POST_SQ):
            h += Pos(sx, sy, top - 1) * Cylinder(          # standoff post; overlap 1mm into the wall to fuse
                POST_OD / 2, POST_H + 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
            h -= Pos(sx, sy, top + POST_H - INSERT_H) * Cylinder(   # M3 insert, in from the post top
                INSERT_D / 2, INSERT_H + 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return h


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = arbor_mount()
    export_stl(part, "build/arbor_mount.stl")
    bb = part.bounding_box()
    print("arbor_mount (Housing_v1 + 4 corner standoff posts):", [round(v, 1) for v in bb.size],
          f"| posts ±{POST_SQ}mm square, {POST_H}mm tall, M3 inserts | solids:", len(part.solids()))
