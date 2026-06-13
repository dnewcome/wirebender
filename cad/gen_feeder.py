"""gen_feeder.py — convert the image-derived feeder GLB to a sim mesh.

extruder.glb is an approximate body model (generated from photos of the 1KGSSJ-B
feeder). Scaled so its long axis = the 122mm feeder length, centred, exported for
the sim. Native orientation already matches the machine: nose (wire output) on -X,
mounting face (motor side) on -Z, tension knob up (+Z).

    py/bin/python cad/gen_feeder.py   ->  build/feeder_body.stl   (gitignored)
"""
import os
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FEEDER_L = 122.0

if __name__ == "__main__":
    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    s = trimesh.load(os.path.join(ROOT, "extruder.glb"))
    g = s.to_geometry() if hasattr(s, "to_geometry") else s
    g.apply_scale(FEEDER_L / g.extents.max())
    g.apply_translation(-g.bounding_box.centroid)
    out = os.path.join(ROOT, "build", "feeder_body.stl")
    g.export(out)
    print("wrote", out, "bbox(mm)", [round(x, 1) for x in g.extents])
