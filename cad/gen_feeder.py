"""gen_feeder.py — convert the image-derived feeder GLB to a sim mesh.

extruder.glb is an approximate body model (generated from photos of the 1KGSSJ-B
feeder). Scaled so its long axis = the 122mm feeder length, centred, exported for
the sim. Native orientation already matches the machine: nose (wire output) on -X,
mounting face (motor side) on -Z, tension knob up (+Z).

The photo model comes out a thin slab in Z (~30mm), but the real feeder is taller:
its wire output sits FEEDER_OUTPUT_H (35mm) above the mounting face. We stretch Z so
the (centred) output axis is 35mm above the -Z face — total height 2*35mm — so when
the sim puts the output on the wire axis the mounting face seats flat on the deck.

    py/bin/python cad/gen_feeder.py   ->  build/feeder_body.stl   (gitignored)
"""
import os
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FEEDER_L = 122.0
FEEDER_OUTPUT_H = 35.0   # wire output axis above the mounting (-Z) face (real feeder)

if __name__ == "__main__":
    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    s = trimesh.load(os.path.join(ROOT, "extruder.glb"))
    g = s.to_geometry() if hasattr(s, "to_geometry") else s
    g.apply_scale(FEEDER_L / g.extents.max())          # length sets the overall scale
    g.apply_translation(-g.bounding_box.centroid)      # centre on origin
    # the photo model is a thin slab in Z; stretch it so the (centred) output axis
    # is FEEDER_OUTPUT_H above the -Z mounting face -> face seats flat on the deck.
    g.apply_scale([1.0, 1.0, 2.0 * FEEDER_OUTPUT_H / g.extents[2]])
    out = os.path.join(ROOT, "build", "feeder_body.stl")
    g.export(out)
    print("wrote", out, "bbox(mm)", [round(x, 1) for x in g.extents])
