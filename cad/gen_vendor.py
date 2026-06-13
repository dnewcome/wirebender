"""gen_vendor.py — convert the purchased cycloidal STEP into local sim meshes.

Splits the Sweep Dynamics 20:1 micro-cycloidal into:
  - cyclo_body.stl : everything EXCEPT the output End Cap (fixed to the head = Axis 2)
  - (the rotating output + bend pin come from bend_endcap.py = Axis 3)

Vendor geometry — NOT committed. Regenerate locally:
    py/bin/python cad/gen_vendor.py   ->  build/cyclo_body.stl  (gitignored)
"""
import os
from build123d import import_step, export_stl, Compound

CYCLO_STEP = os.path.expanduser(
    "~/Downloads/cad-files/sweepdynamics/micro-cycloidal/20-1 Micro Cycloidal.step")

if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    s = import_step(CYCLO_STEP)
    # just the external envelope: motor base + housing. The fine internal discs/
    # shaft (the heavy geometry) are skipped; the output End Cap is Axis 3.
    keep = ("Base_-_Nema_17", "Housing_v1")
    body = Compound(children=[c for c in s.children if c.label in keep])
    eb = [c for c in s.children if c.label == "End_Cap"][0].bounding_box()
    cb = body.bounding_box()
    print("body  z:", round(cb.min.Z, 1), "..", round(cb.max.Z, 1))
    print("output (End_Cap) at", "+Z (top)" if eb.center().Z > cb.center().Z else "-Z (bottom)")
    export_stl(body, "build/cyclo_body.stl", tolerance=0.05, angular_tolerance=0.3)
    print("wrote build/cyclo_body.stl")
