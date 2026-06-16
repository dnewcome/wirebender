"""bend_plate.py — the vendor cyclo base PLATE with heat-set inserts added.

The wirebender analog of 3dprint-robot/angle_mount's mount_plate: it starts from
the REAL vendor cycloid NEMA base (nema-17-cycloid-base.stl, the same part
rothead.cyclo_base imports) and adds 2 M3 heat-set inserts ORTHOGONAL to the plate
face (axial, blind). It bolts to rothead.bend_piece's vertical slotted face — the
inserts sit at the slot spacing (2*BMNT_SLOT_Y), and the slots give the
±BMNT_SLOT mandrel-height adjustment. Inverts the robot pattern: the inserts live
on the PLATE, not the cyclo body.

The vendor base is only available as a mesh (STL), and OCC/build123d segfaults
booleaning a tessellated import, so the insert holes are cut with trimesh +
manifold3d. Vendor geometry is NOT committed — needs the local STL.

    py/bin/python cad/bend_plate.py   ->  build/bend_plate.stl
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import trimesh
import rothead as R

CYC_BASE_STL = R.CYC_BASE_STL
INSERT_D, INSERT_H = R.IFACE_INS_D, R.IFACE_INS_H   # 4.6 hole, 5 deep
BOLT_SP = 2 * R.BMNT_SLOT_Y    # 16 — match bend_piece's 2 slots (y = ±8)
INSERT_Y = 12.0                # insert row: solid material, clear of the output bore + corner cuts


def bend_plate():
    if not os.path.exists(CYC_BASE_STL):
        raise SystemExit(f"vendor plate missing: {CYC_BASE_STL}\n"
                         "  (nema-17-cycloid-base.stl is paid/not redistributed)")
    m = trimesh.load(CYC_BASE_STL)
    c = m.bounds.mean(axis=0)
    m.apply_translation([-c[0], -c[1], -m.bounds[0][2]])       # centre XY, base at z=0
    top = m.bounds[1][2]
    cuts = []
    for sx in (BOLT_SP / 2, -BOLT_SP / 2):                     # 2 axial inserts from the top face
        cyl = trimesh.creation.cylinder(radius=INSERT_D / 2, height=INSERT_H + 0.5)
        cyl.apply_translation([sx, INSERT_Y, top - INSERT_H / 2 + 0.25])
        cuts.append(cyl)
    return trimesh.boolean.difference([m] + cuts, engine="manifold")


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = bend_plate()
    part.export("build/bend_plate.stl")
    print("bend_plate (vendor cyclo base + inserts):", (part.bounds[1] - part.bounds[0]).round(1),
          f"| 2x M3 inserts @ {BOLT_SP}mm, orthogonal to the face | watertight:", part.is_watertight)
