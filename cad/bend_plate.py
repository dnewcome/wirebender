"""bend_plate.py — the vendor cyclo base PLATE with a side BOSS + heat-set inserts.

Mirrors 3dprint-robot/angle_mount's body_boss, but on the cyclo PLATE instead of
the body: start from the REAL vendor cycloid NEMA base (nema-17-cycloid-base.stl,
the same part rothead.cyclo_base imports) and EXTRUDE A BOSS off its side. Two M3
heat-set inserts are drilled into the boss's outer FACE (horizontal, blind back) —
NOT through the plate's broad face.

The boss is a COPLANAR extension of the flat flange band only (z = 0 .. flange
top), so it clears BOTH the motor mounted on the flat (NEMA) face underneath and
the round cyclo body / output boss raised on the other face. It bolts to
rothead.bend_piece's vertical slotted face: the inserts sit at the slot spacing
(2*BMNT_SLOT_Y), and the slots give the ±BMNT_SLOT mandrel-height adjustment.

The vendor base is only a mesh (STL) and OCC segfaults booleaning a tessellated
import, so the union/cut is done with trimesh + manifold3d. Vendor STL not committed.

    py/bin/python cad/bend_plate.py   ->  build/bend_plate.stl
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import trimesh
import rothead as R

CYC_BASE_STL = R.CYC_BASE_STL
INSERT_D, INSERT_H = R.IFACE_INS_D, R.IFACE_INS_H   # 4.6 hole, 5 deep
BOLT_SP = 2 * R.BMNT_SLOT_Y    # 16 — match bend_piece's 2 slots (y = ±8)
BOSS_OUT = 11.0                # how far the boss mounting face sits beyond the plate edge
                               #   (was 8.0; +3mm to push the insert face clear of the turned-down end cap)
BOSS_W = BOLT_SP + 12          # 28 — boss width (Y): the 2 inserts at ±8 + margin
BOSS_BOND = 3.0                # overlap back into the plate so the boss fuses to the wall


def _xcyl(d, x0, x1, y, z):
    """Cylinder Ø d along +X from x0 to x1 at (y, z)."""
    cyl = trimesh.creation.cylinder(radius=d / 2, height=x1 - x0)
    cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))  # Z->X
    cyl.apply_translation([(x0 + x1) / 2, y, z])
    return cyl


def _flange_top(m, x):
    """Top of the flat flange (solid from z=0 up to the first gap) at the +X edge."""
    top = m.bounds[1][2]
    z = 0.2
    while z < top and m.contains([[x, 0.0, z]])[0]:
        z += 0.2
    return round(z - 0.2, 2)


def bend_plate():
    if not os.path.exists(CYC_BASE_STL):
        raise SystemExit(f"vendor plate missing: {CYC_BASE_STL}\n"
                         "  (nema-17-cycloid-base.stl is paid/not redistributed)")
    m = trimesh.load(CYC_BASE_STL)
    c = m.bounds.mean(axis=0)
    m.apply_translation([-c[0], -c[1], -m.bounds[0][2]])       # centre XY, base at z=0
    edge_x = m.bounds[1][0]                                    # +X edge
    face_x = edge_x + BOSS_OUT
    flange = _flange_top(m, edge_x - 1.0)                      # flat band height (~4.8mm)
    lip_z = INSERT_D + 3.0                                     # protruding-lip height (insert + wall)
    # pad: coplanar flange extension across the whole boss footprint (z 0..flange) —
    # flush with the flange where it bonds, so it clears the motor (under the flat
    # face) and the round body (above the flange), both within the plate footprint.
    pad_x0 = edge_x - BOSS_BOND
    pad = trimesh.creation.box(extents=[face_x - pad_x0, BOSS_W, flange])
    pad.apply_translation([(pad_x0 + face_x) / 2, 0, flange / 2])
    # lip: the part that PROTRUDES past the plate edge is clear of both the motor and
    # the round body, so give it real depth for the M3 insert. Its TOP is flush with
    # the flange/boss-base plane (z=flange) — nothing proud of the output side, so the
    # round body can grow — and it extends DOWN toward the motor side for insert wall.
    lip = trimesh.creation.box(extents=[BOSS_OUT, BOSS_W, lip_z])
    lip.apply_translation([(edge_x + face_x) / 2, 0, flange - lip_z / 2])
    part = trimesh.boolean.union([m, pad, lip], engine="manifold")
    # 2 M3 heat-set inserts drilled -X into the lip FACE (horizontal, blind back)
    zc = flange - lip_z / 2
    cuts = [_xcyl(INSERT_D, face_x - INSERT_H, face_x + 0.25, s * BOLT_SP / 2, zc) for s in (1, -1)]
    return trimesh.boolean.difference([part] + cuts, engine="manifold"), flange, lip_z


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part, flange, lip_z = bend_plate()
    part.export("build/bend_plate.stl")
    print("bend_plate (vendor cyclo base + side boss):", (part.bounds[1] - part.bounds[0]).round(1),
          f"| pad flush in the flange band (z=0..{flange})",
          f"| lip top flush at z={flange} (output side clear), extends down to "
          f"z={round(flange - lip_z, 1)} (motor side) holding the 2 M3 inserts @ {BOLT_SP}mm",
          "| watertight:", part.is_watertight)
