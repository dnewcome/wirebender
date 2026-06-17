"""bend_plate_90.py — bend_plate variant: boss extrudes out, then turns 90° UP into a riser
carrying a CROSS-AXIS (Y) adjustment slot.

Same start as bend_plate.py (vendor cyclo NEMA base + a coplanar flange extension off the +X
edge), but instead of ending in an outward insert face, the boss bends 90° up into a vertical
riser tab. The tab has ONE horizontal slot running in the cross (Y) direction, long enough to
hold both bolts (at the ±BMNT_SLOT_Y spacing) plus ±SLOT_TRAVEL of slide.

Bolted to rothead's vertical bending plate — whose slots run in Z — the two perpendicular
slots give the bend actuator full 2-axis (Y + Z) positioning: Z sets the disk height/reach,
Y sets the mandrel-center-to-wire offset. Through-bolts + nuts (no inserts), so both slots
can slide.

Vendor base is a mesh (STL, not redistributed); union/cut via trimesh + manifold3d.

    py/bin/python cad/bend_plate_90.py   ->  build/bend_plate_90.stl
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import trimesh
import rothead as R

CYC_BASE_STL = R.CYC_BASE_STL
M3_CLEAR = R.M3_CLEAR                 # 3.4 — through-bolt clearance
BOLT_SP = 2 * R.BMNT_SLOT_Y          # 16 — bolt spacing in Y (matches rothead's 2 slots)
BOSS_OUT = 17.0                      # how far the horizontal arm reaches past the plate edge
                                     #   (was 11; +6mm total to push the riser out so the head clears)
BOSS_BOND = 3.0                      # overlap back into the plate so the arm fuses to the wall
RISER_T = 6.0                        # riser tab thickness (X) — the mounting plate
RISER_H = 22.0                       # riser tab height (Z) above the flange plane
SLOT_TRAVEL = 8.0                    # ± cross-axis (Y) slide beyond the bolt span (full width gives room)
SLOT_HALF = BOLT_SP / 2 + SLOT_TRAVEL  # 16 — Y half-length of the slot
SLOT_Z = 11.0                        # slot centre height up the riser from the flange plane

# the full-width pad reaches the 2 bolt-hole countersinks on the +X end (NEMA 31mm pattern);
# re-cut them so the vendor plate's flat-head bolts still seat.
NEMA_HOLE = 15.5                     # ±15.5 hole pattern; the +X pair (x=+15.5, y=±15.5) is covered
CSK_THRU, CSK_TOP = 2.8, 6.4         # re-cut through-Ø and countersink major Ø


def _flange_top(m, x):
    top = m.bounds[1][2]
    z = 0.2
    while z < top and m.contains([[x, 0.0, z]])[0]:
        z += 0.2
    return round(z - 0.2, 2)


def _yslot(half, d, x0, x1, zc):
    """Clearance slot bored along +X (x0..x1), elongated ±half in Y, Ø d, centred at z=zc."""
    L = x1 - x0
    parts = [trimesh.creation.box([L, 2 * half, d],
                                  transform=trimesh.transformations.translation_matrix([(x0 + x1) / 2, 0, zc]))]
    for sy in (half, -half):
        c = trimesh.creation.cylinder(radius=d / 2, height=L)            # along Z
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))  # -> along X
        c.apply_translation([(x0 + x1) / 2, sy, zc])
        parts.append(c)
    return trimesh.boolean.union(parts, engine="manifold")


def _csk_cut(x, y, ztop):
    """Through bore + a 90° countersink opening upward at z=ztop, to re-clear a covered hole."""
    thru = trimesh.creation.cylinder(
        radius=CSK_THRU / 2, height=ztop + 6,
        transform=trimesh.transformations.translation_matrix([x, y, (ztop + 6) / 2 - 2]))
    cone = trimesh.creation.cone(radius=CSK_TOP / 2, height=CSK_TOP / 2)   # 90°: base r, apex at z=r
    cone.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))  # apex down
    cone.apply_translation([x, y, ztop])                                  # base (Ø CSK_TOP) at z=ztop
    return trimesh.boolean.union([thru, cone], engine="manifold")


def bend_plate_90():
    if not os.path.exists(CYC_BASE_STL):
        raise SystemExit(f"vendor plate missing: {CYC_BASE_STL}\n"
                         "  (nema-17-cycloid-base.stl is paid/not redistributed)")
    m = trimesh.load(CYC_BASE_STL)
    c = m.bounds.mean(axis=0)
    m.apply_translation([-c[0], -c[1], -m.bounds[0][2]])       # centre XY, base at z=0
    edge_x = m.bounds[1][0]                                    # +X edge
    full_w = float(m.bounds[1][1] - m.bounds[0][1])           # full NEMA plate width — square off the
    #                                                           +X-end bevels and extrude across it all
    face_x = edge_x + BOSS_OUT
    flange = _flange_top(m, edge_x - 1.0)                      # flat flange-band height (~4.8)

    # horizontal arm: full-width coplanar flange extension off the +X edge (z 0..flange); fills the
    # corner bevels at that end, clears the motor below and the round body above (like bend_plate's pad).
    pad_x0 = edge_x - BOSS_BOND
    arm = trimesh.creation.box(
        extents=[face_x - pad_x0, full_w, flange],
        transform=trimesh.transformations.translation_matrix([(pad_x0 + face_x) / 2, 0, flange / 2]))
    # vertical riser: the 90° bend-up tab at the outer end, full width, rising from the flange plane.
    riser = trimesh.creation.box(
        extents=[RISER_T, full_w, RISER_H + 2],               # +2 overlaps down into the arm
        transform=trimesh.transformations.translation_matrix(
            [face_x - RISER_T / 2, 0, flange - 2 + (RISER_H + 2) / 2]))
    part = trimesh.boolean.union([m, arm, riser], engine="manifold")

    # one cross-axis (Y) slot bored through the riser along X
    cuts = [_yslot(SLOT_HALF, M3_CLEAR, face_x - RISER_T - 1, face_x + 1, flange + SLOT_Z)]
    # re-cut the 2 +X bolt-hole countersinks the full-width pad covered
    for hy in (NEMA_HOLE, -NEMA_HOLE):
        cuts.append(_csk_cut(NEMA_HOLE, hy, flange))
    return trimesh.boolean.difference([part] + cuts, engine="manifold"), flange


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part, flange = bend_plate_90()
    if not part.is_watertight:
        part.merge_vertices(); trimesh.repair.fix_winding(part); trimesh.repair.fill_holes(part)
    part.export("build/bend_plate_90.stl")
    b = part.bounds
    print(f"bend_plate_90 (cyclo base + out+up riser): {(b[1]-b[0]).round(1)}"
          f"  riser {RISER_T}x{RISER_H} above flange z={flange}"
          f"  cross-axis Y-slot ±{SLOT_HALF} (bolts ±{BOLT_SP/2}, travel ±{SLOT_TRAVEL})"
          f"  bodies:{len(part.split(only_watertight=False))}  watertight:{part.is_watertight}")
