"""check_cyclo.py — fit check for the cycloid internals against the ring, plus a viewable
assembly STL (housing + 2 discs meshed + eccentric shaft).

Asserts the parametric disc/shaft (cycloid.py) actually drop into the parametric ring
(housing.py) and base (cyclo_base.py):
  * MESH: each disc lobe set contacts the 20 ring pins tangentially (gap ≈ Rr) at the right
    phase, with no penetration — this is what transmits torque.
  * SHELL: the orbiting disc (Rmax + E) clears the Ø37 shell bore.
  * AXIAL: N_DISCS * face width fits the ring's toothed band (PIN_Z0..PIN_Z1).
  * CARRIER: the carrier holes clear the rollers over the full orbit.
  * BORE/BEARING: the disc bore matches the eccentric-bearing OD and the shaft journal.

    py/bin/python cad/check_cyclo.py   ->  PASS/FAIL + build/cycloid_assembly.stl
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import Compound, Pos, Rot, export_stl
import housing as H
import cyclo_base as CB
import cycloid as C


def _mesh_phase(offset_dir=0.0, nrot=2880):
    """Disc offset by E along offset_dir; return (best disc rotation, min pin->disc gap)."""
    pts = np.array(C._profile())
    px, py = pts[:, 0], pts[:, 1]
    pa = np.linspace(0, 2 * np.pi, H.N_PINS, endpoint=False)
    pins = np.stack([H.PIN_PITCH * np.cos(pa), H.PIN_PITCH * np.sin(pa)], 1)
    ox, oy = C.E * math.cos(offset_dir), C.E * math.sin(offset_dir)

    def gap(a):
        ca, sa = math.cos(a), math.sin(a)
        A = np.stack([px * ca - py * sa + ox, px * sa + py * ca + oy], 1)
        B = np.roll(A, -1, 0); AB = B - A; L2 = (AB ** 2).sum(1)
        return min(np.min(np.hypot(*(c - (A + np.clip(((c - A) * AB).sum(1) / L2, 0, 1)[:, None] * AB)).T))
                   for c in pins)

    rots = np.linspace(0, 2 * np.pi, nrot, endpoint=False)
    g = [gap(a) for a in rots]
    i = int(np.argmax(g))
    return rots[i], g[i]


def check():
    rad = [math.hypot(x, y) for x, y in C._profile()]
    rmax = max(rad)
    rot, gap = _mesh_phase(0.0)
    band = H.PIN_Z1 - H.PIN_Z0
    axial = C.N_DISCS * C.DISC_T
    carrier_clear = (C.CARRIER_R - C.CARRIER_HOLE_D / 2) - (C.ROLLER_D / 2 + C.E)  # roller far edge vs hole wall
    checks = [
        ("mesh (pin tangent: gap≈Rr, no penetration)", abs(gap - H.PIN_R) < 0.10, f"gap={gap:.3f} Rr={H.PIN_R} (Δ{gap-H.PIN_R:+.3f})"),
        ("disc clears shell bore (Rmax+E < R_BORE)",    rmax + C.E < H.R_BORE,      f"{rmax+C.E:.2f} < {H.R_BORE} ({H.R_BORE-(rmax+C.E):+.2f}mm)"),
        ("N_DISCS fit the toothed band",                axial <= band + 1e-6,       f"{axial:.1f} <= {band:.1f}mm ({band-axial:+.1f})"),
        ("carrier holes clear rollers over orbit",      carrier_clear >= -1e-6,     f"slack {carrier_clear:+.2f}mm"),
        ("disc bore == eccentric-bearing OD",           abs(C.BORE_D - C.ECC_BRG[1]) < 1e-6, f"Ø{C.BORE_D} == Ø{C.ECC_BRG[1]}"),
        ("shaft ecc journal == bearing ID",             abs(C.JOURNAL_ECC_D - C.ECC_BRG[0]) < 1e-6, f"Ø{C.JOURNAL_ECC_D} == Ø{C.ECC_BRG[0]}"),
        ("6-bolt carrier matches base pattern",         abs(C.CARRIER_R - CB.CYC_BOLT_R) < 1e-6, f"r{C.CARRIER_R} == r{CB.CYC_BOLT_R}"),
    ]
    bad = 0
    for label, ok, detail in checks:
        bad += not ok
        print(f"  [{'OK' if ok else 'FAIL'}] {label}: {detail}")
    return bad, rot


def assembly(rot0):
    """housing + 2 discs (meshed, 180° apart) + eccentric shaft, in the shaft frame.
    Shaft at z=0; discs centred on its two eccentric journals; housing shifted so its
    toothed band wraps the journals."""
    rot1, _ = _mesh_phase(math.pi)
    jc1 = C.ECC1_Z0 + C.ECC_L / 2                       # eccentric journal 1 centre z
    jc2 = C.ECC1_Z0 + C.ECC_L + C.ECC_L / 2            # journal 2 centre z
    housing_z = C.ECC1_Z0 - H.PIN_Z0                    # band start aligns to journal start
    d = C.disc()
    parts = [
        Pos(0, 0, housing_z) * H.housing(),
        Pos(C.E, 0, jc1 - C.DISC_T / 2) * Rot(0, 0, math.degrees(rot0)) * d,
        Pos(-C.E, 0, jc2 - C.DISC_T / 2) * Rot(0, 0, math.degrees(rot1)) * d,
        C.eccentric_shaft(),
    ]
    return Compound(children=parts)


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    bad, rot0 = check()
    export_stl(assembly(rot0), "build/cycloid_assembly.stl")
    print("  -> build/cycloid_assembly.stl (housing + 2 discs meshed + shaft)")
    if bad:
        print(f"{bad} fit problem(s).")
        raise SystemExit(1)
    print("cycloid internals fit the ring.")
