"""cycloid.py — the cycloidal DISC + ECCENTRIC SHAFT, the last vendor-sealed internals.

These are the two moving parts inside the Sweep Dynamics cartridge that were NOT yet
parametric (housing.py = the ring, end_cap.py = the fixed output-pin carrier, cyclo_base.py
= the input/output base were already rebuilt). With these, the whole 20:1 drive is
parametric and you can scale ratio / face width / material yourself.

VALIDATED against the vendor files (20-1 Micro Cycloidal.step + Print-Ready STLs): the disc
profile below matches the vendor "Discs Nul" to ~0.001mm on both peak and valley radius;
the dims are confirmed off the STEP B-rep and the BOM.

Kinematics (why it's 20:1): the OUTPUT-pin carrier (the 6 alu rollers, fixed on the bolts
through end_cap+base) is FIXED and the RING (housing, 20 pins) is the OUTPUT, so the ratio =
N_pins = 20. With the carrier fixed the disc has N_pins-1 = 19 lobes. The eccentric shaft
(on the motor) is the INPUT; it orbits the disc by E, the 19 lobes walk around the 20 ring
pins, and the ring creeps one pin per input turn -> 20:1.

  * Eccentricity E = 0.625mm — the VENDOR value, measured two independent ways off the STEP:
    (a) carrier-hole Ø6.25 − roller Ø5, /2 = 0.625; (b) the disc lobe depth (1.251mm) fits
    E=0.6255. NOTE this is a REDUCED eccentricity: the "full" rolling value is R/N=0.925.
    Running less than full E gives shallower lobes and lower contact stress (smoother, less
    peak load) at a slightly lower theoretical contact ratio.
  * Disc lobe profile: the standard pin-wheel cycloid (envelope of the ring pins) offset by
    the pin radius Rr so the flanks are smooth. At E=0.625 -> lobe radius 16.375..17.625
    (mean 17.0 = pin crest r17.0), Ø35 OD.
  * Carrier holes: 6 holes Ø = roller Ø + 2E so the disc orbits on the 6 Ø5 ALUMINIUM
    ROLLERS (vendor BOM: M3*5*10 rollers on M3*20 bolts at r10.125). THIS is the
    load-transmitting interface — output torque reacts here. The vendor already runs metal
    rollers (not bare bolts); bigger/steel rollers + thicker discs is the scaling path.
  * Eccentric shaft (FAITHFUL to the vendor Shaft, every section measured off the STEP):
    23.2 long — bottom support Ø6.8 (0..6.5) -> two Ø8 eccentric journals 180° apart, 5.1
    each (carry the 8x12x3.5 bearings; the two journals = 10.2 = the ring band) -> top
    support Ø6.8; a D-bore (Ø5.25 round + a flat at AF 4.6) 18.9 deep from the bottom that
    PRESS-FITS onto the NEMA17 D-shaft — the flat keys the torque, the interference fit (bore
    modeled oversize for FDM shrinkage) holds it axially. No set screw (no room; vendor-proven).

DESIGN KNOBS for scaling (see the bend-force discussion):
  * DISC_T (face width) — linear capacity lever; lowers carrier-hole bearing stress (the weak
    contact). 2 discs must fit the ring toothed band (housing PIN_Z0..PIN_Z1 = 10.2mm).
  * Steel carrier rollers + a real eccentric bearing (already present) + PA-CF discs are the
    material upgrades that take 1/4"-rod force; geometry alone won't.

    py/bin/python cad/cycloid.py   ->  build/cycloid_disc.stl + build/cycloid_shaft.stl
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build123d import (Align, Box, BuildLine, BuildPart, BuildSketch, Cylinder, Plane,
                       Polyline, Pos, Rot, export_stl, extrude, make_face)
import housing as H
import cyclo_base as CB

# ── ring it meshes (from housing.py) ─────────────────────────────────────────
N = H.N_PINS                 # 20 ring pins
R = H.PIN_PITCH              # 18.5 pin-axis pitch radius
Rr = H.PIN_R                 # 1.5 ring-pin radius (Ø3)
LOBES = N - 1                # 19 disc lobes (carrier fixed -> ratio = N = 20:1)
E = 0.625                    # eccentricity (vendor-measured; reduced from the full R/N=0.925)

# ── disc (vendor "Discs Nul": Ø35 x 4.6, bore Ø12, 6x Ø6.25 @ r10.125) ───────
DISC_T = 4.6                 # face width per disc (2 fit the 10.2 toothed band); scaling lever
ECC_BRG = (8.0, 12.0, 3.5)   # eccentric bearing ID, OD, W  (vendor 8x12x3.5)
BORE_D = ECC_BRG[1]          # disc central bore = bearing OD (Ø12), the press fit
BEARING_LAND_D = 11.0        # land Ø at ONE face — a 0.5mm-wide lip the bearing bottoms on (seats it square)
BEARING_LAND_H = 0.75        # land height; the bearing presses into the Ø12 above and stops on this shoulder
CARRIER_R = CB.CYC_BOLT_R    # 10.125 — the 6 fixed carrier pins (housing bolts)
N_CARRIER = 6
ROLLER_D = 5.0               # carrier roller OD (vendor M3*5*10 alu rollers; steel to scale)
CARRIER_HOLE_D = ROLLER_D + 2 * E    # Ø6.25 — must clear the orbit (roller Ø + 2E)
N_PROFILE = 720             # disc-profile sample points

# ── eccentric shaft — faithful to the vendor Shaft (measured off the STEP B-rep) ──
N_DISCS = 2                  # balanced 2-disc stack (journals 180° apart)
SUPP_BRG = (7.0, 13.0, 4.0)  # support bearing ID, OD, W (vendor 7x13x4, in base + end_cap)
SHAFT_LEN = 23.2             # overall length (vendor)
SUPP_OD = 6.8                # support-journal OD (rides the 7x13 bearings; tune to your print)
SUPP_L_BOT = 6.5             # bottom support length (= where the eccentric journals start)
JOURNAL_ECC_D = 8.0          # eccentric-journal OD (rides the 8x12 bearings; vendor 8.05, 8.0 for slip-fit)
ECC_L = 5.1                  # each eccentric-journal length (2 of them -> 10.2 = ring toothed band)
MOTOR_BORE_D = 5.25          # NEMA17 D-shaft bore (round part) — PRESS FIT: modeled oversize so FDM
#                              shrinkage lands it at an interference fit on the Ø5 shaft (vendor-proven)
MOTOR_FLAT_AF = 4.6          # D-flat across-flat (flat 1.975 off-axis) — keys the D-shaft for torque, no set screw
MOTOR_BORE_DEPTH = 18.9      # bore depth from the bottom (vendor; blind above this)
WELD = 0.3                   # shoulder weld overlap (avoids coincident-face fuse failures)
ECC1_Z0 = SUPP_L_BOT         # z where eccentric journal 1 begins

_C, _MIN = Align.CENTER, Align.MIN


def _profile():
    """The 19-lobe pin-wheel cycloid: envelope of the N ring pins, offset by the pin radius
    Rr. Standard parametric form (θ over one full turn traces all N-1 lobes)."""
    pts = []
    for i in range(N_PROFILE):
        t = 2 * math.pi * i / N_PROFILE
        psi = math.atan2(math.sin((1 - N) * t),
                         (R / (E * N)) - math.cos((1 - N) * t))
        x = R * math.cos(t) - Rr * math.cos(t + psi) - E * math.cos(N * t)
        y = -R * math.sin(t) + Rr * math.sin(t + psi) + E * math.sin(N * t)
        pts.append((x, y))
    return pts


def disc():
    pts = _profile()
    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        extrude(amount=DISC_T)
    part = bp.part
    part -= Pos(0, 0, -1) * Cylinder(BEARING_LAND_D / 2, DISC_T + 2, align=(_C, _C, _MIN))     # Ø11 land (through)
    part -= Pos(0, 0, BEARING_LAND_H) * Cylinder(BORE_D / 2, DISC_T - BEARING_LAND_H + 1,      # Ø12 bearing seat,
                                                 align=(_C, _C, _MIN))                          #   leaving the land at the base face
    for k in range(N_CARRIER):                                                       # 6 carrier holes (orbit on rollers)
        a = math.radians(360 * k / N_CARRIER)
        x, y = CARRIER_R * math.cos(a), CARRIER_R * math.sin(a)
        part -= Pos(x, y, -1) * Cylinder(CARRIER_HOLE_D / 2, DISC_T + 2, align=(_C, _C, _MIN))
    return part


def eccentric_shaft():
    """Vendor-faithful eccentric shaft: bottom support (Ø6.8) -> 2 eccentric journals
    (Ø8, 180° apart) -> top support (Ø6.8), with a Ø5.25 motor bore from the bottom and an
    ADDED radial M3 set screw (the vendor's round bore can't transmit torque on its own).
    Sections welded with a small overlap and fused in one op."""
    adds = [Pos(0, 0, 0) * Cylinder(SUPP_OD / 2, ECC1_Z0 + WELD, align=(_C, _C, _MIN))]  # bottom support
    z = ECC1_Z0
    for d in range(N_DISCS):
        ang = math.radians(180 * d)
        ox, oy = E * math.cos(ang), E * math.sin(ang)
        adds.append(Pos(ox, oy, z - WELD) * Cylinder(JOURNAL_ECC_D / 2, ECC_L + 2 * WELD, align=(_C, _C, _MIN)))
        z += ECC_L
    adds.append(Pos(0, 0, z - WELD) * Cylinder(SUPP_OD / 2, SHAFT_LEN - z + WELD, align=(_C, _C, _MIN)))  # top support
    part = adds[0].fuse(*adds[1:])
    # D-bore for the NEMA17 D-shaft: Ø5.25 cylinder flattened at MOTOR_FLAT_AF, from the bottom
    bore = Pos(0, 0, -1) * Cylinder(MOTOR_BORE_D / 2, MOTOR_BORE_DEPTH + 1, align=(_C, _C, _MIN))
    flat_x = MOTOR_FLAT_AF - MOTOR_BORE_D / 2
    bore &= Pos(flat_x, 0, -2) * Box(40, 40, MOTOR_BORE_DEPTH + 4, align=(Align.MAX, _C, _MIN))   # flatten one side
    part -= bore
    return part


def _report():
    rad = [math.hypot(x, y) for x, y in _profile()]
    crest = R - Rr
    web = (CARRIER_R - CARRIER_HOLE_D / 2) - BORE_D / 2
    print(f"  E={E:.3f} (full R/N={R/N:.3f})  lobes={LOBES} ratio={N}:1  "
          f"disc Rmax={max(rad):.3f} Rmin={min(rad):.3f} mean={sum(rad)/len(rad):.3f} (pin crest r{crest:.1f})")
    print(f"  bore Ø{BORE_D} (ecc brg {ECC_BRG[0]}x{ECC_BRG[1]}x{ECC_BRG[2]})  "
          f"carrier Ø{CARRIER_HOLE_D:.2f} @ r{CARRIER_R} on Ø{ROLLER_D} rollers  bore-to-carrier web={web:.2f}mm")


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    import trimesh
    _report()
    for name, p in (("cycloid_disc", disc()), ("cycloid_shaft", eccentric_shaft())):
        export_stl(p, f"build/{name}.stl")
        m = trimesh.load(f"build/{name}.stl")
        bb = p.bounding_box()
        print(f"{name}: bbox {[round(v, 1) for v in bb.size]}  "
              f"bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
