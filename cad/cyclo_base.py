"""cyclo_base.py — parametric rebuild of the cycloidal drive's integrated NEMA17 base.

Fully-parametric replacement for the purchased base part (same approach as housing.py /
end_cap.py), so nothing in the build reads a vendor STEP/STL. Every dimension below was
measured ONCE off the vendor B-rep; the build never looks at the vendor source again.

The base is a 42x42x9 plate on the NEMA17 footprint with:
  - a round Ø30 output/register boss on the cyclo side (the bend-disk seat), standing on a
    thin Ø33 land (0.25mm proud of the face) that the bearing/disk seats on — not the flat face,
  - a stepped central output-bearing bore (Ø28 motor-side recess -> Ø11/13/14 bore),
  - the NEMA17 4-bolt pattern (±15.5), countersunk for M3 FLAT-head screws driven from the
    cyclo side into the motor's tapped face (heads flush at the plate top so the parts clear),
  - a 6-bolt cyclo-housing pattern at r=10.125 (matches end_cap.py): a clearance hole from the
    cyclo side into a captive M3 hex-nut pocket that opens into the motor-side recess (nut
    pressed in from the back, housing screw threads into it from the top).

Orientation: NEMA (motor) face at z=0, cyclo/boss face at z=T, output axis on Z, centred XY.

    py/bin/python cad/cyclo_base.py   ->  build/cyclo_base.stl
"""
import math
import os

from build123d import (Align, Axis, BuildLine, BuildPart, BuildSketch, Box, Cone, Cylinder,
                        Plane, Polyline, Pos, RegularPolygon, export_stl, extrude, make_face,
                        revolve)

SQ = 42.0                 # NEMA17 footprint (square plate)
T = 9.0                   # overall thickness (z = 0 .. T)
PLATE_T = 4.85            # square-plate TOP face (z); the stand-off land + boss rise above it
LAND_OD = 33.0            # thin stand-off land at the boss base — LARGER Ø than the boss, so the
LAND_H = 0.25             #   bearing/disk seats on this 0.25mm-proud land, stood off the flat face
BOSS_OD = 30.05           # cyclo output / register boss Ø (≈ rothead CYC_OUT_D=30)

# central output-bearing bore: (radius, z) profile, revolved about Z. Motor-side Ø28 recess
# (z 0..2.5), then the bearing bore up to the top, finished with a 45° lead-in CHAMFER at the
# mouth (Ø13->Ø14 over z 8.5..9) — NOT a counterbore relief, so the bearing pushes straight in.
BORE = [(0.0, 0.0), (14.0, 0.0), (14.0, 2.5),
        (6.0, 2.5), (5.5, 3.0), (5.5, 4.9),
        (6.5, 4.9), (6.5, 8.5), (7.0, 9.0), (0.0, 9.0)]

NEMA_BOLT = 15.5          # NEMA17 4-bolt pattern at (±15.5, ±15.5)
NEMA_CLEAR = 3.4          # M3 clearance (screws from the cyclo side into the motor's tapped face)
NEMA_CSK = 6.5            # M3 flat-head countersink major Ø (90°) — Ø6.5 sinks the head ~0.25mm
#                           below the face so it fully clears (a nominal Ø6 left the head slightly proud)
CYC_BOLT_R = 10.125       # 6-bolt cyclo-housing pattern (matches end_cap.py BOLT_R)
CYC_BOLT_D = 3.4          # M3 screw clearance through the boss (screws thread into the captive nuts)
RECESS_FLOOR = 2.5        # Ø28 motor-side recess floor — the nut pockets open into it (pressed in from the back)
NUT_AF = 5.6              # M3 hex-nut pocket across-flats (5.5 nut + a little press clearance)
NUT_POCKET_TOP = 6.5      # hex-pocket ceiling; the nut seats here, the Ø3.4 hole runs up to the top

_C, _MIN = Align.CENTER, Align.MIN


def _ring_above(z0, z1, dia):
    """Remove the square-plate corners between z0..z1, leaving a Ø dia round step."""
    return (Pos(0, 0, (z0 + z1) / 2) * Box(SQ, SQ, z1 - z0)
            - Pos(0, 0, z0) * Cylinder(dia / 2, z1 - z0, align=(_C, _C, _MIN)))


def cyclo_base():
    # square plate (z 0..PLATE_T) -> thin Ø33 stand-off land (LAND_H) -> Ø30 boss to the top
    part = Pos(0, 0, T / 2) * Box(SQ, SQ, T)
    part -= _ring_above(PLATE_T, PLATE_T + LAND_H, LAND_OD)   # Ø33 land, 0.25mm proud of the face
    part -= _ring_above(PLATE_T + LAND_H, T, BOSS_OD)         # Ø30 output boss above the land
    # central stepped output bore (revolve the profile about Z)
    with BuildPart() as bp:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(*BORE, close=True)
            make_face()
        revolve(axis=Axis.Z)
    part -= bp.part
    # NEMA17 4-bolt: clearance through the plate + 90° flat-head countersink opening at the plate top
    csk_depth = (NEMA_CSK - NEMA_CLEAR) / 2
    for sx in (NEMA_BOLT, -NEMA_BOLT):
        for sy in (NEMA_BOLT, -NEMA_BOLT):
            part -= Pos(sx, sy, -1) * Cylinder(NEMA_CLEAR / 2, PLATE_T + 2, align=(_C, _C, _MIN))
            part -= Pos(sx, sy, PLATE_T - csk_depth) * Cone(
                NEMA_CLEAR / 2, NEMA_CSK / 2, csk_depth, align=(_C, _C, _MIN))
    # 6-bolt cyclo-housing pattern: screw clearance hole from the cyclo (top) side into a captive
    # M3 hex-nut pocket that opens DOWN into the motor-side recess — the nut is pressed in from the
    # back, the housing screw threads into it from the top.
    for i in range(6):
        a = math.radians(60 * i)
        x, y = CYC_BOLT_R * math.cos(a), CYC_BOLT_R * math.sin(a)
        part -= Pos(x, y, NUT_POCKET_TOP) * Cylinder(
            CYC_BOLT_D / 2, T - NUT_POCKET_TOP + 1, align=(_C, _C, _MIN))               # screw clearance to top
        part -= Pos(x, y, RECESS_FLOOR) * extrude(
            RegularPolygon(NUT_AF / 2, 6, major_radius=False), NUT_POCKET_TOP - RECESS_FLOOR)  # hex nut pocket
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = cyclo_base()
    export_stl(part, "build/cyclo_base.stl")
    bb = part.bounding_box()
    import trimesh
    m = trimesh.load("build/cyclo_base.stl")
    print(f"cyclo_base: {SQ}x{SQ}x{T} plate + Ø{LAND_OD}x{LAND_H} stand-off land + Ø{BOSS_OD} boss  "
          f"4x NEMA M3 csk @±{NEMA_BOLT}  6x cyclo bolts @ r{CYC_BOLT_R} w/ captive M3 nut pockets (AF{NUT_AF})  "
          f"bbox {[round(v,1) for v in bb.size]}  bodies:{len(m.split(only_watertight=False))}  "
          f"watertight:{m.is_watertight}")
