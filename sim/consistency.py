"""consistency.py — assert sim/machine.py matches the CAD constants, so the sim,
forward model, slicer, and manufacturability rules can't silently drift from the
printed parts. machine.py is the single source of truth for the toolchain; this
test is what keeps it honest against cad/.

    py/bin/python sim/consistency.py        (or: make check-consistency)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)                          # sim/  -> machine.py
sys.path.insert(0, os.path.join(ROOT, "cad"))     # cad/  -> build123d parts

import machine as M
import base
import rothead
import bend_disc
import cycloid

CHECKS = [
    ("wire axis above deck (AXIS_Z)",        M.AXIS_Z,              base.AXIS_Z),
    ("plate thickness (PLATE_TH=BASE_TH)",   M.PLATE_TH,            base.BASE_TH),
    ("wire axis world height mm",            M.WIRE_AXIS_WORLD_MM,  base.AXIS_Z + base.BASE_TH),
    ("feeder X (m)",                         M.FEEDER_X_WORLD,      base.FEEDER_X / 1000.0),
    ("feed tube Ø (TUBE_D)",                 M.TUBE_D,              rothead.TUBE_D),
    ("mandrel Ø (MANDREL_D)",                M.MANDREL_D,           rothead.MANDREL_D),
    ("pin Ø (PIN_D)",                        M.PIN_D,               bend_disc.PIN_D),
    ("pin sweep radius (PIN_SWEEP_R=PIN_OFFSET)", M.PIN_SWEEP_R,    bend_disc.PIN_OFFSET),
    ("fixed gear teeth (base)",              M.FIXED_GEAR_TEETH,    base.FG_TEETH),
    ("fixed gear teeth (rothead)",           M.FIXED_GEAR_TEETH,    rothead.FG_TEETH),
    ("pinion teeth (PIN_TEETH)",             M.PINION_TEETH,        rothead.PIN_TEETH),
    ("gear module (FG_MODULE)",              M.GEAR_MODULE,         base.FG_MODULE),
    ("gear mesh radius (MESH_R)",            M.MESH_R,              rothead.MESH_R),
    ("pinion mount X (PINION_MOUNT_X)",      M.PINION_MOUNT_X,      rothead.PINION_MOUNT_X),
    # ── bend-drive reduction (cad/cycloid.py) ──
    ("drive ratio (= ring pins)",            M.DRIVE_RATIO,         cycloid.N),
    ("ring pins (N_RING_PINS)",              M.N_RING_PINS,         cycloid.N),
    ("disc lobes (N_LOBES)",                 M.N_LOBES,             cycloid.LOBES),
    ("eccentricity (ECCENTRICITY=E)",        M.ECCENTRICITY,        cycloid.E),
    ("disc face width (DISC_FACE_W=DISC_T)", M.DISC_FACE_W,         cycloid.DISC_T),
    ("disc count (N_DISCS)",                 M.N_DISCS,             cycloid.N_DISCS),
    ("carrier radius (CARRIER_R)",           M.CARRIER_R,           cycloid.CARRIER_R),
    ("carrier roller Ø (CARRIER_ROLLER_D)",  M.CARRIER_ROLLER_D,    cycloid.ROLLER_D),
    ("carrier count (N_CARRIER)",            M.N_CARRIER,           cycloid.N_CARRIER),
]


def main():
    bad = 0
    for label, got, want in CHECKS:
        ok = abs(got - want) < 1e-6
        bad += not ok
        print(f"  [{'OK' if ok else 'MISMATCH'}] {label}: machine={got} cad={want}")
    if bad:
        print(f"{bad} mismatch(es) — machine.py is out of sync with the CAD.")
        raise SystemExit(1)
    print("machine.py is consistent with the CAD.")


if __name__ == "__main__":
    main()
