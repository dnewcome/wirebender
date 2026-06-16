"""feeder_motor_mount.py — pivoting NEMA17 plate to set the feeder gear mesh.

A printed plate that fixes to the feeder housing and carries the NEMA17 stepper on TWO
DIAGONAL bolts: one is a fixed PIVOT, the other rides a countersunk ARC slot. Loosen the arc
screw, swing the motor about the pivot to set the pinion/gear backlash, then lock it — so you
never have to drill blind holes into the unit and hope.

  * Motor bolts: pivot hole + arc slot, both COUNTERSUNK on the back (+Z) face for M3 flat
    heads threading into the motor (motor on the -Z side).
  * Central opening: a SLOT (not a round hole) sized to clear the Ø22 pilot boss as the shaft
    swings through it. The pinion sits beyond the plate, on the gear side.
  * Plate→housing: 2 plain THROUGH holes — screws from inside the housing, out through the
    plate, to nuts on the back.

NEMA17 dims are standard and baked in. The HOUSING-SPECIFIC values are flagged below: MEASURE
them on your unit (the 1KGSSJ-B body in this repo is only a photo-approximation, so the code
can't know them) and edit the constants.

    py/bin/python cad/feeder_motor_mount.py   ->  build/feeder_motor_mount.stl
"""
import math
import os

from build123d import Align, Box, Cone, Cylinder, Pos, export_stl

_C, _MIN = Align.CENTER, Align.MIN

# ── NEMA17 standard (usually leave alone) ───────────────────────────
NEMA_BOLT = 31.0          # square bolt pitch -> holes at ±15.5; diagonal pair = 43.8mm apart
PILOT_D = 22.0            # centering pilot boss Ø — the central slot must clear this as it swings
M3_CLEAR = 3.4            # motor-screw clearance
CSK_OD = 6.0             # M3 flat-head countersink major Ø (90°)

# ── plate + adjustment (tune to taste) ──────────────────────────────
T = 4.0                  # plate thickness
CLEAR = 0.6              # clearance around the swinging pilot boss
SWEEP = 6.0             # total pivot sweep in degrees (±3°); shaft travel ≈ R_shaft·sweep

# ── MEASURE ON YOUR FEEDER (placeholders — edit these) ──────────────
PLATE = (60.0, 60.0)                              # plate outline W×H, centred on the motor pattern
MOUNT_HOLE_D = 3.4                               # housing-screw clearance (M3)
HOUSING_HOLES = [(-24.0, 24.0), (24.0, -24.0)]   # the 2 plate→housing holes (plate-plane X,Y)

# pivot at one diagonal corner, arc bolt at the opposite corner; shaft/pilot at the centre
PX, PY = -NEMA_BOLT / 2, -NEMA_BOLT / 2
AX, AY = NEMA_BOLT / 2, NEMA_BOLT / 2
R_ARC = math.hypot(AX - PX, AY - PY)             # arc-slot radius (≈43.8)
R_SHAFT = math.hypot(PX, PY)                     # shaft swing radius about the pivot (≈21.9)
A0 = math.degrees(math.atan2(AY - PY, AX - PX))  # 45° — P, centre and A are colinear


def _through(part, x, y, r):
    return part - Pos(x, y, -1) * Cylinder(r, T + 2, align=(_C, _C, _MIN))


def _csk(part, x, y, r):
    """Through bore + 90° countersink opening on the +Z (back) face."""
    rh, d = CSK_OD / 2, CSK_OD / 2 - r
    part = part - Pos(x, y, -1) * Cylinder(r, T + 2, align=(_C, _C, _MIN))
    part = part - Pos(x, y, T - d) * Cone(r, rh, d, align=(_C, _C, _MIN))
    return part


def _arc(cx, cy, radius, n=17):
    """Points along the swing arc (centre cx,cy, given radius), swept ±SWEEP/2 about A0."""
    return [(cx + radius * math.cos(math.radians(A0 + t)),
             cy + radius * math.sin(math.radians(A0 + t)))
            for t in (-SWEEP / 2 + SWEEP * i / (n - 1) for i in range(n))]


def feeder_motor_mount():
    plate = Pos(0, 0, T / 2) * Box(PLATE[0], PLATE[1], T)
    plate = _csk(plate, PX, PY, M3_CLEAR / 2)                       # fixed pivot bolt
    for x, y in _arc(PX, PY, R_ARC):                               # countersunk arc slot (other bolt)
        plate = _csk(plate, x, y, M3_CLEAR / 2)
    for x, y in _arc(PX, PY, R_SHAFT):                            # central slot: clears the pilot, swung
        plate = plate - Pos(x, y, -1) * Cylinder(PILOT_D / 2 + CLEAR, T + 2, align=(_C, _C, _MIN))
    for hx, hy in HOUSING_HOLES:                                   # plate→housing through holes
        plate = _through(plate, hx, hy, MOUNT_HOLE_D / 2)
    return plate


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    part = feeder_motor_mount()
    export_stl(part, "build/feeder_motor_mount.stl")
    travel = R_SHAFT * math.radians(SWEEP)
    import trimesh
    m = trimesh.load("build/feeder_motor_mount.stl")
    print(f"feeder_motor_mount: {PLATE[0]}x{PLATE[1]}x{T} plate  NEMA17 diagonal pivot+arc (sweep ±{SWEEP/2:.0f}°"
          f" -> ±{travel/2:.2f}mm mesh adj)  pilot slot Ø{PILOT_D + 2*CLEAR:.1f}  2 housing holes"
          f"  bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
