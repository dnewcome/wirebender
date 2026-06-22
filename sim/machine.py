"""machine.py — single source of truth for the wire bender's machine parameters.

Every part of the toolchain reads its machine numbers from here so the CAD, the
MuJoCo sim, the forward model, the slicer, and the manufacturability rule-checker
all describe ONE machine:

    bend_model.py   BEND_RADIUS                      (the corner radius it lays)
    interference.py mandrel/pin/die limits           (the rules it enforces)
    check_pin.py    wire-axis height, tube Ø         (mesh clearance probe)
    make_mjcf.py    world axis height, feeder X, tube (the MuJoCo model)
    animate_bend.py wire-axis height, BEND_RADIUS    (the animation)
    drive_model.py  ratio/disc/motor                 (what stock can it bend?)

The geometry constants MIRROR the CAD (cad/base.py, cad/rothead.py,
cad/bend_disc.py, cad/cycloid.py). `sim/consistency.py` (`make check-consistency`)
asserts they stay in sync, so this file can't silently drift from the printed parts.

Units: mm and degrees unless a name says otherwise (`_WORLD_*` are MuJoCo metres).
"""

# ── frame / axis heights (cad/base.py) ──────────────────────────────────────
AXIS_Z = 30.0                 # wire axis above the deck top      (base.py AXIS_Z; lowered 35->30)
PLATE_TH = 6.0                # base plate thickness               (base.py BASE_TH)
# MuJoCo world: the deck sits on the floor (z=0), so the wire axis is AXIS_Z above
# the deck top, which is PLATE_TH above the floor.
WIRE_AXIS_WORLD_MM = AXIS_Z + PLATE_TH            # 36 mm
WIRE_AXIS_WORLD_Z = WIRE_AXIS_WORLD_MM / 1000.0   # 0.036 m
BASE_WORLD_Z = PLATE_TH / 1000.0                  # 0.006 m (deck bottom on the floor)
FEEDER_X_WORLD = 0.151        # feeder centre on the wire axis (m) (base.py FEEDER_X)

# ── feed tube ───────────────────────────────────────────────────────────────
TUBE_D = 6.35                 # 1/4" feed tube OD (bare tube; sleeved to Ø8 at the bearings)  (rothead.py TUBE_D)

# ── bend cell (cad/rothead.py, cad/bend_disc.py) ────────────────────────────
MANDREL_D = 4.0               # fixed mandrel Ø                    (rothead.py MANDREL_D)
PIN_D = 3.0                   # bending-pin Ø                      (bend_disc.py PIN_D)
PIN_SWEEP_R = 10.0            # pin-centre radius from bend axis   (bend_disc.py PIN_OFFSET)
DIE_TRAVEL_DEG = 270.0        # usable cycloid output rotation (Axis 3)

# ── bend-drive reduction: the 20:1 cycloid (cad/cycloid.py, housing.py) ──────
# These describe the gearbox that multiplies the bend-motor torque AND set the
# printed-disc load capacity. drive_model.py turns them into "what stock can this
# machine bend?"; consistency.py keeps them matched to cad/cycloid.py.
DRIVE_RATIO       = 20.0      # output:input — carrier fixed, ring is output (= N ring pins)
N_RING_PINS       = 20        # housing.py N_PINS
N_LOBES           = 19        # cycloid.py LOBES (= N_RING_PINS - 1)
ECCENTRICITY      = 0.625     # cycloid.py E (vendor-measured; reduced from the full R/N=0.925)
DISC_FACE_W       = 4.6       # cycloid.py DISC_T — face width PER disc (the capacity lever)
N_DISCS           = 2         # cycloid.py N_DISCS (balanced stack)
CARRIER_R         = 10.125    # cycloid.py CARRIER_R — carrier-roller bolt circle
CARRIER_ROLLER_D  = 5.0       # cycloid.py ROLLER_D — Ø of the alu carrier rollers (the load path)
N_CARRIER         = 6         # cycloid.py N_CARRIER
DRIVE_EFF         = 0.80      # cycloidal mechanical efficiency (estimate)

# ── bend motor (Axis 3) + bending mechanics ─────────────────────────────────
MOTOR_HOLDING_TORQUE = 0.22   # N·m — current pancake NEMA17 (the weak link; full NEMA17 ≈0.45, NEMA23 ≈1.5)
MOTOR_USABLE_FRAC    = 0.70   # usable fraction of holding torque before skipping (slow bend)
BEND_PROCESS_FACTOR  = 1.6    # required torque / ideal plastic moment (friction + work-harden + over-bend)

# ── rotation drive: head pinion meshing the fixed gear (cad/base.py, rothead.py) ──
FIXED_GEAR_TEETH = 40         # base.py FG_TEETH (the fixed gear on the deck)
PINION_TEETH = 12             # rothead.py PIN_TEETH (the head's rotation pinion)
GEAR_MODULE = 1.5             # base.py FG_MODULE
MESH_R = (FIXED_GEAR_TEETH + PINION_TEETH) * GEAR_MODULE / 2   # 39 mm, pinion-centre radius
PINION_MOUNT_X = 2.0          # pinion-centre X in the head frame (rothead.py PINION_MOUNT_X)
# pinion spin RELATIVE to the head as the head rolls (planet on a fixed sun):
# the head rolls by θ -> pinion spins θ·(N_fixed/N_pinion) about its own axis.
PINION_RATIO = FIXED_GEAR_TEETH / PINION_TEETH                 # 3.333

# ── stock ───────────────────────────────────────────────────────────────────
WIRE_D = 1.63                 # default stock Ø (14 ga; 16 ga ≈ 1.29)


def bend_radius(wire_d=WIRE_D):
    """The single corner radius the machine makes: the wire wraps the mandrel."""
    return MANDREL_D / 2 + wire_d / 2


BEND_RADIUS = bend_radius()   # ≈ 2.815 mm at 14 ga — the one radius the machine makes

# ── manufacturability tuning (calibratable; see PLAN.md) ────────────────────
MAX_WRAP_DEG = 180.0          # wrap past which the pin swings back toward the tube
MIN_GRAB = 6.0                # min straight (mm) the pin needs between bends
CLEAR = 1.0                   # clearance margin (mm) the pin keeps off other bodies
HEAD_BACK_REACH = 8.0         # head keep-out depth behind the bend point (mm)
HEAD_RADIUS = 22.0            # head keep-out radius around the wire axis (mm)
