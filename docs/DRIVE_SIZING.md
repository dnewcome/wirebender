# Drive sizing & scaling

How to size the bend drive (motor, driver, cycloid ratio, body diameter) for a target
wire, and how the parametric CAD scales to bigger machines. This is the design-explorer
knowledge behind `sim/drive_model.py`, the parametric `cad/cycloid.py` family, and the
clearance planner. **The model numbers are first-cut — calibrate against real bends.**

## Canonical machine

The **canonical bender targets 14 ga steel (1.63 mm) on the current motor** — the existing
**20:1 / Ø42 cycloid drive** with a NEMA17-class bend motor. 14 ga needs only ~0.5 N·m at
the output, so even a weak motor has large margin; this is the configuration to get fully
working first. Everything below is the **scaling path** for heavier stock — documented so
it isn't lost, not a change to the canonical machine.

Generate the canonical drive (defaults): `make cyclo-drive` (or `make cycloid` / `make check-cyclo`).

## The two limits that decide a job

`sim/drive_model.py` (`make drive`) answers *"what stock can this machine bend?"* from two
independent limits — a job is makeable only if it clears **both**:

1. **Torque (motor side).** Required output torque grows as **d³**:
   `required = BEND_PROCESS_FACTOR · σy · Z_plastic`, with `Z = d³/6` (solid round) or
   `(d_o³−d_i³)/6` (tube). Available = `motor_torque · usable_frac · ratio · efficiency`.
   A stepper has **no overload margin** — exceed it and it stalls/skips.
2. **Capacity (printed-disc side).** The output torque reacts through the 6 carrier
   rollers; the roller→disc-hole **bearing stress** must stay under the print material's
   allowable. Face width (`DISC_FACE_W · N_DISCS`) is the linear lever; bigger body Ø moves
   the rollers out (more lever) for more capacity.

Key finding across all our cases: **the machine is motor-bound, not disc-bound.** The
PA-CF disc is good to ~Ø7.6 mm while the motor caps you far below that — so torque/ratio is
the lever, and the cycloid stands up to the load as designed.

## Stepper torque is set by CURRENT (driver sizing)

Holding torque is quoted **at the motor's rated current** and scales ~linearly with phase
current below that, then saturates. So the **driver** — both its set point and its max
capability — delivers the torque, not the motor's physical size. `drive_model.py` works
back from a target wire to the **phase current** it needs, hence the **driver class**:

| driver | usable phase current |
|---|---|
| A4988 | ~1.0 A |
| DRV8825 / TMC2209 | ~1.7 A |
| TB6600 | ~3.5 A |
| DM542 / TMC5160 | ~4.0 A |
| DM860 / closed-loop | ~6.0 A |

**Stall taxonomy** (diagnose every stall this way):
- **Current-deficiency** — required current ≤ motor's rated current but the driver is set
  below it → *raise the current* (and check the driver can source it). A big motor starved
  of current makes a fraction of its torque; it can be weaker than a small motor run right.
- **Motor-insufficiency** — required current exceeds the motor's rated current → no driver
  setting helps; need a **bigger motor or more ratio**.

> Bench evidence: the 0.150″ stall was current-deficiency (that wire needs ~1.6 A; the
> driver was set below it). 1/4″ on the same motor is motor-insufficiency.

### Worked motor examples (20:1 drive, mild steel σy≈400)

| motor | rated | output @ rated | max bendable | notes |
|---|---|---|---|---|
| pancake NEMA17 | 0.22 N·m | ~2.5 N·m | ~Ø2.8 mm | the original weak link |
| NEMA17 hi-torque | 0.65 N·m @ 2 A | ~7.3 N·m | ~Ø4.1 mm | 0.150″ needs ~1.6 A (1.23× margin) |
| NEMA23 | 2.4 N·m @ 4 A | ~27 N·m | ~Ø6.3 mm | 1/4″ mild at the ragged edge (needs ~4.06 A vs 4 A rated) |

Phase current needed (20:1, mild): 14 ga ≈ 0.1 A · 0.150″ ≈ 1.6 A · 1/4″ ≈ 7.5 A. The 1/4″
figure exceeds any single NEMA23 → it's a ratio/body problem, not a current one.

`make drive CURRENT=1.5` shows what a given driver setting bends;
`../py/bin/python sim/drive_model.py --rated-torque 2.4 --rated-current 4` specs a motor.

## Scaling the cycloid: ratio delivers torque, diameter houses it

**A bigger body does NOT increase delivered torque** (that's motor × ratio). It increases
**capacity** and lets you fit **more ratio** while keeping the lobes full-strength. To bend
heavier wire you need **more ratio**; the body diameter then follows.

Single-stage ratio = ring-pin count (carrier fixed). Keeping the Ø3 pins at constant
spacing (so **lobe size/shape is preserved**), the body grows ~linearly with ratio:

| 1/4″ stock | σy | margin | ratio | ring pins | **body Ø** | disc margin |
|---|---|---|---|---|---|---|
| mild | 400 | 1.5× | 30:1 | 31 | **~Ø62** | 2.6× |
| mild | 400 | 2.0× | 41:1 | 41 | ~Ø81 | 3.5× |
| medium | 600 | 1.5× | 46:1 | 46 | ~Ø90 | 2.7× |
| hard | 800 | 1.5× | 61:1 | 61 | ~Ø118 | 2.7× |

(Today's drive is Ø42 / 20:1.) **σy is the wild card** — diameter scales ~(400/σy)^⅓, so
pin down the actual wire temper before committing.

**The big win — keeping lobe size + eccentricity (E=0.625) fixed means the entire eccentric
core is reused unchanged.** Only the ring and disc grow radially:

| part | 20:1 (Ø42) | 30:1 (Ø60) | |
|---|---|---|---|
| ring pins / disc lobes | 20 / 19 | 30 / 29 | grows |
| pin pitch / ring OD / disc OD | 18.5 / Ø42 / Ø35 | 27.7 / Ø60 / Ø53 | grows |
| **lobe shape, E** | 0.625 | 0.625 | **identical** |
| **eccentric shaft + Ø8 journals** | — | — | **reused** |
| **bearings 8×12, 7×13, 30×37** | — | — | **reused** |
| **disc bore Ø12 / 6× Ø5 carrier rollers** | — | — | **reused** |

### Parametric generation

`cad/housing.py` and `cad/cycloid.py` are parametric on **ring-pin count** (`N_PINS`, env
`WB_RING_PINS`); `cad/cyclo_base.py` on **motor** (`WB_MOTOR` = `nema17`/`nema23`);
`cad/end_cap.py` follows the ring OD. The eccentric core (`cycloid.eccentric_shaft`) and the
Ø37 output-bearing seat (added to `housing.py`) stay fixed so the shaft + bearings carry over.

```bash
make cyclo-drive PINS=30 MOTOR=nema23   # complete scaled drive: ring+disc+shaft+base+end_cap + fit-check
make check-cyclo PINS=30                # confirm the disc meshes the scaled ring
```

Default (`nema17`, 20 pins) is byte-identical to the validated drive. `PINS=30 MOTOR=nema23`
→ Ø60.5 ring, 29-lobe disc, Ø56.4 NEMA23 base, Ø50 cap — all watertight, mesh-checked.

### Caveats (this is NEW geometry — verify on a print)

- The **output-bearing seat** (Ø37 step at the ring base) is a design call for reusing the
  30×37 bearing in a bigger bore — sanity-check on hardware.
- NEMA23 csk just kisses the plate edge (~0.4 mm); the Ø60 ring overhangs the 56.4 plate
  ~2 mm/side (centrally supported on the bearing, so cosmetic). Bump the plate to clean up.
- **Compound drives** trade diameter for axial length: 20:1 × 1.5 → 30:1 keeps each stage
  ~Ø42–50 + a second disc set. Footprint-constrained → compound; simplicity → bigger body.
- **The pin & mandrel carry the full reaction** (27–55 N·m for 1/4″). Steel pins + a stout
  mandrel, or the body torque is wasted — see the `bend_disc.py` design notes (the mandrel
  is the strength-constrained pin).

## Clearance-aware motion planner

`sim/clearance.py` + `sim/plan.py` choose collision-safe moves for G-code:

- **`clearance.py`** — ground-truth bend-cell clearance. Exact body poses from the MuJoCo
  model, distances measured against the **real non-convex STL meshes** via scipy KD-trees
  over surface samples (MuJoCo's own collision convexifies meshes → too coarse; trimesh
  `signed_distance`/rtree segfaults under planner call volume). Convention: the formed wire
  is fixed, the head orbits it. Excludes the fixed die exit (`end_cap`, the `arbor_mount`
  placeholder, the mandrel-wrap region) so it flags only *avoidable* swing/rotation hits.
- **`plan.py`** — per bend, picks the head-rotation direction (short vs **long-way
  backtrack**) by worst-case clearance over the swept path, and searches a **feed-advance**
  when both directions foul the formed part. `plan.verify()` is the whole-path safety gate:
  it replays the *entire* interpolated motion through the clearance model and fails if any
  frame dips below margin (catches greedy-planner traps).
- **Visualize:** `make anim-view NAME=chair ARGS=--plan` (or `make anim … ARGS=--plan` for a
  GIF) animates the planned moves with the wire coloured by live clearance (green/amber/red).

**Caveats:** ~1–2 mm sampling resolution (keep margin); pin-vs-part during the bend stroke
is still the parametric `interference.py` rule (no bend-disc mesh in the sim yet).

## Homing (conventions decided; hardware/CAD pending)

Open-loop steppers lose position on a stall, so homing is needed to re-reference. Two
switches (feed/Axis 1 is relative — no home). The home zeros equal the software zeros the
forward model + planner already use (`machine.py`: `ROT_HOME_DEG`, `BEND_HOME_DEG`):

- **Axis 2 (tube rotation / bend plane) — `tube_rot = 0` = mandrel / bend-disc axis pointing
  UP (+Z)** with the wire along X on the XY deck (a bend at home lies in the horizontal
  plane). Flag on the rotating head → fixed switch on the upright. Rotation sets the bend
  PLANE, so ~1° home repeatability suffices (a ramped microswitch is fine).
- **Axis 3 (bend die) — `bend = 0` = bend PIN 90° orthogonal to the wire axis** (the sweep
  start; the die sweeps to `DIE_TRAVEL_DEG` from here). **Must home on the cycloid OUTPUT** —
  the 20:1 makes the motor turn ~15× over the 270° output, so a motor-shaft index is
  ambiguous. Flag on the rotating output (ring/disc) → fixed sensor on the end_cap/base.
  This sets the bend-zero, so favour an **optical/hall** (non-contact) sensor; the 20:1
  reduction gives sub-degree output accuracy from a modest flag.
- **Hardware: microswitches.** Axis 3 — a radial **home tab on the rotating ring**
  (`arbor_mount.py` `HOME_FLAG`, tune `HOME_FLAG_ANGLE` to the pin-orthogonal home) presses
  a switch on the fixed `end_cap`/`cyclo_base`. Axis 2 — a tab on the head presses a switch
  on the upright (head tab pending the head redesign). Both use the adjustable bracket
  `cad/home_switch.py` (subminiature SS-5GL/D2F switch, slotted M3 mounts to trim the trip
  onto the software zero; GRBL pull-off `HOME_PULLOFF_DEG`).
- Touches: `machine.py` (home zeros, pull-off, travel — done) · CAD (`arbor_mount` tab +
  `home_switch` bracket — done; head tab pending) · GRBL (homing cycle). **Closed-loop
  steppers** additionally *detect* skips and recover — the robust endgame given the stalls.
- **Visualize:** `make home` (GIF) / `make home-view` (live) — `sim/home.py` simulates the
  homing cycle (seek → trip → pull-off → zero) and the switch (bracket + body + lever, placed
  in the MuJoCo model at the ring flag's home) so you can see the position + mounting and
  confirm the homed pose (mandrel +Z, pin ⊥ wire).

## Toward a reusable kinematic kernel (3rd-party tools)

So others can build slicers/G-code tools on this machine, the plan mirrors robotics
(URDF + MoveIt) and 3D-printing (Cura profiles): a **declarative, versioned machine profile
(`machine.json`)** as the public contract, generated from `machine.py`, plus a **stable
kernel API** (`forward`, `capabilities`, `clearance`, `plan`) with a thin **JSON-RPC/CLI
boundary** so non-Python tools can call it. The MJCF (`sim/wirebender.xml`) is the ready-made
URDF-equivalent geometry/collision model. Build the planner/clearance first (done), then
lift the validated code into the kernel shape.

## Calibration TODO (before trusting predictions on hardware)

- `BEND_PROCESS_FACTOR` (1.6) — fit to measured stall torque vs ideal plastic moment.
- `MATERIALS` bearing allowables + `PEAK_CARRIER_SHARE` (0.5) — fit to real disc wear.
- Wire **σy** by temper — the dominant lever on every number above.
- Clearance sampling vs real fit; tube model needs mandrel/wiper wrinkle + ovalization.
