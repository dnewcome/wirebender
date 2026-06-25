# CNC Wire Bender

A CNC wire bending machine capable of bending 14 or 16 gauge steel wire into arbitrary 3D shapes using G-code.

![Assembly](build/assembly.png)

## Overview

Wire is fed from a spool using a 3D printer extruder, through a rotating feed tube. The feed tube rotation sets the bend plane (direction), and a bending head fixed to the end of the tube performs the actual bend. Motion is controlled by a GRBL 1.1 board.

## Machine Architecture

### Motion Controller
- **Controller:** GRBL 1.1
- **Input:** Standard G-code

### Wire Feed
- **Component:** Off-the-shelf 3D printer extruder
- **Wire gauge:** 14 or 16 AWG steel wire fed from a roll
- **Feed path:** Extruder → feed tube → bending head

### Feed Tube (Rotation Axis)
- Rotates around the wire axis to set the bend direction
- Driven by a geared stepper motor via toothed belt
- Supported by bearings housed in a U-shaped sheet aluminum bracket
- The U-bracket is extended to also mount the tube rotation motor

### Bending Head
- A cantilevered head clamps the end of the passive feed tube and **rotates about
  the wire axis** (Axis 2), carrying the bend mechanism around with it.
- **Bend drive (Axis 3):** a 20:1 micro-cycloidal drive + pancake
  NEMA17. A pin on the cycloidal output sweeps the wire around a fixed mandrel —
  single-pin bending; bend radius is set by the mandrel.
- **Rotation drive (Axis 2):** a pancake NEMA17 + a 12T pinion meshing a fixed 40T
  gear on the base; the rotation motor hangs at the mesh radius and doubles as a
  counterweight.
- **Tube clamp:** the head grips the feed tube on a short boss with an M3 set screw.

## Building the parts

The machine is modeled in **build123d** (`cad/*.py`), assembled into a **MuJoCo**
sim (`sim/`), and the CAD↔sim machine parameters live in one place
(`sim/machine.py`). Everything is driven by the `Makefile`:

```bash
python3 -m venv py
./py/bin/pip install build123d gggears mujoco trimesh numpy pillow networkx manifold3d

make parts        # base plate / uprights / fixed gear / spacer (printable STLs)
make head         # printable head pieces (rot + bend) + pinion + reference meshes
make cyclo-drive  # the cycloid drive: ring + disc + shaft + base + end-cap (+ fit-check)
make model        # assemble the MuJoCo model (sim/wirebender.xml) from the STLs
make view         # open the interactive sim (head auto-sweeps its limits)
make assembly     # full machine as one build123d STEP (build/assembly.step)
make              # list all targets
```

The **bend drive is fully parametric** — no external CAD file is read at build time. The
**canonical machine targets 14 ga steel on the current motor** (the 20:1 / Ø42 drive), and
the same generator scales to heavier stock by one knob:

```bash
make drive                              # what wire can this machine bend + the driver current it needs
make cyclo-drive PINS=30 MOTOR=nema23   # a complete SCALED drive (Ø60 / 30:1, NEMA23) for 1/4" stock
```

See **`docs/DRIVE_SIZING.md`** for motor/driver sizing, the scaling family, the clearance
planner, and homing. Generated artifacts land in `build/` and `sim/meshes/` (both gitignored
— reproduce from source); `STRUCTURE.md` is the repo layout, `PLAN.md` the roadmap, and
`sim/README.md` the simulation + G-code toolchain.

The cycloid internals were **validated once against a purchased reference part** (not
redistributed), then rebuilt fully parametrically — no external CAD file is needed to build
any part.

## Checklist

*The CAD, the MuJoCo simulation, and the slicer/planner/drive-sizing toolchain are well
advanced. The remaining open items are mostly physical fabrication, wiring, and on-machine
calibration.*

### Mechanical Design
- [x] Parametric bending head — rotation + bend mounts and tube clamp (`cad/rothead.py`)
- [x] Fully parametric cycloid drive — ring, disc, eccentric shaft, base, end cap; scales by
  ratio/motor (`make cyclo-drive PINS=.. MOTOR=..`)
- [x] Bend pin / mandrel geometry defined + clearance-checked in sim (`cad/bend_disc.py`, `make check-pin`)
- [x] Feeder motor mount + bracket (`cad/feeder_motor_mount.py`, `cad/feeder_bracket.py`)
- [x] Feed tube → 608-bearing sleeve adapter + bearing selection (`cad/sleeve.py`)
- [x] Rotation drive sized — 12T pinion on a fixed 40T base gear (replaces the earlier belt/pulley idea)
- [x] Homing-switch brackets, Axes 2 & 3 (`cad/home_switch.py` + `cad/arbor_mount.py` home tab)
- [ ] U-shaped sheet-aluminum feed-tube bearing bracket (fabricate)
- [ ] Wire guide / straightener between spool and feeder

### Electronics & Control
- [x] Axis assignments confirmed — Axis 1 wire feed, Axis 2 head rotation, Axis 3 bend
- [x] Stepper-driver sizing + phase-current requirements modeled per machine (`make drive`, `docs/DRIVE_SIZING.md`)
- [x] Axis limits + homing strategy defined (conventions in `sim/machine.py`, cycle in `sim/home.py`)
- [ ] Wire up GRBL 1.1 board and stepper drivers
- [ ] Set driver current to rated + tune microstepping (bench)
- [ ] Calibrate steps/mm for each axis (bench)

### Software / G-code
- [x] G-code workflow decided — model → slicer → clearance-aware planner → verified program
- [x] Path generator / post-processor (`sim/slicer.py`, `sim/plan.py`)
- [x] Clearance planner + whole-path collision verify (`sim/clearance.py`, `sim/plan.py`)
- [x] Feed / rotate / bend sequences validated in sim (`make anim`, `make run`)
- [x] Homing + initialization routine designed and simulated (`make home`)

### Fabrication (physical)
- [ ] Print the head pieces (`make head`) and fit on the cycloid / rotation motors
- [ ] Bend + drill the U-bracket sheet-aluminum pieces
- [ ] Assemble feed tube with bearings into the bracket
- [ ] Full mechanical assembly + fit check

### Validation (bench)
- [ ] Dry-run all axes, no wire — travel + direction (verified in sim; confirm on hardware)
- [ ] First wire feed test with soft wire (e.g. aluminum) before steel
- [ ] Calibrate bend angle vs. motor steps + `BEND_PROCESS_FACTOR`
- [ ] Measure + compensate springback in 14/16 ga steel
- [ ] Produce a simple test shape (square, circle)
