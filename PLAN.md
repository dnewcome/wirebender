# Wire Bender — Plan & Roadmap

Forward-looking plan for the CNC wire bender, focused on the design + simulation
work. (The build/fabrication checklist lives in `README.md`.) Items are roughly
ordered; check them off as they land.

## Status snapshot (2026-06-10)

- **Sim:** full machine roughed in (`sim/`), all three axes (`feed` / `tube_rot`
  / `bend`) drivable. Bending head uses real CAD meshes; feeder, NEMA17, feed
  tube, pulley, U-bracket, rotation motor, and wire are rough primitives.
- **Forward model:** `sim/bend_model.py` predicts the 3D part from a program.
- **Animation:** `sim/animate_bend.py` runs a program on the machine (wire forms
  out of the head); verified to match the forward model exactly.
- **Interference:** `sim/interference.py` checks self / table / body collisions
  (exact) and head clearance (approximate — see below).

## Wire / bend modeling

- [ ] **Bend radius (filleted corners).** Forward model + animation currently
  make sharp corners. Real bends wrap the mandrel at a finite radius (~the
  wire-to-shaft offset, ≈6 mm). Replace each sharp vertex with an arc of that
  radius; this also fixes the geometry the head-interference check needs.
- [ ] **Wire consumed in the bend arc.** A bend eats a little feed length
  (arc vs chord); account for it so predicted lengths match reality.
- [ ] **Springback calibration.** `springback` is a placeholder (0). Bend test
  coupons of 14/16 ga, measure commanded vs realized angle, and fit a model
  (likely angle- and material-dependent). Feed that back into `bend_model.py`.
- [ ] **Plastic wire physics (optional, later).** Route B: a segmented wire in
  MuJoCo whose hinges yield and hold past the shoe, so the wire looks real in the
  3D view and responds to contact. Route C: full flex/contact. Only if the
  deterministic model isn't enough.

## Interference / collision

- [ ] **Accurate head-clearance check.** Blocked on finalized bend kinematics
  (mandrel position, bend plane, bend radius). The wire bends around the motor
  shaft into the head's rear cutaway; our rough bend model puts it elsewhere, so
  `interference.py` currently only flags wire *re-entering* the head from far off.
  Once bend radius (above) is in, tighten `BEND_ZONE` and trust the mesh check.
- [ ] **Reachability / envelope check.** Beyond collisions: does a program stay
  within axis travel limits (feed length, tube-rotation range)?
- [ ] **Wire-vs-already-formed-part as the part grows** is handled; revisit once
  bends have radius (sharp corners slightly over/understate clearance).

## G-code workflow

- [ ] **G-code front end.** Thin parser mapping GRBL G-code (the three axes) →
  `feed` / `rotate` / `bend` ops that `bend_model.py` / `animate_bend.py` /
  `interference.py` already consume. Then real machine programs can be previewed,
  animated, and collision-checked before cutting wire.
- [ ] **Axis assignment + steps/mm** (mirror the README electronics checklist):
  Axis 1 feed (extruder), Axis 2 tube rotation, Axis 3 bend. Calibrate steps/mm
  per axis; map G-code units → physical motion in the parser.

## Hardware / CAD (replace rough primitives with real geometry)

- [ ] **Stepper mount adapter** (`stepper-adapter.scad`). Parametric NEMA17-on-
  1KGSSJ-B adapter that keeps the existing gear reduction via a self-centering
  register boss. **Needs measurements** off the real feeder:
  - pocket dia + depth (the boss register)
  - axial seat of the existing pinion (mesh depth)
  - housing screw-hole pattern
  - pinion tooth count + module (confirms `feeder-gear.scad`: 12T, m0.5, 5 mm bore)
  - confirm the can motor is located by the round pocket (self-centering assumption)
  - NEMA17 body clearance behind the gearbox
- [ ] **Replace primitives with CAD** in the manifest: feeder body, feed tube,
  U-bracket + bearings, tube-rotation motor + belt/pulley. Then `make_mjcf.py`
  picks them up as real meshes and the sim/interference geometry gets accurate.
- [ ] **Belt/pulley sizing** for the feed-tube rotation axis (README checklist).
- [ ] **Wire guide / straightener** between spool and feeder.

## Validation (once hardware exists)

- [ ] Dry-run all axes (no wire) — verify travel + direction vs sim.
- [ ] First feed test with soft wire, then 14/16 ga steel.
- [ ] Calibrate bend angle vs motor steps; measure springback; feed back to model.
- [ ] Produce a test shape (square / circle) and compare to the predicted part.

## Notes / decisions

- MIG feeder (1KGSSJ-B) chosen for the wire feed; brushed motor + gear reduction
  already feeds 14 ga fine, so a NEMA17 through the same gearbox is plenty.
- Keep the existing gear reduction (torque + resolution); don't direct-drive.
- Sim is kinematic (collisions off) by design; `interference.py` does collision
  checks offline against the formed-wire geometry instead.
