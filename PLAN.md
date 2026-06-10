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

- [x] **Bend radius (filleted corners).** Bends now lay an arc of radius
  `bend_model.BEND_RADIUS` (default 4 mm — the wire wrapping the mandrel) instead
  of a sharp corner, in both the forward model and the machine-frame animation
  (verified congruent). `bend_radius=0` recovers sharp corners. *Still to do:*
  calibrate the radius to the real mandrel/shoe geometry.
- [x] **Wire consumed in the bend arc.** Total wire length now includes the arc
  length at each bend (Σ feed + Σ r·α), so `total_length` reflects real usage.
- [ ] **Springback calibration.** `springback` is a placeholder (0). Bend test
  coupons of 14/16 ga, measure commanded vs realized angle, and fit a model
  (likely angle- and material-dependent). Feed that back into `bend_model.py`.
- [ ] **Plastic wire physics (optional, later).** Route B: a segmented wire in
  MuJoCo whose hinges yield and hold past the shoe, so the wire looks real in the
  3D view and responds to contact. Route C: full flex/contact. Only if the
  deterministic model isn't enough.

## Interference / collision

- [ ] **Accurate head-clearance check.** Bend radius is now modeled, but the
  *bend point* `B` (and mandrel centre) is still placed roughly relative to the
  real head mesh — the wire bends around the motor shaft into the head's rear
  cutaway, which our `B = (6.35, 0, 5.5)` doesn't yet match. So `interference.py`
  still only flags wire *re-entering* the head from far off (`BEND_ZONE = 32`).
  To finish: place `B`/mandrel at the real shoe location, then shrink `BEND_ZONE`
  and trust the mesh signed-distance check near the head.
- [ ] **Reachability / envelope check.** Beyond collisions: does a program stay
  within axis travel limits (feed length, tube-rotation range)?
- [ ] **Wire-vs-already-formed-part as the part grows** is handled; revisit once
  bends have radius (sharp corners slightly over/understate clearance).

## G-code workflow

- [x] **Slicer (CAD path → G-code).** `sim/slicer.py` is the inverse of the
  forward model: a wire path (points / SVG / example) → `feed`/`rotate`/`bend`
  program → G-code, with setback + springback compensation and fit-error
  reporting. Inverse kinematics verified exact (0 mm sharp round-trip). Imports:
  `.json`/`.csv` points, `.svg` paths.
- [ ] **More path imports:** STL (extract the tube centerline), DXF (2D), STEP /
  3D CAD edges/wires. Today curves are polygonised into discrete bends.
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
