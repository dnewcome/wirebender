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

## Head — simpler 2-part redesign (2026-06-14)

The cantilevered rotating head is being **simplified**. The current `cad/rothead.py`
grew into a multi-feature bracket (pinch-clamp hub + rotation arm + bend spine,
then a bolted 2-piece split). We're collapsing it to a cleaner set of parts.
Dan is sketching the geometry by hand; this section is the design intent to build to.

**Target: 2 parts.**

1. **Main part** — one printed bracket that carries everything, with three features:
   - **Slotted NEMA17 mount** (rotation motor) on one side. Radial slots so the
     motor/pinion slides to set the pinion ↔ fixed-gear mesh (backlash). Carries
     over from today's `rot_mount()` (slots already done, ±`ROT_SLOT`).
   - **Slotted bending-head boss** on the *other* side. A boss/socket that the
     cycloid-base part (below) mates into, with slots so the bend head's height
     above the wire is adjustable (±`BEND_SLOT`). Replaces today's two slotted
     tabs + yoke with a single boss.
   - **Central circular boss with a set screw** — mounts the head onto the feed
     tube. Replaces the pinch-slit clamp hub: a plain round bore (tube Ø) + a
     radial set screw (heat-set insert + grub, like the pinion) clamps the tube.

2. **Cycloid-base part** — a **boss built onto the cycloid plate**, roughly as
   already prototyped (`cyclo_base()` import, oriented bearing-boss-down toward
   the wire; the version with 2 heat-set threaded inserts). This boss slides into
   the main part's bending-head boss and bolts through the height slots into the
   inserts.

**Carry over from the current code:** `_slot()` helper, the slotted `rot_mount()`,
the `cyclo_base()` STL import (boss-down), the slot-travel constants.
**Drop:** the pinch-clamp `hub()`, the rotation `bend_spine`/yoke/tabs, and the
bolted 2-piece `rot_piece`/`bend_piece` split (`build/rothead_rot.stl`,
`rothead_bend.stl`). The main part becomes a single body; the cycloid-base boss
is the only other printed piece.

**Open (Dan to pin down in the sketch):** the bending-head boss/socket profile
(how the cycloid boss registers + slides), set-screw boss diameter/wall, and the
exact insert spacing on the cycloid boss (today's slots assume `CYC_INS_Y=±14`,
`CYC_INS_Z=12.5` on the base +X face — re-match once the sketch lands).

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

- [x] **Manufacturability rule-checker** (`sim/interference.py`, rebuilt 2026-06-15).
  Flag-only checker that walks a program (same kinematics as `bend_model`) and
  reports per-bend violations of the single-pin bend-cell limits:
  `travel` (bend cmd > `DIE_TRAVEL_DEG`=270° die range, exact), `radius` + `min_straight`
  (inter-bend straight < pin grab / setback, exact — fixed mandrel radius
  `BEND_RADIUS = mandrel_r + wire_r`), `pin_part` (pin sweep strikes the formed
  part, approx), `pin_tube` (wrap > `MAX_WRAP_DEG` swings the pin toward the feed
  tube, calibratable), `part_head` (formed part swings back into the head, approx).
  Wired into `slicer.py --check`; CLI `interference.py <example|part.gcode>` /
  `--caps`; `make rules`. Verified: square/staple/chair pass, tight `coil` flags
  pin_part + min_straight, 300° flags travel.
- [ ] **Calibrate the clearance constants** against the real head once the head
  redesign lands: `MAX_WRAP_DEG` (180° placeholder — the angle past which the pin
  returns toward the tube), `MIN_GRAB`, `PIN_SWEEP_R`, the `HEAD_BACK_REACH`/
  `HEAD_RADIUS` keep-out. The current mesh-level pin↔tube interference of the
  as-built CAD is measured separately by `check_pin.py` (the bend die is mounted
  ~8.5mm off the wire axis — see that tool); fold its result in here once fixed.
- [ ] **Upgrade `part_head` to the real head mesh.** Today it's a parametric
  keep-out box behind the bend point; place the formed part in the machine frame
  and run a signed-distance check against the actual head STL (like `check_pin`).
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
- [ ] **More path imports:** DXF (2D), STEP / 3D CAD edges/wires. Today curves
  are polygonised into discrete bends.

### Slicing methods (model → wire path(s)) — pluggable framework

Refactor the slicer so a 3D model can be turned into wire path(s) by one of
several interchangeable **extraction methods**, all feeding the shared backend
(simplify → `feed/rotate/bend` → fit error → interference → G-code). Each method
returns one or more paths; the backend already verifies any path.

- [x] **Framework refactor.** `slicer.py` has a method registry (`METHODS`),
  `--method`/auto-detect, and multi-piece G-code (cut/reload `M0` between pieces).
  Registered: `points`, `svg`, `example`, `centerline`.
- [x] **Centerline** (tube/curve → 1D spine). Cross-section marching; round-trips
  well on chair (fit 0.32 mm) and the U-shaped staple (102.9 vs 102.6 mm).
  *Experimental* — tight/self-touching tubes may need a `--tol`/step tweak.
- [x] **Cross-section** (solid/surface → contours at intervals → wire loops, like
  an FDM slicer). `--method cross_section --axis --spacing`; each contour loop is a
  wire piece. Verified: sphere → ring cage (fit ~0.3 mm), box → rectangles.
- [x] **Edge-following / wireframe** (feature edges → wire), pepakura-flavored.
  `--method edge_follow --feature-angle`. Builds a graph of sharp edges and, by
  default, decomposes each component into the fewest open trails covering every
  edge once (separate wire pieces, no overlap). `--single` forces one strand via
  Eulerian augmentation (retraces edges → large fit error, flags impracticality).
  Verified: box → 4 pieces covering all 12 edges. *Future:* include boundary edges
  of open meshes; smarter odd-vertex pairing (shortest-distance) to minimize pieces.
- [ ] **Outline / silhouette** (project → outline) for 2D-ish parts.
- [ ] **Hybrids** (e.g. cross-section ribs + connecting spine).

**Cross-cutting constraint:** the bender makes a *single continuous strand*, so
multi-path output means either separate wire pieces (cut/weld) or one routed path
that doubles back. Methods must also respect manufacturability (reachability, min
straight between bends, bend-radius limit) — reuse `interference.py` per path.
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
