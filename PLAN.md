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

## Mechanical stiffening / robustness (2026-06-18)

The feeder grip and bend-head torque are now strong enough that the surrounding
brackets are the weak links. Stiffening pass:

- [x] **Front upright foot + gussets** (`cad/base.py`). The thin 8mm blade is now a
  braced L-bracket: a 6mm rearward foot (x≈12→36 — can't extend forward, the deck
  edge is at x=10 and the fixed gear hangs in front), two triangular gussets
  (`GUSSET_*`) over the bolt lines rising 22mm up the blade back, and two extra
  deck bolts at the foot rear (`FRONT_FOOT_BOLT_X`) that widen the X bolt base.
  `base_plate()` carries the matching counterbored mounts. One watertight body.
- [x] **Arbor plate** (`cad/bend_disc.py`). Was Ø60 × 2mm. Now **3mm** thick (more
  material around the countersinks for the bend reaction), still bolted to the arbor
  posts with 4× **M3 countersunk** screws into the existing M3 inserts (arbor_mount
  unchanged). (Still optional: a short boss around the mandrel/bend-pin holes if the
  pins work loose.)
- [ ] **Bend-disc tooling variants** (`cad/bend_disc.py`, design notes in the file). The
  disc separates two pins with different jobs, and we'll ship a family of plates:
  - **Outer bend pin** (`PIN_OFFSET`) sets the minimum bend **length** — sweep its radius
    across versions: smaller wire runs it closer in, heavier wire further out. Planned:
    swap the fixed pin for a larger **roller** (shoulder screw / small bearing) so the wire
    rolls rather than drags as it wraps (less marring, lower friction, more force headroom).
  - **Centre mandrel** (`MANDREL_D`) sets the minimum bend **radius** (r ≈ MANDREL_D/2 +
    wire/2) and is **floored by wire size** — it must be stout enough not to deflect under
    the bending force, so it's the strength-constrained pin; size it per wire/bend-radius.
- [ ] **rothead bending neck** (`cad/rothead.py`, `flat_head()`). The bending plate
  is lifted 12mm (`FH_BEND_LIFT`) on a 5mm-thick, 13mm-wide neck (`bneck`) — a
  slender cantilever. Widen the neck in Y and thicken the central spine/web tying
  it to the boss (in-plane stiffening, keeps the flat-print orientation).
- [ ] **Bender right-angle mount** (`cad/bend_plate_90.py`). The riser is a 6mm tab
  (`RISER_T`) cantilevering 22mm (`RISER_H`) up off the flange. Add a pair of
  triangular side gussets from the flange arm up the riser back (trimesh prisms),
  and/or bump `RISER_T`.

## Bend drive (cycloid) + machine sizing

**Canonical machine: 14 ga steel on the current motor (the 20:1 / Ø42 drive).** 14 ga needs
~0.5 N·m so margin is large; get this fully working first. Everything below is the documented
**scaling path** for heavier stock — full details + numbers in **`docs/DRIVE_SIZING.md`**.

- [x] **Fully-parametric cycloid drive** (`cad/cycloid.py` + `housing.py` + `cyclo_base.py` +
  `end_cap.py`). Disc + eccentric shaft validated vs the vendor STEP/BOM (lobe profile RMS
  0.04 mm, E=0.625; press-fit D-bore shaft; support journals Ø7). The whole drive reads no
  vendor file. **Scales by one knob:** `make cyclo-drive PINS=30 MOTOR=nema23` keeps the lobe
  size + eccentric core (shaft/bearings reused), grows the ring/disc, and switches the NEMA
  mount. `make check-cyclo PINS=…` confirms the disc meshes. Default (20 pins / nema17) is
  byte-identical to the validated drive.
- [x] **Design explorer + driver sizing** (`sim/drive_model.py`, `make drive`). Two limits —
  torque (d³·σy·ratio·motor) and printed-disc capacity — plus **phase-current → driver class**
  (stepper torque is current-set). Diagnoses stalls (current-deficiency vs motor-insufficiency)
  and sizes the body Ø for a target wire (1/4″ mild ≈ Ø62 / 30:1 at 1.5× on the NEMA23).
  Reproduces the bench stalls. Machine-bound, not disc-bound, in every case.
- [x] **Clearance-aware motion planner** (`sim/clearance.py` + `sim/plan.py`). Mesh
  ground-truth clearance; per-bend rotation backtrack + feed-to-clear; `plan.verify()`
  whole-path safety gate; `--plan` clearance-coloured animation. (See the Interference section.)
- [ ] **Calibrate the model against real bends.** `BEND_PROCESS_FACTOR` (1.6), `MATERIALS`
  allowables, `PEAK_CARRIER_SHARE`, and wire σy by temper — fit to measured stall torque /
  disc wear once the canonical machine runs.
- [ ] **Stout mandrel / roller-pin `bend_disc`** sized for the 27–55 N·m reaction of heavy
  stock (steel pins) — the strength-constrained part once the drive can deliver the torque.
- [ ] **Homing switches** — Axis 2 (bend plane) + Axis 3 (bend die, the critical one);
  flag-on-output + fixed switch, optical/hall preferred; `machine.py` offsets + GRBL cycle.
- [ ] **Optional disc bearing land** (Ø11 × 0.75 vendor feature) once the press fit is dialed.
- [ ] **Tube bending (future).** `drive_model.py` takes `kind="tube"`, but torque is
  necessary-not-sufficient: needs a mandrel/wiper wrinkle + ovalization model.
- [ ] **Reusable kinematic kernel** for 3rd-party tools — `machine.json` profile + JSON CLI
  (see `docs/DRIVE_SIZING.md`).

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
  `HEAD_RADIUS` keep-out. The mesh-level pin↔tube clearance is measured by
  `check_pin.py` — now **+4mm clear** across the full axis range after orienting
  the bend die body-up (`make_mjcf`); flipped body-down it buried the pin in the
  tube. NOTE: clearing requires the die body to point away from the wire, which
  only fully works once the head redesign ends the feed tube before the bend
  point (today's tube still runs through it). The real die orientation must come
  from the head CAD; the sim quat just reflects the intended (clear) build.
- [ ] **Upgrade `part_head` to the real head mesh.** Today it's a parametric
  keep-out box behind the bend point; place the formed part in the machine frame
  and run a signed-distance check against the actual head STL (like `check_pin`).
- [x] **Single source of truth + sim enforcement** (2026-06-15). `sim/machine.py`
  holds all machine params (axis heights, tube, mandrel/pin/die, wire, derived
  `BEND_RADIUS`, the calibratable limits); `bend_model`, `interference`,
  `check_pin`, `make_mjcf`, `animate_bend` all read from it, so the CAD, sim,
  forward model, slicer, and rules describe one machine. `sim/consistency.py`
  (`make check-consistency`) asserts machine.py matches the CAD constants.
  `animate_bend` now runs the rule-checker first and, on an error-severity
  violation, animates up to the faulting bend then HALTS with a magenta fault
  marker + nonzero exit (`--force` overrides). Fixed drift this exposed:
  `BEND_RADIUS` 4→2.8 (mandrel-derived) and the animator's stale `ZAXIS_MM` 21→41.
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

- [x] **Stepper mount + drive gear** (done — superseded the old `stepper-adapter.scad`
  / `feeder-gear.scad`, both deleted). The NEMA17 mount is now integrated into
  `cad/feeder_bracket.py` (motor pocket / slotted web / boss pocket), and the drive
  pinion is `cad/feeder_gear_press.py` (11T m0.8, press-fit D-bore, 12mm engagement).
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

## Future direction — clamp-on "crawler" bender (north star, not canonical)

The only ground-fixed part of the machine is the **feeder**; the rotation axis, cycloid, and
mandrel/pin all already ride the head. So invert the feed axis — instead of *pushing wire
through a fixed head*, **crawl the self-contained head along fixed stock** (clamp onto a
tube/rod and traverse it, bending in place). A portable / in-situ former: no spool, no bench.

- **The software is already relative.** `bend_model` treats "feed N mm" as the relative
  advance between wire and head, so the forward model, slicer, clearance planner, and homing
  carry over unchanged — feed just remaps from extruder-steps to **crawl distance** along the
  tube. The existing toolchain already describes a crawler.
- **Three mechanical changes:** (1) a **crawl drive** that grips + translates precisely along
  the tube (friction wheels / pinion-on-clamped-rack / leadscrew) — slip = lost feed, so it
  wants a **closed-loop encoder** (dovetails with the closed-loop-stepper idea for the stall
  problem); (2) an **openable / split rotary bearing** so Axis 2 can encircle a clamped tube
  (today it rotates about the wire with the wire passing through); (3) **anchoring** so the
  bend reaction (≈27 N·m at 1/4″) reacts through the grip without the crawler spinning/sliding.
- **Hard limit — tube bending in place.** A free wire forms only at its trailing end, so
  crawling **wire/rod** (or large-radius, thick-wall tube) progressively **from a free end** is
  the same problem already solved (the formed part trails the crawler like it trails the fixed
  head today). But tight-radius thin-wall tube needs an internal draw-mandrel/wiper, which
  **can't be threaded into a continuous tube mid-span while crawling** — so the crawler is
  naturally a wire/rod or conduit-style large-radius machine, not a tight thin-wall tube bender.
- Orthogonal to the canonical 14 ga machine — pursue only after that's working.

## Notes / decisions

- MIG feeder (1KGSSJ-B) chosen for the wire feed; brushed motor + gear reduction
  already feeds 14 ga fine, so a NEMA17 through the same gearbox is plenty.
- Keep the existing gear reduction (torque + resolution); don't direct-drive.
- Sim is kinematic (collisions off) by design; `interference.py` does collision
  checks offline against the formed-wire geometry instead.
- **Manufacturability: the sim assembles the PRINTABLE part STLs** (base, rothead
  rot+bend, pinion, bend die) — what you see is what you print. Purchased parts
  (NEMA motors, cycloid drive) are shown as a clearly-separated REFERENCE mesh
  (`build/head_refs.stl`, from `cad/head_refs.py`, placements imported from
  `rothead.ghosts()`), never fused into a printable STL. CAD print scripts stay
  clean (one printable part each). The rotation pinion is its own animated body
  meshing the fixed gear (spin = head roll × `machine.PINION_RATIO`); motor
  bodies are shown but their internals aren't animated (torque/efficiency is a
  later numerical study, not the visualization).
