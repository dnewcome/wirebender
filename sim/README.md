# MuJoCo simulation + G-code toolchain

A rigid-body MuJoCo model of the wire bender, assembled directly from the
**printable** build123d STLs, plus the forward model / slicer / rule-checker that
turn shapes into G-code and back. All machine parameters come from one source of
truth, `machine.py` (kept in sync with the CAD by `consistency.py`).

## The model (`make_mjcf.py` → `wirebender.xml`)

Assembled from the actual part STLs — what you see is what you print:

| Mesh         | Source                | Role                                   |
|--------------|-----------------------|----------------------------------------|
| `base`       | `build/base.stl`      | printable: deck + uprights + fixed gear + spacer |
| `head`       | `build/rothead.stl`   | printable: head bracket (rot + bend pieces) |
| `pinion`     | `build/pinion.stl`    | printable: rotation pinion (animated, meshes the fixed gear) |
| `arbor_mount`| `build/arbor_mount.stl` | printable: rotating cyclo housing + posts (bend output) |
| `end_cap`    | `build/end_cap.stl`   | printable: turned-down cyclo end cap (fixed, caps the arbor) |
| `bend_plate` | `build/bend_plate.stl`| printable: cyclo base + side boss (fixed; bend motor on its back) |
| `head_refs`  | `build/head_refs.stl` | **reference** (purchased): NEMA motors + cycloid drive |
| `feeder`     | `build/feeder_body.stl` | reference primitive (1KGSSJ-B feeder) |

Joints (collisions are off — kinematic visualization, driven by actuators):

| Joint         | Type  | Axis                | Function                                  |
|---------------|-------|---------------------|-------------------------------------------|
| `tube_rot`    | hinge | wire axis (X)       | Axis 2 — bend **direction** (head roll)   |
| `bend`        | hinge | cycloid axis (Z)    | Axis 3 — bend **degree** (270° travel)    |
| `pinion_spin` | hinge | pinion axis         | display only — rolls on the fixed gear, `= tube_rot × machine.PINION_RATIO` |

Wire **feed** (Axis 1) is kinematic — the formed wire grows out of the head in
`animate_bend.py`, not a sliding joint. A `pin` site + `pin_pos` sensor track the
bend-pin tip.

```bash
cd sim && MUJOCO_GL=osmesa ../py/bin/python make_mjcf.py   # or: make model
DISPLAY=:0 ../py/bin/python view.py                        # or: make view
```

`view.py` auto-sweeps each axis through its limit (head roll + 270° bend),
interference-free; `view.py --manual` gives draggable sliders.

## Predicting the wire shape (`bend_model.py`)

A deterministic forward model: feed it a bending **program** (`feed` / `rotate` /
`bend` ops) and it walks a moving frame along the wire to compute the resulting
**3D shape**, with a springback factor. Bends are arcs of radius
`machine.BEND_RADIUS` (mandrel radius + wire radius ≈ 2.8 mm at 14 ga; pass
`bend_radius=0` for sharp corners), so predicted lengths include the wire consumed
in each arc.

```bash
../py/bin/python bend_model.py                         # simulate the examples
MUJOCO_GL=osmesa ../py/bin/python bend_model.py --png   # render to preview/shapes.png
```

Example programs live in `EXAMPLES` (square, staple, chair, coil).

## Animating a program on the machine (`animate_bend.py`)

Runs a program **on the machine**: the head rolls (Axis 2), the bend die sweeps
(Axis 3), the pinion meshes the fixed gear, and the formed wire grows out of the
head. Before running it **enforces the machine limits** (`interference.py`): on an
error-severity violation it animates up to the faulting bend, then **halts with a
magenta fault marker** and a non-zero exit (`--force` overrides).

```bash
MUJOCO_GL=osmesa ../py/bin/python animate_bend.py staple    # -> preview/bending_staple.gif
../py/bin/python animate_bend.py chair --view               # live playback (needs a display)
../py/bin/python animate_bend.py part.gcode                 # run a sliced program
```

## Slicer: model → G-code (`slicer.py`)

The inverse of `bend_model.py`, a pluggable framework: an **extraction method**
turns a source into one or more wire paths, then a shared backend recovers the
`feed`/`rotate`/`bend` program and emits G-code for the three GRBL axes (X = feed
mm, Y = tube rotation deg, Z = bend deg).

```bash
../py/bin/python slicer.py --list-methods
../py/bin/python slicer.py chair --check                 # built-in example + rule check
../py/bin/python slicer.py drawing.svg --tol 0.5 -o part.gcode
../py/bin/python slicer.py wire.stl --method centerline  # tube mesh -> spine
```

**Extraction methods** (`--method`, else auto-detected by extension):

| method | source | notes |
|---|---|---|
| `points` | `.json` / `.csv` | list of `[x,y(,z)]` |
| `svg` | `.svg` | first `<path>` (M/L/H/V/C/Q/Z), 2D |
| `example` | a name | a built-in `bend_model` example |
| `centerline` | mesh | tube mesh → extracted spine *(experimental)* |
| `cross_section` | mesh | slice a solid into contour loops (`--axis`, `--spacing`) |
| `edge_follow` | mesh | feature edges → wire (`--feature-angle`, `--single`) |

Feeds are shortened by `r·tan(α/2)` (setback) at each bend and bends
over-commanded by `1/(1-springback)`; **fit error** (max deviation from the input)
is reported. `--check` runs the rule-checker on each sliced path.

## Manufacturability rules (`interference.py`)

A **flag-only** checker for the single-pin bend cell. It walks a program (same
kinematics as `bend_model`) and reports per-bend violations of the machine limits:

- `travel` — bend command > `DIE_TRAVEL_DEG` (270° die range)
- `min_straight` — inter-bend straight too short for the setback + pin grab (this
  is where the fixed mandrel radius bites)
- `pin_part` — the pin sweep would strike the already-formed part
- `pin_tube` — wrap past `MAX_WRAP_DEG` swings the pin toward the feed tube
- `part_head` — the formed part swings back into the head body

```bash
../py/bin/python interference.py chair        # check an example (or a .gcode file)
../py/bin/python interference.py --caps        # print the machine limits
```

It's wired into `slicer.py --check` and enforced in `animate_bend.py`. The
clearance constants are calibratable placeholders (see `PLAN.md`).

## Other tools

- **`machine.py`** — single source of truth for the machine parameters (axis
  heights, tube, mandrel/pin/die, wire, gear ratio, `BEND_RADIUS`, the limits).
- **`consistency.py`** (`make check-consistency`) — asserts `machine.py` matches
  the CAD constants so they can't drift.
- **`check_pin.py`** (`make check-pin`) — sweeps the axes and reports the bend
  die's mesh-level clearance to the feed tube (`mj_geomDistance`).
- **`render.py`** — headless montage / still renders.

## Setup

```bash
python3 -m venv py
./py/bin/pip install mujoco trimesh numpy pillow networkx
```

## Notes

- Geometry is mm in CAD; the model scales to metres (`scale="0.001"` on meshes).
- `make_mjcf.py` reads the STLs in `build/` — run `make parts head` first if
  they're missing.
- Collisions are off in the sim (kinematic viz); `interference.py` checks
  manufacturability offline and `check_pin.py` checks mesh clearance.
