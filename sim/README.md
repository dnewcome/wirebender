# MuJoCo simulation

A rigid-body MuJoCo model of the **whole** wire bender. The bending head uses the
real CAD meshes (placed with the same transforms as `assemble.py`, so it tracks
the design); the rest of the machine is **roughed in with primitives** at nominal
dimensions, to be refined as parts are measured/modeled.

![montage](preview/machine.png)

## What's modeled

All three machine axes, driven by position actuators:

| Joint      | Type  | Axis                    | Machine function              |
|------------|-------|-------------------------|-------------------------------|
| `feed`     | slide | wire/feed-tube axis (Y) | Axis 1 — wire **feed**        |
| `tube_rot` | hinge | wire/feed-tube axis (Y) | Axis 2 — bend **direction**   |
| `bend`     | hinge | motor shaft (Z)         | Axis 3 — bend **degree**      |

- **Real meshes:** the bending head — `motor`, `bender-head`, `motor-flange`.
- **Rough primitives** (in the `LAYOUT` block of `make_mjcf.py`, tagged `[ROUGH]`):
  base plate, U-bracket, feed tube, drive pulley, tube-rotation motor, feeder
  body (1KGSSJ-B), NEMA17 drive stepper, and the wire stock.

Collisions are disabled (`contype/conaffinity = 0`) — this is a kinematic
visualization driven by actuators, not a contact/dynamics sim.

## Predicting the wire shape (`bend_model.py`)

A deterministic forward model: feed it a bending **program** (`feed` / `rotate` /
`bend` ops — the three axes) and it walks a moving frame along the wire to compute
the resulting **3D shape**, with a springback factor. This predicts the part the
machine makes — i.e. your G-code previewer.

```bash
../py/bin/python bend_model.py                        # simulate the examples
MUJOCO_GL=osmesa ../py/bin/python bend_model.py --png  # render to preview/shapes.png
../py/bin/python bend_model.py square --springback 0.07
```

Example programs live in `EXAMPLES` (square, staple, chair, coil). The square is a
sanity check — it closes exactly back at the origin. `springback` defaults to 0 and
is a placeholder to calibrate against real 14/16 ga bends.

## Animating the bend on the machine (`animate_bend.py`)

Runs a program **on the machine**: the feed-tube rotation and bend flange move
while the formed wire grows out of the bending head — in the correct machine frame
(head fixed; the formed wire trails out and swings about the bending point as each
bend forms, like wire pushed past a fixed mandrel). The final shape matches
`bend_model.py` exactly (verified on the examples).

```bash
MUJOCO_GL=osmesa ../py/bin/python animate_bend.py staple   # -> preview/bending.gif
../py/bin/python animate_bend.py chair --view              # live playback (display)
```

## Setup

Uses the same virtualenv as the CAD tools, plus `mujoco`:

```bash
python3 -m venv py
./py/bin/pip install mujoco trimesh numpy pyyaml pillow
```

## Use

```bash
cd sim

# (Re)generate wirebender.xml + binary meshes from manifest.yaml
../py/bin/python make_mjcf.py

# Interactive viewer — drag the feed / tube_rot / bend sliders in the Control panel
../py/bin/python view.py
../py/bin/python view.py --demo          # scripted "make a shape" sequence

# Headless renders (no display needed)
MUJOCO_GL=osmesa ../py/bin/python render.py          # 4-pose montage PNG
MUJOCO_GL=osmesa ../py/bin/python render.py --gif     # animated demo GIF
```

## Files

| File                | Description                                              |
|---------------------|----------------------------------------------------------|
| `make_mjcf.py`      | Generates `wirebender.xml` + `meshes/` from the manifest |
| `wirebender.xml`    | The MJCF model (generated — don't hand-edit)             |
| `view.py`           | Interactive viewer / scripted demo (needs a display)     |
| `render.py`         | Headless montage / GIF renderer                          |
| `bend_model.py`     | Forward model: bending program → predicted 3D wire shape |
| `animate_bend.py`   | Animate a program on the machine (wire forms out of the head) |
| `meshes/`           | Binary STL meshes (converted from `build/*.stl`)         |
| `preview/`          | Rendered montage / GIF output                            |

## Notes

- Geometry is in mm in CAD; the model scales to meters (`scale="0.001"` on meshes,
  positions converted) so MuJoCo's defaults behave.
- `make_mjcf.py` reads the intermediate STLs in `build/`. If they're missing, run
  `python3 assemble.py manifest.yaml` from the repo root first.
- The bending flange is nearly 4-fold symmetric, so the `bend` axis is easier to
  see in the live viewer than in a single still.
- `[ROUGH]` dimensions in `make_mjcf.py` are placeholders — refine them (or swap
  primitives for real meshes) as the feeder/bracket/tube parts get measured.
