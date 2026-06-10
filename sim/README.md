# MuJoCo simulation

A rigid-body MuJoCo model of the wire bender's bending head, generated from the
same `manifest.yaml` the CAD assembly uses, so the sim tracks the design.

![montage](preview/montage.png)

## What's modeled

Two actuated axes (the mechanism's moving parts), reusing the existing meshes:

| Joint      | Type  | Axis                         | Machine function                |
|------------|-------|------------------------------|---------------------------------|
| `tube_rot` | hinge | wire/feed-tube axis (Y)      | Axis 2 — bend **direction**     |
| `bend`     | hinge | motor shaft (Z)              | Axis 3 — bend **degree**        |

The motor + bender-head mount rotate together with the feed tube; the bending
flange (`motor-flange`) rotates on the motor shaft inside that.

**Axis 1 (wire feed)** isn't modeled yet — it only translates wire, and there's
no wire body. The XML has a commented `wire`/`feed` slide-joint stub ready to
drive one when added.

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

# Interactive viewer — drag the tube_rot / bend sliders in the Control panel
../py/bin/python view.py
../py/bin/python view.py --demo          # scripted feed/rotate/bend sequence

# Headless renders (no display needed)
MUJOCO_GL=osmesa ../py/bin/python render.py          # 4-pose montage PNG
MUJOCO_GL=osmesa ../py/bin/python render.py --gif     # animated sweep GIF
```

## Files

| File                | Description                                              |
|---------------------|----------------------------------------------------------|
| `make_mjcf.py`      | Generates `wirebender.xml` + `meshes/` from the manifest |
| `wirebender.xml`    | The MJCF model (generated — don't hand-edit)             |
| `view.py`           | Interactive viewer / scripted demo (needs a display)     |
| `render.py`         | Headless montage / GIF renderer                          |
| `meshes/`           | Binary STL meshes (converted from `build/*.stl`)         |
| `preview/`          | Rendered montage / GIF output                            |

## Notes

- Geometry is in mm in CAD; the model scales to meters (`scale="0.001"` on meshes,
  positions converted) so MuJoCo's defaults behave.
- `make_mjcf.py` reads the intermediate STLs in `build/`. If they're missing, run
  `python3 assemble.py manifest.yaml` from the repo root first.
- The bending flange is nearly 4-fold symmetric, so the `bend` axis is easier to
  see in the live viewer than in a single still.
```
