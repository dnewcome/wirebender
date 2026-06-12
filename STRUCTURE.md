# Repo layout

## Source — edit these

| Path | What |
|------|------|
| `cad/gears.py` | involute spur-gear generator (build123d) |
| `cad/base.py` | **the base** — fixed gear + 2 bearings + feeder mount |
| `cad/rothead.py` | **the rotating head** (work in progress) |
| `sim/make_mjcf_v2.py`, `sim/view_v2.py` | **current** MuJoCo sim (build123d parts) |
| `sim/bend_model.py`, `slicer.py`, `render.py` | forward model + "slicer" toolchain |
| `PLAN.md`, `README.md` | docs |

## Generated — gitignored, never edit, reproduce from source

| Path | Made by |
|------|---------|
| `build/*.stl`, `build/*.step` | `py/bin/python cad/<part>.py` |
| `sim/meshes/*.stl`, `sim/wirebender*.xml` | `cd sim && ../py/bin/python make_mjcf_v2.py` |
| `sim/preview/*.png` | render scripts |

If a file is under `build/`, `sim/meshes/`, or is `sim/wirebender*.xml`, it is
**generated** — delete it freely; it comes back from the source above.

## Regenerate everything
```sh
py/bin/python cad/base.py            # -> build/base.{stl,step}
py/bin/python cad/rothead.py         # -> build/rothead.{stl,step}
cd sim && ../py/bin/python make_mjcf_v2.py
DISPLAY=:0 ../py/bin/python view_v2.py
```

## Legacy (OpenSCAD, pre-build123d) — superseded, pending removal
`base.scad`, `base-bracket.scad`, `head.scad`, `bend-gears.scad`,
`bender-head.scad`, `motor.scad`, `motor-flange.scad`, `massing.scad`,
`rothead.scad`, `spindle.scad` — replaced by the build123d parts above.
The old MuJoCo sim (`sim/make_mjcf.py`, `view.py`, `animate_bend.py`,
`interference.py`) drove those `.scad` files. `stepper-adapter.scad` /
`feeder-gear.scad` (feeder motor swap) are kept for review.
