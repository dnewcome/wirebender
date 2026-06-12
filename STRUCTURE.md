# Repo layout

## Source — edit these

| Path | What |
|------|------|
| `cad/gears.py` | involute spur-gear generator (build123d) |
| `cad/base.py` | **the base** — fixed gear + 2 bearings + feeder mount |
| `cad/rothead.py` | **the rotating head** (work in progress) |
| `sim/make_mjcf.py`, `sim/view.py` | MuJoCo sim built from the build123d STLs |
| `sim/bend_model.py`, `slicer.py`, `render.py` | forward model + "slicer" toolchain |
| `feeder-gear.scad`, `stepper-adapter.scad` | MIG-feeder motor swap (still OpenSCAD) |
| `PLAN.md`, `README.md` | docs |

## Generated — gitignored, never edit, reproduce from source

| Path | Made by |
|------|---------|
| `build/*.stl`, `build/*.step` | `py/bin/python cad/<part>.py` |
| `sim/meshes/*.stl`, `sim/wirebender.xml` | `cd sim && ../py/bin/python make_mjcf.py` |
| `sim/preview/*.png` | render scripts |

If a file is under `build/`, `sim/meshes/`, or is `sim/wirebender.xml`, it is
**generated** — delete it freely; it comes back from the source above.

## Regenerate / view
```sh
py/bin/python cad/base.py            # -> build/base.{stl,step}
py/bin/python cad/rothead.py         # -> build/rothead.{stl,step}
cd sim && ../py/bin/python make_mjcf.py        # -> wirebender.xml + meshes
DISPLAY=:0 ../py/bin/python view.py            # drag the `rot` slider
```

## History
The machine was prototyped in OpenSCAD, then ported to build123d for a cleaner
cantilever architecture. The legacy OpenSCAD parts and the old MuJoCo sim that
drove them have been removed; everything above is the current build123d design.
