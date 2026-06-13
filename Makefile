# Wire bender — build the CAD parts, assemble the MuJoCo model, and view it.
#   make sim     build everything that changed and open the simulation
#   make model   (re)generate sim/wirebender.xml only
#   make vendor  re-bake the cycloidal envelope from the purchased STEP (slow)
#   make clean   remove generated artifacts
PY := py/bin/python

.PHONY: sim view model vendor clean help
.DEFAULT_GOAL := help

# ── CAD parts (build123d -> build/*.stl) ────────────────────────────
build/base.stl: cad/base.py cad/gears.py
	$(PY) cad/base.py
build/rothead_assembly.stl: cad/rothead.py cad/parts.py cad/gears.py
	$(PY) cad/rothead.py
build/bend_endcap.stl: cad/bend_endcap.py
	$(PY) cad/bend_endcap.py
build/cyclo_body.stl: cad/gen_vendor.py
	$(PY) cad/gen_vendor.py            # slow; needs the purchased Sweep Dynamics STEP
build/feeder_body.stl: cad/gen_feeder.py extruder.glb
	$(PY) cad/gen_feeder.py

PARTS := build/base.stl build/rothead_assembly.stl build/bend_endcap.stl \
         build/cyclo_body.stl build/feeder_body.stl

# ── MuJoCo model (parts -> wirebender.xml + meshes) ─────────────────
sim/wirebender.xml: sim/make_mjcf.py $(PARTS)
	cd sim && ../$(PY) make_mjcf.py

model: sim/wirebender.xml          ## regenerate the MuJoCo model

sim: sim/wirebender.xml view       ## build what changed, then open the viewer

view:                              ## open the MuJoCo viewer (drag rot/bend sliders)
	cd sim && DISPLAY=:0 ../$(PY) view.py

anim: sim/wirebender.xml           ## render a bend animation (NAME=staple|square|chair|coil)
	cd sim && MUJOCO_GL=osmesa ../$(PY) animate_bend.py $(or $(NAME),staple)

vendor: build/cyclo_body.stl       ## re-bake the cycloidal envelope (slow)

clean:                             ## delete generated STLs/STEPs/meshes/model
	rm -f build/*.stl build/*.step sim/meshes/*.stl sim/wirebender.xml

help:
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
