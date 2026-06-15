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
build/rothead_assembly.stl: cad/rothead.py cad/parts.py cad/gears.py cad/pinion.py cad/base.py
	$(PY) cad/rothead.py
build/bend_endcap.stl: cad/bend_endcap.py
	$(PY) cad/bend_endcap.py
build/cyclo_body.stl: cad/gen_vendor.py
	$(PY) cad/gen_vendor.py            # slow; needs the purchased Sweep Dynamics STEP
build/feeder_body.stl: cad/gen_feeder.py extruder.glb
	$(PY) cad/gen_feeder.py
build/base_proto.stl: cad/base_proto.py cad/base.py
	$(PY) cad/base_proto.py
build/sleeve.stl: cad/sleeve.py cad/base.py
	$(PY) cad/sleeve.py
build/pinion.stl: cad/pinion.py cad/base.py cad/gears.py
	$(PY) cad/pinion.py

PARTS := build/base.stl build/rothead_assembly.stl build/bend_endcap.stl \
         build/feeder_body.stl

# ── MuJoCo model (parts -> wirebender.xml + meshes) ─────────────────
sim/wirebender.xml: sim/make_mjcf.py $(PARTS)
	cd sim && ../$(PY) make_mjcf.py

model: sim/wirebender.xml          ## regenerate the MuJoCo model

parts: build/base.stl              ## (re)build the separately-printed base parts (plate/uprights/gear)
	@echo "  build/: base_plate.stl  upright_front.stl  upright_rear.stl  gear.stl  sleeve.stl"

head: build/rothead_assembly.stl   ## (re)build the 2-piece head (rotation piece + cycloid/bend piece)
	@echo "  build/: rothead_rot.stl (clamp+rotation motor)  rothead_bend.stl (slotted cycloid mount)"

proto: build/base_proto.stl        ## short/narrow front base section for fast gear+upright prototyping

sim: sim/wirebender.xml view       ## build what changed, then open the viewer

view:                              ## open the MuJoCo viewer (drag rot/bend sliders)
	cd sim && DISPLAY=:0 ../$(PY) view.py

anim: sim/wirebender.xml           ## render a bend animation to a GIF (NAME=staple|square|chair|coil)
	cd sim && MUJOCO_GL=osmesa ../$(PY) animate_bend.py $(or $(NAME),staple)

anim-view: sim/wirebender.xml      ## play a bend program LIVE in the viewer (NAME=...; needs a display)
	cd sim && DISPLAY=:0 ../$(PY) animate_bend.py $(or $(NAME),staple) --view

slice:                             ## model -> G-code (IN=model.svg/.stl/example [OUT=part.gcode])
	cd sim && ../$(PY) slicer.py $(IN) $(if $(OUT),-o $(OUT),)

run: sim/wirebender.xml            ## run a sliced part in the sim (IN=part.gcode or a model/example)
	cd sim && MUJOCO_GL=osmesa ../$(PY) animate_bend.py $(IN)

sleeve: build/sleeve.stl           ## printed 1/4"-tube -> 608-bearing adapter (print x2)

pinion: build/pinion.stl           ## stepper pinion (12T, 5mm D-bore + M3 set screw)

vendor: build/cyclo_body.stl       ## re-bake the cycloidal envelope (slow)

clean:                             ## delete generated STLs/STEPs/meshes/model
	rm -f build/*.stl build/*.step sim/meshes/*.stl sim/wirebender.xml

help:
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
