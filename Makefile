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
build/rothead.stl: cad/rothead.py cad/parts.py cad/gears.py cad/pinion.py cad/base.py
	$(PY) cad/rothead.py
build/head_refs.stl: cad/head_refs.py cad/rothead.py cad/parts.py
	$(PY) cad/head_refs.py            # REFERENCE: motors + cyclo (purchased, not printed)
build/bend_endcap.stl: cad/bend_endcap.py
	$(PY) cad/bend_endcap.py
build/cyclo_body.stl: cad/gen_vendor.py
	$(PY) cad/gen_vendor.py            # slow; needs the purchased Sweep Dynamics STEP
build/feeder_body.stl: cad/gen_feeder.py extruder.glb
	$(PY) cad/gen_feeder.py
build/base_proto.stl: cad/base_proto.py cad/base.py
	$(PY) cad/base_proto.py
build/assembly.step: cad/assembly.py cad/base.py cad/rothead.py build/feeder_body.stl
	$(PY) cad/assembly.py
build/sleeve.stl: cad/sleeve.py cad/base.py
	$(PY) cad/sleeve.py
build/pinion.stl: cad/pinion.py cad/base.py cad/gears.py
	$(PY) cad/pinion.py
build/feeder_gear.stl: cad/feeder_gear.py cad/base.py cad/gears.py
	$(PY) cad/feeder_gear.py
build/bend_plate.stl: cad/bend_plate.py cad/rothead.py cad/parts.py
	$(PY) cad/bend_plate.py
build/arbor_mount.stl: cad/arbor_mount.py
	$(PY) cad/arbor_mount.py            # slow; needs the purchased Sweep Dynamics STEP

# printable parts the sim assembles + reference meshes (motors/cyclo, feeder)
PARTS := build/base.stl build/rothead.stl build/pinion.stl build/bend_endcap.stl \
         build/arbor_mount.stl build/bend_plate.stl \
         build/head_refs.stl build/feeder_body.stl

# ── MuJoCo model (parts -> wirebender.xml + meshes) ─────────────────
sim/wirebender.xml: sim/make_mjcf.py $(PARTS)
	cd sim && ../$(PY) make_mjcf.py

model: sim/wirebender.xml          ## regenerate the MuJoCo model

parts: build/base.stl              ## (re)build the separately-printed base parts (plate/uprights/gear)
	@echo "  build/: base_plate.stl  upright_front.stl  upright_rear.stl  gear.stl  sleeve.stl"

head: build/rothead.stl build/head_refs.stl  ## (re)build the flat head + pinion + reference motor/cyclo meshes
	@echo "  printable: rothead.stl (flat head: 2 slotted plates + tube boss)  pinion.stl    reference: head_refs.stl"

proto: build/base_proto.stl        ## short/narrow front base section for fast gear+upright prototyping

assembly: build/assembly.step      ## full machine as one build123d assembly (build/assembly.step + .stl)

sim: sim/wirebender.xml view       ## build what changed, then open the viewer

view:                              ## open the MuJoCo viewer (head auto-rotates; add ARGS=--manual to drag sliders)
	cd sim && DISPLAY=:0 ../$(PY) view.py $(ARGS)

anim: sim/wirebender.xml           ## render a bend GIF; FAULTS+halts on a rule violation (NAME=...; ARGS=--force)
	cd sim && MUJOCO_GL=osmesa ../$(PY) animate_bend.py $(or $(NAME),staple) $(ARGS)

anim-view: sim/wirebender.xml      ## play a bend program LIVE; faults+halts on a violation (NAME=...; needs a display)
	cd sim && DISPLAY=:0 ../$(PY) animate_bend.py $(or $(NAME),staple) --view $(ARGS)

slice:                             ## model -> G-code (IN=model.svg/.stl/example [OUT=part.gcode])
	cd sim && ../$(PY) slicer.py $(IN) $(if $(OUT),-o $(OUT),)

run: sim/wirebender.xml            ## run a sliced part in the sim; FAULTS+halts on a rule violation (IN=part.gcode|model|example)
	cd sim && MUJOCO_GL=osmesa ../$(PY) animate_bend.py $(IN)

check-pin: sim/wirebender.xml      ## track bend-pin position + clearance to the feed tube over the axis range
	cd sim && MUJOCO_GL=osmesa ../$(PY) check_pin.py

rules:                             ## check a program against the bend-cell limits (IN=example|part.gcode; or --caps)
	cd sim && ../$(PY) interference.py $(or $(IN),--caps)

check-consistency:                 ## assert sim/machine.py matches the CAD constants
	$(PY) sim/consistency.py

sleeve: build/sleeve.stl           ## printed 1/4"-tube -> 608-bearing adapter (print x2)

pinion: build/pinion.stl           ## stepper pinion (12T, 5mm D-bore + M3 set screw)

feeder-gear: build/feeder_gear.stl ## feeder stepper gear (11T m0.8, 5mm D-bore + M3 set screw)

bend-plate: build/bend_plate.stl   ## vendor cyclo base + flange-flush side boss w/ 2 M3 inserts (needs the vendor STL)

arbor-mount: build/arbor_mount.stl ## cyclo housing + 4 corner standoff posts w/ M3 inserts (needs the vendor STEP)

vendor: build/cyclo_body.stl       ## re-bake the cycloidal envelope (slow)

clean:                             ## delete generated STLs/STEPs/meshes/model
	rm -f build/*.stl build/*.step sim/meshes/*.stl sim/wirebender.xml

help:
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
