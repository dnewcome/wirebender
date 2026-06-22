# Wire bender — build the CAD parts, assemble the MuJoCo model, and view it.
#   make sim     build everything that changed and open the simulation
#   make model   (re)generate sim/wirebender.xml only
#   make clean   remove generated artifacts
PY := py/bin/python

.PHONY: sim view model clean help
.DEFAULT_GOAL := help

# ── CAD parts (build123d -> build/*.stl) ────────────────────────────
build/base.stl: cad/base.py cad/gears.py
	$(PY) cad/base.py
build/rothead.stl: cad/rothead.py cad/parts.py cad/gears.py cad/pinion.py cad/base.py cad/cyclo_base.py
	$(PY) cad/rothead.py
build/head_refs.stl: cad/head_refs.py cad/rothead.py cad/parts.py
	$(PY) cad/head_refs.py            # REFERENCE: motors + cyclo (purchased, not printed)
build/feeder_body.stl: cad/gen_feeder.py extruder.glb
	$(PY) cad/gen_feeder.py
build/base_proto.stl: cad/base_proto.py cad/base.py
	$(PY) cad/base_proto.py
build/base_proto_l.stl: cad/base_proto_l.py cad/base.py
	$(PY) cad/base_proto_l.py          # L-shaped proto of the NEW deck (bearing + feeder-bracket mounts)
build/assembly.step: cad/assembly.py cad/base.py cad/rothead.py build/feeder_body.stl
	$(PY) cad/assembly.py
build/sleeve.stl: cad/sleeve.py cad/base.py
	$(PY) cad/sleeve.py
build/pinion.stl: cad/pinion.py cad/base.py cad/gears.py
	$(PY) cad/pinion.py
build/feeder_gear.stl: cad/feeder_gear.py cad/base.py cad/gears.py
	$(PY) cad/feeder_gear.py
build/feeder_gear_press.stl: cad/feeder_gear_press.py cad/gears.py
	$(PY) cad/feeder_gear_press.py     # press-fit variant (D-flat drive, 4mm collar, no set screw)
build/feeder_motor_mount.stl: cad/feeder_motor_mount.py
	$(PY) cad/feeder_motor_mount.py    # pivoting NEMA17 mount plate (mesh adjust); MEASURE housing holes
build/nema_template.stl: cad/nema_template.py
	$(PY) cad/nema_template.py         # thin NEMA17 drilling/marking template
build/bend_plate.stl: cad/bend_plate.py cad/cyclo_base.py cad/rothead.py cad/parts.py
	$(PY) cad/bend_plate.py
build/bend_plate_90.stl: cad/bend_plate_90.py cad/cyclo_base.py cad/rothead.py
	$(PY) cad/bend_plate_90.py          # variant: boss bends 90° up into a riser w/ cross-axis (Y) slot
build/feeder_bracket.stl: cad/feeder_bracket.py cad/base.py
	$(PY) cad/feeder_bracket.py        # upright bracket: feeder bolt pattern rotated 90° into a vertical face
build/housing.stl: cad/housing.py
	$(PY) cad/housing.py               # parametric cyclo Housing (20-pin ring); no vendor STEP needed
build/arbor_mount.stl: cad/arbor_mount.py cad/housing.py
	$(PY) cad/arbor_mount.py            # parametric housing + fused 4-bolt posts (OCC)
build/end_cap.stl: cad/end_cap.py
	$(PY) cad/end_cap.py                # cyclo End_Cap rebuilt clean (watertight), flange turned down 2mm
build/cyclo_base.stl: cad/cyclo_base.py
	$(PY) cad/cyclo_base.py            # parametric cyclo NEMA17 base (vendor-independent; no STEP/STL)
build/bend_disc.stl: cad/bend_disc.py cad/arbor_mount.py
	$(PY) cad/bend_disc.py             # bend-pin disc that bolts to the arbor posts
build/cycloid_disc.stl build/cycloid_shaft.stl: cad/cycloid.py cad/housing.py cad/cyclo_base.py
	$(PY) cad/cycloid.py               # cycloidal disc + eccentric shaft (parametric; validated vs vendor STEP)
build/home_switch.stl: cad/home_switch.py
	$(PY) cad/home_switch.py           # adjustable subminiature-microswitch mount (homing, both axes)
build/cycloid_assembly.stl: cad/check_cyclo.py cad/cycloid.py cad/housing.py cad/cyclo_base.py
	$(PY) cad/check_cyclo.py           # fit check + viewable housing+discs+shaft assembly

# printable parts the sim assembles + reference meshes (motors/cyclo, feeder)
PARTS := build/base.stl build/rothead.stl build/pinion.stl build/end_cap.stl \
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

proto-l: build/base_proto_l.stl    ## L-shaped proto of the new deck: bearing mounts + vertical feeder-bracket mounts

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

drive:                             ## what stock can the machine bend + driver current needed (MATERIAL=pa-cf CURRENT=1.5)
	cd sim && ../$(PY) drive_model.py $(if $(MATERIAL),--material $(MATERIAL),) $(if $(CURRENT),--current $(CURRENT),)

sleeve: build/sleeve.stl           ## printed 1/4"-tube -> 608-bearing adapter (print x2)

pinion: build/pinion.stl           ## stepper pinion (12T, 5mm D-bore + M3 set screw)

feeder-gear: build/feeder_gear.stl ## feeder stepper gear (11T m0.8, 5mm D-bore + M3 set screw)

feeder-gear-press: build/feeder_gear_press.stl ## PRESS-FIT feeder pinion (11T m0.8, D-flat drive, 8mm+4mm collar=12mm, no set screw)

feeder-motor-mount: build/feeder_motor_mount.stl ## pivoting NEMA17 plate to set gear mesh (pivot + countersunk arc slot)

nema-template: build/nema_template.stl ## thin NEMA17 drilling/marking template (4-bolt + Ø22 pilot)

feeder-bracket: build/feeder_bracket.stl ## upright bracket holding the feeder vertically (feeder pattern rotated 90° into a vertical face)

bend-plate: build/bend_plate.stl   ## parametric cyclo base + flange-flush side boss w/ 2 M3 inserts (vendor-independent)

bend-plate-90: build/bend_plate_90.stl ## bend_plate variant: boss bends 90° up into a riser w/ a cross-axis (Y) slot

housing: build/housing.stl          ## parametric cyclo Housing (20-pin internal-gear ring; no vendor STEP)

arbor-mount: build/arbor_mount.stl ## cyclo ring + fused 4-bolt post pattern (radial, 4mm proud, M3 inserts) + Axis-3 home tab

home-switch: build/home_switch.stl ## adjustable subminiature-microswitch mount (Axis 2 + Axis 3 homing)

end-cap: build/end_cap.stl          ## parametric cyclo End_Cap (FIXED), OD turned down 2mm to clear the mounts

cyclo-base: build/cyclo_base.stl    ## parametric cycloidal NEMA17 base (reverse-engineered; no vendor file)

bend-disc: build/bend_disc.stl      ## Ø60x2 disc bolting to the arbor posts; csk M3 mounts + centre/10mm bend-pin holes

cycloid:                           ## cycloidal disc + eccentric shaft (PINS=30 scales the ring -> ratio, keeping lobe size; default 20)
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(PY) cad/cycloid.py

check-cyclo:                       ## assert the disc/shaft fit the ring + write build/cycloid_assembly.stl (PINS=30 to check a scaled drive)
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(PY) cad/check_cyclo.py

cyclo-drive:                       ## build a COMPLETE scaled drive (PINS=30 MOTOR=nema23): ring+disc+shaft+base+end_cap, then fit-check
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(if $(MOTOR),WB_MOTOR=$(MOTOR),) $(PY) cad/housing.py
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(PY) cad/cycloid.py
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(if $(MOTOR),WB_MOTOR=$(MOTOR),) $(PY) cad/cyclo_base.py
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(PY) cad/end_cap.py
	$(if $(PINS),WB_RING_PINS=$(PINS),) $(PY) cad/check_cyclo.py

clean:                             ## delete generated STLs/STEPs/meshes/model
	rm -f build/*.stl build/*.step sim/meshes/*.stl sim/wirebender.xml

help:
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
