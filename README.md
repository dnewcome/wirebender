# CNC Wire Bender

A CNC wire bending machine capable of bending 14 or 16 gauge steel wire into arbitrary 3D shapes using G-code.

## Overview

Wire is fed from a spool using a 3D printer extruder, through a rotating feed tube. The feed tube rotation sets the bend plane (direction), and a bending head fixed to the end of the tube performs the actual bend. Motion is controlled by a GRBL 1.1 board.

## Machine Architecture

### Motion Controller
- **Controller:** GRBL 1.1
- **Input:** Standard G-code

### Wire Feed
- **Component:** Off-the-shelf 3D printer extruder
- **Wire gauge:** 14 or 16 AWG steel wire fed from a roll
- **Feed path:** Extruder → feed tube → bending head

### Feed Tube (Rotation Axis)
- Rotates around the wire axis to set the bend direction
- Driven by a geared stepper motor via toothed belt
- Supported by bearings housed in a U-shaped sheet aluminum bracket
- The U-bracket is extended to also mount the tube rotation motor

### Bending Head
- Fixed to the end of the feed tube; rotates with it
- Driven by a dedicated stepper motor
- **Mandrel:** The motor shaft itself
- **Bending flange:** 3D printed, mounted directly to the bending motor shaft; an M3 screw through the edge acts as the bending shoe
- **Motor mount:** 3D printed bracket (`bender-head.scad`) bolts to the feed tube flange

## Files

| File | Description |
|------|-------------|
| `bender-head.scad` | OpenSCAD model of the bending head motor mount / bending flange |

## Bending Head Design Notes (`bender-head.scad`)

- Overall part diameter: 43 mm
- Flange mounting hole spacing: 35 mm on center (M4 holes)
- Feed tube diameter: 6.35 mm (0.25 in)
- Bending flange pocket diameter: 10 mm + 2 mm fudge
- Part height: 11 mm
- An M4 cross-hole allows a set screw or insert perpendicular to the tube axis

## Checklist

### Mechanical Design
- [ ] Finalize extruder motor mounting to the main assembly
- [ ] Design or source the U-shaped sheet aluminum feed tube bearing bracket
- [ ] Integrate tube rotation motor mount into the U-bracket design
- [ ] Verify feed tube bearing selection and fit
- [ ] Confirm toothed belt/pulley sizing for feed tube rotation axis
- [ ] Validate bending head flange fits the chosen bending motor shaft
- [ ] Test M3 bending shoe screw fit and geometry in `bender-head.scad`
- [ ] Design wire guide / straightener between spool and extruder

### Electronics & Control
- [ ] Select stepper drivers and confirm GRBL axis assignments
  - Axis 1: wire feed (extruder)
  - Axis 2: feed tube rotation (bend direction)
  - Axis 3: bending head (bend degree)
- [ ] Wire up GRBL 1.1 board and stepper drivers
- [ ] Tune stepper current limits and microstepping
- [ ] Calibrate steps/mm for each axis
- [ ] Define axis limits and homing strategy

### Software / G-code
- [ ] Decide on G-code generation workflow (CAM tool, custom post-processor, or hand-written)
- [ ] Write or source a wire bending post-processor / path generator
- [ ] Test basic feed, rotate, and bend sequences
- [ ] Develop homing and initialization routine

### Fabrication
- [ ] Print `bender-head.scad` and test fit on bending motor
- [ ] Bend and drill U-bracket sheet aluminum pieces
- [ ] Assemble feed tube with bearings into bracket
- [ ] Full mechanical assembly and fit check

### Validation
- [ ] Dry-run all axes (no wire) to verify travel and direction
- [ ] First wire feed test with soft wire (e.g. aluminum) before steel
- [ ] Calibrate bend angle vs. motor steps
- [ ] Measure and compensate for springback in 14/16 ga steel
- [ ] Produce a simple test shape (e.g. square, circle)
