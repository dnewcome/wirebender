// head.scad — bending-head housing (Axis 3)
// Replaces bender-head.scad + motor.scad + motor-flange.scad. It:
//   - bears the output gear on a 608 (the mandrel journal) — takes the bend load
//   - mounts the pancake NEMA17 behind the back wall; pinion meshes the gear up front
//   - guides the wire in tangent to the Ø4 mandrel (on the spindle/wire axis)
//   - bolts to the spindle front flange (Ø22 M3, Ø8 bore)
// Output gear + mandrel + shoe come from bend-gears.scad.
//
// FIRST PASS — iterate on bearing choice, NEMA pocket, wire guide, proportions.
//
// Frame: Z = bend axis (mandrel, up). Y = wire axis (spindle on -Y, wire feeds +Y
// then bends). Mandrel sits +X of the wire axis by OFFSET so the wire is tangent.

use <MCAD/involute_gears.scad>
show = "assembly";        // "head" | "assembly"
$fn = 72;

// ── shared (keep in sync with bend-gears.scad / spindle.scad) ───────
MODULE_ = 1;  PINION_T = 14;  GEAR_T = 56;  CP = MODULE_*180;
CD      = (PINION_T + GEAR_T) * MODULE_ / 2;          // 35  centre distance
GEAR_OD = (GEAR_T + 2) * MODULE_;                     // ~58
GEAR_TH = 8;
MANDREL_D = 4;  WIRE_OD = 1.63;
OFFSET = MANDREL_D/2 + WIRE_OD/2 + 0.4;               // wire tangent to the mandrel ≈2.8
BRG_OD = 22;  BRG_W = 7;                              // 608 on the mandrel journal
SP_OD = 32;  SP_BOLT = 22;  SP_HOLE = 3.4;  SP_BORE = 8;
NEMA = 42.3;  NEMA_BOLT = 31;  NEMA_PILOT = 22;  NEMA_HOLE = 3.4;  SHAFT_HOLE = 11;

// ── layout (XY = bend plane; Z = bend axis) ─────────────────────────
MANDREL = [OFFSET, 0];           // bend axis / mandrel
PINION  = [OFFSET + CD, 0];      // NEMA17 axis
WALL_T  = 8;                     // back wall thickness (carries bearing + motor)
GEAR_Z  = WALL_T + 1;            // gear sits just in front of the wall
WIRE_Z  = GEAR_Z + GEAR_TH/2;    // wire passes at the gear mid-plane

module wall_outline() {
    // footprint enclosing the gear and the NEMA17, rounded
    hull() {
        translate(MANDREL) circle(d = GEAR_OD + 8);
        translate(PINION)  circle(d = NEMA + 6);
    }
}

module head() {
    difference() {
        union() {
            linear_extrude(WALL_T) wall_outline();                 // back wall
            // spindle boss: Ø32 along -Y at the wire axis, front face for the flange
            translate([0, -GEAR_OD/2 - 6, WIRE_Z]) rotate([90, 0, 0])
                cylinder(d = SP_OD, h = 14);
            // wire-guide rib along Y at the wire axis, up to the mandrel
            translate([-5, -GEAR_OD/2 - 6, WALL_T]) cube([10, GEAR_OD/2 + 6, WIRE_Z - WALL_T + 4]);
        }
        // 608 bearing pocket for the mandrel (from the front)
        translate([MANDREL[0], MANDREL[1], WALL_T - BRG_W])
            cylinder(d = BRG_OD, h = BRG_W + 1);
        translate([MANDREL[0], MANDREL[1], -1]) cylinder(d = SP_BORE + 4, h = WALL_T);  // mandrel clearance
        // NEMA17 mount on the back: pilot + shaft hole + 4 bolts
        translate([PINION[0], PINION[1], -1]) {
            cylinder(d = SHAFT_HOLE, h = WALL_T + 2);
            cylinder(d = NEMA_PILOT, h = 2.5);                     // pilot recess (back face)
            for (a = [[1, 1], [1, -1], [-1, 1], [-1, -1]])
                translate([a[0]*NEMA_BOLT/2, a[1]*NEMA_BOLT/2, 0]) cylinder(d = NEMA_HOLE, h = WALL_T + 2);
        }
        // wire guide bore along Y at the wire axis (x=0), tangent to the mandrel
        translate([0, -GEAR_OD/2 - 7, WIRE_Z]) rotate([-90, 0, 0]) cylinder(d = 3, h = GEAR_OD/2 + 8);
        // spindle bolt circle (M3) on the boss front face
        translate([0, -GEAR_OD/2 - 6, WIRE_Z]) rotate([90, 0, 0])
            for (i = [0:3]) rotate([0, 0, 90*i + 45])
                translate([SP_BOLT/2, 0, -1]) cylinder(d = SP_HOLE, h = 16);
    }
}

// ── reference ghosts ────────────────────────────────────────────────
module ghost() {
    color("0.30 0.55 0.85", 0.5) translate([MANDREL[0], MANDREL[1], GEAR_Z])      // output gear
        gear(number_of_teeth = GEAR_T, circular_pitch = CP, pressure_angle = 20,
             gear_thickness = GEAR_TH, rim_thickness = GEAR_TH, hub_thickness = 0, bore_diameter = 8);
    color("0.85 0.55 0.2", 0.6) translate([PINION[0], PINION[1], GEAR_Z])         // pinion
        gear(number_of_teeth = PINION_T, circular_pitch = CP, pressure_angle = 20,
             gear_thickness = GEAR_TH, rim_thickness = GEAR_TH, hub_thickness = 0, bore_diameter = 5);
    color("0.62 0.64 0.68", 0.7) translate([MANDREL[0], MANDREL[1], GEAR_Z]) cylinder(d = MANDREL_D, h = GEAR_TH + 8);  // mandrel
    color("0.25 0.27 0.30", 0.5) translate([PINION[0], PINION[1], -22]) cube([NEMA, NEMA, 22], center = false);          // pancake NEMA17 (rough)
    color("0.7 0.72 0.76", 0.5) translate([0, -GEAR_OD/2 - 6, WIRE_Z]) rotate([90, 0, 0]) cylinder(d = SP_OD, h = 6);    // spindle flange
}

color([0.2, 0.55, 0.85]) head();
if (show == "assembly") ghost();
