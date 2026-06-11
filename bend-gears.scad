// bend-gears.scad
// 3D-printed spur-gear reduction for the bend axis (Axis 3).
// A pancake NEMA17 carries the PINION; the OUTPUT gear is the bending flange —
// it carries the central mandrel (the wire bends around it) and the shoe screw,
// and rotates about the bend axis. In-plane reduction keeps the head flat.
//
// Default module 1, 14T → 56T (4:1, ~0.7 N·m from a ~0.2 N·m pancake at 0.9 eff).
// Bump GEAR_T to 70 for 5:1 (~0.9 N·m, headroom to 2 mm wire).
//
// Uses MCAD involute_gears (same as feeder-gear.scad).

use <MCAD/involute_gears.scad>
$fn = 64;

show = "pair";        // "pair" | "output_gear" | "pinion"

// ── gear params ─────────────────────────────────────────────────────
MODULE_  = 1.0;
PINION_T = 14;
GEAR_T   = 56;             // 4:1  (use 70 for 5:1)
PA       = 20;            // pressure angle
THK      = 8;            // gear thickness
CP = MODULE_ * 180;       // MCAD circular pitch (PD = teeth * CP / 180 = teeth * module)
CD = (PINION_T + GEAR_T) * MODULE_ / 2;   // centre distance

// ── interfaces ──────────────────────────────────────────────────────
PINION_BORE = 5;          // NEMA17 shaft
PINION_HUB  = 11;         // set-screw collar
MANDREL_D   = 4;          // mandrel pin — wire bends around this (Ø4 -> ~2.8 mm bend radius)
MANDREL_H   = 9;
SHOE_R      = 7;          // bending-shoe radius from the bend axis
SHOE_HOLE   = 3.2;        // M3 shoe screw
GEAR_BORE   = 8;          // output bearing / shaft bore

module pinion() {
    difference() {
        union() {
            gear(number_of_teeth = PINION_T, circular_pitch = CP, pressure_angle = PA,
                 gear_thickness = THK, rim_thickness = THK, hub_thickness = 0, bore_diameter = 0);
            cylinder(d = PINION_HUB, h = THK + 5);              // set-screw collar
        }
        translate([0, 0, -1]) cylinder(d = PINION_BORE, h = THK + 7);
        translate([0, 0, THK + 2]) rotate([90, 0, 0]) cylinder(d = 3, h = 12);  // set screw
    }
}

// printed gear: teeth + Ø8 bore (for the metal mandrel/journal) + shoe screw hole
module output_gear() {
    difference() {
        gear(number_of_teeth = GEAR_T, circular_pitch = CP, pressure_angle = PA,
             gear_thickness = THK, rim_thickness = THK, hub_thickness = 0, bore_diameter = GEAR_BORE);
        translate([SHOE_R, 0, -1]) cylinder(d = SHOE_HOLE, h = THK + 2);     // shoe screw
    }
}

// metal mandrel: Ø8 journal (presses into the gear bore + rides the head bearing)
// stepping to the Ø4 mandrel the wire wraps. Hardened dowel / drill rod.
module mandrel() {
    cylinder(d = GEAR_BORE, h = THK);              // journal in the gear bore
    translate([0, 0, THK]) cylinder(d = MANDREL_D, h = MANDREL_H);
}

// ── output / render ─────────────────────────────────────────────────
ratio = GEAR_T / PINION_T;
if (show == "output_gear") {
    color([0.30, 0.55, 0.85]) output_gear();              // at origin (for the sim bend body)
} else if (show == "pinion") {
    color([0.85, 0.55, 0.2]) pinion();
} else {                                                   // meshed pair
    color([0.85, 0.55, 0.2]) pinion();
    translate([CD, 0, 0]) {
        color([0.30, 0.55, 0.85]) rotate([0, 0, 180/GEAR_T]) output_gear();
        color([0.62, 0.64, 0.68]) mandrel();
    }
}

echo(str("ratio = ", ratio, ":1   centre distance = ", CD, " mm   ",
         "pinion PD ", PINION_T*MODULE_, "  gear PD ", GEAR_T*MODULE_));
