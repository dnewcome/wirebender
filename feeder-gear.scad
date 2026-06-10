// Requires MCAD library (involute_gears.scad)
use <MCAD/involute_gears.scad>

$fn = 120;

teeth = 12;
m = 0.5;
bore = 0;
thickness = 6;

module teeth() {
    gear(
        number_of_teeth=teeth,
        circular_pitch = m * 180,   // MCAD uses circular_pitch in  degrees-ish units
        pressure_angle = 20,
        clearance = 0.0,
        gear_thickness = thickness,
        rim_thickness = thickness,
        hub_thickness = 0,
        bore_diameter = bore
    );
}

module collar() {
    difference() {
        cylinder(d = 10, h = 10);
        cylinder(d = 5, h = 10);
        translate([0,0,5]) rotate([0,90,0]) cylinder(d = 4, h = 10);
    }
}

teeth();
translate([0, 0, -9]) collar();