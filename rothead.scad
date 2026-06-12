// rothead.scad — rotating head MASSING (cantilever architecture)
// Blocks only — confirm the layout/balance before detailing the printed body.
//
// The head clamps the feed tube and rotates about the wire axis (X = Axis 2),
// cantilevered forward of the base. On it, opposite each other for balance:
//   +Z: the BENDER drivetrain (gutted servo gearbox + tiny stepper -> layshaft ->
//       final gear -> Ø4 mandrel + shoe, the mandrel brought IN near the wire axis)
//   -Z: the ROTATION motor, its 12T pinion meshing the base's fixed gear (which
//       sits coaxial with the wire axis at x=0..FG_W); the pinion walks the head.
//
// Frame: X = wire axis (rotation axis, origin on it). Head body is at x<0 (forward
// of the base). rot = head rotation (deg) to check swing.

use <MCAD/involute_gears.scad>
$fn = 48;
rot = 0;

// ── shared with base.scad (keep in sync) ────────────────────────────
FG_TEETH = 40;  FG_MODULE = 1.5;  FG_W = 8;  PIN_TEETH = 12;
MESH_R = (FG_TEETH + PIN_TEETH) * FG_MODULE / 2;     // 39 — rotation-pinion radius
TUBE_D = 8;

// ── head bits ───────────────────────────────────────────────────────
HUB_OD = 16;   HUB_X = [-24, 2];
MANDREL_OFFSET = 2.8;  MANDREL_D = 4;
MANDREL_POS = [-26, MANDREL_OFFSET];                 // near the wire axis, at the front

// estimated motor envelopes (correct later)
ROTMOT = [30, 24, 24];          // small geared stepper (rotation)  L(x)xWxH
BENDER = [40, 24, 32];          // gutted servo + tiny stepper      L(x)xWxH
BENDER_R = 24;                  // bender mass inboard of the 39mm mesh radius -> balance + smaller swing

module head() rotate([rot, 0, 0]) {
    // tube clamp hub
    color([0.55, 0.58, 0.62])
        translate([HUB_X[0], 0, 0]) rotate([0, 90, 0]) cylinder(d = HUB_OD, h = HUB_X[1]-HUB_X[0]);

    // ── rotation motor (-Z) + pinion meshing the fixed gear ──
    color([0.25, 0.27, 0.30])
        translate([-ROTMOT[0] + 3, -ROTMOT[1]/2, -MESH_R - ROTMOT[2]/2]) cube(ROTMOT);
    color([0.85, 0.55, 0.15])      // 12T pinion at x=0..FG_W, radius MESH_R
        translate([0, 0, -MESH_R]) rotate([0, 90, 0])
            gear(number_of_teeth = PIN_TEETH, circular_pitch = FG_MODULE*180, gear_thickness = FG_W,
                 rim_thickness = FG_W, hub_thickness = 0, bore_diameter = 4);
    // arm tying the rotation motor to the hub
    color([0.55, 0.58, 0.62])
        translate([-6, -3, -MESH_R/2]) cube([12, 6, MESH_R], center = false);

    // ── bender drivetrain (+Z): servo+stepper inboard, single gear pair to the mandrel ──
    color([0.30, 0.55, 0.85])
        translate([-BENDER[0] + 3, -BENDER[1]/2, BENDER_R - BENDER[2]/2]) cube(BENDER);
    color([0.6, 0.62, 0.66])       // relocation gear pair span: servo output -> mandrel
        translate([MANDREL_POS[0] + 2, MANDREL_OFFSET, 6]) rotate([0, 0, 0])
            cube([6, 4, BENDER_R - 10]);
    // mandrel (Ø4, axis ⊥ wire) + shoe near the wire axis
    color([0.62, 0.64, 0.68])
        translate([MANDREL_POS[0], MANDREL_POS[1], -6]) cylinder(d = MANDREL_D, h = 16);
    color([0.85, 0.55, 0.15])      // shoe at small radius from the mandrel
        translate([MANDREL_POS[0], MANDREL_POS[1] + 6, 0]) cylinder(d = 3, h = 8);

    // wire guide stub (wire comes along +X to the mandrel)
    color([0.6, 0.62, 0.66])
        translate([MANDREL_POS[0], 0, 0]) rotate([0, 90, 0]) cylinder(d = 3, h = 26);
    // arm tying the bender to the hub
    color([0.55, 0.58, 0.62])
        translate([-6, -3, 0]) cube([12, 6, BENDER_R], center = false);
}

// reference: the base's fixed gear (ghost) the pinion meshes
%color([0.4, 0.55, 0.7, 0.3]) translate([0, 0, 0]) rotate([0, 90, 0])
    gear(number_of_teeth = FG_TEETH, circular_pitch = FG_MODULE*180, gear_thickness = FG_W,
         rim_thickness = FG_W, hub_thickness = 0, bore_diameter = TUBE_D + 2);
// feed tube ghost
%color([0.6, 0.62, 0.66, 0.4]) translate([-40, 0, 0]) rotate([0, 90, 0]) cylinder(d = TUBE_D, h = 80);

head();
