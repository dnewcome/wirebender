// base-bracket.scad
// Base / bracket that holds the wire bender together:
//   - journals the rotating spindle (Axis 2, bend direction) in two 608 bearings
//   - mounts a NEMA17 that belt-drives the spindle (GT2, ~3:1) with tension slots
//   - carries the bending head on the spindle's front flange (cantilevered)
//   - mounts the 1KGSSJ-B feeder at the rear, wire-axis aligned
//
// 3D-printed first; layout kept simple enough to re-do as folded sheet metal.
// Frame: X = wire axis (head at -X, feeder at +X), Y = across, Z = up.
// Base top at Z = 0; wire axis at (Y=0, Z=AXIS_Z).
//
// `show` selects what to render: "bracket" (printed part only) or "assembly"
// (bracket + ghosted bought parts for context).

show = "assembly";          // "bracket" | "assembly"
$fn = 64;

// ── bearings / spindle ──────────────────────────────────────────────
BRG_OD   = 22;   BRG_W = 7;   BRG_BORE = 8;     // 608
BRG_LIP  = 2;    // retaining shoulder width (pocket bottom ID = BRG_OD-2*LIP)
SPINDLE_BORE = 4;                                // PTFE-lined wire bore
AXIS_Z   = 50;                                   // wire-axis height above base

// ── bearing uprights ────────────────────────────────────────────────
FRONT_X  = 0;    REAR_X = 56;                    // upright X positions
UP_T     = 10;                                   // upright thickness (X)
UP_W     = 46;                                   // upright width (Y)
UP_H     = AXIS_Z + BRG_OD/2 + 8;                // upright height
WALL     = 4;

// ── pulleys (GT2 2mm) ───────────────────────────────────────────────
SP_PULLEY_OD = 40;  SP_PULLEY_W = 8;  PULLEY_X = 28;     // 60T on spindle
ST_PULLEY_OD = 14;                                       // 20T on stepper
BELT_CD = 28;                                            // spindle->stepper centres

// ── stepper (NEMA17), mounted below the spindle, belt vertical ──────
NEMA = 42.3;  NEMA_BOLT = 31;  NEMA_PILOT = 22;  NEMA_SHAFT = 5;  NEMA_LEN = 40;
ST_Z = AXIS_Z - BELT_CD;                                 // stepper axis height
ST_X = PULLEY_X;                                         // stepper aligned to pulley
TENSION_SLOT = 8;                                        // belt-tension travel

// ── base plate ──────────────────────────────────────────────────────
BASE_THK = 6;
BASE_X0  = -14;  BASE_X1 = 250;                          // head end -> past feeder
BASE_W   = 60;

// ── feeder (1KGSSJ-B) mount ─────────────────────────────────────────
FEEDER_X   = 150;                                        // feeder body centre X
FEEDER_L   = 122;  FEEDER_H = 85;  FEEDER_T = 30;
FEEDER_HOLE = 5.3;
// TODO: confirm the real bolt pattern off the feeder; placeholder rectangle:
FEEDER_HOLES = [[ -45, 14], [ 45, 14], [ -45, -14], [ 45, -14]];   // [dx along X, dz]

// ── bending head (Ø43, mounts on spindle front flange) ──────────────
HEAD_OD = 43;  HEAD_H = 11;  HEAD_BOLT = 35;             // M4 at 35mm spacing

// ─────────────────────────────────────────────────────────────────────


module bearing_pocket() {
    // pocket opening toward +X; bore through with a retaining shoulder
    translate([-0.01, 0, 0]) rotate([0, 90, 0]) {
        cylinder(d = BRG_OD + 0.1, h = BRG_W + 0.2);                 // bearing seat
        cylinder(d = BRG_OD - 2*BRG_LIP, h = UP_T + 1);              // shoulder bore
    }
}

module upright(x) {
    difference() {
        // rounded-top plate
        translate([x, 0, UP_H/2])
            cube([UP_T, UP_W, UP_H], center = true);
        // bearing pocket on the wire axis
        translate([x - UP_T/2, 0, AXIS_Z]) bearing_pocket();
        // lighten / cable pass + corner mounting holes to base handled by base
    }
}

module nema17_holes(depth) {
    s = NEMA_BOLT/2;
    for (a = [[s, s], [s, -s], [-s, s], [-s, -s]])
        translate([a[0], a[1], -1]) cylinder(d = 3.4, h = depth + 2);
}

module motor_mount() {
    // vertical plate below the spindle holding the NEMA17 face (shaft +X),
    // with vertical slots so the motor slides for belt tension.
    plate_t = 6;
    translate([ST_X + plate_t/2, 0, ST_Z]) rotate([0, -90, 0])
    difference() {
        translate([0, 0, 0]) cube([NEMA + 8, NEMA + 8, plate_t], center = true);
        // pilot clearance
        cylinder(d = NEMA_PILOT + 1, h = plate_t + 2, center = true);
        // tension slots in place of the 4 bolt holes (slide along motor-axis-perp)
        s = NEMA_BOLT/2;
        for (a = [[s, s], [s, -s], [-s, s], [-s, -s]])
            translate([a[0], a[1], 0]) hull()
                for (dy = [-TENSION_SLOT/2, TENSION_SLOT/2])
                    translate([0, dy, 0]) cylinder(d = 3.6, h = plate_t + 2, center = true);
    }
}

module feeder_plate() {
    // upright plate at the rear carrying the feeder's bolt pattern, axis-aligned
    plate_t = 6;
    translate([FEEDER_X, 0, AXIS_Z]) rotate([0, 90, 0])
    difference() {
        translate([0, 0, -plate_t/2]) cube([FEEDER_H, UP_W + 20, plate_t], center = true);
        cylinder(d = 9, h = plate_t + 2, center = true);   // wire pass-through
        for (h = FEEDER_HOLES)
            translate([h[1], h[0], 0]) cylinder(d = FEEDER_HOLE, h = plate_t + 2, center = true);
    }
}

module base() {
    L = BASE_X1 - BASE_X0;
    difference() {
        translate([BASE_X0 + L/2, 0, -BASE_THK/2]) cube([L, BASE_W, BASE_THK], center = true);
        // benchtop mounting holes
        for (x = [BASE_X0 + 16, REAR_X + 24, FEEDER_X, BASE_X1 - 16])
            for (y = [-BASE_W/2 + 8, BASE_W/2 - 8])
                translate([x, y, -BASE_THK - 1]) cylinder(d = 5, h = BASE_THK + 2);
    }
}

module gusset(x) {
    // triangular rib tying an upright to the base
    translate([x, 0, 0]) rotate([90, 0, 0]) linear_extrude(WALL, center = true)
        polygon([[0, 0], [UP_T, 0], [0, AXIS_Z]]);
}

module bracket() {
    color("0.20 0.55 0.85") {
        base();
        upright(FRONT_X);
        upright(REAR_X);
        translate([FRONT_X - UP_T/2 - WALL/2, 0, 0]) gusset(0);
        translate([REAR_X + UP_T/2 - WALL/2, 0, 0]) gusset(0);
        motor_mount();
        feeder_plate();
    }
}

// ── ghosted bought / rotating parts (not printed) ───────────────────

module ghost_rotating() {
    // spindle along X through both bearings
    color("0.7 0.7 0.75", 0.85)
        translate([FRONT_X - 18, 0, AXIS_Z]) rotate([0, 90, 0]) difference() {
            cylinder(d = BRG_BORE, h = REAR_X + 26);
            translate([0,0,-1]) cylinder(d = SPINDLE_BORE, h = REAR_X + 28);
        }
    // 608 bearings
    for (x = [FRONT_X, REAR_X]) color("0.3 0.3 0.32", 0.6)
        translate([x - BRG_W/2, 0, AXIS_Z]) rotate([0, 90, 0])
            difference() { cylinder(d = BRG_OD, h = BRG_W); cylinder(d = BRG_BORE, h = BRG_W+1, center=true); }
    // spindle pulley (60T)
    color("0.15 0.16 0.18", 0.9)
        translate([PULLEY_X - SP_PULLEY_W/2, 0, AXIS_Z]) rotate([0, 90, 0])
            cylinder(d = SP_PULLEY_OD, h = SP_PULLEY_W);
    // bending head on front flange
    color("0.9 0.55 0.15", 0.9)
        translate([FRONT_X - 30, 0, AXIS_Z]) rotate([0, 90, 0]) cylinder(d = HEAD_OD, h = HEAD_H);
}

module ghost_stepper() {
    color("0.25 0.27 0.30", 0.85) {
        translate([ST_X - NEMA_LEN, 0, ST_Z]) rotate([0, 90, 0])
            cube([NEMA, NEMA, NEMA_LEN]);          // body (centred-ish)
        translate([ST_X, 0, ST_Z]) rotate([0, 90, 0]) cylinder(d = ST_PULLEY_OD, h = 8);
    }
    // belt loop (rough band between the two pulleys)
    color("0.05 0.05 0.05", 0.7)
        translate([PULLEY_X, 0, (AXIS_Z + ST_Z)/2]) cube([4, SP_PULLEY_OD, BELT_CD], center = true);
}

module ghost_feeder() {
    color("0.18 0.19 0.22", 0.5)
        translate([FEEDER_X + 8, 0, AXIS_Z]) rotate([0,90,0])
            translate([0,0,0]) cube([FEEDER_H, FEEDER_T, FEEDER_L], center = true);
}

bracket();
if (show == "assembly") { ghost_rotating(); ghost_stepper(); ghost_feeder(); }
