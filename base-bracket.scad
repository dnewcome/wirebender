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
BASE_X0  = -14;  BASE_X1 = 219;                          // head end -> past feeder
BASE_W   = 80;

// ── feeder (1KGSSJ-B): mounts FLAT on TOP of the base ───────────────
// The feeder motor/gearbox drops through a hole in the base; the body bolts to
// the top with vertical screws; the tension knob swings up over the top. Bolt
// pattern is an isosceles trapezoid (major axis = wire axis X), parallel hole
// spacings 47.3 & 27.3 mm, 104.5 mm apart, mirrored about the major axis.
FEEDER_X   = 150;                       // centre of the feeder footprint along X
FEEDER_L   = 122;  FEEDER_W = 85;  FEEDER_T = 30;
FEEDER_HOLE = 5.3;
FEEDER_BOLT_SPAN = 104.5;               // trapezoid spacing along the major axis (X)
FEEDER_BOLT_W1   = 47.3;                // wide-end hole spacing (across, Y) — motor end
FEEDER_BOLT_W2   = 27.3;                // narrow-end hole spacing (Y) — output/head end
FEEDER_MOTOR_HOLE = 60;                 // large cylindrical pocket for the motor dropping in
FEEDER_MOTOR_X = FEEDER_X + 18;         // motor offset toward the wide (47.3) end
FEEDER_WIRE_Z = 16;                     // [VERIFY] feeder wire-path height above base top
// trapezoid corners on the base top, [x, y]: narrow (27.3) end toward the head (-X),
// wide (47.3) + motor end toward +X. Mirrored about the major (X) axis.
FEEDER_HOLES = [
    [FEEDER_X - FEEDER_BOLT_SPAN/2,  FEEDER_BOLT_W2/2],
    [FEEDER_X - FEEDER_BOLT_SPAN/2, -FEEDER_BOLT_W2/2],
    [FEEDER_X + FEEDER_BOLT_SPAN/2,  FEEDER_BOLT_W1/2],
    [FEEDER_X + FEEDER_BOLT_SPAN/2, -FEEDER_BOLT_W1/2],
];

// ── bending head (Ø43, mounts on spindle front flange) ──────────────
// The head is a HORIZONTAL disk (motor shaft vertical) that the wire passes
// through sideways, so it reaches ±HEAD_OD/2 ALONG the wire axis. The front
// bearing must sit clear of that -> set the head this far in front of it.
HEAD_OD = 43;  HEAD_H = 11;  HEAD_BOLT = 35;             // M4 at 35mm spacing
HEAD_SETBACK = 35;                                       // head centre ahead of front bearing
HEAD_X = FRONT_X - HEAD_SETBACK;

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

module base() {
    L = BASE_X1 - BASE_X0;
    difference() {
        translate([BASE_X0 + L/2, 0, -BASE_THK/2]) cube([L, BASE_W, BASE_THK], center = true);
        // benchtop mounting holes (corners, clear of the feeder footprint)
        for (x = [BASE_X0 + 12, REAR_X + 18, BASE_X1 - 12])
            for (y = [-BASE_W/2 + 8, BASE_W/2 - 8])
                translate([x, y, -BASE_THK - 1]) cylinder(d = 5, h = BASE_THK + 2);
        // feeder: large cylindrical pocket for the motor to drop through (offset
        // to the wide/47.3 end), plus the trapezoidal vertical mounting holes
        translate([FEEDER_MOTOR_X, 0, -BASE_THK - 1])
            cylinder(d = FEEDER_MOTOR_HOLE, h = BASE_THK + 2);
        for (h = FEEDER_HOLES)
            translate([h[0], h[1], -BASE_THK - 1]) cylinder(d = FEEDER_HOLE, h = BASE_THK + 2);
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
    }
}

// ── ghosted bought / rotating parts (not printed) ───────────────────

module ghost_rotating() {
    // spindle along X from the head connection through both bearings
    color("0.7 0.7 0.75", 0.85)
        translate([HEAD_X, 0, AXIS_Z]) rotate([0, 90, 0]) difference() {
            cylinder(d = BRG_BORE, h = REAR_X - HEAD_X + 12);
            translate([0,0,-1]) cylinder(d = SPINDLE_BORE, h = REAR_X - HEAD_X + 14);
        }
    // 608 bearings
    for (x = [FRONT_X, REAR_X]) color("0.3 0.3 0.32", 0.6)
        translate([x - BRG_W/2, 0, AXIS_Z]) rotate([0, 90, 0])
            difference() { cylinder(d = BRG_OD, h = BRG_W); cylinder(d = BRG_BORE, h = BRG_W+1, center=true); }
    // spindle pulley (60T)
    color("0.15 0.16 0.18", 0.9)
        translate([PULLEY_X - SP_PULLEY_W/2, 0, AXIS_Z]) rotate([0, 90, 0])
            cylinder(d = SP_PULLEY_OD, h = SP_PULLEY_W);
    // bending head: HORIZONTAL disk (motor shaft vertical), wire through it along X
    color("0.20 0.55 0.85", 0.9)
        translate([HEAD_X, 0, AXIS_Z - HEAD_H/2]) cylinder(d = HEAD_OD, h = HEAD_H);
    color("0.9 0.55 0.15", 0.95)                 // bending flange on top
        translate([HEAD_X, 0, AXIS_Z + HEAD_H/2]) cylinder(d = 22, h = 2);
    color("0.3 0.32 0.36", 0.9)                  // bending motor hanging below
        translate([HEAD_X, 0, AXIS_Z - HEAD_H/2 - 20]) cylinder(d = 25, h = 20);
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
    // body flat on the base top (output/narrow end toward -X / the head)
    color("0.18 0.19 0.22", 0.45)
        translate([FEEDER_X, 0, FEEDER_T/2]) cube([FEEDER_L, FEEDER_W, FEEDER_T], center = true);
    // motor/gearbox dropping through the base pocket (offset to the wide end)
    color("0.30 0.32 0.36", 0.6)
        translate([FEEDER_MOTOR_X, 0, -BASE_THK - 44]) cylinder(d = 53, h = 48);
    // tension knob swinging up over the top
    color("0.08 0.08 0.08", 0.7)
        translate([FEEDER_X - 42, 0, FEEDER_T]) cylinder(d = 22, h = 14);
}

bracket();
if (show == "assembly") { ghost_rotating(); ghost_stepper(); ghost_feeder(); }
