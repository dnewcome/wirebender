// massing.scad — ROUGH massing of the rotating-head architecture (blocks only)
// Goal: eyeball proportions, motor swing clearance, and balance before detailing.
//
//   Rotation axis = X = the wire axis. The HEAD rotates about X (Axis 2). On the
//   head: the bender drivetrain (tiny stepper + servo gearbox -> layshaft -> mandrel)
//   on one side, and the ROTATION motor on the opposite side (counterweight + drive).
//   The rotation motor's pinion meshes a FIXED gear printed into the base; the feed
//   tube spins passively in 2 base bearings. Both motors hang off the FRONT so they
//   clear the base when the head swings.
//
//   front (head) ── -X            +X ── (feeder)
//
// rot = head rotation angle (deg) to check swing clearance.

$fn = 40;
rot = 0;

// ── wire / feed tube ────────────────────────────────────────────────
TUBE_D = 8;  TUBE_X = [-15, 150];

// ── base (fixed) ────────────────────────────────────────────────────
FIXED_GEAR_X = 30;  FIXED_GEAR_D = 46;  FIXED_GEAR_W = 8;   // printed into base
BRG_X = [42, 95];   BRG_OD = 22;  BRG_W = 7;                // 2 passive bearings
FEEDER_X = 165;     FEEDER = [122, 85, 30];                 // flat on base (from before)
BASE_W = 80;  BASE_TH = 6;  BASE_TOP_Z = -34;               // wire axis ~34mm above base
BASE_X0 = FIXED_GEAR_X + 2;                                 // front edge: head cantilevers clear of this
BASE_X1 = FEEDER_X + FEEDER[0]/2 + 10;
SWING_R = 54;                                               // head swing envelope radius
HEAD_X = [-25, FIXED_GEAR_X];                              // head extent along X (forward of base)

// ── rotating head (everything here turns about X) ───────────────────
// bender drivetrain: gutted servo (gearbox) + tiny stepper, on +R side
BENDER = [42, 22, 40];          // rough servo-ish body L(x) x W x H
BENDER_R = 30;                  // radius of its CoM from the wire axis
MANDREL_OFFSET = 2.8;           // wire tangent to the mandrel
MANDREL_D = 4;

// rotation motor: axis ∥ X, pinion at FIXED_GEAR_X meshes the fixed gear
ROTMOT = [40, 24, 24];          // rough geared-stepper body L(x) x W x H
ROT_PINION_D = 18;
ROT_R = FIXED_GEAR_D/2 + ROT_PINION_D/2;                // mesh offset from axis (~32)

module tube() color([0.6,0.62,0.66])
    translate([TUBE_X[0],0,0]) rotate([0,90,0]) cylinder(d=TUBE_D, h=TUBE_X[1]-TUBE_X[0]);

module base() {
    // base plate — front edge at BASE_X0, behind the head's swing
    color([0.30,0.32,0.36])
        translate([(BASE_X0+BASE_X1)/2, 0, BASE_TOP_Z - BASE_TH/2])
            cube([BASE_X1-BASE_X0, BASE_W, BASE_TH], center=true);
    // fixed gear printed into the base (ring, bore for the tube)
    color([0.20,0.55,0.85])
        translate([FIXED_GEAR_X - FIXED_GEAR_W, 0, 0]) rotate([0,90,0])
            difference(){ cylinder(d=FIXED_GEAR_D, h=FIXED_GEAR_W); translate([0,0,-1]) cylinder(d=TUBE_D+3, h=FIXED_GEAR_W+2); }
    // a wall tying the fixed gear down to the base
    color([0.30,0.32,0.36])
        translate([FIXED_GEAR_X-FIXED_GEAR_W, 0, (BASE_TOP_Z + (-FIXED_GEAR_D/2))/2])
            cube([FIXED_GEAR_W, FIXED_GEAR_D, abs(BASE_TOP_Z)+FIXED_GEAR_D/2], center=true);
    // 2 passive bearings (uprights)
    for (x=BRG_X) {
        color([0.66,0.68,0.72])
            translate([x-BRG_W/2,0,0]) rotate([0,90,0]) difference(){ cylinder(d=BRG_OD,h=BRG_W); translate([0,0,-1]) cylinder(d=TUBE_D,h=BRG_W+2); }
        color([0.66,0.68,0.72])
            translate([x,0,(BASE_TOP_Z-12)/2]) cube([BRG_W,BRG_OD+6,abs(BASE_TOP_Z)+12],center=true);
    }
    // feeder block (flat on base)
    color([0.18,0.19,0.22])
        translate([FEEDER_X,0,BASE_TOP_Z+FEEDER[2]/2]) cube([FEEDER[0],FEEDER[1],FEEDER[2]],center=true);
}

module head() rotate([rot,0,0]) {           // rotates about X (wire axis)
    // central hub on the tube
    color([0.85,0.55,0.2]) translate([-12,0,0]) rotate([0,90,0]) cylinder(d=16,h=34);
    // bender drivetrain on +R
    color([0.30,0.55,0.85])
        translate([-2, 0, BENDER_R]) cube([BENDER[0],BENDER[1],BENDER[2]],center=true);
    // mandrel (near the wire axis, offset tangent)
    color([0.62,0.64,0.68])
        translate([-14, MANDREL_OFFSET, 0]) rotate([0,90,0]) cylinder(d=MANDREL_D,h=10);
    // rotation motor opposite (-R), axis ∥ X, pinion meshes the fixed gear
    color([0.25,0.27,0.30])
        translate([FIXED_GEAR_X-ROTMOT[0], 0, -ROT_R]) cube([ROTMOT[0],ROTMOT[1],ROTMOT[2]],center=false);
    color([0.85,0.55,0.2])
        translate([FIXED_GEAR_X-FIXED_GEAR_W, 0, -ROT_R]) rotate([0,90,0]) cylinder(d=ROT_PINION_D,h=FIXED_GEAR_W);
}

tube();
base();
head();

// head swing envelope (forward of the base, in open air) — translucent
%color([1, 0.4, 0.4, 0.18])
    translate([HEAD_X[0], 0, 0]) rotate([0, 90, 0]) cylinder(r = SWING_R, h = HEAD_X[1]-HEAD_X[0]);
