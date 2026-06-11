// base.scad — base for the cantilevered rotating-head architecture
// One printed piece. It:
//   - carries a FIXED external gear (the head's rotation pinion meshes its outside
//     and walks the head around it = Axis 2 drive)
//   - journals the passive feed tube/spindle in two 608 bearings
//   - mounts the 1KGSSJ-B feeder flat on top (motor drops through, trapezoid bolts)
//   - leaves the front open so the cantilevered head swings clear
//
// Frame: X = wire axis (head at -X, feeder at +X), Z = up. Wire axis at z=AXIS_Z
// above the base top (z=0). The fixed gear front face is at x=0; everything
// structural is BEHIND it so the head (x<0) swings in open air.

use <MCAD/involute_gears.scad>
show = "assembly";        // "base" | "assembly"
$fn = 64;

// ── wire axis / feed tube ───────────────────────────────────────────
AXIS_Z   = 35;        // wire axis height (clears the fixed gear above the base)
TUBE_BORE = 10;       // clearance for the Ø8 feed tube through fixed bits

// ── fixed gear (external; head pinion meshes the outside) ───────────
FG_MODULE = 1.5;  FG_TEETH = 40;  FG_W = 8;  FG_PA = 20;
FG_X = 0;             // gear front face
HUB_OD = 20;          // central hub behind the gear (stays inside the swing zone)

// ── 608 bearings for the passive feed tube ──────────────────────────
BRG_OD = 22;  BRG_W = 7;  BRG_BORE = 8;  BRG_LIP = 2;
FB_X = 16;  RB_X = 72;            // front, rear bearing X
UP_T = 8;   UP_W = BRG_OD + 14;   // upright thickness, width
UP_H = AXIS_Z + BRG_OD/2 + 6;

// ── feeder (1KGSSJ-B) — flat on top, motor through a pocket ─────────
FEEDER_X = 145;  FEEDER_L = 122;
FEEDER_BOLT_SPAN = 104.5;  FEEDER_BOLT_W1 = 47.3;  FEEDER_BOLT_W2 = 27.3;
FEEDER_HOLE = 5.3;  FEEDER_MOTOR_HOLE = 56;
FEEDER_MOTOR_X = FEEDER_X + FEEDER_BOLT_SPAN/2 - 32;
FEEDER_HOLES = [
    [FEEDER_X - FEEDER_BOLT_SPAN/2,  FEEDER_BOLT_W2/2],   // narrow end toward head
    [FEEDER_X - FEEDER_BOLT_SPAN/2, -FEEDER_BOLT_W2/2],
    [FEEDER_X + FEEDER_BOLT_SPAN/2,  FEEDER_BOLT_W1/2],   // wide/motor end
    [FEEDER_X + FEEDER_BOLT_SPAN/2, -FEEDER_BOLT_W1/2],
];

// ── base plate ──────────────────────────────────────────────────────
BASE_W = 80;  BASE_TH = 6;
BASE_X0 = FG_X + FG_W + 2;                 // front edge: head swings clear of this
BASE_X1 = FEEDER_X + FEEDER_L/2 + 8;

// ─────────────────────────────────────────────────────────────────────

module bore(d, x0, x1) translate([x0, 0, AXIS_Z]) rotate([0, 90, 0]) cylinder(d = d, h = x1 - x0);

module upright(x) {
    difference() {
        translate([x, 0, UP_H/2 - 1]) cube([UP_T, UP_W, UP_H + 2], center = true);
        // 608 seat from the front face + clearance bore through
        translate([x - UP_T/2 - 0.1, 0, AXIS_Z]) rotate([0, 90, 0]) {
            cylinder(d = BRG_OD, h = BRG_W + 0.2);
            cylinder(d = BRG_OD - 2*BRG_LIP, h = UP_T + 1);
        }
    }
}

module fixed_gear_unit() {
    // external gear coaxial with the wire axis, front face at FG_X
    translate([FG_X, 0, AXIS_Z]) rotate([0, 90, 0]) difference() {
        gear(number_of_teeth = FG_TEETH, circular_pitch = FG_MODULE*180, pressure_angle = FG_PA,
             gear_thickness = FG_W, rim_thickness = FG_W, hub_thickness = 0, bore_diameter = 0);
        translate([0, 0, -1]) cylinder(d = TUBE_BORE, h = FG_W + 2);
    }
    // backing annulus tying the gear to the front upright AROUND the bearing seat
    // (the feed tube + 608 are coaxial, so the support can't be a central hub)
    difference() {
        bore(42, FG_X + FG_W - 0.1, FB_X - UP_T/2 + 1);
        bore(BRG_OD + 3, FG_X, FB_X + 1);
    }
}

module base_plate() {
    L = BASE_X1 - BASE_X0;
    difference() {
        translate([BASE_X0 + L/2, 0, -BASE_TH/2]) cube([L, BASE_W, BASE_TH], center = true);
        // feeder motor pocket + trapezoid bolt holes
        translate([FEEDER_MOTOR_X, 0, -BASE_TH - 1]) cylinder(d = FEEDER_MOTOR_HOLE, h = BASE_TH + 2);
        for (h = FEEDER_HOLES) translate([h[0], h[1], -BASE_TH - 1]) cylinder(d = FEEDER_HOLE, h = BASE_TH + 2);
        // benchtop mounting holes (clear of the feeder footprint)
        for (x = [BASE_X0 + 12, RB_X + 16, BASE_X1 - 12])
            for (y = [-BASE_W/2 + 8, BASE_W/2 - 8])
                translate([x, y, -BASE_TH - 1]) cylinder(d = 5, h = BASE_TH + 2);
    }
}

module gusset(x, dir) {
    // triangular rib tying an upright to the base (overlaps both for one solid)
    translate([x, 0, 0]) rotate([90, 0, 0]) linear_extrude(6, center = true)
        polygon([[-1.5, -2], [dir*14, -2], [-1.5, AXIS_Z - BRG_OD/2]]);
}

module base() color([0.62, 0.66, 0.72]) {
    base_plate();
    fixed_gear_unit();
    upright(FB_X);
    upright(RB_X);
    gusset(FB_X + UP_T/2, 1);     // behind the front upright
    gusset(RB_X - UP_T/2, -1);    // in front of the rear upright
}

// reference ghosts: feed tube + the head's rotation pinion meshing the gear
module ghost() {
    color([0.6, 0.62, 0.66, 0.6]) bore(8, FG_X - 25, RB_X + 12);   // feed tube
    // pinion meshing the fixed gear (on the rotating head, shown at one position)
    pinion_pd = 12 * FG_MODULE;
    R = (FG_TEETH*FG_MODULE + pinion_pd)/2;
    color([0.9, 0.55, 0.15, 0.8]) translate([FG_X, R, AXIS_Z]) rotate([0, 90, 0])
        gear(number_of_teeth = 12, circular_pitch = FG_MODULE*180, pressure_angle = FG_PA,
             gear_thickness = FG_W, rim_thickness = FG_W, hub_thickness = 0, bore_diameter = 5);
}

base();
if (show == "assembly") ghost();
