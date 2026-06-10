// spindle.scad
// The rotating spindle (Axis 2 — bend direction). It rides in the two 608
// bearings of base-bracket.scad, carries the 60T GT2 pulley, and presents a
// front flange that the bending-head adapter bolts to. The wire passes through
// the Ø4 bore (PTFE-lined). Print axis-vertical (as modelled) so the bearing
// journals come out round — an 8 mm metal tube/rod bored 4 mm is better for the
// journals if you have it; this models the full part for fit/reference.
//
// Frame: Z = spindle axis, z = 0 at the front-flange face (head end), increasing
// toward the rear (feeder). Z positions are derived from base-bracket.scad —
// keep the shared block in sync.

show = "spindle";        // "spindle" | "assembly"
$fn = 96;

// ── shared with base-bracket.scad (keep in sync) ────────────────────
BRG_OD = 22;  BRG_W = 7;  BRG_BORE = 8;     // 608 bearing
BORE   = 4;                                  // wire bore
FLANGE_X = -19;                              // bracket X of the front-flange face
FRONT_X  = 0;   REAR_X = 56;                 // bracket X of the bearing centres
PULLEY_X = 28;  PULLEY_OD = 40;  PULLEY_W = 8;   // 60T GT2 pulley

// ── derived axial stations (z = bracket_X − FLANGE_X) ───────────────
Z_FRONT_BRG = FRONT_X - FLANGE_X;            // 19
Z_PULLEY    = PULLEY_X - FLANGE_X;           // 47
Z_REAR_BRG  = REAR_X  - FLANGE_X;            // 75
Z_REAR_END  = Z_REAR_BRG + 14;               // 89  (rear stub for the wire entry)

// ── spindle dimensions ──────────────────────────────────────────────
JOURNAL   = BRG_BORE;        // 8 mm — bearing bore / pulley seat
SHOULDER  = 11;              // shoulder OD that axially locates the bearings
FLANGE_OD = 32;  FLANGE_T = 6;
FLANGE_BOLT = 22;            // head-adapter bolt circle
FLANGE_HOLE = 3.2;          // M3 clearance
FLAT_DEPTH  = 0.6;          // set-screw flat on the pulley seat

module seg(d, z0, z1) translate([0, 0, z0]) cylinder(d = d, h = z1 - z0);

module spindle() {
    difference() {
        union() {
            seg(FLANGE_OD, 0, FLANGE_T);                                    // front flange
            seg(SHOULDER,  FLANGE_T, Z_FRONT_BRG - BRG_W/2);                // front shoulder
            seg(JOURNAL,   Z_FRONT_BRG - BRG_W/2, Z_FRONT_BRG + BRG_W/2);   // front journal
            seg(SHOULDER,  Z_FRONT_BRG + BRG_W/2, Z_PULLEY - PULLEY_W/2);   // mid
            seg(JOURNAL,   Z_PULLEY - PULLEY_W/2, Z_PULLEY + PULLEY_W/2);   // pulley seat
            seg(SHOULDER,  Z_PULLEY + PULLEY_W/2, Z_REAR_BRG - BRG_W/2);    // mid
            seg(JOURNAL,   Z_REAR_BRG - BRG_W/2, Z_REAR_BRG + BRG_W/2);     // rear journal
            seg(JOURNAL,   Z_REAR_BRG + BRG_W/2, Z_REAR_END);              // rear stub
        }
        // wire bore through the whole length
        seg(BORE, -1, Z_REAR_END + 1);
        // head-adapter bolt holes in the flange
        for (i = [0:3]) rotate([0, 0, 90*i + 45])
            translate([FLANGE_BOLT/2, 0, -1]) cylinder(d = FLANGE_HOLE, h = FLANGE_T + 2);
        // set-screw flat on the pulley seat (so the GT2 pulley grub screw bites)
        translate([JOURNAL/2 - FLAT_DEPTH, -PULLEY_W/2 - 1, Z_PULLEY - PULLEY_W/2])
            cube([5, PULLEY_W + 2, PULLEY_W]);
    }
}

// ── reference ghosts (bearings + pulley at their seats) ─────────────
module ghost() {
    for (z = [Z_FRONT_BRG, Z_REAR_BRG]) color("0.3 0.3 0.32", 0.5)
        translate([0, 0, z - BRG_W/2]) difference() {
            cylinder(d = BRG_OD, h = BRG_W);
            translate([0, 0, -1]) cylinder(d = BRG_BORE, h = BRG_W + 2);
        }
    color("0.15 0.16 0.18", 0.55)
        translate([0, 0, Z_PULLEY - PULLEY_W/2]) difference() {
            cylinder(d = PULLEY_OD, h = PULLEY_W);
            translate([0, 0, -1]) cylinder(d = JOURNAL, h = PULLEY_W + 2);
        }
}

color([0.7, 0.72, 0.76]) spindle();
if (show == "assembly") ghost();
