// stepper-adapter.scad
// Adapter to mount a NEMA17 stepper in place of the brushed can motor on the
// 1KGSSJ-B MIG wire-feed gearbox, KEEPING the existing plastic gear reduction.
//
// Concept: a register boss drops into the round pocket the can motor sat in.
// Because the can motor's shaft is concentric with its can, the boss auto-centers
// the NEMA17 shaft on the original motor axis -> the pinion meshes with the first
// gear at the original center distance, no fussy hole alignment required. The
// reused/printed pinion (see feeder-gear.scad: 12T, module 0.5, 5mm bore) goes on
// the NEMA17 shaft.
//
// Origin = NEMA17 shaft axis. +Z points from the stepper into the gearbox.
//   z < 0           : NEMA17 body (not drawn)
//   z = 0 .. plate_t: adapter plate (NEMA17 bolts here; pilot recess on z=0 face)
//   z = plate_t ..  : register boss that enters the gearbox motor pocket
//
// >>> Values tagged [MEASURE] are placeholders until measured off the real unit. <<<

$fn = 64;

// ───────── Measured parameters — FILL IN from the actual feeder ─────────
pocket_dia      = 35;   // [MEASURE] dia of the round pocket the can motor registered in
pocket_depth    = 6;    // [MEASURE] usable depth of that pocket (boss length)
shaft_seat_z    = 10;   // [MEASURE] axial dist from gearbox face to where the pinion
                        //           must sit, so it meshes at the right depth.
                        //           (sets how far the NEMA17 shaft reaches in)

// Fastening to the housing. Either reuse existing screw bosses around the pocket
// (set their [x,y] here, relative to the motor/shaft axis) or omit and retain the
// boss with a clamp/set screw. Leave list empty [] to skip ears.
mount_holes     = [ [24, 0], [-24, 0] ];  // [MEASURE] housing screw-hole centers
mount_hole_dia  = 3.4;  // [MEASURE] clearance for the housing screws (M3 -> 3.4)
ear_t           = 4;    // thickness of the mounting ears

// ───────── NEMA17 standard dimensions (no need to measure) ─────────
nema_body       = 42.3;  // body square
nema_bolt_sq    = 31.0;  // bolt circle is a 31mm square
nema_bolt_dia   = 3.4;   // M3 clearance
nema_pilot_dia  = 22.0;  // raised pilot boss on the motor face
nema_pilot_h    = 2.2;   // pilot height -> recess depth in the plate
nema_shaft_dia  = 5.0;

// ───────── Adapter geometry ─────────
plate_t         = 6;     // adapter plate thickness
plate_corner_r  = 4;     // rounded corners
clear           = 0.3;   // general slip-fit clearance
pinion_od       = 7.0;   // 12T @ m0.5 -> ~7mm OD; bore clears shaft + pinion

// ─────────────────────────────────────────────────────────────────────────

module rounded_square_plate(size, t, r) {
    linear_extrude(t)
        offset(r) offset(-r)
            square([size, size], center = true);
}

module nema17_bolt_pattern(d, depth) {
    s = nema_bolt_sq / 2;
    for (x = [-s, s], y = [-s, s])
        translate([x, y, -1]) cylinder(d = d, h = depth + 2);
}

module mounting_ears() {
    for (p = mount_holes) {
        // a stubby arm from the boss out to each screw hole
        hull() {
            cylinder(d = pocket_dia * 0.5, h = ear_t);
            translate([p[0], p[1], 0]) cylinder(d = mount_hole_dia + 6, h = ear_t);
        }
    }
}

module adapter() {
    difference() {
        union() {
            // main plate
            translate([0, 0, 0]) rounded_square_plate(nema_body, plate_t, plate_corner_r);
            // register boss into the gearbox pocket
            translate([0, 0, plate_t])
                cylinder(d = pocket_dia - clear, h = pocket_depth);
            // mounting ears sit on the gearbox face (top of plate)
            if (len(mount_holes) > 0)
                translate([0, 0, plate_t]) mounting_ears();
        }

        // NEMA17 pilot recess on the motor side (z=0 face)
        translate([0, 0, -0.01]) cylinder(d = nema_pilot_dia + clear, h = nema_pilot_h);

        // NEMA17 bolt holes
        nema17_bolt_pattern(nema_bolt_dia, plate_t);

        // central through-bore: shaft + pinion clearance, full depth
        translate([0, 0, -1])
            cylinder(d = pinion_od + 2*clear, h = plate_t + pocket_depth + 2);

        // housing fastener holes through the ears
        for (p = mount_holes)
            translate([p[0], p[1], plate_t - 1])
                cylinder(d = mount_hole_dia, h = ear_t + 2);
    }
}

adapter();

// Reference ghost of the NEMA17 shaft + pinion seat (not exported; comment out for STL)
%translate([0, 0, plate_t + pocket_depth - shaft_seat_z + pocket_depth])
    cylinder(d = nema_shaft_dia, h = shaft_seat_z);
