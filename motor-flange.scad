$fn = 50;

flange_thickness = 2;

module setscrew() {
    cylinder(d = 3, h = 10);
}

module flange() {
    f_od = 22;  // overall dia
    bcd = 15;   // bolt circle dia
    f_t = 2;    // flange thickness
    h_id = 3;   // bolt hole ID
    id = 5;     // bore ID
    
    difference() {
        cylinder(d = f_od, h = f_t);
        // bolt circle
        for(i = [0:3]) {
            angle = 360/4*i;
            echo(angle);
            rotate([0, 0, angle]) translate([bcd/2, 0]) cylinder(d = h_id, h = f_t);
        }
        // bore
        cylinder(h=f_t, d = id);
    }
}

module barrel() {
    id = 5;
    od = 8;
    l = 12; // hub barrel length
    difference() {
        cylinder(h = l, d = od);
        cylinder(h = l, d = id);
        translate([0,0, l/2]) rotate([0, 90]) setscrew();
    }
}

flange();
translate([0, 0, flange_thickness]) barrel();