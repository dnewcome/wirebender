$fn = 30;
hole = 4; // flange mounting hole
width = 43; // overall width
spacing = 39 - hole; // flange hole spacing on center
height = 11;

tube_offset = 5;
tube_dia = 25.4 * 0.25;
flange_dia = 10;

fudge = 2;


module m4_insert(h) {
        cylinder(d = 4, h = h);
     
}

difference() {
    difference() {
        cylinder(h = height, d = width);
        translate([spacing/2, 0]) cylinder(h = height, d = hole);
        translate([-spacing/2, 0]) cylinder(h = height, d = hole);
    }
    translate([-width/2, 3]) cube([width,width,width]);
    translate([tube_dia, width/2, height/2]) rotate([90, 0, 0]) cylinder(d = tube_dia, h = width);
    translate([0, 6]) cylinder(d = flange_dia + fudge, h = height);
    translate([0,-width/4, height/2]) rotate([0, 90, 0]) m4_insert(h = width);
}
