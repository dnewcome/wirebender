$fn = 50;

bolt_pattern = 30;
diameter = 25;
height = 20;

module flange() {
        
    difference() {
        linear_extrude(2) hull() {
            translate([0, -bolt_pattern/2]) circle(d=6);
            translate([0, bolt_pattern/2]) circle(d=6);
        }
        translate([0, -bolt_pattern/2]) cylinder(d=4, h=2);
        translate([0, bolt_pattern/2]) cylinder(d=4, h=2);
        
    }
}

module shaft() {
    translate([5, 0, -10]) cylinder(h = 10, d = 5);    
}

module body() {
    cylinder(h = height, d = diameter);
}   

module wiring_block() {
    translate([-diameter/2,0, height/2]) cube(center=true, [10, 15, height]);
}

flange();
shaft();
body();
wiring_block();
