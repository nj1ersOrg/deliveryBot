width = 40;
depth = 20; 
height = 3;
margin = 3;
thickness = 3;
slotsize = 3.5;
fixheight = 30;
nutslot_width = 5.6;
magnet_radius = 3;
board_mounting_hole_radius = 1.3;
sensor_board_width = 22;
sensor_board_height = 11+5.2; 
total_width = sensor_board_width + 4*margin + 2*nutslot_width;
total_height = sensor_board_height + 2*margin;
nutsize = 3.5;
$fn=50;

union() {
    difference() {
        cube([total_width, total_height, thickness]);
        translate([margin, margin, 0]) {
            cube([nutslot_width, total_height-2*margin,0.5*thickness]);
        }
        translate([margin+0.5*nutslot_width-0.5*nutsize, margin,0]) {
            cube([nutsize, total_height-2*margin, 2*thickness]);
        }
        translate([total_width - margin - nutslot_width, margin, 0]) {
            cube([nutslot_width, total_height-2*margin,0.5*thickness]);
        }
        translate([total_width-nutsize-margin-1, margin,0]) {
            cube([nutsize, total_height-2*margin, 2*thickness]);
        }
        translate([0.5*total_width-0.5*sensor_board_width, margin,0]) {
            cube([sensor_board_width, sensor_board_height+20,0.5*thickness]);
        }
        translate([0.5*total_width, 0.5*total_height,0]) {
            cylinder(h=2*thickness,d=6,center=true);
        }
    }
}

/*
union() {
		difference() {
			cube([width, slotsize,fixheight]);
            rotate([0,0,0]) {
			translate([nutslot_width*0.5, 0,0.5*slotsize]) {
				nutslot();
			}}
            rotate([0,0,0]) {
			translate([width-nutslot_width-(0.5*nutslot_width), 0,0.5*slotsize]) {
				nutslot();	
			}
        }
			translate([0.5*width,-0.1,0.5*fixheight]) {
				rotate([90,0,0]) {
					cylinder(h=10,r=magnet_radius, center=true);
				}
			}
		
            translate([0.5*width-0.5*22.5, slotsize+1, slotsize]) {
            rotate([90,0,0]) {
                cube([22.5,30,3]);
                }
            }}
        }
*/
module nutslot() {
    
	translate([(0.5 * nutslot_width) - (0.5 * slotsize), 0, 0]) {
		cube([slotsize, 0.5*slotsize, fixheight - slotsize]);
	}
	translate([0, 0.5*slotsize, 0]) {
		cube([nutslot_width, 0.5*slotsize, fixheight - slotsize]);
	}
}