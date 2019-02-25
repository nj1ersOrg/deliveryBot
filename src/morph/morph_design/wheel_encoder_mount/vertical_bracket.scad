$fn=50;
width = 40;
depth = 20; 
height = 3;
slotsize = 3.5;
fixheight = 20;
margin = 3;
nutslot_width = 5.6;
sensor_board_width = 22;
sensor_board_height = 11+5.2; 
total_width = sensor_board_width + 4*margin + 2*nutslot_width;
magnet_diameter = 6;
mountBarWidth = 0.5*total_width-0.5*magnet_diameter-3*slotsize;
nutsize = 3;

difference() {
    // Adjustable mounting slots for vertical plate
    slottedPlate(total_width, depth, height, slotsize);
    // Slot which allows alignment nut to pass through
    translate([0.5*total_width-0.5*magnet_diameter,0,0]) {
        cube([magnet_diameter,fixheight-margin, height]);
    }
}

// Support mounting bars to mount the vertical plate to the base
// This way the plate can be removed after the separete vertical plate
// has been aligned properly to make it easy to attach the sensor to the seperate mounting plate
translate([3*slotsize,0,height]) {
    mountBar();
}
translate([total_width*0.5+0.5*magnet_diameter,0,height]) {
    mountBar();
}

module mountBar() {
    difference() {
        cube([mountBarWidth,2*margin,height*3]);
        translate([0.5*mountBarWidth,0.5*mountBarWidth,height*1.5]) {
            rotate([90,0,0]) {
                cylinder(h=4*height,d=nutsize, center=true);
            }
        }
    }  
}

module slottedPlate(plateWidth, plateDepth, plateHeight, slotWidth) {
    difference() {
        cube([plateWidth,plateDepth,plateHeight]);
        translate([slotWidth, plateHeight, 0]) {
            cube([slotWidth,plateDepth-2*plateHeight,plateHeight+1]);
        }
        translate([plateWidth - 2*slotWidth, plateHeight,0]) {
            cube([slotWidth,plateDepth-2*plateHeight,plateHeight+1]);
        }
    }
}

module nutslot() {
	translate([(0.5 * nutslot_width) - (0.5 * slotsize), 0, 0]) {
		cube([slotsize, 0.5*slotsize, fixheight - slotsize]);
	}
	translate([0, 0.5*slotsize, 0]) {
		cube([nutslot_width, 0.5*slotsize, fixheight - slotsize]);
	}
}