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
nutslot_mountbar_offset = 0.5*(mountBarWidth-nutslot_width);

difference() {
    slottedPlate(total_width,depth, height, slotsize);

    translate([3*slotsize+0.5*mountBarWidth,height+0.5*mountBarWidth]) {
        cylinder(h=height*2, d=nutsize,center=true);
    }
    translate([3*slotsize+nutslot_mountbar_offset,height+nutslot_mountbar_offset,0.5*height]) {
        cube([nutslot_width,nutslot_width,0.5*height]);
    }
    translate([total_width*0.5+0.5*magnet_diameter+0.5*mountBarWidth,height+0.5*mountBarWidth]) {
        cylinder(h=height*2, d=nutsize,center=true);
    }
    translate([total_width*0.5+0.5*magnet_diameter+nutslot_mountbar_offset,height+nutslot_mountbar_offset,0.5*height]) {
        cube([nutslot_width,nutslot_width,0.5*height]);
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