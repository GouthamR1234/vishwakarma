/*
  SPINAL POSTURE SENSOR  -  parametric OpenSCAD model
  ---------------------------------------------------------------------------
  Two thin, low-profile pods joined by a flat flexible spine ribbon.

  LOWER POD (lumbar / sacrum)      UPPER POD (upper thoracic, ~T3)
    - XIAO ESP32-C3                  - IMU upper (MPU6050 @0x69, AD0->3V3)
    - IMU lower (MPU6050 @0x68)      - 3.7 V LiPo (502030 class, 5x20x30)
    - DRV2605L driver
    - Coin ERM motor (against skin)
    - SPST slide switch (side wall)
    - Pushbutton (plunger through lid)
    - LED (side-emitting) + 220 ohm resistor

  SPINE RIBBON (print in TPU): carries 6 thin wires between the pods
    3V3, GND, SDA, SCL  (to upper IMU)   +   BAT+, BAT-  (to upper LiPo)
    BAT+ returns through the slide switch in the lower pod to XIAO BAT+.

  Nets / pins used (no pin used twice, INT pins left open):
    SDA = D4 (GPIO6)   SCL = D5 (GPIO7)   Button = D1 (GPIO3) -> GND
    LED = D2 (GPIO4) -> 220R -> LED anode, cathode -> GND
    IMU lower AD0 -> GND (0x68)   IMU upper AD0 -> 3V3 (0x69)   DRV2605L EN -> 3V3

  Coordinates: z = 0 is the SKIN SIDE, +y runs UP the spine, x is left/right.
  Mount both IMUs with the same axis orientation (X along pod width).

  view: "assembly" | "exploded" | "print_plate"
  Print: pods + lids in PETG/PLA (0.16-0.2 mm), ribbon + plunger in TPU 95A.
  Lids sit in a rebate flush with the top; glue or tape them shut.
*/

$fn = 48;

view       = "assembly";   // "assembly", "exploded", "print_plate"
show_parts = true;         // translucent electronics
section    = false;        // cut away x>0 half to see inside
spine_len  = 200;          // mm between pod centres (adjust to the wearer)

// ------------------------------------------------------------- shell params
wall = 1.6;  floor_t = 1.2;  lid_t = 1.2;  cav_d = 5.4;
total_t = floor_t + cav_d + lid_t;          // 7.8 mm overall thickness
corner_r = 6;  skin_chamfer = 1.2;  top_round = 0.8;
ledge = 0.8;  lid_clear = 0.15;
standoff = 0.8;            // lifts XIAO so BAT pads/wires fit underneath
gap = 2.0;  margin = 1.0;  anchor_zone = 3.2;

// straps
strap_t = 2.0;  strap_clear = 0.6;
strap_w_low = 20;  strap_w_up = 16;
ear_t = 2.4;  ear_len = 10;

// spine ribbon
ribbon_w = 14;  ribbon_t = 2.6;
fl_w = 18;  fl_l = 1.2;                 // anchor flange (wider than wall slot)
tunnel_w = 8;  tunnel_h = 1.2;          // wire tunnel
slot_w = ribbon_w + 0.4;  slot_h = ribbon_t + 0.3;

// ------------------------------------------------------------ part sizes
xiao  = [21.0, 17.8, 4.0];
imu   = [21.2, 16.4, 3.2];   // GY-521, header removed / low profile
drv   = [17.8, 17.8, 3.0];
motor_d = 10;  motor_h = 2.7;
lipo  = [30, 20, 5];
sw    = [8.5, 3.6, 3.5];
btn   = [6, 6, 3.5];
led_d = 3;  led_l = 6;
res   = [3.2, 1.6, 0.8];

// ------------------------------------------------------------ lower layout
low_iw = 2*margin + xiao.x + imu.x + drv.x + 2*gap;
low_ih = 34;
low_ow = low_iw + 2*wall;  low_oh = low_ih + 2*wall;

top_y  = low_ih/2 - anchor_zone - xiao.y/2;
xiao_x = -low_iw/2 + margin + xiao.x/2;
imu_x  = xiao_x + xiao.x/2 + gap + imu.x/2;
drv_x  = imu_x + imu.x/2 + gap + drv.x/2;

mot_x = -low_iw/2 + margin + motor_d/2;
mot_y = -low_ih/2 + margin + motor_d/2;
sw_x  = mot_x + motor_d/2 + gap + sw.x/2;
sw_y  = -low_ih/2 + sw.y/2;
btn_x = sw_x + sw.x/2 + gap + btn.x/2;
btn_y = -low_ih/2 + margin + btn.y/2;
led_x = btn_x + btn.x/2 + gap + led_d/2;
led_z = floor_t + led_d/2 + 0.3;
res_x = led_x + led_d/2 + gap + res.x/2;
res_y = -low_ih/2 + margin + res.y/2;

// ------------------------------------------------------------ upper layout
up_iw = 2*margin + lipo.x + imu.x + gap;
up_ih = lipo.y + 2*margin + anchor_zone;
up_ow = up_iw + 2*wall;  up_oh = up_ih + 2*wall;
row_y = anchor_zone/2;
lipo_x  = -up_iw/2 + margin + lipo.x/2;
imu_u_x = lipo_x + lipo.x/2 + gap + imu.x/2;

// ribbon extents (world y)
yA = low_oh/2 - wall - 0.1 - fl_l;
yB = spine_len - up_oh/2 + wall + 0.1 + fl_l;

// ================================================================ helpers
module rrect2d(w, h, r) { offset(r = r) square([w - 2*r, h - 2*r], center = true); }

module pod_outer(w, h, t, r = corner_r) {
  hull() for (sx = [-1, 1], sy = [-1, 1])
    translate([sx*(w/2 - r), sy*(h/2 - r), 0]) {
      cylinder(r1 = r - skin_chamfer, r2 = r, h = skin_chamfer);
      translate([0, 0, skin_chamfer]) cylinder(r = r, h = t - skin_chamfer - top_round);
      translate([0, 0, t - top_round]) cylinder(r1 = r, r2 = r - top_round, h = top_round);
    }
}

module ear2d(ow, eh) {
  hull() {
    translate([ow/2 - 1.2, -eh/2]) square([1.2, eh]);
    for (s = [-1, 1]) translate([ow/2 + ear_len - 4, s*(eh/2 - 4)]) circle(r = 4);
  }
}

module ear(ow, strap_w) {
  eh = strap_w + 10;
  difference() {
    hull() {
      linear_extrude(0.4) offset(delta = -0.8) ear2d(ow, eh);
      translate([0, 0, 0.4]) linear_extrude(ear_t - 0.8) ear2d(ow, eh);
      translate([0, 0, ear_t - 0.4]) linear_extrude(0.4) offset(delta = -0.4) ear2d(ow, eh);
    }
    translate([ow/2 + ear_len - 4.5, 0, -1])
      linear_extrude(ear_t + 2) square([strap_t + strap_clear, strap_w], center = true);
  }
}

// ribbon slot through the wall + open-bottom pocket for the anchor flange
module ribbon_cuts(oh, ay) {
  if (ay > 0) {
    translate([-slot_w/2, oh/2 - wall - 0.01, -1]) cube([slot_w, wall + 1.01, slot_h + 1]);
    translate([-(fl_w + 0.4)/2, oh/2 - wall - fl_l - 0.4, -1]) cube([fl_w + 0.4, fl_l + 0.41, slot_h + 1]);
  } else {
    translate([-slot_w/2, -oh/2 - 1, -1]) cube([slot_w, wall + 1.01, slot_h + 1]);
    translate([-(fl_w + 0.4)/2, -oh/2 + wall - 0.01, -1]) cube([fl_w + 0.4, fl_l + 0.41, slot_h + 1]);
  }
}

module base_shell(iw, ih, strap_w, ay) {
  ow = iw + 2*wall;  oh = ih + 2*wall;
  difference() {
    union() {
      pod_outer(ow, oh, total_t);
      for (s = [-1, 1]) scale([s, 1, 1]) ear(ow, strap_w);
    }
    translate([0, 0, floor_t]) linear_extrude(cav_d + lid_t + 1) rrect2d(iw, ih, corner_r - wall);
    translate([0, 0, total_t - lid_t]) linear_extrude(lid_t + 1)
      rrect2d(iw + 2*ledge, ih + 2*ledge, corner_r - wall + ledge);
    ribbon_cuts(oh, ay);
    children();
  }
}

module lid_plate(iw, ih) {
  linear_extrude(lid_t)
    rrect2d(iw + 2*ledge - 2*lid_clear, ih + 2*ledge - 2*lid_clear, corner_r - wall + ledge - lid_clear);
}

// corner brackets that locate a component on the floor
module holder(size, h = 1.6, t = 0.7, c = 0.2) {
  linear_extrude(h) difference() {
    square([size.x + 2*(c + t), size.y + 2*(c + t)], center = true);
    square([size.x + 2*c, size.y + 2*c], center = true);
    square([size.x + 2*(c + t) + 1, size.y*0.4], center = true);
    square([size.x*0.4, size.y + 2*(c + t) + 1], center = true);
  }
}

// ============================================================== LOWER POD
module base_lower() {
  union() {
    base_shell(low_iw, low_ih, strap_w_low, 1) {
      // USB-C opening (-x wall)
      translate([-low_ow/2 - 1, top_y - 4.9, floor_t + standoff - 0.3])
        cube([wall + 2.5, 9.8, 3.9]);
      // slide switch window (-y wall)
      translate([sw_x - 3.5, -low_oh/2 - 1, floor_t + 0.4])
        cube([7, wall + 2, 2.6]);
      // LED window (-y wall)
      translate([led_x, -low_oh/2 - 1, led_z]) rotate([-90, 0, 0])
        cylinder(d = led_d + 0.3, h = wall + 2);
    }
    // locating brackets
    translate([xiao_x, top_y, floor_t - 0.01]) holder([xiao.x, xiao.y]);
    translate([imu_x,  top_y, floor_t - 0.01]) holder([imu.x, imu.y]);
    translate([drv_x,  top_y, floor_t - 0.01]) holder([drv.x, drv.y]);
    translate([mot_x,  mot_y, floor_t - 0.01]) holder([motor_d, motor_d]);
    translate([btn_x,  btn_y, floor_t - 0.01]) holder([btn.x, btn.y]);
    // XIAO standoff rails (leave room for BAT+/BAT- pad wires underneath)
    for (s = [-1, 1]) translate([xiao_x - (xiao.x - 4)/2, top_y + s*7 - 0.75, floor_t - 0.01])
      cube([xiao.x - 4, 1.5, standoff + 0.01]);
  }
}

module lid_lower() {
  difference() {
    lid_plate(low_iw, low_ih);
    translate([btn_x, btn_y, -1]) cylinder(d = 4.8, h = lid_t + 2);
  }
}

module plunger() {      // TPU button cap
  cylinder(d = 7, h = 0.6);
  cylinder(d = 4.4, h = total_t - (floor_t + btn.z) - 0.1);
}

// ============================================================== UPPER POD
module base_upper() {
  union() {
    base_shell(up_iw, up_ih, strap_w_up, -1);
    translate([lipo_x,  row_y, floor_t - 0.01]) holder([lipo.x, lipo.y]);
    translate([imu_u_x, row_y, floor_t - 0.01]) holder([imu.x, imu.y]);
  }
}
module lid_upper() { lid_plate(up_iw, up_ih); }

// ============================================================ SPINE RIBBON
module ribbon() {
  difference() {
    union() {
      translate([-ribbon_w/2, yA, 0]) cube([ribbon_w, yB - yA, ribbon_t]);
      translate([-fl_w/2, yA, 0]) cube([fl_w, fl_l, ribbon_t]);
      translate([-fl_w/2, yB - fl_l, 0]) cube([fl_w, fl_l, ribbon_t]);
    }
    translate([-tunnel_w/2, yA - 1, (ribbon_t - tunnel_h)/2]) cube([tunnel_w, yB - yA + 2, tunnel_h]);
  }
}

// ============================================================ GHOST PARTS
module gbox(size, c, a = 0.6) { color(c, a) translate([-size.x/2, -size.y/2, 0]) cube(size); }

module ghost_lower() {
  // XIAO ESP32-C3 + USB-C
  translate([xiao_x, top_y, floor_t + standoff]) color("#2f6fdd", 0.65) {
    translate([-xiao.x/2, -xiao.y/2, 0]) cube([xiao.x, xiao.y, 1]);
    translate([-xiao.x/2 - 0.3, -4.5, 1]) cube([7.5, 9, 3.0]);
    translate([xiao.x/2 - 9.5, -4, 1]) cube([8, 8, 1.5]);
  }
  translate([imu_x, top_y, floor_t]) gbox(imu, "#18a058");           // IMU lower
  translate([drv_x, top_y, floor_t]) gbox(drv, "#c0392b");           // DRV2605L
  translate([mot_x, mot_y, floor_t]) color("#888888", 0.7) cylinder(d = motor_d, h = motor_h);
  translate([sw_x, sw_y, floor_t]) gbox(sw, "#f39c12");
  translate([btn_x, btn_y, floor_t]) gbox(btn, "#333333");
  translate([led_x, -low_ih/2, led_z]) rotate([-90, 0, 0]) color("#ff3030", 0.8) cylinder(d = led_d, h = led_l);
  translate([res_x, res_y, floor_t]) gbox(res, "#d2b48c", 0.9);       // 220 ohm
}

module ghost_upper() {
  translate([imu_u_x, row_y, floor_t]) gbox(imu, "#18a058");         // IMU upper
  translate([lipo_x,  row_y, floor_t]) gbox(lipo, "#8e44ad");        // LiPo
}

// ============================================================== ASSEMBLY
module maybe_cut() {
  if (section) difference() { children(); translate([0, -100, -50]) cube([300, 600, 200]); }
  else children();
}

module assembly() {
  e = (view == "exploded") ? 1 : 0;
  maybe_cut() {
    // lower pod
    base_lower();
    if (show_parts) translate([0, 0, e*6]) ghost_lower();
    translate([0, 0, total_t - lid_t + e*16]) lid_lower();
    color("#222") translate([btn_x, btn_y, floor_t + btn.z + e*10]) plunger();
    // upper pod
    translate([0, spine_len, 0]) {
      base_upper();
      if (show_parts) translate([0, 0, e*6]) ghost_upper();
      translate([0, 0, total_t - lid_t + e*16]) lid_upper();
    }
    // spine ribbon (flat, flush with skin)
    color("#e8a33d") ribbon();
  }
}

module print_plate() {
  base_lower();
  translate([0, -48, 0]) lid_lower();
  translate([52, -48, 0]) plunger();
  translate([110, 12, 0]) base_upper();
  translate([110, -26, 0]) lid_upper();
  translate([-45, 62, 0]) rotate([0, 0, -90]) translate([0, -yA, 0]) ribbon();
}

if (view == "print_plate") print_plate(); else assembly();
