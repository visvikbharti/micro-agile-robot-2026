// ============================================================================
// micro-agile-robot-2026 — 3D-printable X-frame for a 33 g micro quadrotor
//
// Geometry is BINDING to the simulation model in quadsim/params.py:
//   m           = 0.033 kg   -> 33 g all-up mass target (36 g hard max)
//   L           = 0.046 m    -> arm_length = 46 mm (center to motor axis)
//   f_motor_max = 0.16  N    -> >= 16.5 gf per motor required from hardware
//   X configuration, rotors numbered to match the sim mixer:
//     rotor 1: +x+y arm (45 deg),  CCW   (1 engraved dot on arm)
//     rotor 2: -x+y arm (135 deg), CW    (2 engraved dots on arm)
//     rotor 3: -x-y arm (225 deg), CCW   (1 engraved dot on arm)
//     rotor 4: +x-y arm (315 deg), CW    (2 engraved dots on arm)
//
// Single printable part. Flat base at z = 0, every feature rises straight
// from the bed (arms, ribs, motor rings, guards), all chamfers/slots are
// cut from above -> prints support-free, flat side down.
//
// Requires OpenSCAD 2019.05 or newer (rotate_extrude(angle=...), assert()).
// Units: mm.
// ============================================================================

/* [Core X geometry] */
arm_length = 46;                    // frame center to motor axis (sim L = 0.046 m)
arm_angles = [45, 135, 225, 315];   // deg; index i = sim rotor i+1

/* [Motor press-fit rings — 7x16 mm brushed coreless] */
motor_diam        = 7.0;   // nominal motor can diameter
motor_fit_tol     = 0.1;   // diametral press-fit clearance added to the bore
motor_ring_height = 8;     // grip length on the 16 mm can (must be >= 8)
motor_ring_wall   = 1.2;   // ring wall thickness
ring_chamfer      = 0.6;   // lead-in chamfer at the ring top (insertion aid)
wire_slot_width   = 2.2;   // vertical side slot for the motor wires

/* [Arms] */
arm_width     = 7.5;       // in-plane width
arm_thickness = 2.8;       // vertical thickness (also the plate thickness)
arm_ribbed    = true;      // stiffening rib on top of each arm
rib_width     = 2.0;
rib_height    = 1.2;       // rib rises this much above the arm top surface
rib_start_radius = 21;     // ribs begin outboard of the FC footprint

/* [Center plate + whoop AIO FC mount] */
plate_diam        = 30;    // central disc; FC holes land on the arms beyond it
fc_hole_pitch     = 25.5;  // square M2 pattern, whoop AIO standard
fc_hole_diam      = 2.1;   // M2 clearance; set 1.7-1.8 for self-tapping screws
center_hole_diam  = 8;     // wire pass-through / lightening
lightening_hole_diam   = 5;
lightening_hole_offset = 9;   // on the y axis, between arms

/* [Battery strap slots — battery rides under the plate, along the x axis] */
strap_slot_length  = 12;   // slot extent across the battery (y direction)
strap_slot_width   = 3.2;  // fits a 10 mm hook-and-loop strap or rubber band
strap_slot_spacing = 16;   // slot center-to-center along the battery (x)

/* [Optional prop guards (adds ~5 g — bench/indoor tuning only)] */
prop_guards      = false;  // default off: flight config must stay <= 36 g AUW
guard_prop_diam  = 55;     // largest prop the guard envelope is sized for
guard_clearance  = 3;      // radial gap beyond the prop tip (must be >= 3)
guard_wall       = 1.4;
guard_height     = 4;
guard_arc        = 200;    // deg of partial ring, centered on the outboard axis
guard_spoke_width = 2.5;

/* [Rotor direction markers (engraved dots: 1 = CCW, 2 = CW)] */
marker_diam   = 1.8;
marker_depth  = 0.6;
marker_radius = 24;        // radial position of the first dot on the arm
marker_pitch  = 3;         // spacing between dots (CW arms get two)
marker_offset = 2.4;       // sideways offset so dots clear the rib

/* [Render quality] */
$fa = 2;
$fs = 0.4;
eps = 0.01;

// ------------------- derived values (never hardcoded) -----------------------
ring_id  = motor_diam + motor_fit_tol;          // press-fit bore
ring_od  = ring_id + 2 * motor_ring_wall;
motor_pos = [for (a = arm_angles) arm_length * [cos(a), sin(a)]];
adjacent_spacing = norm(motor_pos[0] - motor_pos[1]);   // = arm_length*sqrt(2)
diagonal_spacing = norm(motor_pos[0] - motor_pos[2]);   // = 2*arm_length
fc_hole_radius   = fc_hole_pitch / 2 * sqrt(2); // FC holes lie on the arm axes
guard_inner_r    = guard_prop_diam / 2 + guard_clearance;
guard_outer_r    = guard_inner_r + guard_wall;
check_prop_diam  = 55;     // spec prop (55 mm Hubsan X4 class); verified below for >= 10 mm tip gap

echo(str("adjacent motor spacing  = ", adjacent_spacing, " mm"));
echo(str("motor-to-motor diagonal = ", diagonal_spacing, " mm"));
echo(str("tip gap with ", check_prop_diam, " mm prop = ",
         adjacent_spacing - check_prop_diam, " mm"));
echo(str("tip gap with ", guard_prop_diam, " mm prop = ",
         adjacent_spacing - guard_prop_diam, " mm"));
echo(str("motor ring bore = ", ring_id, " mm, OD = ", ring_od, " mm"));
echo(str("guard inner radius = ", guard_inner_r, " mm"));

// ------------------------- design assertions --------------------------------
assert(abs(adjacent_spacing - 65.05) <= 0.1,
       str("adjacent motor spacing ", adjacent_spacing,
           " mm violates 65.05 +/- 0.1 mm"));
assert(abs(diagonal_spacing - 92) <= 0.1,
       str("motor diagonal ", diagonal_spacing, " mm != 92 mm"));
assert(adjacent_spacing - check_prop_diam >= 10,
       str("tip gap with a ", check_prop_diam, " mm prop is ",
           adjacent_spacing - check_prop_diam, " mm; need >= 10 mm"));
assert(adjacent_spacing - guard_prop_diam >= 10,
       str("tip gap with the ", guard_prop_diam,
           " mm guard-envelope prop is under 10 mm"));
assert(motor_ring_height >= 8, "motor_ring_height must be >= 8 mm");
assert(guard_clearance >= 3, "guard_clearance must be >= 3 mm beyond prop tip");
assert(2 * guard_outer_r < adjacent_spacing,
       "adjacent prop guards would intersect");
assert(fc_hole_radius + fc_hole_diam / 2 + 2 <= arm_length,
       "FC hole pattern does not fit on the arms");
assert(rib_start_radius > fc_hole_radius + fc_hole_diam / 2 + 1.5,
       "ribs would foul the FC mounting screws");

// ------------------------------- modules ------------------------------------

// One arm + rib + motor ring, laid along local +x; motor axis at x = arm_length.
module arm_solid() {
    translate([0, -arm_width / 2, 0])
        cube([arm_length, arm_width, arm_thickness]);
    if (arm_ribbed)
        translate([rib_start_radius, -rib_width / 2, 0])
            cube([arm_length - ring_od / 2 - rib_start_radius,
                  rib_width, arm_thickness + rib_height]);
    translate([arm_length, 0, 0])
        cylinder(h = motor_ring_height, d = ring_od);
}

// Partial-ring prop guard around the motor at local [arm_length, 0].
module guard_solid() {
    translate([arm_length, 0, 0]) {
        rotate([0, 0, -guard_arc / 2])
            rotate_extrude(angle = guard_arc, $fn = 180)
                translate([guard_inner_r, 0])
                    square([guard_wall, guard_height]);
        for (s = [-1, 1])
            rotate([0, 0, s * (guard_arc / 2 - 12)])
                translate([ring_id / 2, -guard_spoke_width / 2, 0])
                    cube([guard_inner_r - ring_id / 2 + guard_wall / 2,
                          guard_spoke_width, guard_height]);
    }
}

// All cuts local to one arm (bore, chamfer, wire slot, FC hole, marker dots).
// n_dots: 1 = CCW rotor, 2 = CW rotor.
module arm_cuts(n_dots) {
    translate([arm_length, 0, 0]) {
        // motor bore, straight through so seat depth is adjustable
        translate([0, 0, -eps])
            cylinder(h = motor_ring_height + 2 * eps, d = ring_id);
        // internal lead-in chamfer, cut from the top (support-free)
        translate([0, 0, motor_ring_height - ring_chamfer])
            cylinder(h = ring_chamfer + eps,
                     d1 = ring_id, d2 = ring_id + 2 * ring_chamfer);
        // full-height wire exit slot on the +y side of the ring
        translate([-wire_slot_width / 2, 0, -eps])
            cube([wire_slot_width, ring_od / 2 + eps,
                  motor_ring_height + 2 * eps]);
    }
    // FC mounting hole (pattern corner lies on the arm centerline)
    translate([fc_hole_radius, 0, -eps])
        cylinder(h = arm_thickness + 2 * eps, d = fc_hole_diam);
    // engraved rotor-direction dots
    for (k = [0 : n_dots - 1])
        translate([marker_radius + k * marker_pitch, marker_offset,
                   arm_thickness - marker_depth])
            cylinder(h = marker_depth + eps, d = marker_diam);
}

module frame() {
    difference() {
        union() {
            cylinder(h = arm_thickness, d = plate_diam);
            for (a = arm_angles)
                rotate([0, 0, a]) {
                    arm_solid();
                    if (prop_guards) guard_solid();
                }
        }
        // per-arm cuts; even index = rotors 1,3 = CCW (1 dot), odd = CW (2)
        for (i = [0 : len(arm_angles) - 1])
            rotate([0, 0, arm_angles[i]])
                arm_cuts(i % 2 == 0 ? 1 : 2);
        // center wire pass-through / lightening hole
        translate([0, 0, -eps])
            cylinder(h = arm_thickness + 2 * eps, d = center_hole_diam);
        // lightening holes between the arms
        for (s = [-1, 1])
            translate([0, s * lightening_hole_offset, -eps])
                cylinder(h = arm_thickness + 2 * eps, d = lightening_hole_diam);
        // battery strap slots (strap loops under the plate, battery along x)
        for (s = [-1, 1])
            translate([s * strap_slot_spacing / 2 - strap_slot_width / 2,
                       -strap_slot_length / 2, -eps])
                cube([strap_slot_width, strap_slot_length,
                      arm_thickness + 2 * eps]);
    }
}

frame();
