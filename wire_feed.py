"""
1KGSSJ-B Wire Feed Unit — Dimensional Reference Model
======================================================
Generates a STEP file with named components representing the wire
feed extruder head used in MIG welders.

Key dimensions from manufacturer drawing (1KGSSJ-B):
  Body:        104.5 L × 53 D × 85 H  mm
  Wire axis:   47.3 mm above base
  Total len:   122 mm  (incl. nozzle protrusion)
  Total depth:  93 mm  (incl. motor)
  Feed roller: Ø25 × 7 mm  (OD × width)
  Output bore: Ø5.3 mm

All dimensions in mm.  X = wire feed direction,
Y = depth (front→back), Z = height.
"""

import math
import warnings
import cadquery as cq

warnings.filterwarnings("ignore", category=FutureWarning, module="cadquery")

# ── Primary dimensions (manufacturer drawing) ────────────────────────────
BODY_L      = 104.5   # housing length (X)
BODY_H      =  85.0   # housing height (Z)
BODY_D      =  53.0   # housing depth front-to-back (Y)
TOTAL_L     = 122.0   # total length including nozzle tip
WIRE_Z      =  47.3   # wire-feed axis height above base (Z)
WIRE_Y      = BODY_D / 2   # wire axis centred in depth

# ── Chamfer on bottom-right corner of housing body ───────────────────────
CHAMFER_X_START = 87.0   # where chamfer begins along bottom edge
CHAMFER_Z_END   = 18.0   # chamfer height on right face

# ── Beak (tapered nozzle-carrier on right end) ───────────────────────────
NOZZLE_EXT  = TOTAL_L - BODY_L   # = 17.5 mm protrusion past body
NOZZLE_D    =   5.3               # output bore
NOZZLE_OD   =  12.0               # outer tube diameter
# beak spans z = 32..65 at x=BODY_L, tapering to z = 40..55 at x=TOTAL_L

# ── Feed rollers ─────────────────────────────────────────────────────────
ROLLER_D    = 25.0                       # OD
ROLLER_W    =  7.0                       # face width
ROLLER_CX   = 52.0                       # X position of both roller axes
DRIVE_CZ    = WIRE_Z - ROLLER_D / 2     # drive roller centre Z  (34.8)
TENS_CZ     = WIRE_Z + ROLLER_D / 2     # tensioner roller centre Z (59.8)
ROLLER_Y0   = (BODY_D - ROLLER_W) / 2   # Y start (rollers centred in body)

# ── Motor / gearbox ──────────────────────────────────────────────────────
MOTOR_CX    = ROLLER_CX    # motor axis directly behind drive roller
MOTOR_CZ    = WIRE_Z       # motor axis at wire height
GEARBOX_D   = 52.0         # gearbox flange OD
GEARBOX_H   =  5.0         # gearbox plate thickness
MOTOR_D     = 36.0         # 775-type motor body OD
MOTOR_L     = 35.0         # motor length beyond gearbox face
# stack: BODY_D(53) + GEARBOX_H(5) + MOTOR_L(35) = 93 mm ✓

# ── Mounting holes (thru Y, front-to-back) ───────────────────────────────
MOUNT_D     =   5.0    # hole diameter
MOUNT_XL    =  12.0    # left column X
MOUNT_XR    =  88.0    # right column X  (inside chamfer region)
MOUNT_ZB    =   9.0    # bottom row Z
MOUNT_ZT    =  76.0    # top row Z

# ── Wire inlet fitting (left face) ───────────────────────────────────────
INLET_OD    = 10.0
INLET_ID    =  5.5
INLET_STICK = 15.0     # protrudes this far in –X from left face (x=0)

# ── Tension adjustment bolt (extends –X from left face) ──────────────────
TBOLT_Z     = 42.0     # height of bolt centre
TBOLT_L     = 20.0     # protrusion length
TBOLT_D     =  8.0     # bolt diameter

# ── Tensioner arm (locked / wire-engaged position) ───────────────────────
ARM_PIVOT_X =  28.0    # pivot X inside housing
ARM_PIVOT_Z =  78.0    # pivot Z (near housing top)
ARM_W       =  12.0    # arm width in XZ plane
ARM_T       =   8.0    # arm thickness (Y direction)

# ── Tensioner adjustment knob (sits on top of pivot assembly) ────────────
KNOB_D      = 28.0
KNOB_H      = 18.0


# ============================================================
# HELPER – cylinder along Y axis
# ============================================================
def cyl_y(cx, cz, diameter, y_start=0.0, depth=BODY_D):
    """Cylinder whose axis is parallel to Y, centre at world (cx, ·, cz)."""
    return (
        cq.Workplane("XZ")
        .workplane(offset=y_start)
        .center(cx, cz)
        .circle(diameter / 2)
        .extrude(depth)
    )


# ============================================================
# HELPER – cylinder along X axis
# ============================================================
def cyl_x(cy, cz, diameter, x_start, length):
    """Cylinder whose axis is parallel to X, starting at x_start."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_start)
        .center(cy, cz)
        .circle(diameter / 2)
        .extrude(length)
    )


# ============================================================
# 1.  MAIN HOUSING
# ============================================================
# Side profile (XZ), extruded BODY_D in +Y.
# Rectangle with bottom-right chamfer.
housing_pts = [
    (0,               0),
    (CHAMFER_X_START, 0),            # bottom edge to chamfer start
    (BODY_L,          CHAMFER_Z_END), # diagonal chamfer
    (BODY_L,          BODY_H),
    (0,               BODY_H),
]
housing = (
    cq.Workplane("XZ")
    .polyline(housing_pts)
    .close()
    .extrude(BODY_D)
)

# Beak: tapered rectangular section carrying the output nozzle
beak_pts = [
    (BODY_L,   32.0),
    (TOTAL_L,  40.0),
    (TOTAL_L,  55.0),
    (BODY_L,   65.0),
]
beak = (
    cq.Workplane("XZ")
    .polyline(beak_pts)
    .close()
    .extrude(BODY_D)
)
housing = housing.union(beak)

# ── Mounting holes (thru Y) ──────────────────────────────────────────────
for mx, mz in [
    (MOUNT_XL, MOUNT_ZB),
    (MOUNT_XL, MOUNT_ZT),
    (MOUNT_XR, MOUNT_ZB),
    (MOUNT_XR, MOUNT_ZT),
]:
    housing = housing.cut(cyl_y(mx, mz, MOUNT_D))

# ── Nozzle bore (thru X through beak) ────────────────────────────────────
housing = housing.cut(
    cyl_x(WIRE_Y, WIRE_Z, NOZZLE_D, BODY_L - 3, NOZZLE_EXT + 6)
)

# ── Inlet bore (thru left wall) ──────────────────────────────────────────
housing = housing.cut(
    cyl_x(WIRE_Y, WIRE_Z, INLET_ID, -2, 25)
)


# ============================================================
# 2.  OUTPUT NOZZLE (hollow tube protruding from right face)
# ============================================================
nozzle = (
    cyl_x(WIRE_Y, WIRE_Z, NOZZLE_OD, BODY_L, NOZZLE_EXT)
    .cut(cyl_x(WIRE_Y, WIRE_Z, NOZZLE_D, BODY_L - 1, NOZZLE_EXT + 2))
)


# ============================================================
# 3.  WIRE INLET FITTING (hollow tube protruding from left face)
# ============================================================
inlet = (
    cyl_x(WIRE_Y, WIRE_Z, INLET_OD, -INLET_STICK, INLET_STICK + 3)
    .cut(cyl_x(WIRE_Y, WIRE_Z, INLET_ID, -INLET_STICK, INLET_STICK + 3))
)


# ============================================================
# 4.  WIRE PATH AXIS INDICATOR
#     Thin rod tracing the wire centreline from inlet to outlet.
#     Useful for alignment / offset measurement in downstream CAD.
# ============================================================
wire_path = cyl_x(
    WIRE_Y, WIRE_Z,
    diameter=2.0,
    x_start=-(INLET_STICK + 5),
    length=TOTAL_L + INLET_STICK + 10,
)


# ============================================================
# 5.  GEARBOX FLANGE PLATE (on back face, y = BODY_D)
# ============================================================
gearbox = cyl_y(MOTOR_CX, MOTOR_CZ, GEARBOX_D,
                y_start=BODY_D, depth=GEARBOX_H)


# ============================================================
# 6.  MOTOR BODY (extends beyond gearbox plate)
# ============================================================
motor = cyl_y(MOTOR_CX, MOTOR_CZ, MOTOR_D,
              y_start=BODY_D + GEARBOX_H, depth=MOTOR_L)


# ============================================================
# 7.  DRIVE ROLLER
#     Driven by motor; centre is ROLLER_D/2 below wire axis so
#     the roller rim just contacts the wire centreline.
# ============================================================
drive_roller = cyl_y(ROLLER_CX, DRIVE_CZ, ROLLER_D,
                     y_start=ROLLER_Y0, depth=ROLLER_W)


# ============================================================
# 8.  TENSIONER ROLLER  (in locked / wire-engaged position)
#     Centre is ROLLER_D/2 above wire axis; rim contacts wire
#     centreline from above, pinching wire against drive roller.
# ============================================================
tens_roller = cyl_y(ROLLER_CX, TENS_CZ, ROLLER_D,
                    y_start=ROLLER_Y0, depth=ROLLER_W)


# ============================================================
# 9.  TENSIONER ARM  (locked position – simplified flat plate)
#     Arm runs from pivot near housing top to tensioner roller axle.
# ============================================================
pivot       = (ARM_PIVOT_X, ARM_PIVOT_Z)
roller_att  = (ROLLER_CX,   TENS_CZ)          # roller axle centre

dx      = roller_att[0] - pivot[0]
dz      = roller_att[1] - pivot[1]
arm_len = math.sqrt(dx**2 + dz**2)
# unit vector perpendicular to arm (for width offset)
nx =  -dz / arm_len
nz =   dx / arm_len
hw = ARM_W / 2

arm_pts = [
    (pivot[0]      - nx * hw,  pivot[1]      - nz * hw),
    (pivot[0]      + nx * hw,  pivot[1]      + nz * hw),
    (roller_att[0] + nx * hw,  roller_att[1] + nz * hw),
    (roller_att[0] - nx * hw,  roller_att[1] - nz * hw),
]
arm = (
    cq.Workplane("XZ")
    .workplane(offset=(BODY_D - ARM_T) / 2)
    .polyline(arm_pts)
    .close()
    .extrude(ARM_T)
)

# Pivot pin cylinder (full depth, Y-axis)
arm_pivot_pin = cyl_y(ARM_PIVOT_X, ARM_PIVOT_Z, diameter=6.0)


# ============================================================
# 10. TENSIONER KNOB (knurled cap above pivot assembly)
# ============================================================
knob_z_center = BODY_H + KNOB_H / 2 + 3   # sits above housing top
knob = cyl_y(
    ARM_PIVOT_X, knob_z_center,
    KNOB_D,
    y_start=(BODY_D - KNOB_H * 0.55) / 2,
    depth=KNOB_H * 0.55,
)

# ============================================================
# 11. TENSION ADJUSTMENT BOLT (threaded rod extending left)
# ============================================================
tension_bolt = cyl_x(WIRE_Y, TBOLT_Z, TBOLT_D, -TBOLT_L, TBOLT_L)


# ============================================================
# ASSEMBLY
# ============================================================
assy = (
    cq.Assembly()
    .add(housing,       name="housing",
         color=cq.Color(0.12, 0.12, 0.12))
    .add(nozzle,        name="output_nozzle",
         color=cq.Color(0.12, 0.12, 0.12))
    .add(inlet,         name="inlet_fitting",
         color=cq.Color(0.12, 0.12, 0.12))
    .add(wire_path,     name="wire_feed_axis",
         color=cq.Color(0.9, 0.7, 0.1))       # gold – easy to spot
    .add(gearbox,       name="gearbox_flange",
         color=cq.Color(0.12, 0.12, 0.12))
    .add(motor,         name="motor",
         color=cq.Color(0.25, 0.25, 0.25))
    .add(drive_roller,  name="drive_roller",
         color=cq.Color(0.55, 0.55, 0.55))
    .add(tens_roller,   name="tensioner_roller",
         color=cq.Color(0.55, 0.55, 0.55))
    .add(arm,           name="tensioner_arm",
         color=cq.Color(0.12, 0.12, 0.12))
    .add(arm_pivot_pin, name="arm_pivot_pin",
         color=cq.Color(0.7,  0.7,  0.7))
    .add(knob,          name="tensioner_knob",
         color=cq.Color(0.08, 0.08, 0.08))
    .add(tension_bolt,  name="tension_adj_bolt",
         color=cq.Color(0.8,  0.8,  0.8))
)

out_file = "wire_feed.step"
assy.save(out_file)
print(f"Wrote {out_file}")
print()
print("Component summary:")
print(f"  Housing body   : {BODY_L} × {BODY_D} × {BODY_H} mm  (L×D×H)")
print(f"  Total length   : {TOTAL_L} mm  (incl. nozzle protrusion)")
print(f"  Total depth    : {BODY_D + GEARBOX_H + MOTOR_L} mm  (incl. motor)")
print(f"  Wire axis (Z)  : {WIRE_Z} mm above base")
print(f"  Wire axis (Y)  : {WIRE_Y} mm  (centred in body depth)")
print(f"  Drive roller   : Ø{ROLLER_D} mm, centre Z={DRIVE_CZ:.1f} mm")
print(f"  Tensioner roller: Ø{ROLLER_D} mm, centre Z={TENS_CZ:.1f} mm")
print(f"  Output bore    : Ø{NOZZLE_D} mm")
print(f"  Mounting holes : Ø{MOUNT_D} mm at "
      f"({MOUNT_XL},{MOUNT_ZB}), ({MOUNT_XL},{MOUNT_ZT}), "
      f"({MOUNT_XR},{MOUNT_ZB}), ({MOUNT_XR},{MOUNT_ZT})  [X,Z]")
