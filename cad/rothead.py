"""rothead.py — rotating head, build123d (cycloidal bend axis).

Massing of the cantilever head. It clamps the feed tube and rotates about the
wire axis (X = Axis 2). On it:
  +Z: BEND actuator = pancake stepper (24mm) + 20:1 micro-cycloidal (26mm), 42x42,
      bend axis = Z; the cycloidal output carries the bend disk + Ø4 mandrel, set
      so the wire runs tangent to the mandrel.
  -Z: ROTATION motor — a stepper at the 39mm mesh radius; its 12T pinion meshes the
      base's fixed gear AND it serves as the counterweight for the bend actuator.

Balance: bend 188g at CoM radius ~33  vs  rotation motor mass at radius 39.

Frame: X = wire axis (rotation axis, origin on it). Motors/cycloidal/mandrel are
GHOSTS (bought/printed-separately parts); the printed bracket ties them together.
Run:  py/bin/python cad/rothead.py   ->  build/rothead.{stl,step}
"""
import os
from build123d import *
from gears import spur_gear
from parts import nema17, NEMA, NEMA_BOLT
from pinion import pinion

# Sweep Dynamics 20:1 micro-cycloidal STEP (vendor/paid geometry — NOT committed).
# Point this at your local copy; the assembly falls back to a block if it's absent.
CYCLO_STEP = os.path.expanduser(
    "~/Downloads/cad-files/sweepdynamics/micro-cycloidal/20-1 Micro Cycloidal.step")
CYCLO_H = 25.9          # cycloidal body depth; output face on one end

# ── shared with base.py ─────────────────────────────────────────────
FG_TEETH, FG_MODULE, FG_W, PIN_TEETH = 40, 1.5, 8.0, 12
MESH_R = (FG_TEETH + PIN_TEETH) * FG_MODULE / 2            # 39
TUBE_D = 8.0

# ── bend actuator (pancake + micro-cycloidal) ───────────────────────
CYC_SQ = 42.0                       # 42x42 NEMA17 / cycloidal footprint
CYC_D = 26.0                        # cycloidal body depth
PANCAKE_D = 24.0                    # pancake stepper depth
BEND_STACK = CYC_D + PANCAKE_D      # 50mm along the bend axis (+Z)
CYC_OUT_D = 30.0                    # output bearing OD (bend-disk seat)
MANDREL_D, MANDREL_OFFSET = 4.0, 2.8
BEND_X = -30.0                      # bend axis X (at the wire exit, front of head)
BEND_Y = MANDREL_OFFSET             # bend axis offset so the wire is tangent
OUT_Z = 8.0                         # cycloidal output face height above the wire

# ── rotation motor ──────────────────────────────────────────────────
ROTMOT = (PANCAKE_D, CYC_SQ, CYC_SQ)   # pancake on -Z, axis ∥ X (L along X)
ROT_X = -14.0                          # motor front face (X); body extends -X, shaft +X
MOUNT_T = 6.0                          # face-plate thickness (bolt through it into the motor)
PILOT_CLEAR = 22.5                     # clears the Ø22 pilot boss; shaft passes through too
M3_CLEAR = 3.4                         # NEMA17 face is tapped M3, so the plate gets clearance

# ── tube clamp hub ──────────────────────────────────────────────────
HUB_OD = 18.0
HUB_X0, HUB_X1 = -26.0, -2.0


def xcyl(d, x0, x1, y=0.0, z=0.0):
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(
        d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def zcyl(d, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def hub():
    h = xcyl(HUB_OD, HUB_X0, HUB_X1)
    h -= xcyl(TUBE_D, HUB_X0 - 1, HUB_X1 + 1)
    h -= Pos((HUB_X0 + HUB_X1) / 2, 0, HUB_OD / 2) * Box(HUB_X1 - HUB_X0 + 2, 1.6, HUB_OD)  # pinch slit
    return h


def rot_mount():
    """NEMA17 face mount for the rotation motor. The motor seats on the -X side
    (its pilot boss into the bore), bolts pass +X through the plate into the
    motor's tapped face holes, and the shaft passes through to the pinion. Built
    in the motor's local frame (face at z=0, +Z toward the gear), then placed."""
    plate = Pos(0, 0, MOUNT_T / 2) * Box(NEMA + 2, NEMA + 2, MOUNT_T)
    plate -= Pos(0, 0, -1) * Cylinder(PILOT_CLEAR / 2, MOUNT_T + 2,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sx in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
        for sy in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
            plate -= Pos(sx, sy, -1) * Cylinder(M3_CLEAR / 2, MOUNT_T + 2,
                                                align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(ROT_X, 0, -MESH_R) * Rot(0, 90, 0) * plate


def bracket():
    """Rough printed bracket: hub + web up to the cycloidal mount + web down to the
    rotation-motor mount. Detail (bolt patterns, mandrel support) comes next."""
    b = hub()
    # web up to the cycloidal base plate (+Z)
    b += Pos(BEND_X + 6, BEND_Y, OUT_Z / 2) * Box(20, 10, OUT_Z + 4)
    b += Pos(BEND_X, BEND_Y, OUT_Z - 2) * Box(CYC_SQ, CYC_SQ, 4)            # cycloidal mount plate
    # spine tying the hub to the bend mount
    b += Pos((HUB_X0 + BEND_X) / 2 - 2, BEND_Y / 2, 1) * Box(28, 10, 8)
    # web down to the rotation-motor mount (-Z)
    b += Pos(ROT_X, 0, -MESH_R / 2) * Box(10, 12, MESH_R)
    b += rot_mount()                                                        # NEMA17 face mount
    return b


CYC_BODY_H = 23.3       # real cycloidal envelope height (base->output), from gen_vendor


def ghosts():
    # the real cycloidal body + bend die are placed in the sim (make_mjcf); here just
    # the pancake on the cycloidal base + the rotation pancake/pinion
    g = {}
    g["bend_pancake"] = Pos(BEND_X, BEND_Y, OUT_Z + CYC_BODY_H) * Rot(180, 0, 0) * nema17(depth=PANCAKE_D, shaft_len=18)
    g["rot_pancake"] = Pos(ROT_X, 0, -MESH_R) * Rot(0, 90, 0) * nema17(depth=PANCAKE_D, shaft_len=14)
    g["pinion"] = Pos(2, 0, -MESH_R) * Rot(0, 90, 0) * pinion()   # real printable part (set screw + D-bore)
    return g


def build_head():
    return bracket()


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    head = build_head()
    export_stl(head, "build/rothead.stl")
    export_step(head, "build/rothead.step")
    asm = Compound(children=[head] + list(ghosts().values()))
    export_stl(asm, "build/rothead_assembly.stl")
    bb = asm.bounding_box()
    print("assembly bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          "swing R ~", round(max(abs(bb.min.Z), abs(bb.max.Z)), 1))
