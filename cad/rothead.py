"""rothead.py — rotating head, build123d (cantilever architecture).

First structural pass as REAL connected solids (replaces the OpenSCAD massing).
The printed head body clamps the feed tube and rotates about the wire axis
(X = Axis 2). On it, opposite for balance:
  +Z: bender drivetrain (gutted servo + tiny stepper -> gear train -> Ø4 mandrel),
      inboard at BENDER_R so its moment ~ matches the rotation motor
  -Z: rotation motor, its 12T pinion meshing the base's fixed gear (radius MESH_R)

Frame: X = wire axis (rotation axis, origin on it); head body at x<0, forward of
the base. Motors + gears are GHOSTS (bought parts); only the body is printed.

Run:  py/bin/python cad/rothead.py   ->  build/rothead.{stl,step}
NOTE: the bender drivetrain ORIENTATION (servo output -> Y-axis mandrel gear train)
is still to be resolved — here it's mounts + a mandrel bearing boss + ghosts.
"""
import os
from build123d import *
from gears import spur_gear

# ── shared with base.py (keep in sync) ──────────────────────────────
FG_TEETH, FG_MODULE, FG_W, PIN_TEETH = 40, 1.5, 8.0, 12
MESH_R = (FG_TEETH + PIN_TEETH) * FG_MODULE / 2           # 39
TUBE_D = 8.0

# ── head ────────────────────────────────────────────────────────────
HUB_OD = 18.0
HUB_X0, HUB_X1 = -28.0, -2.0
CLAMP_GAP = 1.6                              # pinch slit
MANDREL_D, MANDREL_OFFSET = 4.0, 2.8
MANDREL_X = -24.0
MBRG_OD, MBRG_W = 12.0, 4.0                  # MR small bearing for the mandrel journal

ROTMOT = (30.0, 24.0, 24.0)                  # rotation geared stepper  (L along X)
BENDER = (40.0, 24.0, 32.0)                  # gutted servo + tiny stepper
BENDER_R = 24.0                              # inboard -> balances the lighter rotation motor at 39
ARM_T = 6.0                                  # connecting-arm thickness


def xcyl(d, x0, x1, y=0.0, z=0.0):
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(
        d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def hub():
    h = xcyl(HUB_OD, HUB_X0, HUB_X1)
    h -= xcyl(TUBE_D, HUB_X0 - 1, HUB_X1 + 1)               # tube bore
    # pinch slit + lug with an M3 cross-bolt
    h -= Pos((HUB_X0 + HUB_X1) / 2, 0, HUB_OD / 2) * Box(HUB_X1 - HUB_X0 + 2, CLAMP_GAP, HUB_OD)
    lug = Pos((HUB_X0 + HUB_X1) / 2, 0, HUB_OD / 2 + 3) * Box(14, 12, 8)
    lug -= Pos((HUB_X0 + HUB_X1) / 2, 0, HUB_OD / 2 + 3) * Rot(90, 0, 0) * Cylinder(1.6, 14)
    return h + lug


def motor_pad(z, depth):
    """Mounting pad at radius |z| on the wire axis side, normal to X (face at x=0)."""
    pad = Pos(-depth / 2, 0, z) * Box(depth, ROTMOT[1] + 8, ROTMOT[2] + 8)
    return pad


def arm(z0, z1):
    """Web in the X-Z plane tying the hub to a motor pad."""
    lo, hi = sorted((z0, z1))
    return Pos((HUB_X0 + HUB_X1) / 2, 0, (lo + hi) / 2) * Box(HUB_X1 - HUB_X0, ARM_T, hi - lo)


def mandrel_boss():
    # boss around the mandrel journal (mandrel axis = Z here, near the wire axis)
    boss = Pos(MANDREL_X, MANDREL_OFFSET, -MBRG_W) * Cylinder(
        MBRG_OD / 2 + 3, MBRG_W + 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    boss -= Pos(MANDREL_X, MANDREL_OFFSET, -MBRG_W) * Cylinder(
        MBRG_OD / 2, MBRG_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    boss -= Pos(MANDREL_X, MANDREL_OFFSET, -MBRG_W - 1) * Cylinder(MANDREL_D / 2 + 0.6, MBRG_W + 10)
    # wire-guide rib along X at the wire axis, up to the mandrel
    rib = Pos(MANDREL_X - 1, 0, 1) * Box(8, 6, 6)
    rib -= xcyl(3, MANDREL_X - 6, MANDREL_X + 6, z=1)
    return boss + rib


def build_head():
    b = hub()
    b += arm(HUB_OD / 2 - 2, BENDER_R)             # up to bender
    b += motor_pad(BENDER_R + BENDER[2] / 2 - 4, 6)
    b += arm(-MESH_R, -(HUB_OD / 2 - 2))           # down to rotation motor
    b += motor_pad(-MESH_R, 6)
    b += mandrel_boss()
    try:
        b = fillet(b.edges().filter_by(Axis.X).group_by(SortBy.LENGTH)[-1], 1.5)
    except Exception:
        pass
    return b


def ghosts():
    """Bought parts, for the assembly view only."""
    g = {}
    g["pinion"] = Pos(0, 0, -MESH_R) * Rot(0, 90, 0) * spur_gear(PIN_TEETH, FG_MODULE, FG_W, bore=4)
    g["rotmot"] = Pos(-ROTMOT[0] + 2, 0, -MESH_R) * Box(*ROTMOT)
    g["bender"] = Pos(-BENDER[0] / 2 + 2, 0, BENDER_R) * Box(*BENDER)
    g["mandrel"] = Pos(MANDREL_X, MANDREL_OFFSET, -8) * Cylinder(
        MANDREL_D / 2, 18, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return g


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    head = build_head()
    export_stl(head, "build/rothead.stl")
    export_step(head, "build/rothead.step")
    # combined assembly STL for viewing (body + ghosts)
    asm = head
    for s in ghosts().values():
        asm += s
    export_stl(asm, "build/rothead_assembly.stl")
    bb = head.bounding_box()
    print("head body volume", round(head.volume, 1),
          "bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)])
