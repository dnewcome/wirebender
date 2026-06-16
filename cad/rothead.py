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
TUBE_CLEAR = 0.4        # slip fit so the feed tube slides into the bore (set screw clamps it)

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
CYC_BODY_H = 23.3                   # real cycloidal envelope height (output->input), from gen_vendor
BEND_PLATE_Z = OUT_Z + CYC_BODY_H   # cycloid INPUT/motor face (top) — head hangs below it
CYC_BASE_T = 9.0                    # nema-17-cycloid-base.stl thickness (NEMA face <-> bearing boss)
CYC_BASE_STL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "nema-17-cycloid-base.stl")

# ── rotation motor ──────────────────────────────────────────────────
ROTMOT = (PANCAKE_D, CYC_SQ, CYC_SQ)   # pancake on -Z, axis ∥ X (L along X)
ROT_X = -14.0                          # motor front face (X); body extends -X, shaft +X
MOUNT_T = 6.0                          # face-plate thickness (bolt through it into the motor)
PILOT_CLEAR = 22.5                     # clears the Ø22 pilot boss; shaft passes through too
M3_CLEAR = 3.4                         # NEMA17 face is tapped M3, so the plate gets clearance

# ── 2-piece split + adjustment slots ────────────────────────────────
ROT_SLOT = 2.0          # rotation NEMA mount: ±travel to set the pinion/fixed-gear mesh
IFACE_INS_D, IFACE_INS_H = 4.6, 5.0    # M3 heat-set inserts joining piece 2 -> piece 1
IFACE_X = -5.0                          # hub-top join pad (clear of the base edge -9 and hub end -2)

# bend-motor mount (piece 2): a vertical face the bend-actuator plate bolts to,
# extended UP ~MESH_R (mirroring the rotation arm that hangs down) with 2 vertical
# slots so the bend actuator's distance from the wire is adjustable (±BMNT_SLOT)
# for various mandrels. The motor mounts at a fixed X ("out"); the slots set Z.
BMNT_X = BEND_X + CYC_SQ / 2    # -9, the cyclo +X face plane = the mount-face position
BMNT_T = MOUNT_T                # 6mm plate
BMNT_SLOT = 10.0               # ±travel (distance-from-wire adjustment)
BMNT_SLOT_Y = 8.0              # the 2 vertical slots at ±Y
BMNT_Z_TOP = MESH_R + 3        # top of the mount (up the same distance the rot motor drops)
BMNT_SLOT_Z = 28.0            # slot centre height

# ── flat head (the simpler single printable part) ───────────────────
# Two coplanar slotted plates (rotation NEMA mount below at -MESH_R, bending mount
# above) joined by a central tube-clamp boss that sticks up FH_BOSS_UP above the
# FH_PT-thick plates. Radial M3 set screw at x=FH_SET_X (mid of the boss). Prints
# flat on its x=0 face; wire/tube axis = X (the boss bore).
FH_PT = 5.0            # plate thickness (x = 0 .. FH_PT)
FH_BOSS_UP = 3.0       # boss sticks up this much past the plates
FH_BOSS_X1 = FH_PT + FH_BOSS_UP        # 8 — boss top (set screw at the 4mm mid)
FH_SET_X = 4.0         # set screw at x=4 (middle of the 8mm clamp)
FH_HUB_OD = 16.0       # tube-clamp boss Ø
FH_BEND_ZC = 21.0      # bending-plate slot centre (Z, above the boss)

# ── tube clamp hub ──────────────────────────────────────────────────
# Short (8mm) boss on the bend-head/attachment end of the tube (its old long -X
# protrusion fouled the rotation motor). Clamped by a radial M3 heat-set insert +
# grub set screw facing +Y (reachable from the narrow side) — no pinch slit.
HUB_OD = 18.0
HUB_X0, HUB_X1 = -10.0, -2.0       # 8mm long


def xcyl(d, x0, x1, y=0.0, z=0.0):
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(
        d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def zcyl(d, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def ycyl(d, y0, y1, x=0.0, z=0.0):
    """Solid cylinder Ø d along +Y from y0 to y1, at (x, z)."""
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(
        d / 2, y1 - y0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def _slot(d, travel, depth):
    """Slot cutter centred at the origin: Ø d hole elongated ±travel along X, bored
    along Z. Pos/Rot into place (Rot(0,90,0) -> bored along X, elongated along Z)."""
    s = Box(2 * travel, d, depth)
    s += Pos(travel, 0, 0) * Cylinder(d / 2, depth)
    s += Pos(-travel, 0, 0) * Cylinder(d / 2, depth)
    return s


def hub():
    """Short tube-clamp boss with a radial M3 heat-set set screw (no pinch slit)."""
    h = xcyl(HUB_OD, HUB_X0, HUB_X1)                       # boss along the tube
    h -= xcyl(TUBE_D + TUBE_CLEAR, HUB_X0 - 1, HUB_X1 + 1)  # tube bore (slip fit)
    xc = (HUB_X0 + HUB_X1) / 2
    # +Y radial set screw: one Ø IFACE_INS_D hole for the M3 heat-set insert that
    # penetrates a couple mm INTO the bore (not tangent to it) so the seated grub
    # presses the tube cleanly. Heat-set inserts press flush, so no shoulder needed.
    h -= ycyl(IFACE_INS_D, TUBE_D / 2 - 2, HUB_OD / 2 + 1, x=xc)
    return h


def rot_mount():
    """NEMA17 face mount for the rotation motor. The motor seats on the -X side
    (its pilot boss into the bore), bolts pass +X through the plate into the
    motor's tapped face holes, and the shaft passes through to the pinion. Built
    in the motor's local frame (face at z=0, +Z toward the gear), then placed."""
    plate = Pos(0, 0, MOUNT_T / 2) * Box(NEMA + 2, NEMA + 2, MOUNT_T)
    # slots run along local X (= the head's radial Z) so the motor slides ±ROT_SLOT
    # to set the pinion/fixed-gear mesh
    plate -= Pos(0, 0, MOUNT_T / 2) * _slot(PILOT_CLEAR, ROT_SLOT, MOUNT_T + 2)
    for sx in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
        for sy in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
            plate -= Pos(sx, sy, MOUNT_T / 2) * _slot(M3_CLEAR, ROT_SLOT, MOUNT_T + 2)
    return Pos(ROT_X, 0, -MESH_R) * Rot(0, 90, 0) * plate


def cyclo_base():
    """The cycloid drive's integrated NEMA17 base (vendor nema-17-cycloid-base.stl),
    oriented BEARING-BOSS-DOWN toward the wire — the pancake mounts on the NEMA face
    (up), the Ø30 output boss faces the wire. This IS the bend-head mount (the whole
    drive is integrated), not a printed plate. Output boss face at z=OUT_Z."""
    if not os.path.exists(CYC_BASE_STL):                # vendor STL absent -> placeholder block
        b = Pos(0, 0, CYC_BASE_T / 2) * Box(CYC_SQ, CYC_SQ, CYC_BASE_T)
    else:
        b = import_stl(CYC_BASE_STL)
        c = b.bounding_box().center()
        b = Pos(-c.X, -c.Y, 0) * b                      # centre on the output axis (XY)
    return Pos(BEND_X, BEND_Y, OUT_Z + CYC_BASE_T) * Rot(180, 0, 0) * b


PAD_Z = HUB_OD / 2 + 3.5            # join-pad top face (hub top 9 + 3.5 pad)


def rot_piece():
    """Piece 1 (flat-print): full tube clamp + rotation arm + slotted NEMA mount +
    a hub-top pad with 2 heat-set inserts that piece 2 bolts down onto."""
    b = hub()
    # rotation web to the slotted mount plate (top-edge tie, clears the motor envelope)
    plate_top = -MESH_R + (NEMA + 2) / 2
    web_h = 2 - (plate_top - 4)
    b += Pos(ROT_X + MOUNT_T / 2, 0, 2 - web_h / 2) * Box(MOUNT_T, 16, web_h)
    b += rot_mount()
    # join pad on the hub top (between the base edge at -9 and the hub end at -2);
    # overlaps into the hub (z 7..PAD_Z) so it fuses rather than just touching
    b += Pos(IFACE_X, 0, (7 + PAD_Z) / 2) * Box(6, 16, PAD_Z - 7)
    for ix in (IFACE_X - 3, IFACE_X + 3):               # 2 heat-set inserts, open at the pad top
        b -= Pos(ix, 0, PAD_Z - IFACE_INS_H) * Cylinder(
            IFACE_INS_D / 2, IFACE_INS_H + 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # re-cut the tube bore AFTER the web/pad are added, so nothing blocks the tube
    # (the rotation web overlaps the bore region) — keep it clear through the web.
    b -= xcyl(TUBE_D + TUBE_CLEAR, ROT_X - MOUNT_T / 2 - 1, HUB_X1 + 1)
    return b


def bend_piece():
    """Piece 2 (flat-print): bolts down onto piece 1's hub-top pad, then rises as a
    vertical mounting face (up to ~z=BMNT_Z_TOP, mirroring the rotation arm) with two
    vertical M3 slots (±BMNT_SLOT). The bend-actuator plate bolts to it at a fixed X
    ('out') and slides up/down to set the die's distance from the wire per mandrel."""
    # foot on the join pad + clearance for the two join bolts
    b = Pos(IFACE_X, 0, PAD_Z + 2) * Box(6, 16, 4)                  # z PAD_Z..PAD_Z+4
    for ix in (IFACE_X - 3, IFACE_X + 3):
        b -= zcyl(M3_CLEAR, PAD_Z - 0.5, PAD_Z + 5, x=ix, y=0)
    # vertical mounting plate rising from the foot up to BMNT_Z_TOP (normal +X)
    plate_w = 2 * BMNT_SLOT_Y + 12                                  # Y width
    b += Pos(BMNT_X + BMNT_T / 2, 0, (PAD_Z + BMNT_Z_TOP) / 2) * Box(
        BMNT_T, plate_w, BMNT_Z_TOP - PAD_Z)                        # x -9..-3, z PAD_Z..top
    # two vertical slots (bored through the plate along X, elongated ±BMNT_SLOT in Z)
    for sy in (BMNT_SLOT_Y, -BMNT_SLOT_Y):
        b -= (Pos(BMNT_X + BMNT_T / 2, sy, BMNT_SLOT_Z) * Rot(0, 90, 0)
              * _slot(M3_CLEAR, BMNT_SLOT, BMNT_T + 2))
    return b


def flat_head():
    """Simpler flat-printing head: two COPLANAR slotted plates (rotation NEMA17
    mount below at z=-MESH_R, bending mount above) joined by a central tube-clamp
    boss that sticks up FH_BOSS_UP past the FH_PT-thick plates, with a radial M3
    heat-set set screw at x=FH_SET_X. Prints flat on its x=0 face; wire/tube axis = X.
    The plates connect to the boss above/below z=0, leaving the +Y set screw clear."""
    boss = xcyl(FH_HUB_OD, 0.0, FH_BOSS_X1, 0, 0)                       # tube-clamp boss along X
    rotp = Pos(FH_PT / 2, 0, -MESH_R) * Box(FH_PT, NEMA + 2, NEMA + 2)  # rotation NEMA plate (below)
    rneck = Pos(FH_PT / 2, 0, -10.5) * Box(FH_PT, 16, 13)              # neck boss->rot plate (z -17..-4)
    bendp = Pos(FH_PT / 2, 0, FH_BEND_ZC) * Box(FH_PT, 2 * BMNT_SLOT_Y + 16, 34)  # bending plate (z 4..38)
    body = boss + rotp + rneck + bendp
    body -= xcyl(TUBE_D + TUBE_CLEAR, -1, FH_BOSS_X1 + 1, 0, 0)         # tube bore (slip fit)
    # +Y radial M3 heat-set set screw at x=FH_SET_X, penetrating into the bore
    body -= ycyl(IFACE_INS_D, TUBE_D / 2 - 2, FH_HUB_OD / 2 + 1, x=FH_SET_X)
    # rotation NEMA17 slots (bored along X, elongated ±ROT_SLOT in Z for gear mesh)
    body -= Pos(FH_PT / 2, 0, -MESH_R) * Rot(0, 90, 0) * _slot(PILOT_CLEAR, ROT_SLOT, FH_PT + 2)
    for sy in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
        for sz in (NEMA_BOLT / 2, -NEMA_BOLT / 2):
            body -= Pos(FH_PT / 2, sy, -MESH_R + sz) * Rot(0, 90, 0) * _slot(M3_CLEAR, ROT_SLOT, FH_PT + 2)
    # bending plate 2 slots (bored along X, elongated ±BMNT_SLOT in Z for mandrel height)
    for sy in (BMNT_SLOT_Y, -BMNT_SLOT_Y):
        body -= Pos(FH_PT / 2, sy, FH_BEND_ZC) * Rot(0, 90, 0) * _slot(M3_CLEAR, BMNT_SLOT, FH_PT + 2)
    return body


def build_head():
    return Compound(children=[flat_head()])


PINION_MOUNT_X = 2.0       # pinion centre X in the head frame (on the rotation-motor shaft)


def ghosts(motors=True, pinion_part=True):
    """Reference placements around the FLAT head. The rotation motor + pinion sit on
    the rotation NEMA plate (z=-MESH_R); the bend actuator (cyclo + pancake + die) is
    at the bend point out front. NOTE: how the bend actuator REACHES from the bending
    plate (at the boss) forward to the bend point is the bend_plate/arbor you're
    still designing — that reach isn't visualized here."""
    g = {"cyclo_base": cyclo_base()}                          # bend drive body (at the bend point)
    if motors:
        # bend pancake on the cyclo input face (at the bend point, out front)
        g["bend_pancake"] = Pos(BEND_X, BEND_Y, OUT_Z + CYC_BASE_T) * Rot(180, 0, 0) * nema17(depth=PANCAKE_D, shaft_len=18)
        # rotation pancake on the flat head's rotation NEMA plate (face at x=0, shaft +X)
        g["rot_pancake"] = Pos(0, 0, -MESH_R) * Rot(0, 90, 0) * nema17(depth=PANCAKE_D, shaft_len=14)
    if pinion_part:
        g["pinion"] = Pos(PINION_MOUNT_X, 0, -MESH_R) * Rot(0, 90, 0) * pinion()
    return g


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    head = flat_head()                             # single flat-printing part
    export_stl(head, "build/rothead.stl")
    export_step(head, "build/rothead.step")
    bb = head.bounding_box()
    print("flat head: 2 coplanar slotted plates (NEMA mount + bending mount) + tube-clamp boss",
          "| bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          f"| plates {FH_PT}mm, boss +{FH_BOSS_UP}mm, set screw at x={FH_SET_X}")
