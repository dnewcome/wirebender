#!/usr/bin/env python3
"""
make_mjcf.py — Generate a MuJoCo model (MJCF) of the *whole* wire bender.

The bending head reuses the real CAD meshes (placed with the same transforms as
assemble.py, so it stays in sync with the design). Everything else — feeder body,
NEMA17, feed tube, drive pulley, U-bracket, tube-rotation motor, and the wire —
is ROUGHED IN with primitives at nominal dimensions (see the LAYOUT block). Those
are placeholders to refine once the real parts are measured/modeled.

Kinematic tree:
    world
     ├─ (static) base, U-bracket, feeder, NEMA17, rotation motor   [decoration]
     ├─ wire   : slide along the feed axis (Y)            Axis 1: wire feed
     └─ tube   : hinge about the wire/feed-tube axis (Y)  Axis 2: bend direction
        │        carries feed tube + drive pulley + the bending head meshes
        └─ bend_flange : hinge about the motor shaft (Z)  Axis 3: bend degree

Collisions are disabled (contype/conaffinity = 0): this is a kinematic
visualization driven by position actuators, not a contact/dynamics sim.

Run:  ../py/bin/python make_mjcf.py
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MANIFEST = ROOT / "manifest.yaml"
MESH_OUT = HERE / "meshes"
XML_OUT = HERE / "wirebender.xml"

MM_TO_M = 0.001  # MuJoCo works in meters; our CAD is in mm.

# ── Transform math (mirrors assemble.py; numpy-only so we need no cadquery) ──


def deg2rad(v) -> float:
    return float(v) * math.pi / 180.0


def unit_scale(units: str) -> float:
    u = (units or "mm").lower()
    if u in ("mm", "millimeter", "millimeters"):
        return 1.0
    if u in ("in", "inch", "inches"):
        return 25.4
    raise ValueError(f"Unsupported units: {units}")


def xform_to_matrix(t, r_deg, units_mm_per_unit):
    """(R 3x3, t 3-vec in mm). Rotation order X then Y then Z — matches assemble.py."""
    tx, ty, tz = [float(x) * units_mm_per_unit for x in t]
    rx, ry, rz = [deg2rad(x) for x in r_deg]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], float)
    return Rx @ Ry @ Rz, np.array([tx, ty, tz], float)


def rotation_matrix_from_vectors(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(np.dot(a, b))
    if abs(c + 1.0) < 1e-9:
        perp = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0.0, 1, 0])
        ax = np.cross(a, perp)
        ax /= np.linalg.norm(ax)
        return 2.0 * np.outer(ax, ax) - np.eye(3)
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + kmat + kmat @ kmat * ((1.0 - c) / (s * s + 1e-30))


def resolve_mate(p, part_world, part_anchors, units_mm):
    xform = p.get("xform", {}) or {}
    r_deg = xform.get("r_deg", [0, 0, 0])
    my_anchors = p.get("anchors") or {}
    R_pre, _ = xform_to_matrix([0, 0, 0], r_deg, units_mm)

    mate = (p.get("mates") or [])[0]
    my_anch = my_anchors[mate["my_anchor"]]
    P_me = np.array(my_anch.get("t", [0, 0, 0]), float) * units_mm
    AX_me = np.array(my_anch.get("axis", [0, 0, 1]), float)
    AX_me /= np.linalg.norm(AX_me)

    R_target, t_target = part_world[mate["to_part"]]
    to_anch = part_anchors[mate["to_part"]][mate["to_anchor"]]
    P_to = np.array(to_anch.get("t", [0, 0, 0]), float) * units_mm
    AX_to = np.array(to_anch.get("axis", [0, 0, 1]), float)
    AX_to /= np.linalg.norm(AX_to)

    P_to_world = R_target @ P_to + t_target
    AX_to_world = R_target @ AX_to
    P_me_pre = R_pre @ P_me
    AX_me_pre = R_pre @ AX_me
    R_align = rotation_matrix_from_vectors(AX_me_pre, AX_to_world)
    return R_align @ R_pre, P_to_world - R_align @ P_me_pre


def resolve_world_transforms(manifest):
    units_mm = unit_scale(manifest.get("units", "mm"))
    parts = [p for p in manifest.get("parts", []) if p.get("name") and p.get("file")]
    cfgs = {p["name"]: p for p in parts}
    anchors = {n: (cfgs[n].get("anchors") or {}) for n in cfgs}
    world = {}
    remaining = list(cfgs)
    for _ in range(len(cfgs) + 1):
        if not remaining:
            break
        deferred = []
        for name in remaining:
            p = cfgs[name]
            xform = p.get("xform", {}) or {}
            mates = p.get("mates") or []
            if not mates:
                world[name] = xform_to_matrix(
                    xform.get("t", [0, 0, 0]), xform.get("r_deg", [0, 0, 0]), units_mm
                )
            elif all(m.get("to_part") in world for m in mates):
                world[name] = resolve_mate(p, world, anchors, units_mm)
            else:
                deferred.append(name)
        remaining = deferred
    if remaining:
        raise RuntimeError(f"Unresolvable mate dependencies: {remaining}")
    return world, cfgs


# ── Mesh handling ───────────────────────────────────────────────────────────


def intermediate_stl(part_file: Path) -> Path:
    suffix = part_file.suffix.lower()
    build = ROOT / "build"
    if suffix == ".stl":
        return part_file
    if suffix in (".glb", ".gltf", ".obj", ".ply"):
        return build / f"{part_file.stem}.converted.stl"
    if suffix == ".scad":
        return build / f"{part_file.stem}.scad.stl"
    raise ValueError(f"No mesh mapping for {part_file}")


def convert_to_binary_stl(src: Path, dst: Path) -> None:
    mesh = trimesh.load(str(src), force="mesh")
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(dst), file_type="stl")
    print(f"  mesh {src.name:24s} -> meshes/{dst.name}  ({len(mesh.vertices)} verts)")


def mat_to_quat(R):
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1]-m[1, 2])/s, (m[0, 2]-m[2, 0])/s, (m[1, 0]-m[0, 1])/s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1]-m[1, 2])/s, 0.25*s, (m[0, 1]+m[1, 0])/s, (m[0, 2]+m[2, 0])/s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2]-m[2, 0])/s, (m[0, 1]+m[1, 0])/s, 0.25*s, (m[1, 2]+m[2, 1])/s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0]-m[0, 1])/s, (m[0, 2]+m[2, 0])/s, (m[1, 2]+m[2, 1])/s, 0.25*s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def fmt(v):
    if np.isscalar(v):
        return f"{v:.6g}"
    return " ".join(f"{x:.6g}" for x in np.asarray(v, float))


# ── Geom emitters (all inputs in mm; converted to meters) ───────────────────


def mesh_geom(name, R, t_mm, rgba, indent=6):
    pos = np.asarray(t_mm, float) * MM_TO_M
    quat = mat_to_quat(R)
    sp = " " * indent
    return (f'{sp}<geom type="mesh" mesh="{name}" pos="{fmt(pos)}" '
            f'quat="{fmt(quat)}" rgba="{rgba}"/>')


def box_geom(center_mm, half_mm, rgba, indent=6, quat=None):
    pos = np.asarray(center_mm, float) * MM_TO_M
    size = np.asarray(half_mm, float) * MM_TO_M
    q = f' quat="{fmt(quat)}"' if quat is not None else ""
    sp = " " * indent
    return f'{sp}<geom type="box" pos="{fmt(pos)}" size="{fmt(size)}"{q} rgba="{rgba}"/>'


def cyl_geom(p0_mm, p1_mm, r_mm, rgba, indent=6, kind="cylinder"):
    a = np.asarray(p0_mm, float) * MM_TO_M
    b = np.asarray(p1_mm, float) * MM_TO_M
    sp = " " * indent
    return (f'{sp}<geom type="{kind}" fromto="{fmt(a)} {fmt(b)}" '
            f'size="{r_mm*MM_TO_M:.6g}" rgba="{rgba}"/>')


# ── Main ────────────────────────────────────────────────────────────────────

# Bending-head meshes (from the manifest / CAD), grouped by moving body.
TUBE_PARTS = ["motor", "bender-head"]   # rotate together with the feed tube
FLANGE_PARTS = ["motor-flange"]         # bending flange on the motor shaft

COLORS = {
    "motor": "0.30 0.32 0.36 1",
    "bender-head": "0.20 0.55 0.85 1",
    "motor-flange": "0.90 0.55 0.15 1",
}
C_STEEL = "0.62 0.64 0.68 1"
C_DARK = "0.18 0.19 0.22 1"
C_ALU = "0.70 0.72 0.75 1"
C_MOTOR = "0.25 0.27 0.30 1"
C_WIRE = "0.80 0.80 0.85 1"
C_PULLEY = "0.15 0.16 0.18 1"

# ───────────────────────── MACHINE LAYOUT (nominal mm) ──────────────────────
# Wire / feed-tube axis runs along +Y at (x=WAX, z=WAZ). Wire feeds toward -Y
# (out through the bending head near Y=0). All [ROUGH] values are placeholders.
WAX, WAZ = 6.35, 5.5            # wire axis (matches bender-head channel)

# Feed tube (Axis 2 rotates about this axis)
TUBE_Y0, TUBE_Y1 = 12, 158      # [ROUGH] tube extent (head end -> feeder end)
TUBE_OD = 6.35                  # 0.25" feed tube

# Drive pulley on the tube (toothed-belt driven by the rotation motor)
PULLEY_Y, PULLEY_R, PULLEY_W = 150, 16, 6   # [ROUGH]

# U-bracket: two uprights on a base foot; the tube turns in bearings here
BRK_Y = [108, 150]              # [ROUGH] upright Y positions
BRK_HALF = [14, 2.5, 28]        # [ROUGH] upright half-size (X, Y, Z) above base
BASE_TOP = -38                  # [ROUGH] top of base plate / foot height

# Tube-rotation motor (geared stepper, belt to the pulley) — mounted off the bracket
ROT_MOTOR_C = [WAX - 34, 150, WAZ - 8]   # [ROUGH] center
ROT_MOTOR_HALF = [14, 25, 14]            # [ROUGH] body half-size (NEMA-ish)
ROT_PULLEY_R = 9

# Feeder body (1KGSSJ-B ~122 x 85, thickness ~30). Wire passes through at the axis.
FEEDER_Y = [165, 287]           # [ROUGH] feeder extent along Y (122 long)
FEEDER_HALF = [13, 61, 42.5]    # [ROUGH] half-size (X thickness, Y/2 len, Z height)

# Drive stepper (NEMA17) sticking out the side of the feeder, axis along X
NEMA_C = [WAX - 13 - 21, 215, WAZ]   # [ROUGH] body center (-X side of feeder)
NEMA_HALF = [21, 21, 21]             # 42mm cube-ish
NEMA_SHAFT_L = 22

# Base plate the machine sits on
BASE_Y = [95, 290]
BASE_X_HALF = 40
BASE_THK = 6

# Wire stock (Axis 1)
WIRE_OD = 1.63                  # 14 ga
WIRE_Y0, WIRE_Y1 = -55, 230     # extent at rest (feeder roller -> past head)

# Actuator / joint travel
FEED_RANGE_MM = (-120, 40)      # wire slide travel
TUBE_RANGE = (-90, 90)          # feed-tube rotation
BEND_RANGE = (0, 150)           # bend degree


def main():
    manifest = yaml.safe_load(MANIFEST.read_text())
    world, cfgs = resolve_world_transforms(manifest)

    print("Resolving bending-head meshes:")
    mesh_assets = {}
    for name in TUBE_PARTS + FLANGE_PARTS:
        if name not in cfgs:
            raise RuntimeError(f"manifest has no part '{name}'")
        src = intermediate_stl(Path(cfgs[name]["file"]))
        if not src.is_absolute():
            src = ROOT / src
        if not src.exists():
            raise FileNotFoundError(
                f"{src} missing — run `python3 assemble.py manifest.yaml` first")
        dst = MESH_OUT / f"{name}.stl"
        convert_to_binary_stl(src, dst)
        mesh_assets[name] = dst.name

    def head_meshes(parts, indent):
        return "\n".join(
            mesh_geom(n, world[n][0], world[n][1], COLORS.get(n, "0.7 0.7 0.7 1"), indent)
            for n in parts)

    asset_xml = "\n".join(
        f'    <mesh name="{n}" file="{f}" scale="{MM_TO_M} {MM_TO_M} {MM_TO_M}"/>'
        for n, f in mesh_assets.items())

    # ── static decoration (frame, bracket, feeder, motors) ──
    base_c = [WAX, sum(BASE_Y) / 2, BASE_TOP - BASE_THK / 2]
    base_h = [BASE_X_HALF, (BASE_Y[1] - BASE_Y[0]) / 2, BASE_THK / 2]

    static = []
    static.append("      <!-- base plate -->")
    static.append(box_geom(base_c, base_h, C_DARK))
    static.append("      <!-- U-bracket: foot + two bearing uprights -->")
    foot_c = [WAX, (BRK_Y[0] + BRK_Y[1]) / 2, BASE_TOP + 3]
    static.append(box_geom(foot_c, [BRK_HALF[0], (BRK_Y[1]-BRK_Y[0])/2 + 4, 3], C_ALU))
    for yu in BRK_Y:
        up_c = [WAX, yu, (BASE_TOP + WAZ + 6) / 2]
        up_h = [BRK_HALF[0], BRK_HALF[1], (WAZ + 6 - BASE_TOP) / 2]
        static.append(box_geom(up_c, up_h, C_ALU))
    static.append("      <!-- tube-rotation motor (geared stepper, belt to pulley) -->")
    static.append(box_geom(ROT_MOTOR_C, ROT_MOTOR_HALF, C_MOTOR))
    static.append(cyl_geom([ROT_MOTOR_C[0], PULLEY_Y, ROT_MOTOR_C[2]],
                           [ROT_MOTOR_C[0], PULLEY_Y + ROT_PULLEY_R*0 + PULLEY_W, ROT_MOTOR_C[2]],
                           ROT_PULLEY_R, C_PULLEY))
    static.append("      <!-- feeder body (1KGSSJ-B MIG wire feed) -->")
    feeder_c = [WAX, (FEEDER_Y[0] + FEEDER_Y[1]) / 2, WAZ]
    static.append(box_geom(feeder_c, FEEDER_HALF, C_DARK))
    static.append("      <!-- drive stepper (NEMA17) + shaft -->")
    static.append(box_geom(NEMA_C, NEMA_HALF, C_MOTOR))
    static.append(cyl_geom([NEMA_C[0] + NEMA_HALF[0], NEMA_C[1], NEMA_C[2]],
                           [NEMA_C[0] + NEMA_HALF[0] + NEMA_SHAFT_L, NEMA_C[1], NEMA_C[2]],
                           2.5, C_STEEL))
    static.append("      <!-- wire inlet guide -->")
    static.append(cyl_geom([WAX, FEEDER_Y[1], WAZ], [WAX, FEEDER_Y[1] + 18, WAZ],
                           4, C_STEEL))
    static_xml = "\n".join(static)

    # ── tube body (Axis 2): feed tube + pulley + bending-head meshes ──
    tube = []
    tube.append("      <!-- feed tube -->")
    tube.append(cyl_geom([WAX, TUBE_Y0, WAZ], [WAX, TUBE_Y1, WAZ], TUBE_OD/2, C_STEEL))
    tube.append("      <!-- drive pulley -->")
    tube.append(cyl_geom([WAX, PULLEY_Y - PULLEY_W/2, WAZ],
                         [WAX, PULLEY_Y + PULLEY_W/2, WAZ], PULLEY_R, C_PULLEY))
    tube.append("      <!-- bending head (real CAD meshes) -->")
    tube.append(head_meshes(TUBE_PARTS, 6))
    tube_xml = "\n".join(tube)

    tube_pivot = [WAX, 0, WAZ]
    bend_pivot = world["motor-flange"][1].copy()  # flange center; bend axis = world Z
    bend_pivot[1] = 6.0

    xml = f"""<mujoco model="wirebender">
  <!-- Generated by sim/make_mjcf.py. Bending head = real meshes; rest = rough primitives. -->
  <compiler angle="degree" meshdir="meshes" autolimits="true" inertiafromgeom="true"/>
  <option gravity="0 0 0" integrator="implicitfast"/>

  <visual>
    <global offwidth="1280" offheight="960" azimuth="120" elevation="-20"/>
    <quality shadowsize="2048"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.4 0.4 0.4"/>
  </visual>
  <statistic center="{WAX*MM_TO_M:.4g} {sum(BASE_Y)/2*MM_TO_M:.4g} {WAZ*MM_TO_M:.4g}" extent="0.25"/>

  <default>
    <joint damping="0.5"/>
    <position kp="8" forcerange="-80 80"/>
    <!-- collisions off: kinematic visualization -->
    <geom density="1000" contype="0" conaffinity="0"/>
  </default>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.25 0.3 0.4" rgb2="0.05 0.06 0.1"
             width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3"
             width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="10 10" reflectance="0.1"/>
{asset_xml}
  </asset>

  <worldbody>
    <light pos="0.2 0.1 0.5" dir="-0.3 -0.1 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="1 1 0.05" pos="0 {sum(BASE_Y)/2*MM_TO_M:.4g} {(BASE_TOP-BASE_THK)*MM_TO_M:.4g}"
          material="grid" contype="1" conaffinity="1"/>

    <!-- ───────── static frame / decoration ───────── -->
    <body name="frame">
{static_xml}
    </body>

    <!-- ───────── Axis 1: wire feed (slide along Y) ───────── -->
    <body name="wire" pos="{fmt(np.array([WAX,0,WAZ])*MM_TO_M)}">
      <joint name="feed" type="slide" axis="0 1 0" range="{FEED_RANGE_MM[0]*MM_TO_M:.4g} {FEED_RANGE_MM[1]*MM_TO_M:.4g}"/>
      <geom type="capsule" fromto="0 {WIRE_Y0*MM_TO_M:.4g} 0  0 {WIRE_Y1*MM_TO_M:.4g} 0"
            size="{WIRE_OD/2*MM_TO_M:.6g}" rgba="{C_WIRE}"/>
    </body>

    <!-- ───────── Axis 2: feed-tube rotation (bend direction) ───────── -->
    <body name="tube" pos="0 0 0">
      <joint name="tube_rot" type="hinge" axis="0 1 0" pos="{fmt(np.array(tube_pivot)*MM_TO_M)}"
             range="{TUBE_RANGE[0]} {TUBE_RANGE[1]}"/>
{tube_xml}

      <!-- ───────── Axis 3: bend head ───────── -->
      <body name="bend_flange" pos="0 0 0">
        <joint name="bend" type="hinge" axis="0 0 1" pos="{fmt(np.array(bend_pivot)*MM_TO_M)}"
               range="{BEND_RANGE[0]} {BEND_RANGE[1]}"/>
{head_meshes(FLANGE_PARTS, 8)}
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="feed"     joint="feed"     ctrlrange="{FEED_RANGE_MM[0]*MM_TO_M:.4g} {FEED_RANGE_MM[1]*MM_TO_M:.4g}" kp="200"/>
    <position name="tube_rot" joint="tube_rot" ctrlrange="{TUBE_RANGE[0]} {TUBE_RANGE[1]}"/>
    <position name="bend"     joint="bend"     ctrlrange="{BEND_RANGE[0]} {BEND_RANGE[1]}"/>
  </actuator>
</mujoco>
"""
    XML_OUT.write_text(xml)
    print(f"\nWrote {XML_OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
