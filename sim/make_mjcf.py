#!/usr/bin/env python3
"""
make_mjcf.py — Generate a MuJoCo model (MJCF) of the wire bender from the
same manifest the CAD assembly uses, so the sim stays in sync with the design.

What it does:
  1. Reads ../manifest.yaml and resolves each part's world transform using the
     exact same math as assemble.py (pure numpy: xform + anchor mating).
  2. Converts the build/ STL meshes (ASCII, from OpenSCAD) to binary STL in
     sim/meshes/ — MuJoCo's STL reader wants binary.
  3. Writes sim/wirebender.xml: a 2-DOF articulated model.

Kinematic tree (see README "Machine Architecture"):
    world
     └─ tube   : hinge about the wire/feed-tube axis  (Axis 2: bend direction)
        │        carries the motor + bender-head mount (they rotate with the tube)
        └─ bend_flange : hinge about the motor shaft   (Axis 3: bend degree)
                         carries the bending flange (motor-flange)

Axis 1 (wire feed) only translates wire, which isn't modeled yet, so it has no
geometry here. The feed slide joint is left commented in the XML, ready to drive
a wire body once one exists.

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
    """Mirror assemble.py: where the build/ STL for a given source part lives."""
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
    mesh.export(str(dst), file_type="stl")  # trimesh writes binary STL by default
    print(f"  mesh {src.name:24s} -> meshes/{dst.name}  ({len(mesh.vertices)} verts)")


# ── Quaternion helper (MuJoCo order: w x y z) ───────────────────────────────


def mat_to_quat(R):
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def geom_xml(name, R, t_mm, rgba):
    """A mesh geom placed at its world pose (positions converted mm->m)."""
    pos = t_mm * MM_TO_M
    quat = mat_to_quat(R)
    return (
        f'      <geom type="mesh" mesh="{name}" pos="{fmt(pos)}" '
        f'quat="{fmt(quat)}" rgba="{rgba}"/>'
    )


# ── Main ────────────────────────────────────────────────────────────────────

# Which manifest part goes on which moving body.
TUBE_PARTS = ["motor", "bender-head"]   # rotate together with the feed tube
FLANGE_PARTS = ["motor-flange"]         # the bending flange on the motor shaft

# Joint pivots, in CAD/world mm (see README + bender-head.scad geometry):
#   tube axis: wire channel at x=tube_dia(6.35), z=height/2(5.5), running along Y
#   bend axis: motor shaft / flange bore at x=0, y=6, running along Z
TUBE_PIVOT_MM = np.array([6.35, 0.0, 5.5])
BEND_PIVOT_MM = np.array([0.0, 6.0, 0.0])

# Slider travel, in degrees. The MuJoCo viewer spreads the whole range across
# the slider width, so a smaller range = finer (less twitchy) control.
# Widen these if you need more reach (e.g. full feed-tube rotation = -180 180).
TUBE_RANGE = (-90, 90)   # feed-tube rotation / bend direction
BEND_RANGE = (0, 150)    # bend degree (wire bends one way, 0 -> ~150 deg)

COLORS = {
    "motor": "0.30 0.32 0.36 1",
    "bender-head": "0.20 0.55 0.85 1",
    "motor-flange": "0.90 0.55 0.15 1",
}


def main():
    manifest = yaml.safe_load(MANIFEST.read_text())
    world, cfgs = resolve_world_transforms(manifest)
    active = [n for n in world if not str(cfgs[n].get("file", "")).startswith("#")]

    print("Resolving meshes:")
    mesh_assets = {}
    for name in TUBE_PARTS + FLANGE_PARTS:
        if name not in cfgs:
            raise RuntimeError(f"manifest has no part '{name}'")
        src = intermediate_stl(Path(cfgs[name]["file"]))
        if not src.is_absolute():
            src = ROOT / src
        if not src.exists():
            raise FileNotFoundError(
                f"{src} missing — run `python3 assemble.py manifest.yaml` first to build it"
            )
        dst = MESH_OUT / f"{name}.stl"
        convert_to_binary_stl(src, dst)
        mesh_assets[name] = dst.name

    def geoms_for(parts):
        return "\n".join(
            geom_xml(n, world[n][0], world[n][1], COLORS.get(n, "0.7 0.7 0.7 1"))
            for n in parts
        )

    tube_pivot = TUBE_PIVOT_MM * MM_TO_M
    bend_pivot = BEND_PIVOT_MM * MM_TO_M

    # meshes are in mm; scale to meters to match the (mm->m) geom positions.
    asset_xml = "\n".join(
        f'    <mesh name="{n}" file="{f}" scale="{MM_TO_M} {MM_TO_M} {MM_TO_M}"/>'
        for n, f in mesh_assets.items()
    )

    xml = f"""<mujoco model="wirebender">
  <!-- Generated by sim/make_mjcf.py from manifest.yaml — do not edit by hand. -->
  <compiler angle="degree" meshdir="meshes" autolimits="true" inertiafromgeom="true"/>
  <option gravity="0 0 0" integrator="implicitfast"/>

  <visual>
    <global offwidth="1280" offheight="960" azimuth="35" elevation="-20"/>
    <quality shadowsize="2048"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.4 0.4 0.4"/>
  </visual>
  <statistic center="0 0.005 0.007" extent="0.06"/>

  <default>
    <joint damping="0.5"/>
    <position kp="5" forcerange="-50 50"/>
    <geom density="1000"/>
  </default>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.25 0.3 0.4" rgb2="0.05 0.06 0.1"
             width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3"
             width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
{asset_xml}
  </asset>

  <worldbody>
    <light pos="0.2 -0.2 0.4" dir="-0.4 0.4 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="1 1 0.05" pos="0 0 -0.05"
          material="grid"/>

    <!-- Axis 2: feed-tube rotation (bend direction). Hinge about the wire axis (Y). -->
    <body name="tube" pos="0 0 0">
      <joint name="tube_rot" type="hinge" axis="0 1 0" pos="{fmt(tube_pivot)}"
             range="{TUBE_RANGE[0]} {TUBE_RANGE[1]}"/>
{geoms_for(TUBE_PARTS)}

      <!-- Axis 3: bend head. Hinge about the motor shaft (Z). -->
      <body name="bend_flange" pos="0 0 0">
        <joint name="bend" type="hinge" axis="0 0 1" pos="{fmt(bend_pivot)}"
               range="{BEND_RANGE[0]} {BEND_RANGE[1]}"/>
{geoms_for(FLANGE_PARTS)}
      </body>
    </body>

    <!-- Axis 1: wire feed (slide along the tube/Y axis). No wire geometry yet;
         add a wire body here and uncomment to drive it.
    <body name="wire" pos="{fmt(tube_pivot)}">
      <joint name="feed" type="slide" axis="0 1 0" range="-0.2 0.2"/>
      <geom type="capsule" fromto="0 0 0  0 -0.1 0" size="0.0008" rgba="0.8 0.8 0.85 1"/>
    </body>
    -->
  </worldbody>

  <actuator>
    <position name="tube_rot" joint="tube_rot" ctrlrange="{TUBE_RANGE[0]} {TUBE_RANGE[1]}"/>
    <position name="bend"     joint="bend"     ctrlrange="{BEND_RANGE[0]} {BEND_RANGE[1]}"/>
  </actuator>
</mujoco>
"""
    XML_OUT.write_text(xml)
    print(f"\nWrote {XML_OUT.relative_to(ROOT)}")
    print("World transforms (mm):")
    for n in active:
        R, t = world[n]
        print(f"  {n:14s} t={fmt(t)}")


if __name__ == "__main__":
    main()
