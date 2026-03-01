#!/usr/bin/env python3
"""
cad_assembly.py — Programmatic "mish-mash" CAD assembly builder.

Goals:
- Take a manifest (YAML or JSON) describing parts + transforms
- Accept input formats: STEP, STL, GLB/GLTF, OpenSCAD (.scad)
- Optionally accept CadQuery generator scripts (.py) that output a cq.Assembly or cq.Shape
- Build a CadQuery Assembly and export:
    - assembly.step (best for CAD)
    - assembly.glb  (best for visual inspection)

Reality check:
- STL/GLB are meshes. Converting meshes to true analytic STEP solids is *not* automatic here.
  This script will embed faceted geometry in STEP (still useful for placement/assembly),
  but it won’t magically rebuild NURBS/BReps. For true reverse-engineered solids, you’d
  use DesignX/Fusion/FreeCAD reconstruction.

Dependencies:
  pip install cadquery trimesh numpy pyyaml

Optional:
- OpenSCAD CLI installed (for .scad -> .stl):
  macOS: brew install openscad

Usage:
  python cad_assembly.py manifest.yaml --out assembly.step --outglb assembly.glb

Manifest example (YAML):
  units: "mm"          # "mm" or "inch"
  out_dir: "build"
  parts:
    - name: "bracket"
      file: "assets/bracket.step"
      xform: {t: [0,0,0], r_deg: [0,0,0], s: 1.0}

    - name: "motor_mesh"
      file: "assets/motor.glb"
      xform: {t: [50,0,0], r_deg: [0,0,90], s: 1.0}

    - name: "laser_panel"
      file: "assets/panel.scad"
      scad:
        defines: {thickness: 6, w: 200, h: 120}
      xform: {t: [0,80,0], r_deg: [90,0,0], s: 1.0}

    - name: "generated_part"
      file: "assets/gen_part.py"
      cq:
        entry: "make"   # function returning cq.Shape or cq.Workplane or cq.Assembly
      xform: {t: [0,0,30], r_deg: [0,0,0], s: 1.0}
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

try:
    import yaml  # type: ignore
except Exception:
    yaml = None

import trimesh  # type: ignore
import cadquery as cq  # type: ignore


# ----------------------------
# Helpers: IO / manifest
# ----------------------------

def load_manifest(path: Path) -> Dict[str, Any]:
    data = path.read_text(encoding="utf-8")
    if path.suffix.lower() in [".yaml", ".yml"]:
        if yaml is None:
            raise RuntimeError("pyyaml not installed. Run: pip install pyyaml")
        return yaml.safe_load(data)
    if path.suffix.lower() == ".json":
        return json.loads(data)
    raise ValueError(f"Unsupported manifest type: {path.suffix} (use .yaml/.yml/.json)")


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def which(cmd: str) -> Optional[str]:
    return shutil.which(cmd)


# ----------------------------
# Units and transforms
# ----------------------------

def unit_scale(units: str) -> float:
    u = (units or "mm").lower()
    if u in ("mm", "millimeter", "millimeters"):
        return 1.0
    if u in ("in", "inch", "inches"):
        return 25.4  # inch -> mm
    raise ValueError(f"Unsupported units: {units}")


def deg2rad(v: Union[float, int]) -> float:
    return float(v) * math.pi / 180.0


def xform_to_location(
    t: Tuple[float, float, float],
    r_deg: Tuple[float, float, float],
    s: float,
    units_mm_per_unit: float,
) -> cq.Location:
    """
    CadQuery Location:
    - translation in mm
    - rotation is applied in ZYX order here (yaw/pitch/roll-ish)
    """
    tx, ty, tz = [float(x) * units_mm_per_unit for x in t]
    rx, ry, rz = [deg2rad(x) for x in r_deg]

    # CadQuery uses gp_Trsf via Location; easiest is combine:
    loc = cq.Location()

    # scale: CadQuery Location doesn't do uniform scale directly.
    # We'll scale geometry itself pre-add (for meshes + shapes).
    # Translation + rotation in loc only.
    # Rotation order: X then Y then Z.
    if abs(rx) > 1e-12:
        loc = loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), rx)
    if abs(ry) > 1e-12:
        loc = loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), ry)
    if abs(rz) > 1e-12:
        loc = loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), rz)

    loc = cq.Location(cq.Vector(tx, ty, tz)) * loc
    return loc


def xform_to_matrix(
    t: Tuple[float, float, float],
    r_deg: Tuple[float, float, float],
    units_mm_per_unit: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Returns (R 3×3, t_mm 3-vector) matching the rotation convention of xform_to_location."""
    tx, ty, tz = [float(x) * units_mm_per_unit for x in t]
    rx, ry, rz = [deg2rad(x) for x in r_deg]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return Rx @ Ry @ Rz, np.array([tx, ty, tz], dtype=float)


def rotation_matrix_from_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minimal rotation matrix that rotates unit vector a onto unit vector b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(np.dot(a, b))
    if abs(c + 1.0) < 1e-9:  # antiparallel: 180° around any perpendicular axis
        perp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        ax = np.cross(a, perp)
        ax /= np.linalg.norm(ax)
        return 2.0 * np.outer(ax, ax) - np.eye(3)
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    kmat = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + kmat + kmat @ kmat * ((1.0 - c) / (s * s + 1e-30))


def matrix_to_location(R: np.ndarray, t: np.ndarray) -> cq.Location:
    """Build a cq.Location from a 3×3 rotation matrix R and translation vector t (mm)."""
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    trsf = gp_Trsf()
    trsf.SetValues(
        float(R[0, 0]), float(R[0, 1]), float(R[0, 2]), float(t[0]),
        float(R[1, 0]), float(R[1, 1]), float(R[1, 2]), float(t[1]),
        float(R[2, 0]), float(R[2, 1]), float(R[2, 2]), float(t[2]),
    )
    return cq.Location(TopLoc_Location(trsf))


def resolve_mate(
    p: Dict[str, Any],
    part_world: Dict[str, Tuple[np.ndarray, np.ndarray]],
    part_anchors: Dict[str, Dict[str, Any]],
    units_mm: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute world (R, t_mm) for part p using its first mate constraint.

    The part's xform.r_deg is applied as a pre-rotation before mating,
    letting you orient a part's "natural" axis to match the mount direction.
    xform.t is ignored when mates are defined (position comes from the mate).
    """
    mates = p.get("mates") or []
    xform = p.get("xform", {}) or {}
    r_deg = xform.get("r_deg", [0, 0, 0])
    my_name = p.get("name", "?")
    my_anchors = p.get("anchors") or {}

    R_pre, _ = xform_to_matrix([0, 0, 0], r_deg, units_mm)

    mate = mates[0]
    my_anchor_name = mate.get("my_anchor")
    to_part_name = mate.get("to_part")
    to_anchor_name = mate.get("to_anchor")

    if my_anchor_name not in my_anchors:
        raise RuntimeError(f"Part '{my_name}': anchor '{my_anchor_name}' not defined")
    my_anch = my_anchors[my_anchor_name]
    P_me = np.array(my_anch.get("t", [0, 0, 0]), dtype=float) * units_mm
    AX_me = np.array(my_anch.get("axis", [0, 0, 1]), dtype=float)
    AX_me /= np.linalg.norm(AX_me)

    R_target, t_target = part_world[to_part_name]
    to_anch = part_anchors[to_part_name].get(to_anchor_name)
    if to_anch is None:
        raise RuntimeError(f"Part '{to_part_name}': anchor '{to_anchor_name}' not defined")
    P_to = np.array(to_anch.get("t", [0, 0, 0]), dtype=float) * units_mm
    AX_to = np.array(to_anch.get("axis", [0, 0, 1]), dtype=float)
    AX_to /= np.linalg.norm(AX_to)

    P_to_world = R_target @ P_to + t_target
    AX_to_world = R_target @ AX_to

    P_me_pre = R_pre @ P_me
    AX_me_pre = R_pre @ AX_me

    R_align = rotation_matrix_from_vectors(AX_me_pre, AX_to_world)
    R_world = R_align @ R_pre
    t_world = P_to_world - R_align @ P_me_pre

    return R_world, t_world


def mesh_apply_transform(
    mesh: trimesh.Trimesh,
    t: Tuple[float, float, float],
    r_deg: Tuple[float, float, float],
    s: float,
    units_mm_per_unit: float,
) -> trimesh.Trimesh:
    """
    Apply scale, rotation (XYZ), translation, and units to a Trimesh.
    Outputs mesh in mm space.
    """
    # uniform scale + units
    scale = float(s) * float(units_mm_per_unit)
    mesh.apply_scale(scale)

    # rotations
    rx, ry, rz = [deg2rad(x) for x in r_deg]
    if abs(rx) > 1e-12:
        mesh.apply_transform(trimesh.transformations.rotation_matrix(rx, [1, 0, 0]))
    if abs(ry) > 1e-12:
        mesh.apply_transform(trimesh.transformations.rotation_matrix(ry, [0, 1, 0]))
    if abs(rz) > 1e-12:
        mesh.apply_transform(trimesh.transformations.rotation_matrix(rz, [0, 0, 1]))

    # translation in mm
    tx, ty, tz = [float(x) * float(units_mm_per_unit) for x in t]
    mesh.apply_translation([tx, ty, tz])
    return mesh


# ----------------------------
# Part loaders / converters
# ----------------------------

def convert_scad_to_stl(scad_path: Path, out_stl: Path, defines: Dict[str, Any] | None) -> None:
    openscad = (
        which("openscad")
        or (
            "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"
            if os.path.exists("/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD")
            else None
        )
    )
    if not openscad:
        raise RuntimeError(
            f"OpenSCAD CLI not found, needed to convert {scad_path.name}. "
            f"Install OpenSCAD or convert to STL yourself."
        )

    cmd = [openscad, "-o", str(out_stl)]
    if defines:
        for k, v in defines.items():
            # OpenSCAD -D supports expressions; strings should be quoted
            if isinstance(v, str):
                expr = f'{k}="{v}"'
            elif isinstance(v, bool):
                expr = f"{k}={str(v).lower()}"
            else:
                expr = f"{k}={v}"
            cmd += ["-D", expr]
    cmd += [str(scad_path)]
    subprocess.run(cmd, check=True)


def load_mesh_any(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(str(path), force="mesh")
    if isinstance(mesh, trimesh.Scene):
        # If it's a scene, merge into one mesh
        mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError(f"Could not load mesh: {path}")
    return mesh


def load_cq_generator(py_path: Path, entry: str = "make") -> Union[cq.Assembly, cq.Shape]:
    """
    Runs a CadQuery generator python file.
    The file should define a function named `entry` that returns:
      - cq.Assembly OR
      - cq.Workplane OR
      - cq.Shape
    """
    # Execute in a clean globals dict with cadquery imported as cq
    g: Dict[str, Any] = {"cq": cq}
    code = py_path.read_text(encoding="utf-8")
    exec(compile(code, str(py_path), "exec"), g, g)

    if entry not in g or not callable(g[entry]):
        raise RuntimeError(f"{py_path.name} must define a callable '{entry}()'")

    obj = g[entry]()
    if isinstance(obj, cq.Assembly):
        return obj
    if isinstance(obj, cq.Workplane):
        return obj.val()
    if isinstance(obj, cq.Shape):
        return obj
    raise RuntimeError(f"{py_path.name}:{entry} returned unsupported type: {type(obj)}")


def cq_shape_from_mesh(mesh: trimesh.Trimesh, tmp_stl: Path) -> cq.Shape:
    """
    CadQuery doesn't import GLB directly. We convert mesh -> STL -> OCC StlAPI_Reader.
    """
    from OCP.StlAPI import StlAPI_Reader
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Shape

    mesh.export(str(tmp_stl))
    occ_shape = TopoDS_Shape()
    builder = BRep_Builder()
    reader = StlAPI_Reader()
    reader.Read(occ_shape, str(tmp_stl))
    return cq.Shape(occ_shape)


def cq_load_part(
    part_file: Path,
    part_cfg: Dict[str, Any],
    build_dir: Path,
    units_mm_per_unit: float,
) -> Tuple[Union[cq.Shape, cq.Assembly], Optional[str]]:
    """
    Returns (shape_or_assembly, kind) where kind is "assembly" or None
    """
    suffix = part_file.suffix.lower()

    # transforms in manifest
    xform = part_cfg.get("xform", {}) or {}
    t = tuple(xform.get("t", [0, 0, 0]))
    r = tuple(xform.get("r_deg", [0, 0, 0]))
    s = float(xform.get("s", 1.0))

    if suffix in [".step", ".stp"]:
        shp = cq.importers.importStep(str(part_file))
        # importStep returns Workplane; take first shape
        shape = shp.val()
        # apply scale to shape itself (since Location has no scale)
        if abs(s * units_mm_per_unit - 1.0) > 1e-12:
            shape = shape.scale(s * units_mm_per_unit)
        # apply translation+rotation via Location later (assembly add loc)
        return shape, None

    if suffix == ".stl":
        # STL is mesh -> faceted shell/solid representation in cq
        from OCP.StlAPI import StlAPI_Reader
        from OCP.BRep import BRep_Builder
        from OCP.TopoDS import TopoDS_Shape
        occ_shape = TopoDS_Shape()
        BRep_Builder()
        StlAPI_Reader().Read(occ_shape, str(part_file))
        shape = cq.Shape(occ_shape)
        if abs(s * units_mm_per_unit - 1.0) > 1e-12:
            shape = shape.scale(s * units_mm_per_unit)
        return shape, None

    if suffix in [".glb", ".gltf", ".obj", ".ply"]:
        mesh = load_mesh_any(part_file)
        # Only bake scale; rotation+translation are applied via Location in build_assembly
        mesh.apply_scale(float(s) * float(units_mm_per_unit))
        tmp_stl = build_dir / f"{part_file.stem}.converted.stl"
        shape = cq_shape_from_mesh(mesh, tmp_stl)
        return shape, None

    if suffix == ".scad":
        defines = (part_cfg.get("scad", {}) or {}).get("defines", None)
        tmp_stl = build_dir / f"{part_file.stem}.scad.stl"
        convert_scad_to_stl(part_file, tmp_stl, defines)
        from OCP.StlAPI import StlAPI_Reader
        from OCP.TopoDS import TopoDS_Shape
        occ_shape = TopoDS_Shape()
        StlAPI_Reader().Read(occ_shape, str(tmp_stl))
        shape = cq.Shape(occ_shape)
        # apply placement via Location later; apply scale via shape.scale
        if abs(s * units_mm_per_unit - 1.0) > 1e-12:
            shape = shape.scale(s * units_mm_per_unit)
        return shape, None

    if suffix == ".py":
        entry = (part_cfg.get("cq", {}) or {}).get("entry", "make")
        obj = load_cq_generator(part_file, entry=entry)
        # If it’s an Assembly, we’ll nest it later.
        if isinstance(obj, cq.Assembly):
            return obj, "assembly"
        shape = obj
        if abs(s * units_mm_per_unit - 1.0) > 1e-12:
            shape = shape.scale(s * units_mm_per_unit)
        return shape, None

    raise ValueError(f"Unsupported part file type: {part_file}")


# ----------------------------
# Main build
# ----------------------------

def build_assembly(manifest: Dict[str, Any], manifest_path: Path) -> Tuple[cq.Assembly, Path]:
    units = manifest.get("units", "mm")
    units_mm = unit_scale(units)

    out_dir = Path(manifest.get("out_dir", "build"))
    if not out_dir.is_absolute():
        out_dir = manifest_path.parent / out_dir
    ensure_dir(out_dir)

    assy = cq.Assembly(name=manifest.get("name", "root"))

    parts = manifest.get("parts", [])
    if not isinstance(parts, list) or not parts:
        raise RuntimeError("Manifest must contain a non-empty 'parts' list")

    # Validate and index parts
    part_cfgs: Dict[str, Dict[str, Any]] = {}
    part_order: List[str] = []
    for p in parts:
        if not isinstance(p, dict):
            raise RuntimeError("Each item in 'parts' must be an object/dict")
        name = p.get("name")
        file_ = p.get("file")
        if not name or not file_:
            raise RuntimeError("Each part needs 'name' and 'file'")
        part_path = Path(file_)
        if not part_path.is_absolute():
            part_path = manifest_path.parent / part_path
        if not part_path.exists():
            raise FileNotFoundError(part_path)
        cfg = dict(p)
        cfg["_path"] = part_path
        part_cfgs[name] = cfg
        part_order.append(name)

    # Phase 1: Load all shapes
    shapes: Dict[str, Tuple[Any, Optional[str]]] = {}
    for name in part_order:
        obj, kind = cq_load_part(part_cfgs[name]["_path"], part_cfgs[name], out_dir, units_mm)
        shapes[name] = (obj, kind)

    # Phase 2: Resolve world transforms (topological, supports mate dependencies)
    part_anchors: Dict[str, Dict[str, Any]] = {
        name: (part_cfgs[name].get("anchors") or {}) for name in part_order
    }
    part_world: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    remaining = list(part_order)
    for _ in range(len(part_order) + 1):
        if not remaining:
            break
        deferred: List[str] = []
        for name in remaining:
            p = part_cfgs[name]
            xform = p.get("xform", {}) or {}
            t = tuple(xform.get("t", [0, 0, 0]))
            r = tuple(xform.get("r_deg", [0, 0, 0]))
            mates = p.get("mates") or []
            if not mates:
                part_world[name] = xform_to_matrix(t, r, units_mm)
            elif all(m.get("to_part") in part_world for m in mates):
                part_world[name] = resolve_mate(p, part_world, part_anchors, units_mm)
            else:
                deferred.append(name)
        remaining = deferred
    if remaining:
        raise RuntimeError(f"Circular or unresolvable mate dependencies: {remaining}")

    # Phase 3: Add parts to assembly with resolved locations
    for name in part_order:
        obj, kind = shapes[name]
        R, t_mm = part_world[name]
        assy.add(obj, name=name, loc=matrix_to_location(R, t_mm))

    return assy, out_dir


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest", type=str, help="Path to manifest.yaml/.yml/.json")
    ap.add_argument("--out", type=str, default="assembly.step", help="Output STEP path")
    ap.add_argument("--outglb", type=str, default="", help="Optional output GLB path")
    args = ap.parse_args()

    manifest_path = Path(args.manifest).resolve()
    manifest = load_manifest(manifest_path)

    assy, out_dir = build_assembly(manifest, manifest_path)

    out_step = Path(args.out)
    if not out_step.is_absolute():
        out_step = out_dir / out_step.name

    assy.save(str(out_step))
    print(f"Wrote STEP: {out_step}")

    if args.outglb:
        out_glb = Path(args.outglb)
        if not out_glb.is_absolute():
            out_glb = out_dir / out_glb.name
        # CadQuery exports GLB via exporters (works for quick visual sanity checks)
        try:
            assy.save(str(out_glb))
            print(f"Wrote GLB:  {out_glb}")
        except Exception as e:
            print(f"GLB export failed (STEP still written): {e}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())