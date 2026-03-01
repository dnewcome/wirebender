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
from typing import Any, Dict, Optional, Tuple, Union

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
        # apply scale/rot/trans directly to mesh so we don't need loc scale
        mesh = mesh_apply_transform(mesh, t, r, s, units_mm_per_unit)
        tmp_stl = build_dir / f"{part_file.stem}.converted.stl"
        shape = cq_shape_from_mesh(mesh, tmp_stl)
        # already transformed into place; we'll add with identity loc
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

        xform = p.get("xform", {}) or {}
        t = tuple(xform.get("t", [0, 0, 0]))
        r = tuple(xform.get("r_deg", [0, 0, 0]))
        s = float(xform.get("s", 1.0))

        obj, kind = cq_load_part(part_path, p, out_dir, units_mm)

        if kind == "assembly" and isinstance(obj, cq.Assembly):
            # Nest assembly with placement
            loc = xform_to_location(t, r, 1.0, units_mm_per_unit=units_mm)
            assy.add(obj, name=name, loc=loc)
            continue

        # For GLB/mesh path, we already baked transform into mesh; detect that by suffix
        baked = part_path.suffix.lower() in [".glb", ".gltf", ".obj", ".ply"]

        loc = cq.Location() if baked else xform_to_location(t, r, 1.0, units_mm_per_unit=units_mm)
        # Note: scale already applied to shape in loader (shape.scale), except baked meshes.
        assy.add(obj, name=name, loc=loc)

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