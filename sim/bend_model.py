#!/usr/bin/env python3
"""
bend_model.py — Deterministic forward model of the wire bender (Route A).

Given a bending *program* (a sequence of the machine's three operations), walk a
moving frame along the wire and compute the resulting 3D shape as a polyline.
This predicts the part the machine will make — your G-code previewer.

Operations (map 1:1 to the GRBL axes):
    ("feed",   mm )   Axis 1 — advance the wire by `mm` along the current heading
    ("rotate", deg)   Axis 2 — roll the bend plane about the wire axis by `deg`
    ("bend",   deg)   Axis 3 — bend the wire by `deg` in the current bend plane

Moving frame (right-handed, orthonormal):
    h = heading      (unit tangent; the direction wire feeds)
    b = bend_dir     (unit; the direction a bend deflects the wire toward)
    a = h × b        (bend axis; bending rotates h toward b about a)
  feed   : p += h * d
  rotate : roll b about h            (chooses the plane of the next bend)
  bend   : rotate h and b about a    (turns the heading toward b)

Springback: a real bend relaxes after the shoe releases. We model the *realized*
angle as  commanded * (1 - springback).  `springback` is a placeholder to
calibrate against real 14/16 ga bends (see the repo README validation checklist).

Usage:
    python bend_model.py                       # list + simulate the examples
    MUJOCO_GL=osmesa python bend_model.py --png # render shapes to preview/shapes.png
    python bend_model.py square --springback 0.07
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "preview"


# ── core forward model ──────────────────────────────────────────────────────


def _rot(v, axis, ang):
    """Rodrigues rotation of vector v about unit-ish axis by ang radians."""
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def simulate(program, springback=0.0):
    """Run a program -> (N,3) polyline of points in mm (the wire centerline)."""
    p = np.zeros(3)
    h = np.array([1.0, 0.0, 0.0])   # heading (wire lays out along +X to start)
    b = np.array([0.0, 1.0, 0.0])   # initial bends deflect toward +Y (XY plane)
    pts = [p.copy()]
    for op, val in program:
        if op == "feed":
            p = p + h * float(val)
            pts.append(p.copy())
        elif op == "rotate":
            b = _rot(b, h, math.radians(float(val)))
        elif op == "bend":
            a = np.cross(h, b)
            ang = math.radians(float(val) * (1.0 - springback))
            h = _rot(h, a, ang)
            b = _rot(b, a, ang)
        else:
            raise ValueError(f"unknown op: {op!r}")
    return np.array(pts)


def total_length(pts):
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


# ── example programs ────────────────────────────────────────────────────────

EXAMPLES = {
    # planar closed square (sanity check: should return to the origin)
    "square": [("feed", 30), ("bend", 90)] * 3 + [("feed", 30)],
    # a 2D "staple" / bracket
    "staple": [("feed", 25), ("bend", 90), ("feed", 40), ("bend", 90), ("feed", 25)],
    # 3D: bend, roll the plane 90, bend again -> leaves the plane (like a chair leg)
    "chair": [
        ("feed", 30), ("bend", 90), ("feed", 30),
        ("rotate", 90), ("bend", 90), ("feed", 30),
        ("rotate", 90), ("bend", 90), ("feed", 30),
    ],
    # a helix-ish coil: many small bends with a constant roll between them
    "coil": sum(([("feed", 8), ("bend", 40), ("rotate", 35)] for _ in range(12)), []),
}


# ── visualization (MuJoCo capsules, headless via osmesa) ────────────────────


PALETTE = ["0.85 0.85 0.9 1", "0.95 0.6 0.25 1",
           "0.35 0.75 0.95 1", "0.55 0.85 0.4 1"]


def _seg_geoms(pts, radius_mm, rgba):
    mm = 0.001
    out = []
    for i in range(len(pts) - 1):
        a, b = pts[i] * mm, pts[i + 1] * mm
        if np.linalg.norm(b - a) < 1e-9:
            continue
        out.append(
            f'      <geom type="capsule" fromto="{a[0]:.6g} {a[1]:.6g} {a[2]:.6g} '
            f'{b[0]:.6g} {b[1]:.6g} {b[2]:.6g}" size="{radius_mm*mm:.6g}" rgba="{rgba}"/>')
    return out


def to_mjcf(items, radius_mm=0.8):
    """items: a single (N,3) polyline, or a list of (N,3) polylines (mm)."""
    if isinstance(items, np.ndarray):
        items = [items]
    mm = 0.001
    geoms = []
    for i, pts in enumerate(items):
        geoms += _seg_geoms(pts, radius_mm, PALETTE[i % len(PALETTE)])
    allpts = np.vstack(items)
    c = allpts.mean(axis=0) * mm
    ext = float(np.max(allpts.max(0) - allpts.min(0))) * mm
    return f"""<mujoco model="wireshape">
  <option gravity="0 0 0"/>
  <visual><global offwidth="1280" offheight="960"/>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.4 0.4 0.4"/></visual>
  <statistic center="{c[0]:.5g} {c[1]:.5g} {c[2]:.5g}" extent="{max(ext,0.01):.5g}"/>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.25 0.3 0.4" rgb2="0.05 0.06 0.1"
             width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.18 0.2" rgb2="0.26 0.26 0.3"
             width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0.1 0.1 0.5" dir="-0.2 -0.2 -1"/>
    <geom type="plane" size="2 2 0.05" pos="0 0 {(allpts.min(0)[2]*mm-0.005):.5g}" material="grid"/>
    <body>
{chr(10).join(geoms)}
    </body>
  </worldbody>
</mujoco>
"""


def layout(names, springback=0.0, gap_mm=40):
    """Simulate each example and lay them out in a row along Y, recentered."""
    shapes = []
    y = 0.0
    for name in names:
        pts = simulate(EXAMPLES[name], springback=springback)
        pts = pts - pts.mean(axis=0)            # center on its own origin
        span = pts.max(0)[1] - pts.min(0)[1]
        pts = pts + np.array([0, y + span / 2, 0])
        shapes.append(pts)
        y += span + gap_mm
    return shapes


def render_shapes(names, springback=0.0, png=OUT / "shapes.png"):
    import mujoco
    from PIL import Image

    tiles = []
    for name in names:
        pts = simulate(EXAMPLES[name], springback=springback)
        model = mujoco.MjModel.from_xml_string(to_mjcf(pts))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        r = mujoco.Renderer(model, 480, 480)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, cam)
        cam.azimuth, cam.elevation, cam.distance = 130, -22, cam.distance * 1.1
        r.update_scene(data, cam)
        tiles.append(r.render())
        r.close()
    grid = np.concatenate(tiles, axis=1)
    OUT.mkdir(exist_ok=True)
    Image.fromarray(grid).save(png)
    print(f"wrote {png}")


def view_shapes(names, springback=0.0):
    """Open the predicted shape(s) in the interactive MuJoCo viewer (needs a display)."""
    import mujoco
    import mujoco.viewer

    shapes = layout(names, springback=springback) if len(names) > 1 \
        else [simulate(EXAMPLES[names[0]], springback=springback)]
    model = mujoco.MjModel.from_xml_string(to_mjcf(shapes))
    data = mujoco.MjData(model)
    print(f"viewing: {', '.join(names)}  (orbit: drag · zoom: scroll · close window to exit)")
    mujoco.viewer.launch(model, data)


# ── CLI ─────────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("example", nargs="?", default=None,
                    help=f"one of: {', '.join(EXAMPLES)} (default: all)")
    ap.add_argument("--springback", type=float, default=0.0,
                    help="fraction a bend relaxes (0 = perfect; calibrate to material)")
    ap.add_argument("--png", action="store_true", help="render shape(s) to preview/")
    ap.add_argument("--view", action="store_true",
                    help="open in the interactive MuJoCo viewer (needs a display)")
    args = ap.parse_args()

    names = [args.example] if args.example else list(EXAMPLES)
    for name in names:
        pts = simulate(EXAMPLES[name], springback=args.springback)
        closed = np.linalg.norm(pts[0] - pts[-1]) < 1e-6
        print(f"{name:8s}  pts={len(pts):3d}  length={total_length(pts):6.1f}mm  "
              f"{'closed' if closed else 'open':6s}  end={np.round(pts[-1],1)}")
    if args.png:
        render_shapes(names, springback=args.springback)
    if args.view:
        view_shapes(names, springback=args.springback)


if __name__ == "__main__":
    main()
