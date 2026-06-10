#!/usr/bin/env python3
"""
animate_bend.py — Animate a bending program ON the machine (Route A → motion).

Couples the deterministic forward model to the live machine: as the program runs,
the feed-tube rotation and bend flange move, and the formed wire grows out of the
bending head — in the *machine frame* (the head/bending point stays fixed; the
already-formed wire trails out and swings about the bending point as each bend
forms, exactly like the real machine pushing wire past a fixed mandrel).

    MUJOCO_GL=osmesa ../py/bin/python animate_bend.py            # staple -> preview/bending.gif
    MUJOCO_GL=osmesa ../py/bin/python animate_bend.py chair
    DISPLAY=:0       ../py/bin/python animate_bend.py chair --view  # live playback

Programs come from bend_model.EXAMPLES (feed / rotate / bend ops).
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import mujoco

import make_mjcf as mk
from bend_model import EXAMPLES

HERE = Path(__file__).resolve().parent
OUT = HERE / "preview"

# Bending point at the head (on the wire axis), and the wire's exit direction.
B = np.array([mk.WAX, 0.0, mk.WAZ])      # mandrel / bend point (fixed in world)
E = np.array([0.0, -1.0, 0.0])           # formed wire trails in -Y
B0 = np.array([1.0, 0.0, 0.0])           # bend-deflect direction at tube_rot = 0
YAXIS = np.array([0.0, 1.0, 0.0])        # feed-tube rotation axis

C_FORMED = "0.78 0.80 0.85 1"
C_UNFORMED = "0.55 0.57 0.62 1"


def _rot(v, axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def _rot_about(p, center, axis, ang):
    return center + _rot(p - center, axis, ang)


def frames_for(program, springback=0.0, feed_steps=6, bend_steps=8, rot_steps=8):
    """Step the program in the machine frame.

    Yields (tube_deg, bend_deg, W) per animation frame, where W is the formed-wire
    polyline (Nx3, mm, world) and tube_deg/bend_deg drive the machine joints.
    """
    W = [B.copy()]      # W[0] is the live point at the bending head B
    tube = 0.0          # absolute feed-tube angle (deg)
    out = [(tube, 0.0, np.array(W))]

    for op, val in program:
        if op == "feed":
            d = float(val)
            base = [p.copy() for p in W]
            for s in range(1, feed_steps + 1):
                t = s / feed_steps
                W = [B.copy()] + [p + E * d * t for p in base]
                out.append((tube, 0.0, np.array(W)))

        elif op == "rotate":
            t0, t1 = tube, tube + float(val)
            for s in range(1, rot_steps + 1):
                tube = t0 + (t1 - t0) * s / rot_steps
                out.append((tube, 0.0, np.array(W)))
            tube = t1

        elif op == "bend":
            a = math.radians(float(val) * (1.0 - springback))
            b = _rot(B0, YAXIS, math.radians(tube))
            axis = np.cross(E, b)               # bend axis (= rotated flange Z)
            base = [p.copy() for p in W]
            for s in range(1, bend_steps + 1):   # shoe pushes in: wire bends 0 -> a
                ang = a * s / bend_steps
                W = [_rot_about(p, B, axis, ang) for p in base]
                out.append((tube, math.degrees(ang), np.array(W)))
            held = [p.copy() for p in W]
            for s in range(bend_steps - 1, -1, -1):  # shoe retracts; wire stays bent
                out.append((tube, math.degrees(a * s / bend_steps), np.array(held)))
        else:
            raise ValueError(f"unknown op: {op!r}")
    return out


def _wire_xml(W, radius_mm=0.82):
    geoms = []
    # unformed stock: a stub from the bend point back up the tube toward the feeder
    geoms.append(mk.cyl_geom(B, [mk.WAX, mk.TUBE_Y1, mk.WAZ], radius_mm, C_UNFORMED,
                             indent=6, kind="capsule"))
    for i in range(len(W) - 1):
        if np.linalg.norm(W[i + 1] - W[i]) < 1e-6:
            continue
        geoms.append(mk.cyl_geom(W[i], W[i + 1], radius_mm, C_FORMED,
                                 indent=6, kind="capsule"))
    return '    <body name="formed_wire">\n' + "\n".join(geoms) + "\n    </body>"


def _set_joint(model, data, name, val_rad):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[model.jnt_qposadr[jid]] = val_rad


def _fixed_camera(frames):
    allW = np.vstack([f[2] for f in frames])
    lo = np.minimum(allW.min(0), B - 50)
    hi = np.maximum(allW.max(0), B + 30)
    center = (lo + hi) / 2 * 0.001
    size = float(np.max(hi - lo)) * 0.001
    cam = mujoco.MjvCamera()
    cam.lookat[:] = center
    cam.distance = max(size, 0.05) * 1.45
    cam.azimuth, cam.elevation = 125, -38
    return cam


def render_gif(name, program, springback=0.0):
    from PIL import Image

    frames = frames_for(program, springback=springback)
    world, cfgs, ma = mk.ensure_meshes(convert=False)
    cam = _fixed_camera(frames)

    imgs = []
    renderer = None
    for tube_deg, bend_deg, W in frames:
        xml = mk.build_model_xml(world, cfgs, ma, extra_world=_wire_xml(W),
                                 with_wire=False, with_actuators=False)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        _set_joint(model, data, "tube_rot", math.radians(tube_deg))
        _set_joint(model, data, "bend", math.radians(bend_deg))
        mujoco.mj_forward(model, data)
        if renderer is None:
            renderer = mujoco.Renderer(model, 600, 800)
        renderer.update_scene(data, cam)
        imgs.append(Image.fromarray(renderer.render()))
    OUT.mkdir(exist_ok=True)
    path = OUT / "bending.gif"
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=55, loop=0)
    print(f"wrote {path}  ({len(imgs)} frames)")


def _quat_z_to(d):
    """Quaternion (w,x,y,z) rotating local +z onto unit direction d."""
    z = np.array([0.0, 0.0, 1.0])
    d = d / (np.linalg.norm(d) + 1e-12)
    c = float(np.dot(z, d))
    if c > 1 - 1e-9:
        return np.array([1.0, 0, 0, 0])
    if c < -1 + 1e-9:
        return np.array([0.0, 1, 0, 0])  # 180° about x
    axis = np.cross(z, d)
    axis /= np.linalg.norm(axis)
    half = math.acos(c) / 2
    return np.array([math.cos(half), *(axis * math.sin(half))])


def view_live(name, program, springback=0.0):
    """Live playback. Builds one model with a fixed pool of wire-segment geoms and
    repositions them each frame (a live viewer can't change topology mid-run)."""
    import mujoco.viewer

    frames = frames_for(program, springback=springback)
    nmax = max(len(W) - 1 for _, _, W in frames)
    world, cfgs, ma = mk.ensure_meshes(convert=False)

    mm = 0.001
    pool = [f'      <geom name="seg{i}" type="capsule" '
            f'fromto="0 0 0 0 0 0.001" size="{0.82*mm:.6g}" rgba="{C_FORMED}"/>'
            for i in range(nmax)]
    stub = mk.cyl_geom(B, [mk.WAX, mk.TUBE_Y1, mk.WAZ], 0.82, C_UNFORMED, 6, "capsule")
    extra = '    <body name="formed_wire">\n' + stub + "\n" + "\n".join(pool) + "\n    </body>"
    model = mujoco.MjModel.from_xml_string(
        mk.build_model_xml(world, cfgs, ma, extra_world=extra,
                           with_wire=False, with_actuators=False))
    data = mujoco.MjData(model)
    seg_gid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"seg{i}")
               for i in range(nmax)]

    def apply(tube_deg, bend_deg, W):
        _set_joint(model, data, "tube_rot", math.radians(tube_deg))
        _set_joint(model, data, "bend", math.radians(bend_deg))
        for i, gid in enumerate(seg_gid):
            if i < len(W) - 1 and np.linalg.norm(W[i+1] - W[i]) > 1e-6:
                a, b = W[i] * mm, W[i+1] * mm
                model.geom_pos[gid] = (a + b) / 2
                model.geom_quat[gid] = _quat_z_to(b - a)
                model.geom_size[gid, 1] = np.linalg.norm(b - a) / 2
                model.geom_rgba[gid, 3] = 1.0
            else:
                model.geom_rgba[gid, 3] = 0.0   # hide unused segments
        mujoco.mj_forward(model, data)

    print(f"live playback: {name}  (close the window to exit)")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.azimuth, viewer.cam.elevation = 120, -20
        while viewer.is_running():
            for tube_deg, bend_deg, W in frames:
                if not viewer.is_running():
                    break
                apply(tube_deg, bend_deg, W)
                viewer.sync()
                time.sleep(0.05)
            time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("example", nargs="?", default="staple",
                    help=f"program: {', '.join(EXAMPLES)} (default: staple)")
    ap.add_argument("--springback", type=float, default=0.0)
    ap.add_argument("--view", action="store_true", help="live viewer (needs a display)")
    args = ap.parse_args()
    program = EXAMPLES[args.example]
    if args.view:
        view_live(args.example, program, args.springback)
    else:
        render_gif(args.example, program, args.springback)


if __name__ == "__main__":
    main()
