#!/usr/bin/env python3
"""
render.py — Headless renders of the wire bender (no display needed).

    MUJOCO_GL=osmesa ../py/bin/python render.py            # 4-pose montage PNG
    MUJOCO_GL=osmesa ../py/bin/python render.py --gif       # animated sweep GIF

Outputs to sim/preview/. Use this when you don't have a GUI; otherwise view.py
gives a live interactive window.
"""
import argparse
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
XML = HERE / "wirebender.xml"
OUT = HERE / "preview"


def model_bbox(model, data):
    """Center + max extent of the machine (excludes the floor)."""
    mujoco.mj_forward(model, data)
    lo = np.full(3, 1e9)
    hi = np.full(3, -1e9)
    for g in range(model.ngeom):
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) == "floor":
            continue
        c = data.geom_xpos[g]
        rb = model.geom_rbound[g]
        lo = np.minimum(lo, c - rb)
        hi = np.maximum(hi, c + rb)
    return (lo + hi) / 2, float(np.max(hi - lo))


def setq(model, data, name, deg):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[model.jnt_qposadr[jid]] = np.deg2rad(deg)


def camera(center, size, azimuth=130, elevation=-18):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = center
    cam.distance = size * 2.4
    cam.azimuth = azimuth
    cam.elevation = elevation
    return cam


def render_montage(model, data, renderer):
    center, size = model_bbox(model, data)
    poses = {
        "home (0,0)": (0, 0),
        "tube 90": (90, 0),
        "bend 120": (0, 120),
        "tube 45 / bend 90": (45, 90),
    }
    imgs = []
    for tube, bend in poses.values():
        mujoco.mj_resetData(model, data)
        setq(model, data, "tube_rot", tube)
        setq(model, data, "bend", bend)
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera(center, size))
        imgs.append(renderer.render())
    h, w, _ = imgs[0].shape
    grid = np.zeros((h * 2, w * 2, 3), np.uint8)
    for i, px in enumerate(imgs):
        r, c = divmod(i, 2)
        grid[r * h:(r + 1) * h, c * w:(c + 1) * w] = px
    OUT.mkdir(exist_ok=True)
    Image.fromarray(grid).save(OUT / "montage.png")
    print(f"wrote {OUT/'montage.png'}")


def render_gif(model, data, renderer):
    center, size = model_bbox(model, data)
    cam = camera(center, size)
    frames = []
    # one revolution of the feed tube, with the head bending as it goes
    n = 72
    for i in range(n):
        t = i / n
        tube = 360 * t
        bend = 110 * (0.5 - 0.5 * np.cos(2 * np.pi * 3 * t))  # 3 bends per rev
        mujoco.mj_resetData(model, data)
        setq(model, data, "tube_rot", tube)
        setq(model, data, "bend", bend)
        mujoco.mj_forward(model, data)
        cam.azimuth = 130 + 20 * np.sin(2 * np.pi * t)
        renderer.update_scene(data, cam)
        frames.append(Image.fromarray(renderer.render()))
    OUT.mkdir(exist_ok=True)
    path = OUT / "sweep.gif"
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=50, loop=0)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gif", action="store_true", help="render an animated sweep GIF")
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--height", type=int, default=540)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, args.height, args.width)
    if args.gif:
        render_gif(model, data, renderer)
    else:
        render_montage(model, data, renderer)


if __name__ == "__main__":
    main()
