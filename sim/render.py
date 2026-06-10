#!/usr/bin/env python3
"""
render.py — Headless renders of the wire bender (no display needed).

    MUJOCO_GL=osmesa ../py/bin/python render.py            # 4-pose montage PNG
    MUJOCO_GL=osmesa ../py/bin/python render.py --gif       # animated demo GIF

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


def setq(model, data, name, val):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[model.jnt_qposadr[jid]] = val


def pose(model, data, feed=0.0, tube_deg=0.0, bend_deg=0.0):
    mujoco.mj_resetData(model, data)
    setq(model, data, "feed", feed)
    setq(model, data, "tube_rot", np.deg2rad(tube_deg))
    setq(model, data, "bend", np.deg2rad(bend_deg))
    mujoco.mj_forward(model, data)


def camera(model, azimuth=120, elevation=-20, zoom=1.0):
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)   # frames using <statistic>
    cam.azimuth = azimuth
    cam.elevation = elevation
    cam.distance *= zoom
    return cam


def render_montage(model, data, renderer):
    shots = [
        ("home",            dict(feed=0.00, tube_deg=0,  bend_deg=0)),
        ("feed + bend",     dict(feed=-0.03, tube_deg=0,  bend_deg=110)),
        ("rotate 60",       dict(feed=-0.03, tube_deg=60, bend_deg=110)),
        ("rotate -60 / bend", dict(feed=-0.06, tube_deg=-60, bend_deg=70)),
    ]
    imgs = []
    for _, kw in shots:
        pose(model, data, **kw)
        renderer.update_scene(data, camera(model))
        imgs.append(renderer.render())
    h, w, _ = imgs[0].shape
    grid = np.zeros((h * 2, w * 2, 3), np.uint8)
    for i, px in enumerate(imgs):
        r, c = divmod(i, 2)
        grid[r * h:(r + 1) * h, c * w:(c + 1) * w] = px
    OUT.mkdir(exist_ok=True)
    Image.fromarray(grid).save(OUT / "machine.png")
    print(f"wrote {OUT/'machine.png'}")


def render_gif(model, data, renderer):
    """A 'make a shape' demo: feed a bit, bend, unbend, rotate the plane, repeat."""
    cam = camera(model, zoom=1.05)
    keys = [  # (feed_mm, tube_deg, bend_deg)
        (0,   0,   0),   (15, 0, 0),   (15, 0, 110), (15, 0, 0),
        (30,  0,   0),   (30, 90, 0),  (30, 90, 110),(30, 90, 0),
        (45,  90,  0),   (45, -90, 0), (45, -90, 90),(45, -90, 0),
        (60,  0,   0),
    ]
    feeds = [-k[0] * 0.001 for k in keys]
    tubes = [np.deg2rad(k[1]) for k in keys]
    bends = [np.deg2rad(k[2]) for k in keys]
    frames, per = [], 10
    for i in range(len(keys) - 1):
        for s in range(per):
            t = s / per
            pose(model, data,
                 feed=feeds[i] + (feeds[i+1]-feeds[i])*t,
                 tube_deg=0, bend_deg=0)
            setq(model, data, "tube_rot", tubes[i] + (tubes[i+1]-tubes[i])*t)
            setq(model, data, "bend", bends[i] + (bends[i+1]-bends[i])*t)
            mujoco.mj_forward(model, data)
            cam.azimuth = 120 + 25 * np.sin(2*np.pi*i/len(keys))
            renderer.update_scene(data, cam)
            frames.append(Image.fromarray(renderer.render()))
    OUT.mkdir(exist_ok=True)
    path = OUT / "machine.gif"
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=60, loop=0)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gif", action="store_true", help="render an animated demo GIF")
    ap.add_argument("--width", type=int, default=900)
    ap.add_argument("--height", type=int, default=600)
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
