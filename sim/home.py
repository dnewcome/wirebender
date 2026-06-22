"""home.py — simulate the homing cycle in the MuJoCo model.

Kinematic homing (matches the rest of the sim — joints are set, not contact-solved): each
axis seeks toward its home switch from an unknown power-on position, TRIPS at the switch
(the home angle in machine.py), backs off HOME_PULLOFF_DEG, and that becomes zero. This
validates the home conventions — Axis 2 (tube_rot) home = mandrel / bend-disc axis UP (+Z),
Axis 3 (bend) home = pin 90° orthogonal to the wire — and the seek directions + pull-off for
the GRBL homing cycle. Feed (Axis 1) is relative, so it isn't homed.

The bend home tab lives on the rotating ring (arbor_mount HOME_FLAG); a small magenta marker
shows where each fixed switch sits, flashing as the axis trips.

    cd sim && MUJOCO_GL=osmesa ../py/bin/python home.py        # -> preview/homing.gif
    cd sim && DISPLAY=:0 ../py/bin/python home.py --view
"""
import argparse
import math
from pathlib import Path

import numpy as np
import mujoco

import animate_bend as ab
from machine import (ROT_HOME_DEG, BEND_HOME_DEG, HOME_PULLOFF_DEG,
                     ROT_TRAVEL_DEG, BEND_TRAVEL_DEG)

HERE = Path(__file__).resolve().parent
XML = HERE / "wirebender.xml"
OUT = HERE / "preview"
SEEK_STEPS, OFF_STEPS, HOLD = 30, 8, 10


def _ramp(a, b, n):
    return [a + (b - a) * k / n for k in range(1, n + 1)]


def home_frames(start_rot, start_bend):
    """(tube_deg, bend_deg, label, tripped) per frame: home bend, then rotation."""
    f = [(start_rot, start_bend, "power-on (position unknown)", False)]
    # Axis 3 — bend die seeks the switch (pin ⊥ wire), then pulls off
    for b in _ramp(start_bend, BEND_HOME_DEG, SEEK_STEPS):
        f.append((start_rot, b, "Axis 3: seeking bend home", False))
    f += [(start_rot, BEND_HOME_DEG, "Axis 3: TRIP (pin ⊥ wire)", True)] * HOLD
    for b in _ramp(BEND_HOME_DEG, BEND_HOME_DEG + HOME_PULLOFF_DEG, OFF_STEPS):
        f.append((start_rot, b, "Axis 3: pull-off → zero", False))
    bz = BEND_HOME_DEG + HOME_PULLOFF_DEG
    # Axis 2 — head seeks the switch (mandrel up +Z), then pulls off
    for t in _ramp(start_rot, ROT_HOME_DEG, SEEK_STEPS):
        f.append((t, bz, "Axis 2: seeking rotation home", False))
    f += [(ROT_HOME_DEG, bz, "Axis 2: TRIP (mandrel +Z)", True)] * HOLD
    for t in _ramp(ROT_HOME_DEG, ROT_HOME_DEG + HOME_PULLOFF_DEG, OFF_STEPS):
        f.append((t, bz, "Axis 2: pull-off → zero", False))
    f += [(ROT_HOME_DEG + HOME_PULLOFF_DEG, bz, "HOMED", False)] * HOLD
    return f


BX, BY = -0.030, 0.0028        # bend axis in the head frame (mm->m), matches make_mjcf

def _flash_pos(model, data, label):
    """World pos (mm) to flash on a TRIP — the real bend-switch lever (Axis 3, modelled in
    make_mjcf) or, for Axis 2, a point above the head marking mandrel-up. Both ride the head
    body pose so they track tube_rot."""
    if "Axis 3" in label:                       # bend switch — fixed on the head, tracks tube_rot
        hd = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head")
        R = data.xmat[hd].reshape(3, 3); t = data.xpos[hd]
        return (t + R @ np.array([BX + 0.0275, BY, 0.0150])) * 1000.0
    return np.array([20.0, 0.0, ab.ZAXIS_MM + 31.5])   # rotation switch — WORLD-fixed lever


def _report(start_rot, start_bend):
    print("Homing cycle (kinematic):")
    print(f"  Axis 3 (bend): seek {start_bend:+.0f}° → switch {BEND_HOME_DEG:.0f}° (pin ⊥ wire), "
          f"pull off +{HOME_PULLOFF_DEG:.0f}° → ZERO   [travel {BEND_TRAVEL_DEG}]")
    print(f"  Axis 2 (rot):  seek {start_rot:+.0f}° → switch {ROT_HOME_DEG:.0f}° (mandrel +Z), "
          f"pull off +{HOME_PULLOFF_DEG:.0f}° → ZERO   [travel {ROT_TRAVEL_DEG}]")
    print("  Axis 1 (feed): relative — not homed")
    print("  → homed pose: bend pin orthogonal to the wire, mandrel up (+Z)")


def view_live(frames, fps=20):
    import time
    import mujoco.viewer
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.azimuth, v.cam.elevation, v.cam.distance = 150, -25, 0.34
        v.cam.lookat[:] = np.array([-0.02, 0, ab.ZAXIS_MM]) * 0.001
        while v.is_running():
            for t, b, label, trip in frames:
                if not v.is_running():
                    break
                ab._set_machine(model, data, t, b)
                v.user_scn.ngeom = 0
                if trip:
                    ab._add_sphere_scn(v.user_scn, _flash_pos(model, data, label), 3.0, ab._rgba("1 0.1 1 1"))
                v.sync()
                time.sleep(1.0 / fps)
            time.sleep(0.5)


def render_gif(frames):
    from PIL import Image
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 150, -25, 0.34
    cam.lookat[:] = np.array([-0.02, 0, ab.ZAXIS_MM]) * 0.001
    r = mujoco.Renderer(model, 720, 720)
    imgs = []
    for t, b, label, trip in frames:
        ab._set_machine(model, data, t, b)
        r.update_scene(data, cam)
        if trip:
            ab._add_sphere_scn(r.scene, _flash_pos(model, data, label), 3.0, ab._rgba("1 0.1 1 1"))
        imgs.append(Image.fromarray(r.render().copy()))
    r.close()
    OUT.mkdir(exist_ok=True)
    path = OUT / "homing.gif"
    pal = [im.convert("P", palette=Image.ADAPTIVE) for im in imgs]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=60, loop=0, disposal=2)
    print(f"wrote {path}  ({len(imgs)} frames)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start-rot", type=float, default=60.0, help="power-on Axis-2 angle (deg)")
    ap.add_argument("--start-bend", type=float, default=120.0, help="power-on Axis-3 angle (deg)")
    ap.add_argument("--view", action="store_true", help="play live (needs a display)")
    a = ap.parse_args()
    _report(a.start_rot, a.start_bend)
    frames = home_frames(a.start_rot, a.start_bend)
    (view_live if a.view else render_gif)(frames)


if __name__ == "__main__":
    main()
