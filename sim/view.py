#!/usr/bin/env python3
"""
view.py — Interactively view / animate the wire bender in MuJoCo.

Usage:
    ../py/bin/python view.py            # interactive: drag the Control sliders
    ../py/bin/python view.py --demo     # play a scripted feed/rotate/bend sequence

In interactive mode, open the "Control" panel in the viewer (left side) and drag
the `tube_rot` (feed-tube rotation / bend direction) and `bend` (bend degree)
sliders. The model is position-actuated, so the parts follow the sliders.

Requires a display. If you're headless, use render.py instead.
"""
import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

XML = Path(__file__).resolve().parent / "wirebender.xml"


def ctrl_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def run_demo(model, data, viewer):
    """A 'make a shape' sequence: feed wire, rotate the bend plane, bend, repeat."""
    feed = ctrl_id(model, "feed")
    tube = ctrl_id(model, "tube_rot")
    bend = ctrl_id(model, "bend")

    def ramp(act, a, b, seconds):
        steps = max(1, int(seconds / model.opt.timestep))
        for i in range(steps):
            data.ctrl[act] = a + (b - a) * (i / steps)
            mujoco.mj_step(model, data)
            if i % 8 == 0:
                viewer.sync()
                time.sleep(model.opt.timestep * 8)
            if not viewer.is_running():
                return

    # feed wire, bend, unbend, rotate the plane, feed, bend again — like a real program
    sequence = [
        (feed, 0.0, -0.02, 0.6),
        (bend, 0, 110, 0.5), (bend, 110, 0, 0.4),
        (feed, -0.02, -0.04, 0.6),
        (tube, 0, 90, 0.7),
        (bend, 0, 110, 0.5), (bend, 110, 0, 0.4),
        (feed, -0.04, -0.06, 0.6),
        (tube, 90, -90, 1.1),
        (bend, 0, 90, 0.5), (bend, 90, 0, 0.4),
        (tube, -90, 0, 0.7),
        (feed, -0.06, 0.0, 0.8),
    ]
    while viewer.is_running():
        for act, a, b, secs in sequence:
            ramp(act, a, b, secs)
            if not viewer.is_running():
                break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="play a scripted bend sequence")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)

    if args.demo:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)  # frame the whole machine
            viewer.cam.azimuth = 120
            viewer.cam.elevation = -20
            run_demo(model, data, viewer)
    else:
        # Managed viewer: gives the Control panel with actuator sliders.
        mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
