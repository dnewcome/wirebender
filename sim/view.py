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
    """A simple 'bend a few times in different directions' sequence."""
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

    # bend, unbend, rotate the feed tube, bend again — a few cycles
    sequence = [
        (bend, 0, 110, 0.6),
        (bend, 110, 0, 0.6),
        (tube, 0, 90, 0.8),
        (bend, 0, 110, 0.6),
        (bend, 110, 0, 0.6),
        (tube, 90, -90, 1.2),
        (bend, 0, 110, 0.6),
        (bend, 110, 0, 0.6),
        (tube, -90, 0, 0.8),
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
            viewer.cam.azimuth = 130
            viewer.cam.elevation = -18
            viewer.cam.distance = 0.17
            run_demo(model, data, viewer)
    else:
        # Managed viewer: gives the Control panel with actuator sliders.
        mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
