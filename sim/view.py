"""Interactive viewer for the build123d cantilever architecture (wirebender.xml).

Launch yourself (the GUI thread doesn't survive the agent's background runner):
    cd sim && DISPLAY=:0 ../py/bin/python view.py            # auto-motion demo
    cd sim && DISPLAY=:0 ../py/bin/python view.py --manual    # drag the sliders

By default the head rotates about the wire axis (Axis 2) and the bend die wags
(Axis 3) on their own so it's obvious what moves — paced to real time. Pass
--manual for a static model you drive with the `rot`/`bend` sliders.
Regenerate the model first with: ../py/bin/python make_mjcf.py
"""
import math
import sys
import time
from pathlib import Path
import mujoco
import mujoco.viewer

XML = Path(__file__).resolve().parent / "wirebender.xml"

# demo sweep — amplitude (rad) and frequency (Hz) per axis
ROT_AMP, ROT_HZ = 1.5, 0.12      # head rotation about the wire axis (Axis 2; joint range ±1.6)
BEND_AMP, BEND_HZ = 2.8, 0.22    # bend die wag (Axis 3; joint range ±3.2)


def main(manual=False):
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    if manual:
        mujoco.viewer.launch(model, data)
        return
    rot = model.actuator("rot").id
    bend = model.actuator("bend").id
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            t = data.time
            data.ctrl[rot] = ROT_AMP * math.sin(2 * math.pi * ROT_HZ * t)
            data.ctrl[bend] = BEND_AMP * math.sin(2 * math.pi * BEND_HZ * t)
            mujoco.mj_step(model, data)
            viewer.sync()
            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main(manual="--manual" in sys.argv)
