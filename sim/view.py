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

import machine

XML = Path(__file__).resolve().parent / "wirebender.xml"

ROT_HZ, BEND_HZ = 0.12, 0.22     # sweep frequencies (Hz) per axis


def _amplitudes(model):
    """Sweep each axis back and forth through its limit: the head-roll (Axis 2)
    spans its joint range; the bend die (Axis 3) spans the ±half of the 270° die
    travel (machine.DIE_TRAVEL_DEG), capped by the joint range. These stay inside
    the interference-free envelope (verified in make_mjcf/check_pin)."""
    jr = model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "tube_rot")]
    jb = model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "bend")]
    rot_amp = min(abs(jr[0]), abs(jr[1]))
    bend_amp = min(math.radians(machine.DIE_TRAVEL_DEG) / 2, abs(jb[0]), abs(jb[1]))
    return rot_amp, bend_amp


def main(manual=False):
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    if manual:
        mujoco.viewer.launch(model, data)
        return
    rot = model.actuator("rot").id
    bend = model.actuator("bend").id
    rot_amp, bend_amp = _amplitudes(model)
    print(f"sweeping: head roll ±{math.degrees(rot_amp):.0f}°, bend die ±{math.degrees(bend_amp):.0f}° "
          f"(270° travel) — interference-free")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            t = data.time
            data.ctrl[rot] = rot_amp * math.sin(2 * math.pi * ROT_HZ * t)
            data.ctrl[bend] = bend_amp * math.sin(2 * math.pi * BEND_HZ * t)
            mujoco.mj_step(model, data)
            viewer.sync()
            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main(manual="--manual" in sys.argv)
