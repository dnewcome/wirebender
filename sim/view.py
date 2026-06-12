"""Interactive viewer for the build123d cantilever architecture (wirebender.xml).

Launch yourself (the GUI thread doesn't survive the agent's background runner):
    cd sim && DISPLAY=:0 ../py/bin/python view.py

Drag the `rot` actuator slider (or the joint) to rotate the head about the wire
axis (Axis 2). Regenerate the model first with: ../py/bin/python make_mjcf.py
"""
from pathlib import Path
import mujoco
import mujoco.viewer

XML = Path(__file__).resolve().parent / "wirebender.xml"

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.viewer.launch(model, data)
