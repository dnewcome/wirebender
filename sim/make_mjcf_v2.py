"""make_mjcf_v2.py — MuJoCo model for the build123d cantilever architecture.

Assembles build/base.stl (static) + build/rothead.stl (rotates about the wire
axis = Axis 2) + the passive feed tube. Mesh frames are in mm (scale 0.001).
Base frame: wire axis at z=35mm. Head frame: wire axis at the mesh origin.
"""
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MESH = ROOT / "sim" / "meshes"
MESH.mkdir(parents=True, exist_ok=True)
for src, dst in [("build/base.stl", "base_v2.stl"), ("build/rothead.stl", "head_v2.stl")]:
    shutil.copy(ROOT / src, MESH / dst)

ZAXIS = 0.041          # wire-axis height in the world (base lifted 6mm onto the floor)
BASE_Z = ZAXIS - 0.035  # base frame wire axis is at 35mm

XML = f"""<mujoco model="wirebender_v2">
  <compiler angle="radian" meshdir="meshes"/>
  <option gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.4 0.4 0.4"/>
  </visual>
  <asset>
    <mesh name="base" file="base_v2.stl" scale="0.001 0.001 0.001"/>
    <mesh name="head" file="head_v2.stl" scale="0.001 0.001 0.001"/>
  </asset>
  <worldbody>
    <light pos="0.2 -0.3 0.6" dir="-0.3 0.5 -1"/>
    <geom name="floor" type="plane" size="0.6 0.6 0.05" pos="0.08 0 0" rgba="0.27 0.28 0.30 1"/>
    <geom name="base" type="mesh" mesh="base" pos="0 0 {BASE_Z}" rgba="0.66 0.68 0.72 1"
          contype="0" conaffinity="0"/>
    <geom name="tube" type="cylinder" fromto="-0.03 0 {ZAXIS} 0.095 0 {ZAXIS}" size="0.004"
          rgba="0.6 0.62 0.66 1" contype="0" conaffinity="0"/>
    <body name="head" pos="0 0 {ZAXIS}">
      <joint name="tube_rot" type="hinge" axis="1 0 0" range="-1.6 1.6"/>
      <geom type="mesh" mesh="head" pos="0 0 0" rgba="0.86 0.6 0.2 1" contype="0" conaffinity="0"/>
      <inertial pos="0 0 0" mass="0.2" diaginertia="2e-4 2e-4 2e-4"/>
    </body>
  </worldbody>
  <actuator>
    <position name="rot" joint="tube_rot" kp="3" ctrlrange="-1.6 1.6"/>
  </actuator>
</mujoco>
"""
out = ROOT / "sim" / "wirebender_v2.xml"
out.write_text(XML)
print("wrote", out)

# sanity-load
import mujoco
m = mujoco.MjModel.from_xml_path(str(out))
print("loaded OK — geoms", m.ngeom, "joints", m.njnt)
