"""make_mjcf.py — MuJoCo model for the build123d cantilever architecture.

Assembles the machine from the PRINTABLE part STLs (what you actually print) plus
clearly-separated REFERENCE meshes for the purchased parts:

    printable:  base.stl (deck+uprights+gear+spacer), rothead.stl (head bracket),
                pinion.stl (rotation pinion), bend_endcap.stl (bend die + pin)
    reference:  head_refs.stl (NEMA motor bodies + cycloid drive — bought, not printed)

The head rotates about the wire axis (Axis 2 = tube_rot); the pinion is a child of
the head with its own spin joint (Axis 2b) so the gear mesh with the fixed gear is
visible; the bend die spins about the bend axis (Axis 3 = bend). Motor INTERNALS
aren't modeled — only their bodies are shown. Mesh frames are mm (scale 0.001).

Run:  cd sim && ../py/bin/python make_mjcf.py   ->  sim/wirebender.xml
"""
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MESH = ROOT / "sim" / "meshes"
MESH.mkdir(parents=True, exist_ok=True)
for src, dst in [("build/base.stl", "base.stl"),              # printable
                 ("build/rothead.stl", "head.stl"),           # printable (flat head)
                 ("build/pinion.stl", "pinion.stl"),          # printable (animated, meshes the gear)
                 ("build/bend_endcap.stl", "benddie.stl"),    # bend die + pin (rotating output)
                 ("build/arbor_mount.stl", "arbor_mount.stl"), # cyclo housing + posts (bend body)
                 ("build/bend_plate.stl", "bend_plate.stl"),  # cyclo base + boss (bend mount)
                 ("build/head_refs.stl", "head_refs.stl"),    # REFERENCE: motors (purchased)
                 ("build/feeder_body.stl", "feeder.stl")]:    # reference primitive
    shutil.copy(ROOT / src, MESH / dst)

from machine import (WIRE_AXIS_WORLD_Z as ZAXIS, BASE_WORLD_Z as BASE_Z,
                     FEEDER_X_WORLD as FEEDER_X, TUBE_D, MESH_R, PINION_MOUNT_X)
# bend axis in the head frame (mm, from rothead.py): BEND_X, BEND_Y, output height.
BX, BY, BZ = -0.030, 0.0028, 0.008
FEEDER_Z = ZAXIS                  # feeder output (mesh mid-Z) on the wire axis
# pinion body: head-frame Pos(PINION_MOUNT_X, 0, -MESH_R) * Rot(0,90,0) — the Y-rotation
# (quat 0.70711 0 0.70711 0) lays the pinion's native +Z gear axis along the wire axis X.
PIN_X, PIN_Z = PINION_MOUNT_X / 1000.0, -MESH_R / 1000.0

XML = f"""<mujoco model="wirebender">
  <compiler angle="radian" meshdir="meshes"/>
  <option gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.4 0.4 0.4"/>
  </visual>
  <asset>
    <mesh name="base" file="base.stl" scale="0.001 0.001 0.001"/>
    <mesh name="head" file="head.stl" scale="0.001 0.001 0.001"/>
    <mesh name="pinion" file="pinion.stl" scale="0.001 0.001 0.001"/>
    <mesh name="benddie" file="benddie.stl" scale="0.001 0.001 0.001"/>
    <mesh name="arbor_mount" file="arbor_mount.stl" scale="0.001 0.001 0.001"/>
    <mesh name="bend_plate" file="bend_plate.stl" scale="0.001 0.001 0.001"/>
    <mesh name="head_refs" file="head_refs.stl" scale="0.001 0.001 0.001"/>
    <mesh name="feeder" file="feeder.stl" scale="0.001 0.001 0.001"/>
  </asset>
  <worldbody>
    <light pos="0.2 -0.3 0.6" dir="-0.3 0.5 -1"/>
    <geom name="floor" type="plane" size="0.6 0.6 0.05" pos="0.08 0 0" rgba="0.27 0.28 0.30 1"/>
    <geom name="base" type="mesh" mesh="base" pos="0 0 {BASE_Z}" rgba="0.66 0.68 0.72 1"
          contype="0" conaffinity="0"/>
    <geom name="tube" type="cylinder" fromto="-0.03 0 {ZAXIS} 0.095 0 {ZAXIS}" size="{TUBE_D/2/1000}"
          rgba="0.6 0.62 0.66 1" contype="0" conaffinity="0"/>
    <geom name="feeder" type="mesh" mesh="feeder" pos="{FEEDER_X} 0 {FEEDER_Z}" quat="0 0 0 1"
          rgba="0.3 0.31 0.34 1" contype="0" conaffinity="0"/>
    <body name="head" pos="0 0 {ZAXIS}">
      <joint name="tube_rot" type="hinge" axis="1 0 0" range="-1.6 1.6"/>
      <geom type="mesh" mesh="head" pos="0 0 0" rgba="0.86 0.6 0.2 1" contype="0" conaffinity="0"/>
      <!-- purchased parts: motors, shown for context (not animated internally) -->
      <geom type="mesh" mesh="head_refs" pos="0 0 0" rgba="0.42 0.46 0.54 1" contype="0" conaffinity="0"/>
      <!-- bend actuator (APPROX placement, pending the arbor/reach design): cyclo housing
           + posts (arbor_mount) with the output/posts toward the wire, and the cyclo base
           + boss (bend_plate) above it -->
      <geom type="mesh" mesh="arbor_mount" pos="{BX} {BY} 0.026" quat="0 1 0 0"
            rgba="0.5 0.7 0.55 1" contype="0" conaffinity="0"/>
      <geom type="mesh" mesh="bend_plate" pos="{BX} {BY} 0.030" rgba="0.5 0.7 0.55 1"
            contype="0" conaffinity="0"/>
      <inertial pos="0 0 0" mass="0.2" diaginertia="2e-4 2e-4 2e-4"/>
      <!-- Axis 2b: rotation pinion, spins as the head rolls -> meshes the fixed gear -->
      <body name="pinion" pos="{PIN_X} 0 {PIN_Z}" quat="0.70711 0 0.70711 0">
        <joint name="pinion_spin" type="hinge" axis="0 0 1"/>
        <geom type="mesh" mesh="pinion" pos="0 0 0" rgba="0.80 0.82 0.30 1" contype="0" conaffinity="0"/>
        <inertial pos="0 0 0" mass="0.02" diaginertia="2e-6 2e-6 2e-6"/>
      </body>
      <!-- Axis 3: bend die (cycloidal output + pin) about the bend axis -->
      <body name="benddie" pos="{BX} {BY} {BZ}">
        <joint name="bend" type="hinge" axis="0 0 1" range="-3.2 3.2"/>
        <geom type="mesh" mesh="benddie" pos="0 0 0" rgba="0.95 0.5 0.15 1" contype="0" conaffinity="0"/>
        <!-- bending-pin tip = mesh pin tip (15,0,15.5), no flip (die body points +Z, clear of the tube) -->
        <site name="pin" pos="0.015 0 0.0155" size="0.0025" rgba="1 0.1 0.1 1"/>
        <inertial pos="0 0 0" mass="0.05" diaginertia="2e-5 2e-5 2e-5"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="rot" joint="tube_rot" kp="3" ctrlrange="-1.6 1.6"/>
    <position name="bend" joint="bend" kp="2" ctrlrange="-3.2 3.2"/>
  </actuator>
  <sensor>
    <framepos name="pin_pos" objtype="site" objname="pin"/>
  </sensor>
</mujoco>
"""
out = ROOT / "sim" / "wirebender.xml"
out.write_text(XML)
print("wrote", out)

# sanity-load
import mujoco
m = mujoco.MjModel.from_xml_path(str(out))
print("loaded OK — geoms", m.ngeom, "joints", m.njnt)
