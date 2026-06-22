"""make_mjcf.py — MuJoCo model for the build123d cantilever architecture.

Assembles the machine from the PRINTABLE part STLs (what you actually print) plus
clearly-separated REFERENCE meshes for the purchased parts:

    printable:  base.stl (deck+uprights+gear+spacer), rothead.stl (head bracket),
                pinion.stl (rotation pinion), bend_plate + arbor_mount + end_cap (bend cell)
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
                 ("build/arbor_mount.stl", "arbor_mount.stl"), # rotating cyclo housing + posts (output)
                 ("build/end_cap.stl", "end_cap.stl"),         # turned-down cyclo end cap (on the arbor)
                 ("build/bend_plate.stl", "bend_plate.stl"),  # cyclo base + boss (fixed; motor on its back)
                 ("build/head_refs.stl", "head_refs.stl"),    # REFERENCE: motors (purchased)
                 ("build/home_switch.stl", "home_switch.stl"),  # Axis-3 home microswitch bracket
                 ("build/feeder_body.stl", "feeder.stl")]:    # reference primitive
    if (ROOT / src).exists():
        shutil.copy(ROOT / src, MESH / dst)

from machine import (WIRE_AXIS_WORLD_Z as ZAXIS, BASE_WORLD_Z as BASE_Z,
                     FEEDER_X_WORLD as FEEDER_X, TUBE_D, MESH_R, PINION_MOUNT_X)
# bend axis in the head frame (mm, from rothead.py): BEND_X, BEND_Y, output height.
BX, BY, BZ = -0.030, 0.0028, 0.008
FEEDER_Z = ZAXIS                  # feeder output (mesh mid-Z) on the wire axis
# pinion body: head-frame Pos(PINION_MOUNT_X, 0, -MESH_R) * Rot(0,90,0) — the Y-rotation
# (quat 0.70711 0 0.70711 0) lays the pinion's native +Z gear axis along the wire axis X.
PIN_X, PIN_Z = PINION_MOUNT_X / 1000.0, -MESH_R / 1000.0

# bend-actuator stack (head frame). Cyclo/bend axis = head +Z at (BX, BY), motor-side UP
# (matches rothead's bend reference: output near the wire at OUT_Z=8mm, input/motor face up
# near BEND_MOTOR_Z=40mm). bend_plate is FIXED to the head — its NEMA back faces the bend
# motor at the top, its round boss faces down. The cyclo housing (arbor_mount) + end_cap are
# the ROTATING output on the bend joint, hung on that boss with the posts pointing DOWN (away
# from the plate, toward the wire) and the end_cap capping the posts at the bottom. The printed
# parts are flipped 180° about X (FLIP) so their local +Z (away from the motor) points to head -Z.
#   bend_plate: motor-back z=-2.8mm, round boss-top z=+9.0mm (local)
#   arbor_mount: base z=0, ring top face z=18.2mm, posts proud to z=20.2mm (local)
#   end_cap:    base z=0, height 6.5mm (local), sits on the arbor ring top face
FLIP  = "0 1 0 0"                # 180° about X: a part's local +Z -> head -Z (motor-side up)
ZBOSS = 0.030                    # head-frame Z of the boss mating plane (bend_plate boss-top = arbor base)
BP_Z  = ZBOSS + 0.009            # bend_plate (flipped) origin so its boss-top (local 9mm) lands on ZBOSS
EC_Z  = 0.0182                   # end_cap base offset from the arbor ring top face (local z=18.2mm)
EC_HEAD_Z = ZBOSS - EC_Z         # end_cap is FIXED (does NOT rotate); head-frame origin so it caps the arbor top

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
    <mesh name="arbor_mount" file="arbor_mount.stl" scale="0.001 0.001 0.001"/>
    <mesh name="end_cap" file="end_cap.stl" scale="0.001 0.001 0.001"/>
    <mesh name="bend_plate" file="bend_plate.stl" scale="0.001 0.001 0.001"/>
    <mesh name="head_refs" file="head_refs.stl" scale="0.001 0.001 0.001"/>
    <mesh name="home_switch" file="home_switch.stl" scale="0.001 0.001 0.001"/>
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
      <!-- bend actuator, FIXED parts: cyclo base + boss (bend_plate; motor bolts to its
           back) and the turned-down end_cap. The end_cap stays fixed and caps the output
           on the bend axis; the rotating arbor_mount posts sweep around its turned-down OD. -->
      <geom type="mesh" mesh="bend_plate" pos="{BX} {BY} {BP_Z}" quat="{FLIP}"
            rgba="0.5 0.7 0.55 1" contype="0" conaffinity="0"/>
      <geom type="mesh" mesh="end_cap" pos="{BX} {BY} {EC_HEAD_Z}" quat="{FLIP}"
            rgba="0.95 0.5 0.15 1" contype="0" conaffinity="0"/>
      <!-- Axis-3 HOME microswitch — FIXED on the head; the rotating ring's HOME_FLAG
           (arbor_mount) sweeps to this lever at the pin-orthogonal home (bend=0). PROPOSED
           placement to iterate on: bracket cantilevered from the fixed side, lever at the
           ring-flag radius (~r24mm) on the +X home spoke, in the flag's z-plane. -->
      <geom type="mesh" mesh="home_switch" pos="{BX+0.034} {BY} 0.0102" euler="0 0 0"
            rgba="0.30 0.32 0.36 1" contype="0" conaffinity="0"/>
      <geom type="box" size="0.0065 0.0032 0.0030" pos="{BX+0.031} {BY} 0.0150"
            rgba="0.12 0.12 0.15 1" contype="0" conaffinity="0"/>            <!-- switch body -->
      <geom type="box" size="0.0035 0.0006 0.0015" pos="{BX+0.0275} {BY} 0.0150"
            rgba="0.85 0.15 0.15 1" contype="0" conaffinity="0"/>            <!-- switch lever -->
      <inertial pos="0 0 0" mass="0.2" diaginertia="2e-4 2e-4 2e-4"/>
      <!-- Axis 2b: rotation pinion, spins as the head rolls -> meshes the fixed gear -->
      <body name="pinion" pos="{PIN_X} 0 {PIN_Z}" quat="0.70711 0 0.70711 0">
        <joint name="pinion_spin" type="hinge" axis="0 0 1"/>
        <geom type="mesh" mesh="pinion" pos="0 0 0" rgba="0.80 0.82 0.30 1" contype="0" conaffinity="0"/>
        <inertial pos="0 0 0" mass="0.02" diaginertia="2e-6 2e-6 2e-6"/>
      </body>
      <!-- Axis 3: cyclo output about the bend axis = rotating arbor_mount (housing + 4
           posts). The posts sweep around the fixed end_cap; the bend-pin part that bolts
           to the posts isn't designed yet, so the "pin" site is a placeholder so the bend
           actuator/animation still resolve. -->
      <body name="benddie" pos="{BX} {BY} {ZBOSS}" quat="{FLIP}">
        <!-- axis "0 0 -1": the body is FLIP'd 180° about X, so local -Z = head +Z; this makes a
             +bend command sweep the die/pin the SAME way the wire curls (+Y), matching the real machine -->
        <joint name="bend" type="hinge" axis="0 0 -1" range="-3.2 3.2"/>
        <geom type="mesh" mesh="arbor_mount" pos="0 0 0" rgba="0.5 0.7 0.55 1" contype="0" conaffinity="0"/>
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
