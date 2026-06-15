"""check_pin.py — track the bending-pin position and its clearance to the feed
tube across the full axis range.

The bend die is mounted ~8.5mm OFF the wire axis (benddie body at y=2.8, z=8) and
the output cap is Ø37, so it straddles the axis. Rolling the head (Axis 2,
tube_rot) swings the die — and the central bore the tube threads through — off
axis, sweeping the die into the on-axis feed tube. This sweeps both joints and
reports the worst penetration, where it happens, the pin-tip position, and how
far back (toward the feeder) the die intrudes into the tube corridor — i.e.
where the tube must end to clear it.

    cd sim && ../py/bin/python check_pin.py
"""
from pathlib import Path
import numpy as np
import mujoco
import trimesh

XML = Path(__file__).resolve().parent / "wirebender.xml"
ZAXIS = 0.041          # wire-axis world height (m); keep in sync with make_mjcf.py
TUBE_R = 0.004         # feed-tube radius (m)
N = 49                 # samples per axis


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    g_tube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "tube")
    b_bd = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "benddie")
    g_bd = next(i for i in range(m.ngeom) if m.geom_bodyid[i] == b_bd)
    pin = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pin")
    V = trimesh.load(str(XML.parent / "meshes" / "benddie.stl")).vertices * 0.001
    jr, jb = m.joint("tube_rot"), m.joint("bend")
    qr, qb = jr.qposadr[0], jb.qposadr[0]
    rr, br = m.jnt_range[jr.id], m.jnt_range[jb.id]

    worst = (1e9, None)
    maxx = -1e9
    for rot in np.linspace(rr[0], rr[1], N):
        for bend in np.linspace(br[0], br[1], N):
            d.qpos[qr], d.qpos[qb] = rot, bend
            mujoco.mj_forward(m, d)
            ft = np.zeros(6)
            dist = mujoco.mj_geomDistance(m, d, g_tube, g_bd, 0.05, ft)
            if dist < worst[0]:
                worst = (dist, (rot, bend, d.site_xpos[pin].copy()))
            R = d.geom_xmat[g_bd].reshape(3, 3)
            W = V @ R.T + d.geom_xpos[g_bd]
            inside = W[np.hypot(W[:, 1], W[:, 2] - ZAXIS) < TUBE_R]
            if len(inside):
                maxx = max(maxx, inside[:, 0].max())

    dist, (rot, bend, p) = worst
    print(f"min tube <-> die clearance: {dist * 1000:+.1f} mm   (negative = penetration)")
    print(f"  worst at  tube_rot={rot:+.2f} rad  bend={bend:+.2f} rad")
    print(f"  pin tip there: ({p[0] * 1000:.1f}, {p[1] * 1000:.1f}, {p[2] * 1000:.1f}) mm")
    if maxx > -1e8:
        print(f"rearmost die intrusion into the tube corridor: x = {maxx * 1000:.1f} mm")
        print(f"  -> tube must end at x >= {maxx * 1000:.1f} mm to clear (it currently runs to x=-30)")
    else:
        print("die never enters the tube corridor — clear.")


if __name__ == "__main__":
    main()
