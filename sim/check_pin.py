"""check_pin.py — track the bending-pin position and its clearance to the feed
tube across the full axis range, using MuJoCo's own geom-distance query.

The bend die is mounted ~8.5mm off the wire axis (benddie body at y=2.8, z=8), so
rolling the head (Axis 2) orbits it about the axis. With the die oriented body-up
(make_mjcf benddie geom, no flip) it clears the on-axis feed tube by ~4mm across
the full range; flipped body-down it buried the pin in the tube. This sweeps both
joints and reports the worst clearance, where it happens, and the pin-tip position.

    cd sim && ../py/bin/python check_pin.py
"""
from pathlib import Path
import numpy as np
import mujoco

XML = Path(__file__).resolve().parent / "wirebender.xml"
N = 49                 # samples per axis


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    g_tube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "tube")
    b_bd = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "benddie")
    g_bd = next(i for i in range(m.ngeom) if m.geom_bodyid[i] == b_bd)
    pin = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pin")
    jr, jb = m.joint("tube_rot"), m.joint("bend")
    qr, qb = jr.qposadr[0], jb.qposadr[0]
    rr, br = m.jnt_range[jr.id], m.jnt_range[jb.id]

    # mj_geomDistance is the authoritative clearance — it's the same query MuJoCo's
    # own collision uses, and it accounts for MuJoCo's internal mesh recentering
    # (raw-STL-vertex math does NOT, so don't reintroduce it here).
    worst = (1e9, None)
    for rot in np.linspace(rr[0], rr[1], N):
        for bend in np.linspace(br[0], br[1], N):
            d.qpos[qr], d.qpos[qb] = rot, bend
            mujoco.mj_forward(m, d)
            ft = np.zeros(6)
            dist = mujoco.mj_geomDistance(m, d, g_tube, g_bd, 0.05, ft)
            if dist < worst[0]:
                worst = (dist, (rot, bend, d.site_xpos[pin].copy(), ft.copy()))

    dist, (rot, bend, p, ft) = worst
    print(f"min tube <-> die clearance: {dist * 1000:+.1f} mm   (negative = penetration)")
    print(f"  worst at  tube_rot={rot:+.2f} rad  bend={bend:+.2f} rad")
    print(f"  pin tip there: ({p[0] * 1000:.1f}, {p[1] * 1000:.1f}, {p[2] * 1000:.1f}) mm")
    if dist < 0:
        print(f"  closest points (mm): tube {(ft[:3]*1000).round(1)}  die {(ft[3:]*1000).round(1)}")
        print("  -> die intrudes on the tube; reorient/raise the bend stack or pull the tube back")
    else:
        print("  die clears the feed tube across the full axis range.")


if __name__ == "__main__":
    main()
