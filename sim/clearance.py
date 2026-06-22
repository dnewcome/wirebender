"""clearance.py — ground-truth bend-cell clearance from the real part meshes.

For a machine state (head rotation `tube_deg`, bend die `bend_deg`) and the already
formed wire, measure the minimum distance between the formed wire and every machine
body. Body poses come from the MuJoCo model (its validated kinematics — the head and
bend die move on their joints); distances are measured against the real NON-CONVEX
STL meshes with trimesh, because MuJoCo's own collision convexifies each mesh (the
head's hull would be one big over-conservative blob).

Convention (matches bend_model / animate_bend): the formed wire is FIXED in the world
and the head ORBITS it to set the bend plane. So a head rotation sweeps the head
bodies past the fixed part — this is the "rotate the long way so the head clears the
part" clearance the G-code planner needs. `feed` translates the part along the wire
axis (advance = away from the head).

Negative clearance = interference. This is the ground truth the planner trusts and the
fast parametric envelopes get calibrated against.

    cd sim && ../py/bin/python clearance.py            # self-test on the staple
"""
import functools
from pathlib import Path

import numpy as np
import trimesh
import mujoco
from scipy.spatial import cKDTree

import animate_bend as ab
from machine import WIRE_AXIS_WORLD_MM

HERE = Path(__file__).resolve().parent
XML = HERE / "wirebender.xml"
MESH = HERE / "meshes"
ZAXIS = WIRE_AXIS_WORLD_MM                 # wire axis height (mm)
WIRE_R = 0.8                               # default stock radius (mm) for the gap (Ø1.6)
AXIS_DIR = np.array([1.0, 0.0, 0.0])       # wire axis (X); feed advances the part along -X (out front)
SAMPLE_MM = 2.0                            # formed-wire sampling pitch for distance queries

# Bodies that move with the head (rotate/bend) vs the world-fixed structure.
HEAD_GEOMS = ["head", "head_refs", "bend_plate", "end_cap", "arbor_mount", "pinion"]
WORLD_GEOMS = ["base", "feeder"]           # deck + feeder (floor handled separately)


@functools.lru_cache(maxsize=1)
def _model():
    return mujoco.MjModel.from_xml_path(str(XML))


@functools.lru_cache(maxsize=None)
def _surface_pts(stl):
    """Dense surface point cloud (metres, mesh-local): vertices + area-weighted samples.
    Distance is measured to these points (a scipy KD-tree), NOT trimesh.proximity —
    signed_distance/rtree segfaults under the planner's call volume. Sampling resolution
    is ~1mm, so keep a margin."""
    m = trimesh.load(MESH / stl, process=False)
    m.apply_scale(0.001)
    samp, _ = trimesh.sample.sample_surface(m, 12000)
    return np.vstack([m.vertices, samp])


@functools.lru_cache(maxsize=512)
def _geom_trees(tube_deg, bend_deg):
    """{body: KD-tree of its world-frame surface points} at the given joint angles.
    Cached (rounded by the caller) so the trees are built once and reused across the
    planner's feed/rotation sweeps."""
    model = _model()
    data = mujoco.MjData(model)
    _set(model, data, "tube_rot", np.radians(tube_deg))
    _set(model, data, "bend", np.radians(bend_deg))
    mujoco.mj_forward(model, data)
    trees = {}
    for g in range(model.ngeom):
        mid = model.geom_dataid[g]
        if mid < 0:
            continue
        stl = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid) + ".stl"
        R = data.geom_xmat[g].reshape(3, 3)
        Pw = _surface_pts(stl) @ R.T + data.geom_xpos[g]
        trees[stl[:-4]] = cKDTree(Pw)
    return trees


def _set(model, data, joint, val):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    data.qpos[model.jnt_qposadr[jid]] = val


def _resample(pts, pitch=SAMPLE_MM):
    """Densify a polyline (mm) to ~`pitch` spacing so distance queries don't skip gaps."""
    pts = np.asarray(pts, float)
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = np.linalg.norm(b - a)
        n = max(1, int(d / pitch))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def formed_world(program):
    """The finished formed wire (world mm) for a program prefix — same geometry the
    animation draws (head-orbits-fixed-part convention)."""
    return ab.frames_for(program)[-1][2]


def clearance(part_pts, tube_deg=0.0, bend_deg=0.0, feed=0.0, wire_r=WIRE_R):
    """Min gap (mm) from the formed wire to each machine body at this state.
    `feed` advances the part along the wire axis (out the front, away from the head).
    Returns {body: gap, 'floor': gap, 'min': overall, 'hit': worst body}."""
    pts = _resample(part_pts) + AXIS_DIR * (-feed)         # advance = out the front (-X)
    qpts_m = pts * 0.001                                    # to metres
    gaps = {}
    for name, tree in _geom_trees(round(tube_deg, 1), round(bend_deg, 1)).items():
        d, _ = tree.query(qpts_m)                              # nearest surface sample (m)
        gaps[name] = float(d.min()) * 1000.0 - wire_r          # closest-approach gap (mm)
    gaps["floor"] = float(pts[:, 2].min()) - wire_r            # part vs deck floor (z=0)
    worst = min(gaps, key=gaps.get)
    gaps["min"] = gaps[worst]
    gaps["hit"] = worst
    return gaps


if __name__ == "__main__":
    from bend_model import EXAMPLES
    prog = EXAMPLES["staple"]
    part = formed_world(prog)
    print(f"staple: {len(part)} pts, bbox(mm) {np.round(part.max(0) - part.min(0), 1)}")
    print(f"{'tube°':>6} {'min gap':>8}  worst body   (bend=0)")
    for t in range(0, 361, 30):
        g = clearance(part, tube_deg=t)
        flag = "  <-- INTERFERENCE" if g["min"] < 0 else ""
        print(f"{t:6d} {g['min']:8.1f}  {g['hit']:12}{flag}")
