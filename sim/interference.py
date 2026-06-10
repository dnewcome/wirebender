#!/usr/bin/env python3
"""
interference.py — Will a bending program run without colliding?

Steps a feed/rotate/bend program through the machine (same machine-frame motion
as animate_bend.py) and, at every frame, checks the formed wire against:

  • the bending head   — the REAL head mesh (motor + bender-head + flange) in its
                          current pose, via signed-distance. The channel and rear
                          cutaway are real geometry, so wire threading through the
                          head reads as clearance, not a collision.
  • machine bodies     — feeder, NEMA17, rotation motor, U-bracket, base (boxes)
  • the table/floor    — downward bends
  • itself             — non-adjacent wire passing within a wire diameter

A small bending-zone sphere around the bend point is excluded from the head check
(the wire is *meant* to be at the head there). Needs trimesh + scipy + rtree.

    ../py/bin/python interference.py                 # check every program
    ../py/bin/python interference.py spiral_flat     # check one
    MUJOCO_GL=osmesa ../py/bin/python interference.py curl_in --png  # render worst frame
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import trimesh

import make_mjcf as mk
import animate_bend as ab
from bend_model import EXAMPLES

HERE = Path(__file__).resolve().parent
OUT = HERE / "preview"

WIRE_R = mk.WIRE_OD / 2.0
# Exclude wire within BEND_ZONE of the bend point from the head check. The real
# wire bends around the mandrel into the head's cutaway region; our rough bend
# kinematics put it elsewhere, so until the mandrel/bend-radius/bend-plane are
# finalized the head check only catches wire RE-ENTERING the head from far away
# (see PLAN.md). Self-collision / table / body checks are exact.
BEND_ZONE = 32.0
TIP_MM = 3.0                                # ignore the free wire tips in self-check
FLOOR_Z = mk.BASE_TOP - mk.BASE_THK         # table surface
TUBE_PIVOT = np.array([mk.WAX, 0.0, mk.WAZ])
BEND_PIVOT = np.array([0.0, 6.0, 14.0])     # flange bore (bend axis = world Z here)

EXTRA = {
    "spiral_flat": [("feed", 12), ("bend", 40)] * 10,   # >360° planar -> self-overlap
    "curl_in": [("feed", 28), ("bend", 90), ("feed", 22),
                ("bend", 90), ("feed", 40), ("bend", 90), ("feed", 20)],
}
ALL_PROGRAMS = {**EXAMPLES, **EXTRA}


# ── machine keep-out boxes (mm), from make_mjcf LAYOUT ───────────────────────


def _boxes():
    boxes = [("feeder", [mk.WAX, sum(mk.FEEDER_Y) / 2, mk.WAZ], mk.FEEDER_HALF),
             ("nema17", mk.NEMA_C, mk.NEMA_HALF),
             ("rot_motor", mk.ROT_MOTOR_C, mk.ROT_MOTOR_HALF)]
    up_z = (mk.BASE_TOP + mk.WAZ + 6) / 2
    up_hz = (mk.WAZ + 6 - mk.BASE_TOP) / 2
    for y in mk.BRK_Y:
        boxes.append(("bracket", [mk.WAX, y, up_z], [mk.BRK_HALF[0], mk.BRK_HALF[1], up_hz]))
    boxes.append(("base", [mk.WAX, sum(mk.BASE_Y) / 2, mk.BASE_TOP - mk.BASE_THK / 2],
                  [mk.BASE_X_HALF, (mk.BASE_Y[1] - mk.BASE_Y[0]) / 2, mk.BASE_THK / 2]))
    return [(n, np.array(c, float), np.array(h, float)) for n, c, h in boxes]


BOXES = _boxes()


def _dist_box(p, c, h):
    q = np.clip(p, c - h, c + h)
    return float(np.linalg.norm(p - q))     # 0 if inside


# ── head mesh in a given pose, with cached signed-distance queries ──────────

_BASE_PARTS = None
_PQ_CACHE = {}


def _base_parts():
    global _BASE_PARTS
    if _BASE_PARTS is None:
        world, cfgs, _ = mk.ensure_meshes(convert=False)
        _BASE_PARTS = {}
        for n in ("motor", "bender-head", "motor-flange"):
            m = trimesh.load(str(HERE / "meshes" / f"{n}.stl"), force="mesh")
            R, t = world[n]
            V = (R @ m.vertices.T).T + t           # base world pose (tube=0, bend=0), mm
            _BASE_PARTS[n] = (V, m.faces)
    return _BASE_PARTS


def _rot_about(points, axis, through, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    p = points - through
    c, s = math.cos(ang), math.sin(ang)
    out = p * c + np.cross(a, p) * s + np.outer(p @ a, a) * (1 - c)
    return out + through


def _head_pq(tube_deg, bend_deg):
    key = (round(tube_deg, 1), round(bend_deg, 1))
    if key not in _PQ_CACHE:
        parts = _base_parts()
        tube = math.radians(tube_deg)
        bend = math.radians(bend_deg)
        meshes = []
        for n, (V, F) in parts.items():
            W = V.copy()
            if n == "motor-flange":                # flange: bend (Z) then tube (Y)
                W = _rot_about(W, [0, 0, 1], BEND_PIVOT, bend)
            W = _rot_about(W, [0, 1, 0], TUBE_PIVOT, tube)
            meshes.append(trimesh.Trimesh(vertices=W, faces=F, process=False))
        head = trimesh.util.concatenate(meshes)
        _PQ_CACHE[key] = trimesh.proximity.ProximityQuery(head)
    return _PQ_CACHE[key]


def _resample(W, step=1.0):
    pts, s, acc = [W[0]], [0.0], 0.0
    for i in range(len(W) - 1):
        a, b = W[i], W[i + 1]
        L = np.linalg.norm(b - a)
        if L < 1e-9:
            continue
        n = max(1, int(L / step))
        for k in range(1, n + 1):
            pts.append(a + (b - a) * (k / n))
            s.append(acc + L * (k / n))
        acc += L
    return np.array(pts), np.array(s)


# ── the check ───────────────────────────────────────────────────────────────


def check(program, springback=0.0, self_gap=6.0):
    frames = ab.frames_for(program, springback=springback)
    machine_min, machine_at = math.inf, None
    self_min, self_at = math.inf, None

    for fi, (tube_deg, bend_deg, W) in enumerate(frames):
        P, S = _resample(W)
        # head: signed distance to the real mesh (clearance = -signed_distance)
        outside_zone = np.linalg.norm(P - ab.B, axis=1) > BEND_ZONE
        if np.any(outside_zone):
            sd = _head_pq(tube_deg, bend_deg).signed_distance(P[outside_zone])
            clear = -np.asarray(sd)                # >0 outside solid, <0 penetrating
            k = int(np.argmin(clear))
            if clear[k] < machine_min:
                machine_min = float(clear[k])
                machine_at = (fi, "head", P[outside_zone][k].copy())
        # boxes + table
        for p in P:
            d = math.inf
            obst = None
            for name, c, h in BOXES:
                db = _dist_box(p, c, h)
                if db < d:
                    d, obst = db, name
            fz = p[2] - FLOOR_Z
            if fz < d:
                d, obst = fz, "table"
            if d < machine_min:
                machine_min, machine_at = d, (fi, obst, p.copy())
        # self (non-adjacent samples; ignore the free tips so closed shapes don't
        # read their intended start/end closure as a collision)
        smax = S[-1]
        for i in range(len(P)):
            if S[i] < TIP_MM or S[i] > smax - TIP_MM:
                continue
            far = ((S[i + 1:] - S[i]) > self_gap) & (S[i + 1:] < smax - TIP_MM)
            if not np.any(far):
                continue
            dj = np.linalg.norm(P[i + 1:][far] - P[i], axis=1)
            d = float(np.min(dj))
            if d < self_min:
                self_min, self_at = d, (fi, P[i].copy())

    return {
        "frames": len(frames),
        "machine_min": machine_min, "machine_at": machine_at,
        "machine_hit": machine_min < WIRE_R,
        "self_min": self_min, "self_at": self_at,
        "self_hit": self_min < mk.WIRE_OD,
        "ok": not (machine_min < WIRE_R or self_min < mk.WIRE_OD),
    }


def _fmt_report(name, r):
    status = "OK  " if r["ok"] else "FAIL"
    m = r["machine_at"]
    t1 = "  <-- COLLISION" if r["machine_hit"] else ""
    t2 = "  <-- COLLISION" if r["self_hit"] else ""
    return "\n".join([
        f"[{status}] {name}",
        f"        machine clearance: {r['machine_min']:6.1f} mm"
        + (f"  (nearest: {m[1]}, frame {m[0]})" if m else "") + t1,
        f"        self clearance:    {r['self_min']:6.1f} mm"
        + (f"  (frame {r['self_at'][0]})" if r["self_at"] else "") + t2,
    ])


# ── render the worst frame, offending wire in red ───────────────────────────


def render_worst(name, program, r, springback=0.0):
    import mujoco
    from PIL import Image

    frames = ab.frames_for(program, springback=springback)
    fi = 0
    if r["machine_at"]:
        fi = r["machine_at"][0]
    if r["self_at"] and r["self_hit"]:
        fi = r["self_at"][0]
    tube_deg, bend_deg, W = frames[fi]
    vpt = None
    if not r["ok"]:
        vpt = r["machine_at"][2] if r["machine_hit"] else r["self_at"][1]

    bad, good = "0.95 0.15 0.15 1", ab.C_FORMED
    geoms = [mk.cyl_geom(ab.B, [mk.WAX, mk.TUBE_Y1, mk.WAZ], 0.82, ab.C_UNFORMED, 6, "capsule")]
    for i in range(len(W) - 1):
        if np.linalg.norm(W[i + 1] - W[i]) < 1e-6:
            continue
        mid = (W[i] + W[i + 1]) / 2
        col = bad if (vpt is not None and np.linalg.norm(mid - vpt) < 14) else good
        geoms.append(mk.cyl_geom(W[i], W[i + 1], 0.82, col, 6, "capsule"))
    extra = '    <body name="formed_wire">\n' + "\n".join(geoms) + "\n    </body>"

    world, cfgs, ma = mk.ensure_meshes(convert=False)
    model = mujoco.MjModel.from_xml_string(
        mk.build_model_xml(world, cfgs, ma, extra_world=extra,
                           with_wire=False, with_actuators=False))
    data = mujoco.MjData(model)
    ab._set_joint(model, data, "tube_rot", math.radians(tube_deg))
    ab._set_joint(model, data, "bend", math.radians(bend_deg))
    mujoco.mj_forward(model, data)
    rnd = mujoco.Renderer(model, 600, 800)
    rnd.update_scene(data, ab._fixed_camera(frames))
    OUT.mkdir(exist_ok=True)
    Image.fromarray(rnd.render()).save(OUT / "interference.png")
    print(f"wrote {OUT/'interference.png'}  (worst frame {fi})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("program", nargs="?", default=None,
                    help=f"one of: {', '.join(ALL_PROGRAMS)} (default: all)")
    ap.add_argument("--springback", type=float, default=0.0)
    ap.add_argument("--png", action="store_true", help="render the worst frame")
    args = ap.parse_args()

    names = [args.program] if args.program else list(ALL_PROGRAMS)
    print(f"wire Ø{mk.WIRE_OD} mm — collision if machine clearance < {WIRE_R:.2f} mm "
          f"or self clearance < {mk.WIRE_OD:.2f} mm")
    print("(self / table / body checks are exact; head clearance is approximate "
          "until bend kinematics are finalized — see PLAN.md)\n")
    last = None
    for name in names:
        r = check(ALL_PROGRAMS[name], springback=args.springback)
        print(_fmt_report(name, r))
        last = (name, r)
    if args.png and last:
        render_worst(last[0], ALL_PROGRAMS[last[0]], last[1], springback=args.springback)


if __name__ == "__main__":
    main()
