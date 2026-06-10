#!/usr/bin/env python3
"""
slicer.py — a "slicer" for the wire bender: a 3D wire path -> machine program -> G-code.

The inverse of bend_model.py. Given the desired wire centerline as a polyline
(from CAD: a sampled curve, an SVG path, or a list of points), it walks the path
and recovers the machine operations:

    feed   = straight run length
    bend   = turn angle at a corner (angle between adjacent segments)
    rotate = roll about the wire axis to bring that bend into plane

then emits G-code for the three GRBL axes (X = feed mm, Y = tube rotation deg,
Z = bend deg), with springback over-bend compensation.

Smooth curves are polygonised (Ramer–Douglas–Peucker) into straights + discrete
bends — a simple bender can only do that; tighten `--tol` for a finer fit.

    python slicer.py chair                         # slice a built-in example
    python slicer.py path.json -o part.gcode        # from a points file
    python slicer.py drawing.svg --tol 0.5 --check   # from SVG, with collision check
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

import bend_model as bm


# ── geometry helpers ────────────────────────────────────────────────────────


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _any_perp(h):
    ref = np.array([0.0, 1.0, 0.0]) if abs(h[1]) < 0.9 else np.array([0.0, 0.0, 1.0])
    return _unit(np.cross(np.cross(h, ref), h))


def _signed_angle(u, v, axis):
    """Signed angle from u to v measured in the plane perpendicular to `axis`."""
    axis = _unit(axis)
    u = _unit(u - axis * np.dot(u, axis))
    v = _unit(v - axis * np.dot(v, axis))
    ang = math.acos(float(np.clip(np.dot(u, v), -1, 1)))
    if np.dot(np.cross(u, v), axis) < 0:
        ang = -ang
    return ang


def _rdp(P, tol):
    """Ramer–Douglas–Peucker polyline simplification."""
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]
    ab = b - a
    L = np.linalg.norm(ab)
    if L < 1e-12:
        d = np.linalg.norm(P - a, axis=1)
    else:
        d = np.linalg.norm(np.cross(P - a, ab / L), axis=1)
    k = int(np.argmax(d))
    if d[k] > tol:
        return np.vstack([_rdp(P[:k + 1], tol)[:-1], _rdp(P[k:], tol)])
    return np.vstack([a, b])


# ── the slicer: path -> program ─────────────────────────────────────────────


def slice_path(points, simplify_tol=0.4, min_bend_deg=0.5, bend_radius=None):
    """Polyline (Nx3 mm) -> program [(op, value), ...].

    `bend_radius` is the machine's bend radius (defaults to bm.BEND_RADIUS). Feeds
    are setback-compensated by r·tan(α/2) at each adjacent bend so the produced
    (rounded-corner) part matches the design dimensions. Set 0 for sharp corners.
    """
    r = bm.BEND_RADIUS if bend_radius is None else float(bend_radius)
    P = np.asarray(points, float)
    if P.shape[1] == 2:                       # 2D path -> z = 0
        P = np.column_stack([P, np.zeros(len(P))])
    P = _rdp(P, simplify_tol)
    seg = np.diff(P, axis=0)
    L = np.linalg.norm(seg, axis=1)
    keep = L > 1e-6
    seg, L = seg[keep], L[keep]
    dirs = seg / L[:, None]
    if len(dirs) == 0:
        return []

    feeds = [float(L[0])]
    rolls, bends = [], []        # roll (rad) before each bend; bend angle (rad)
    h = dirs[0]
    b = _any_perp(h)
    prev = dirs[0]
    for i in range(1, len(dirs)):
        nd = dirs[i]
        c = float(np.clip(np.dot(prev, nd), -1, 1))
        alpha = math.acos(c)
        if math.degrees(alpha) < min_bend_deg:        # ~straight: merge into feed
            feeds[-1] += float(L[i])
            prev = nd
            continue
        bend_dir = _unit(nd - c * prev)               # deflection dir at this corner
        rolls.append(_signed_angle(b, bend_dir, prev))
        bends.append(alpha)
        feeds.append(float(L[i]))
        b = _rot_arr(bend_dir, np.cross(prev, bend_dir), alpha)   # frame update
        prev = nd

    # setback compensation: feed k is shortened by the bends on either side of it
    T = [r * math.tan(a / 2) for a in bends]
    comp = list(feeds)
    for k in range(len(comp)):
        if k > 0:
            comp[k] -= T[k - 1]
        if k < len(bends):
            comp[k] -= T[k]
    comp = [max(0.0, c) for c in comp]

    prog = [("feed", comp[0])]
    for k in range(len(bends)):
        if abs(math.degrees(rolls[k])) > 1e-3:
            prog.append(("rotate", math.degrees(rolls[k])))
        prog.append(("bend", math.degrees(bends[k])))
        prog.append(("feed", comp[k + 1]))
    return prog


def _rot_arr(v, axis, ang):
    axis = _unit(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


# ── program -> G-code ───────────────────────────────────────────────────────


def to_gcode(program, springback=0.0, feed_rate=400, name="part"):
    """Emit GRBL G-code. X=feed(mm), Y=tube rotation(deg, absolute), Z=bend(deg)."""
    out = [f"; wirebender program: {name}",
           "; axes: X=feed(mm)  Y=tube rotation(deg)  Z=bend(deg)",
           f"; springback compensation: {springback*100:.0f}%",
           "G21 ; mm", "G90 ; absolute", "G92 X0 Y0 Z0 ; zero all axes", ""]
    x = y = 0.0
    nb = 0
    for op, val in program:
        if op == "feed":
            x += val
            out.append(f"G1 X{x:.3f} F{feed_rate} ; feed {val:.2f}")
        elif op == "rotate":
            y += val
            out.append(f"G1 Y{y:.3f} ; rotate {val:+.2f} -> tube {y:.2f}")
        elif op == "bend":
            cmd = val / (1.0 - springback) if springback < 1 else val
            out.append(f"G1 Z{cmd:.3f} ; bend {val:.2f} (cmd {cmd:.2f} w/ springback)")
            out.append("G1 Z0 ; release")
            nb += 1
    out += ["", f"M2 ; end  ({nb} bends)"]
    return "\n".join(out) + "\n"


# ── path importers ──────────────────────────────────────────────────────────


def load_points(path: Path):
    text = path.read_text()
    if path.suffix.lower() == ".json":
        return np.array(json.loads(text), float)
    rows = [r for r in (ln.strip() for ln in text.splitlines()) if r and not r.startswith("#")]
    return np.array([[float(x) for x in re.split(r"[,\s]+", r)] for r in rows], float)


def load_svg(path: Path, samples=24):
    """Sample the first path's d-attribute into a polyline (2D, mm = user units).
    Supports M/L/H/V/C/Q/Z (abs + rel). Arcs (A) are not handled."""
    d = re.search(r'<path[^>]*\bd="([^"]+)"', path.read_text())
    if not d:
        raise ValueError("no <path d=...> found in SVG")
    toks = re.findall(r"[MmLlHhVvCcQqZz]|-?\d*\.?\d+(?:e-?\d+)?", d.group(1))
    pts, cur, start = [], np.zeros(2), np.zeros(2)
    i, cmd = 0, None

    def num():
        nonlocal i
        v = float(toks[i]); i += 1
        return v

    while i < len(toks):
        t = toks[i]
        if t.isalpha():
            cmd = t; i += 1
        rel = cmd.islower()
        C = cmd.upper()
        base = cur if rel else np.zeros(2)
        if C == "M":
            cur = base + [num(), num()]; start = cur.copy(); pts.append(cur.copy()); cmd = "l" if rel else "L"
        elif C == "L":
            cur = base + [num(), num()]; pts.append(cur.copy())
        elif C == "H":
            cur = np.array([(cur[0] if rel else 0) + num(), cur[1]]); pts.append(cur.copy())
        elif C == "V":
            cur = np.array([cur[0], (cur[1] if rel else 0) + num()]); pts.append(cur.copy())
        elif C in ("C", "Q"):
            p0 = cur.copy()
            ctrl = [base + [num(), num()] for _ in range(2 if C == "C" else 1)]
            end = base + [num(), num()]
            P = [p0] + ctrl + [end]
            for s in range(1, samples + 1):
                u = s / samples
                pts.append(_bezier(P, u))
            cur = end.copy()
        elif C == "Z":
            cur = start.copy(); pts.append(cur.copy())
        else:
            i += 1
    # SVG y grows downward; flip so it reads naturally
    arr = np.array(pts, float)
    arr[:, 1] *= -1
    return arr


def _bezier(P, u):
    P = [np.asarray(p, float) for p in P]
    while len(P) > 1:
        P = [P[i] * (1 - u) + P[i + 1] * u for i in range(len(P) - 1)]
    return P[0]


# ── fit error: does the sliced program reproduce the input? ─────────────────


def fit_error(points, program, bend_radius=None):
    """Max deviation (mm) of the input points from the sliced+resimulated shape,
    after best-fit rigid alignment (Procrustes without scale)."""
    src = np.asarray(points, float)
    if src.shape[1] == 2:
        src = np.column_stack([src, np.zeros(len(src))])
    out = bm.simulate(program, bend_radius=bend_radius)
    # resample both to the same count along arc length, then rigid-align
    A = _resample_n(src, 200)
    B = _resample_n(out, 200)
    R, t = _rigid_align(B, A)
    Bt = (R @ B.T).T + t
    return float(np.max(np.linalg.norm(A - Bt, axis=1)))


def _resample_n(P, n):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    if s[-1] < 1e-9:
        return np.repeat(P[:1], n, axis=0)
    u = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(u, s, P[:, k]) for k in range(3)])


def _rigid_align(B, A):
    cb, ca = B.mean(0), A.mean(0)
    H = (B - cb).T @ (A - ca)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, ca - R @ cb


# ── CLI ─────────────────────────────────────────────────────────────────────


def _load(src):
    p = Path(src)
    if p.exists():
        if p.suffix.lower() == ".svg":
            return load_svg(p), p.stem
        return load_points(p), p.stem
    if src in bm.EXAMPLES:                     # a built-in example, sharp-cornered
        return bm.simulate(bm.EXAMPLES[src], bend_radius=0), src
    raise SystemExit(f"no file or example named {src!r} (examples: {', '.join(bm.EXAMPLES)})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="path file (.json/.csv/.svg) or a bend_model example name")
    ap.add_argument("-o", "--out", help="write G-code here (default: stdout)")
    ap.add_argument("--tol", type=float, default=0.4, help="simplify tolerance mm")
    ap.add_argument("--springback", type=float, default=0.0)
    ap.add_argument("--feed-rate", type=float, default=400)
    ap.add_argument("--check", action="store_true", help="also run the collision checker")
    args = ap.parse_args()

    points, name = _load(args.input)
    program = slice_path(points, simplify_tol=args.tol)
    nb = sum(1 for op, _ in program if op == "bend")
    err = fit_error(points, program)
    total = bm.total_length(bm.simulate(program))
    print(f"# {name}: {len(points)} input pts -> {nb} bends, "
          f"{total:.1f} mm wire, fit error {err:.2f} mm", flush=True)

    if args.check:
        import interference as it
        r = it.check(program)
        print("# collision: " + ("OK" if r["ok"] else "FAIL")
              + f"  (self {r['self_min']:.1f} mm, machine {r['machine_min']:.1f} mm)")

    gcode = to_gcode(program, springback=args.springback,
                     feed_rate=args.feed_rate, name=name)
    if args.out:
        Path(args.out).write_text(gcode)
        print(f"# wrote {args.out}")
    else:
        print(gcode)


if __name__ == "__main__":
    main()
