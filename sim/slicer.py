#!/usr/bin/env python3
"""
slicer.py — a "slicer" for the wire bender: a model -> wire path(s) -> G-code.

The inverse of bend_model.py, structured as a pluggable framework:

    source ──[ extraction METHOD ]──▶ one or more wire PATHS (Nx3 polylines)
                                          │
              shared backend:  slice_path → feed/rotate/bend program
                               → fit error → interference → G-code

Extraction methods (registered in METHODS, chosen with --method or auto-detected
from the input):

    points      .json / .csv list of [x,y(,z)] points              -> 1 path
    svg         first <path> in an SVG (M/L/H/V/C/Q/Z)              -> 1 path
    example     a built-in bend_model example (sharp-cornered)      -> 1 path
    centerline  tube mesh (.stl/.obj/.ply) -> extracted spine  [experimental]

More methods are planned (cross-section, edge-following/wireframe, hybrids — see
PLAN.md). A method may return several paths (separate wire pieces); the backend
slices each and emits G-code blocks separated by a cut/reload pause.

    python slicer.py chair                          # example -> stdout G-code
    python slicer.py drawing.svg --tol 0.5 --check -o part.gcode
    python slicer.py wire.stl --method centerline
    python slicer.py --list-methods
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

import bend_model as bm

MESH_EXT = {".stl", ".obj", ".ply", ".off", ".glb"}


# ── geometry helpers ────────────────────────────────────────────────────────


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _any_perp(h):
    ref = np.array([0.0, 1.0, 0.0]) if abs(h[1]) < 0.9 else np.array([0.0, 0.0, 1.0])
    return _unit(np.cross(np.cross(h, ref), h))


def _signed_angle(u, v, axis):
    axis = _unit(axis)
    u = _unit(u - axis * np.dot(u, axis))
    v = _unit(v - axis * np.dot(v, axis))
    ang = math.acos(float(np.clip(np.dot(u, v), -1, 1)))
    if np.dot(np.cross(u, v), axis) < 0:
        ang = -ang
    return ang


def _rot_arr(v, axis, ang):
    axis = _unit(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def _rdp(P, tol):
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]
    ab = b - a
    L = np.linalg.norm(ab)
    d = (np.linalg.norm(P - a, axis=1) if L < 1e-12
         else np.linalg.norm(np.cross(P - a, ab / L), axis=1))
    k = int(np.argmax(d))
    if d[k] > tol:
        return np.vstack([_rdp(P[:k + 1], tol)[:-1], _rdp(P[k:], tol)])
    return np.vstack([a, b])


def _bezier(P, u):
    P = [np.asarray(p, float) for p in P]
    while len(P) > 1:
        P = [P[i] * (1 - u) + P[i + 1] * u for i in range(len(P) - 1)]
    return P[0]


# ── extraction method registry ──────────────────────────────────────────────

METHODS = {}            # name -> {fn, exts, doc}


def method(name, exts=(), doc=""):
    def deco(fn):
        METHODS[name] = {"fn": fn, "exts": set(exts), "doc": doc}
        return fn
    return deco


def detect_method(source):
    """Pick a method from the source (file extension, or a built-in example)."""
    p = Path(source)
    if p.exists():
        ext = p.suffix.lower()
        for name, m in METHODS.items():
            if ext in m["exts"]:
                return name
        raise SystemExit(f"no extraction method for '{ext}' files")
    if source in bm.EXAMPLES:
        return "example"
    raise SystemExit(f"no file or example named {source!r}")


def extract(source, method_name=None, **kw):
    """Run an extraction method -> list of paths (each Nx3, mm)."""
    name = method_name or detect_method(source)
    if name not in METHODS:
        raise SystemExit(f"unknown method {name!r} (have: {', '.join(METHODS)})")
    paths = METHODS[name]["fn"](source, **kw)
    out = []
    for P in paths:
        P = np.asarray(P, float)
        if P.shape[1] == 2:
            P = np.column_stack([P, np.zeros(len(P))])
        out.append(P)
    return name, out


@method("points", exts=(".json", ".csv", ".txt"), doc="list of [x,y(,z)] points")
def _m_points(source, **kw):
    p = Path(source)
    text = p.read_text()
    if p.suffix.lower() == ".json":
        return [np.array(json.loads(text), float)]
    rows = [r for r in (ln.strip() for ln in text.splitlines())
            if r and not r.startswith("#")]
    return [np.array([[float(x) for x in re.split(r"[,\s]+", r)] for r in rows], float)]


@method("svg", exts=(".svg",), doc="first <path> in an SVG (2D)")
def _m_svg(source, samples=24, **kw):
    d = re.search(r'<path[^>]*\bd="([^"]+)"', Path(source).read_text())
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
        if toks[i].isalpha():
            cmd = toks[i]; i += 1
        rel = cmd.islower()
        C = cmd.upper()
        base = cur if rel else np.zeros(2)
        if C == "M":
            cur = base + [num(), num()]; start = cur.copy(); pts.append(cur.copy())
            cmd = "l" if rel else "L"
        elif C == "L":
            cur = base + [num(), num()]; pts.append(cur.copy())
        elif C == "H":
            cur = np.array([(cur[0] if rel else 0) + num(), cur[1]]); pts.append(cur.copy())
        elif C == "V":
            cur = np.array([cur[0], (cur[1] if rel else 0) + num()]); pts.append(cur.copy())
        elif C in ("C", "Q"):
            P = [cur.copy()] + [base + [num(), num()] for _ in range(2 if C == "C" else 1)]
            P.append(base + [num(), num()])
            for s in range(1, samples + 1):
                pts.append(_bezier(P, s / samples))
            cur = P[-1].copy()
        elif C == "Z":
            cur = start.copy(); pts.append(cur.copy())
        else:
            i += 1
    arr = np.array(pts, float)
    arr[:, 1] *= -1                 # SVG y is down; flip to read naturally
    return [arr]


@method("example", doc="a built-in bend_model example (sharp)")
def _m_example(source, **kw):
    return [bm.simulate(bm.EXAMPLES[source], bend_radius=0)]


@method("centerline", exts=tuple(MESH_EXT),
        doc="tube mesh -> extracted spine [experimental]")
def _m_centerline(source, step=None, **kw):
    import trimesh
    mesh = trimesh.load(str(source), force="mesh")
    return [_extract_centerline(mesh, step=step)]


def _axis_vec(axis):
    if isinstance(axis, str):
        axis = axis.strip().lower()
        if axis in ("x", "y", "z"):
            return np.eye(3)["xyz".index(axis)]
        axis = [float(t) for t in re.split(r"[,\s]+", axis)]
    return _unit(np.asarray(axis, float))


@method("cross_section",
        doc="solid mesh -> contour loops at intervals, FDM-slicer style "
            "(--axis, --spacing); explicit --method")
def _m_cross_section(source, axis="z", spacing=10.0, **kw):
    """Slice a closed mesh on parallel planes; each contour loop is a wire piece."""
    import trimesh
    mesh = trimesh.load(str(source), force="mesh")
    n = _axis_vec(axis)
    proj = np.asarray(mesh.vertices, float) @ n
    base = np.asarray(mesh.vertices, float).mean(0)
    base_h = float(base @ n)
    heights = np.arange(proj.min() + spacing / 2, proj.max(), spacing)
    paths = []
    for h in heights:
        sec = mesh.section(plane_origin=base + n * (h - base_h), plane_normal=n)
        if sec is None:
            continue
        for loop in sec.discrete:            # one or more closed contours
            loop = np.asarray(loop, float)
            if len(loop) >= 3:
                paths.append(loop)
    return paths


def _extract_centerline(mesh, step=None):
    """Cross-section marching from a tube tip. Experimental — best on clean,
    non-self-touching tubes; U-shapes/tight bends may need a step/tol tweak."""
    V = np.asarray(mesh.vertices, float)
    c = V.mean(0)
    _, _, vt = np.linalg.svd(V - c, full_matrices=False)
    axis = vt[0]
    # tube radius: from a mid section (fallback: smallest bbox extent / 2)
    r = None
    sec = mesh.section(plane_origin=c, plane_normal=axis)
    if sec is not None and len(sec.vertices) >= 3:
        sv = sec.vertices
        d = np.linalg.norm(sv - c, axis=1)
        near = sv[d < np.percentile(d, 60)]
        if len(near) >= 3:
            r = float(np.median(np.linalg.norm(near - near.mean(0), axis=1)))
    if not r or r < 1e-6:
        r = max(float(np.sort(np.ptp(V, axis=0))[0]) / 2, 0.5)
    step = r if step is None else step

    seed = V[np.argmax(np.linalg.norm(V - c, axis=1))]      # a tube tip
    cap = V[np.linalg.norm(V - seed, axis=1) < 1.8 * r]
    p = cap.mean(0) if len(cap) >= 3 else seed.copy()
    fwd = V[(np.linalg.norm(V - p, axis=1) > r) & (np.linalg.norm(V - p, axis=1) < 5 * r)]
    d = _unit(fwd.mean(0) - p) if len(fwd) >= 3 else _unit(c - p)

    pts, misses = [p.copy()], 0
    for _ in range(20000):
        sec = mesh.section(plane_origin=p + d * step, plane_normal=d)
        ok = False
        if sec is not None and len(sec.vertices) >= 3:
            sv = sec.vertices
            near = sv[np.linalg.norm(sv - (p + d * step), axis=1) < 3 * r]
            if len(near) >= 3:
                cent = near.mean(0)
                if np.linalg.norm(cent - p) > 0.25 * step:
                    d = _unit(cent - p); p = cent; pts.append(p.copy()); ok = True
        misses = 0 if ok else misses + 1
        if misses >= 2:
            break
    return np.array(pts)


# ── backend: path -> program ─────────────────────────────────────────────────


def slice_path(points, simplify_tol=0.4, min_bend_deg=0.5, bend_radius=None):
    """Polyline (Nx3 mm) -> program [(op, value), ...], with setback compensation."""
    r = bm.BEND_RADIUS if bend_radius is None else float(bend_radius)
    P = np.asarray(points, float)
    if P.shape[1] == 2:
        P = np.column_stack([P, np.zeros(len(P))])
    P = _rdp(P, simplify_tol)
    seg = np.diff(P, axis=0)
    L = np.linalg.norm(seg, axis=1)
    keep = L > 1e-6
    seg, L = seg[keep], L[keep]
    if len(L) == 0:
        return []
    dirs = seg / L[:, None]

    feeds = [float(L[0])]
    rolls, bends = [], []
    b = _any_perp(dirs[0])
    prev = dirs[0]
    for i in range(1, len(dirs)):
        nd = dirs[i]
        c = float(np.clip(np.dot(prev, nd), -1, 1))
        alpha = math.acos(c)
        if math.degrees(alpha) < min_bend_deg:
            feeds[-1] += float(L[i])
            prev = nd
            continue
        bend_dir = _unit(nd - c * prev)
        rolls.append(_signed_angle(b, bend_dir, prev))
        bends.append(alpha)
        feeds.append(float(L[i]))
        b = _rot_arr(bend_dir, np.cross(prev, bend_dir), alpha)
        prev = nd

    T = [r * math.tan(a / 2) for a in bends]
    comp = list(feeds)
    for k in range(len(comp)):
        if k > 0:
            comp[k] -= T[k - 1]
        if k < len(bends):
            comp[k] -= T[k]
    comp = [max(0.0, x) for x in comp]

    prog = [("feed", comp[0])]
    for k in range(len(bends)):
        if abs(math.degrees(rolls[k])) > 1e-3:
            prog.append(("rotate", math.degrees(rolls[k])))
        prog.append(("bend", math.degrees(bends[k])))
        prog.append(("feed", comp[k + 1]))
    return prog


# ── backend: program(s) -> G-code ────────────────────────────────────────────


def _gcode_block(program, springback, feed_rate):
    lines, x, y, nb = [], 0.0, 0.0, 0
    for op, val in program:
        if op == "feed":
            x += val
            lines.append(f"G1 X{x:.3f} F{feed_rate} ; feed {val:.2f}")
        elif op == "rotate":
            y += val
            lines.append(f"G1 Y{y:.3f} ; rotate {val:+.2f} -> tube {y:.2f}")
        elif op == "bend":
            cmd = val / (1.0 - springback) if springback < 1 else val
            lines.append(f"G1 Z{cmd:.3f} ; bend {val:.2f} (cmd {cmd:.2f})")
            lines.append("G1 Z0 ; release")
            nb += 1
    return lines, nb


def to_gcode(programs, springback=0.0, feed_rate=400, name="part"):
    """Emit GRBL G-code. X=feed(mm) Y=tube rotation(deg) Z=bend(deg). Accepts one
    program or a list; multiple pieces are separated by a cut/reload pause."""
    if programs and isinstance(programs[0], tuple):
        programs = [programs]
    out = [f"; wirebender program: {name}",
           "; axes: X=feed(mm)  Y=tube rotation(deg)  Z=bend(deg)",
           f"; springback compensation: {springback*100:.0f}%   pieces: {len(programs)}",
           "G21 ; mm", "G90 ; absolute"]
    total = 0
    for i, prog in enumerate(programs):
        out += ["", f"; ---- piece {i+1}/{len(programs)} ----"]
        if i > 0:
            out.append("M0 ; pause: cut wire & reload")
        out.append("G92 X0 Y0 Z0 ; zero")
        block, nb = _gcode_block(prog, springback, feed_rate)
        out += block
        total += nb
    out += ["", f"M2 ; end  ({total} bends, {len(programs)} piece(s))"]
    return "\n".join(out) + "\n"


# ── fit error ─────────────────────────────────────────────────────────────────


def fit_error(points, program, bend_radius=None):
    src = np.asarray(points, float)
    if src.shape[1] == 2:
        src = np.column_stack([src, np.zeros(len(src))])
    out = bm.simulate(program, bend_radius=bend_radius)
    if len(out) < 2:
        return float("nan")
    A, B = _resample_n(src, 200), _resample_n(out, 200)
    R, t = _rigid_align(B, A)
    return float(np.max(np.linalg.norm(A - ((R @ B.T).T + t), axis=1)))


def _resample_n(P, n):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    if s[-1] < 1e-9:
        return np.repeat(P[:1], n, axis=0)
    u = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(u, s, P[:, k]) for k in range(3)])


def _rigid_align(B, A):
    cb, ca = B.mean(0), A.mean(0)
    U, _, Vt = np.linalg.svd((B - cb).T @ (A - ca))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, ca - R @ cb


# ── CLI ──────────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", help="path file / SVG / mesh, or an example name")
    ap.add_argument("--method", choices=list(METHODS), help="extraction method (else auto)")
    ap.add_argument("--list-methods", action="store_true")
    ap.add_argument("-o", "--out", help="write G-code here (default: stdout)")
    ap.add_argument("--tol", type=float, default=0.4, help="simplify tolerance mm")
    ap.add_argument("--springback", type=float, default=0.0)
    ap.add_argument("--feed-rate", type=float, default=400)
    ap.add_argument("--axis", default="z", help="cross_section: slicing axis (x/y/z or vec)")
    ap.add_argument("--spacing", type=float, default=10.0, help="cross_section: slice spacing mm")
    ap.add_argument("--check", action="store_true", help="run the collision checker")
    args = ap.parse_args()

    if args.list_methods:
        for n, m in METHODS.items():
            ext = " ".join(sorted(m["exts"])) or "(name)"
            print(f"  {n:11s} {ext:20s} {m['doc']}")
        return
    if not args.input:
        ap.error("input required (or --list-methods)")

    name, paths = extract(args.input, args.method, axis=args.axis, spacing=args.spacing)
    src_name = Path(args.input).stem if Path(args.input).exists() else args.input
    print(f"# method '{name}': {len(paths)} path(s) from {src_name}", flush=True)

    programs = []
    for j, P in enumerate(paths):
        prog = slice_path(P, simplify_tol=args.tol)
        programs.append(prog)
        nb = sum(1 for op, _ in prog if op == "bend")
        err = fit_error(P, prog)
        total = bm.total_length(bm.simulate(prog))
        tag = f" piece {j+1}" if len(paths) > 1 else ""
        print(f"#{tag}: {len(P)} pts -> {nb} bends, {total:.1f} mm wire, fit {err:.2f} mm")
        if args.check:
            import interference as it
            r = it.check(prog)
            print(f"#  collision: {'OK' if r['ok'] else 'FAIL'}  "
                  f"(self {r['self_min']:.1f} mm, machine {r['machine_min']:.1f} mm)")

    gcode = to_gcode(programs, springback=args.springback,
                     feed_rate=args.feed_rate, name=src_name)
    if args.out:
        Path(args.out).write_text(gcode)
        print(f"# wrote {args.out}")
    else:
        print(gcode)


if __name__ == "__main__":
    main()
