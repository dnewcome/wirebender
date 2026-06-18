#!/usr/bin/env python3
"""
interference.py — manufacturability / clearance rule-checker for the wire bender.

The bender is a SINGLE-PIN bender: for each bend the die rotates from home (0°),
a pin at radius PIN_SWEEP_R sweeps the wire around a fixed mandrel by the bend
angle, then returns to 0 (the G-code emits `Z<angle>` then `Z0 ; release`). So:

  * every bend starts from home -> the per-stroke bend angle is capped by the die
    travel (DIE_TRAVEL_DEG, the 20:1 cycloid output's usable range, ~270°);
  * the corner RADIUS is fixed by the mandrel (BEND_RADIUS below) — the machine
    can't make a corner tighter than the mandrel it's fitted with;
  * during a stroke the pin sweeps an arc that must clear the feed tube / incoming
    wire, and the growing part must clear both the pin and the head body.

This module is FLAG-ONLY: `check(program)` walks the program (same kinematics as
bend_model) and returns a list of findings — it does NOT rewrite the program.

The corner RADIUS is fixed by the mandrel (reported as `radius`), so a feature
needing a tighter radius can't be made; that constraint shows up as `min_straight`
(too little straight between bends for the radius's setback + the pin's grab).

Rules (severity: 'error' = unmakeable as written, 'warn' = marginal/approx):
  travel       bend command > DIE_TRAVEL_DEG                          [exact]
  min_straight inter-bend straight too short for setback + pin grab   [exact]
  pin_part     pin sweep would strike the already-formed part         [approx]
  pin_tube     bend wrap exceeds what clears the feed tube/incoming   [calib]
  part_head    formed part swings back into the head body             [approx]

Constants are sourced from the CAD (cad/rothead.py, cad/bend_disc.py) and the
cycloid spec; keep them in sync. The geometric clearances marked [approx]/[calib]
are conservative first-cut models — tighten against the real head once the head
redesign lands (see PLAN.md). The live mesh-level pin↔tube interference of the
*current* CAD is measured separately by check_pin.py.

    python interference.py chair                 # check a built-in example
    python interference.py part.gcode            # check a sliced program
    python interference.py --caps                # print the machine limits
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

import bend_model as bm
from machine import (DIE_TRAVEL_DEG, WIRE_D, MANDREL_D, PIN_D, PIN_SWEEP_R, CLEAR,
                     MIN_GRAB, MAX_WRAP_DEG, HEAD_BACK_REACH, HEAD_RADIUS, bend_radius)

# All machine constants live in machine.py (single source of truth, kept in sync
# with the CAD by sim/consistency.py). This module just applies them as rules.


def per_stroke_limit_deg():
    """Max realizable bend per stroke = the tighter of die travel and tube wrap."""
    return min(DIE_TRAVEL_DEG, MAX_WRAP_DEG)


def capabilities(wire_d=WIRE_D):
    return {
        "die_travel_deg": DIE_TRAVEL_DEG,
        "max_wrap_deg": MAX_WRAP_DEG,
        "per_stroke_limit_deg": per_stroke_limit_deg(),
        "bend_radius_mm": bend_radius(wire_d),
        "min_straight_mm": MIN_GRAB,
        "pin_sweep_r_mm": PIN_SWEEP_R,
        "wire_d_mm": wire_d,
    }


# ── walk the program, recording each bend's frame + the formed part ─────────


def _rot(v, axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def _walk(program, springback=0.0, wire_d=WIRE_D):
    """Mirror bend_model.simulate but record, per bend: the bend point, incoming
    heading/bend-axis, realized & commanded angle, and the feed straights either
    side. Returns (pts Nx3, bends[list of dict])."""
    r = bend_radius(wire_d)
    p = np.zeros(3)
    h = np.array([1.0, 0.0, 0.0])
    b = np.array([0.0, 1.0, 0.0])
    pts = [p.copy()]
    bends = []
    feed_acc = 0.0
    sb = springback
    for op, val in program:
        if op == "feed":
            d = float(val)
            p = p + h * d
            pts.append(p.copy())
            feed_acc += d
            if bends:                       # straight after the previous bend
                bends[-1]["feed_after"] = bends[-1].get("feed_after", 0.0) + d
        elif op == "rotate":
            b = _rot(b, h, math.radians(float(val)))
        elif op == "bend":
            a = np.cross(h, b)
            realized = float(val) * (1.0 - sb)
            cmd = float(val) / (1.0 - sb) if sb < 1 else float(val)
            bends.append({
                "i": len(bends),
                "pos": p.copy(),
                "h": h.copy(),
                "a": a.copy(),
                "deg_realized": realized,
                "deg_cmd": cmd,
                "feed_before": feed_acc,
                "feed_after": 0.0,
                "pt_index": len(pts) - 1,
            })
            ang = math.radians(realized)
            for q in bm._arc_points(p, b, a, ang, r):
                pts.append(q.copy())
            p = pts[-1].copy()
            h = _rot(h, a, ang)
            b = _rot(b, a, ang)
            feed_acc = 0.0
    return np.array(pts), bends


# ── the checker ─────────────────────────────────────────────────────────────


def check(program, springback=0.0, wire_d=WIRE_D):
    """Validate a program against the machine's bend-cell limits. Flag-only.

    Returns {ok, radius, findings:[{bend, rule, severity, detail, value, limit}],
    self_min, machine_min}. self_min/machine_min are the tightest formed-part
    clearances (mm) to the pin sweep and to the head (back-compat with slicer)."""
    pts, bends = _walk(program, springback, wire_d)
    r = bend_radius(wire_d)
    sweep_r = PIN_SWEEP_R + PIN_D / 2 + CLEAR
    limit = per_stroke_limit_deg()
    findings = []
    self_min = float("inf")
    machine_min = float("inf")

    for bd in bends:
        i = bd["i"]
        cmd = bd["deg_cmd"]
        realized = bd["deg_realized"]

        # travel — commanded angle (after springback) must fit the die range
        if abs(cmd) > DIE_TRAVEL_DEG + 1e-6:
            findings.append(dict(bend=i, rule="travel", severity="error",
                                 detail=f"bend cmd {cmd:.1f}° > die travel {DIE_TRAVEL_DEG:.0f}°",
                                 value=round(cmd, 1), limit=DIE_TRAVEL_DEG))
        # pin_tube — wrap that swings the pin back toward the feed tube
        if abs(realized) > MAX_WRAP_DEG + 1e-6 and abs(cmd) <= DIE_TRAVEL_DEG:
            findings.append(dict(bend=i, rule="pin_tube", severity="warn",
                                 detail=f"wrap {realized:.1f}° > {MAX_WRAP_DEG:.0f}° — pin nears the feed tube",
                                 value=round(realized, 1), limit=MAX_WRAP_DEG))

        # min_straight — inter-bend straights must let the pin grab + clear setback
        T = r * math.tan(math.radians(min(abs(realized), 179.0)) / 2)
        for side, feed in (("before", bd["feed_before"]), ("after", bd["feed_after"])):
            inter = (side == "before" and i > 0) or (side == "after" and i < len(bends) - 1)
            if inter and feed + 1e-6 < max(MIN_GRAB, T):
                findings.append(dict(bend=i, rule="min_straight", severity="error",
                                     detail=f"straight {side} = {feed:.1f}mm < need {max(MIN_GRAB,T):.1f}mm "
                                            f"(grab {MIN_GRAB:.0f}, setback {T:.1f})",
                                     value=round(feed, 1), limit=round(max(MIN_GRAB, T), 1)))

        # pin_part — does the pin sweep strike the already-formed part?
        p, a = bd["pos"], bd["a"]
        formed = pts[: bd["pt_index"]]                 # part laid before this bend
        if len(formed):
            rel = formed - p
            arc = np.cumsum(np.r_[0, np.linalg.norm(np.diff(formed, axis=0), axis=1)])[::-1]
            perp = np.linalg.norm(rel - np.outer(rel @ a / (a @ a), a), axis=1)
            far = arc > MIN_GRAB                         # ignore the wrap region near p
            if np.any(far):
                d = float(perp[far].min())
                self_min = min(self_min, d - sweep_r)
                if d < sweep_r:
                    findings.append(dict(bend=i, rule="pin_part", severity="error",
                                         detail=f"formed part within {d:.1f}mm of the pin sweep "
                                                f"(needs {sweep_r:.1f}mm)",
                                         value=round(d, 1), limit=round(sweep_r, 1)))
            # part_head — does the formed part swing back behind the bend (head side)?
            back = rel @ bd["h"]                         # +h = toward the feeder/head
            ring = np.linalg.norm(rel - np.outer(back, bd["h"]), axis=1)
            inhead = (back > HEAD_BACK_REACH) & (ring < HEAD_RADIUS)
            if len(formed) > 1:
                machine_min = min(machine_min, HEAD_BACK_REACH - float(back.max()))
            if np.any(inhead):
                findings.append(dict(bend=i, rule="part_head", severity="warn",
                                     detail=f"formed part reaches {float(back.max()):.1f}mm toward the head "
                                            f"(keep-out {HEAD_BACK_REACH:.0f}mm)",
                                     value=round(float(back.max()), 1), limit=HEAD_BACK_REACH))

    ok = not any(f["severity"] == "error" for f in findings)
    return {
        "ok": ok,
        "radius": r,
        "findings": findings,
        "self_min": (0.0 if self_min == float("inf") else self_min),
        "machine_min": (0.0 if machine_min == float("inf") else machine_min),
    }


def summary(result):
    """One-line-per-finding human report."""
    caps = capabilities()
    lines = [f"bend radius {result['radius']:.1f}mm | per-stroke ≤ {caps['per_stroke_limit_deg']:.0f}° "
             f"(die {caps['die_travel_deg']:.0f}°) | min straight {caps['min_straight_mm']:.0f}mm"]
    if not result["findings"]:
        lines.append("  ✓ no rule violations")
    else:
        errs = sum(f["severity"] == "error" for f in result["findings"])
        warns = len(result["findings"]) - errs
        lines.append(f"  {errs} error(s), {warns} warning(s):")
        for f in result["findings"]:
            mark = "✗" if f["severity"] == "error" else "•"
            lines.append(f"    {mark} bend {f['bend']}: [{f['rule']}] {f['detail']}")
    return "\n".join(lines)


# ── CLI ─────────────────────────────────────────────────────────────────────


def _load_program(src):
    """A built-in example name, or a .gcode file -> a single program."""
    p = Path(src)
    if p.exists() and p.suffix.lower() in (".gcode", ".nc", ".txt"):
        import slicer
        progs = slicer.parse_gcode(p.read_text())
        return progs[0] if progs else []
    if src in bm.EXAMPLES:
        return bm.EXAMPLES[src]
    raise SystemExit(f"no example or .gcode named {src!r} (examples: {', '.join(bm.EXAMPLES)})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("program", nargs="?", help="example name or .gcode file")
    ap.add_argument("--springback", type=float, default=0.0)
    ap.add_argument("--wire-d", type=float, default=WIRE_D, help="stock Ø mm (14ga 1.63 / 16ga 1.29)")
    ap.add_argument("--caps", action="store_true", help="print machine limits and exit")
    args = ap.parse_args()

    if args.caps:
        for k, v in capabilities(args.wire_d).items():
            print(f"  {k:22s} {v}")
        return
    if not args.program:
        ap.error("program required (example name or .gcode), or --caps")

    prog = _load_program(args.program)
    res = check(prog, springback=args.springback, wire_d=args.wire_d)
    print(summary(res))
    raise SystemExit(0 if res["ok"] else 1)


if __name__ == "__main__":
    main()
