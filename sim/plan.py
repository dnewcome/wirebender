"""plan.py — clearance-aware motion planner: a bend program -> safe machine moves.

A bend program says WHAT part to make (feed/rotate/bend). It does NOT say how to move
the machine safely to make it: which way to rotate the head (the short way may sweep
the head bodies straight through the already-formed part), or how far to feed the wire
out so the part clears the head during a rotation and clears the pin during a bend.

This planner fills that in. For each bend it:
  * finds the head-rotation target (cumulative `rotate`) and tries BOTH directions
    (short and long way), scoring each by the worst clearance over the swept path;
  * if the best direction still fouls the formed part, searches a FEED ADVANCE that
    pushes the part out far enough to clear (then notes the wire must be fed back);
  * reports the chosen move + its clearance margin, or flags an unavoidable collision.

Clearance is the ground truth from clearance.py (real meshes, MuJoCo poses). The output
is the basis for collision-safe G-code.

    cd sim && ../py/bin/python plan.py staple      # plan an example
"""
import argparse
import math
from pathlib import Path

import numpy as np

import clearance as cl
import bend_model as bm
from machine import CLEAR

MARGIN = max(CLEAR, 1.0)        # required clearance margin (mm)
ROT_STEP = 6.0                  # head-rotation sweep resolution (deg)
FEED_MAX = 60.0                 # most we'll advance the wire to clear (mm)
FEED_STEP = 4.0                 # feed search step (mm)


def _steps(frm, to, step):
    n = max(1, int(math.ceil(abs(to - frm) / step)))
    return [frm + (to - frm) * k / n for k in range(n + 1)]


def _sweep_min(part, frm, to, feed):
    """Worst clearance (mm) over a head rotation from `frm`->`to` deg at this feed."""
    return min(cl.clearance(part, tube_deg=t, feed=feed)["min"] for t in _steps(frm, to, ROT_STEP))


def _choose_rotation(part, cur, target, feed):
    """Pick the rotation path (short vs long way) with the best worst-case clearance."""
    short = target
    long = target - 360.0 if target > cur else target + 360.0
    cands = [("short", short), ("long", long)]
    scored = [(lbl, dst, _sweep_min(part, cur, dst, feed)) for lbl, dst in cands]
    scored.sort(key=lambda s: -s[2])                  # most clearance first
    return scored[0]                                  # (label, dest_deg, worst_gap)


def plan(program, springback=0.0, wire_r=cl.WIRE_R):
    """Return {moves:[...], ok:bool} — an annotated, clearance-checked motion list."""
    moves = []
    cur_tube = 0.0
    feed_total = 0.0
    cum_rotate = 0.0
    bend_no = -1
    ok = True
    prefix = []                                       # program up to the current point

    for op, val in program:
        prefix = prefix + [(op, val)]
        if op == "feed":
            feed_total += float(val)
            moves.append(dict(kind="feed", mm=float(val), note="advance stock"))
        elif op == "rotate":
            cum_rotate += float(val)
        elif op == "bend":
            bend_no += 1
            part = cl.formed_world(prefix[:-1]) if len(prefix) > 1 else cl.formed_world([("feed", 1)])
            target = cum_rotate
            # 1) try to reach the bend plane at zero extra feed
            lbl, dest, gap = _choose_rotation(part, cur_tube, target, feed=0.0)
            extra_feed = 0.0
            # 2) if the head would sweep through the part, feed the wire out until it clears
            if gap < MARGIN and abs(dest - cur_tube) > 1e-6:
                for f in np.arange(FEED_STEP, FEED_MAX + 1e-6, FEED_STEP):
                    lbl, dest, gap = _choose_rotation(part, cur_tube, target, feed=float(f))
                    if gap >= MARGIN:
                        extra_feed = float(f)
                        break
            if abs(dest - cur_tube) > 1e-6:
                if extra_feed:
                    moves.append(dict(kind="feed", mm=extra_feed, note="clearance advance (feed back after)"))
                moves.append(dict(kind="rotate", deg=round(dest - cur_tube, 1), way=lbl,
                                  to=round(dest % 360, 1), gap=round(gap, 1),
                                  note=f"{lbl} way, worst gap {gap:.1f}mm"))
                if extra_feed:
                    moves.append(dict(kind="feed", mm=-extra_feed, note="retract clearance advance"))
                cur_tube = dest % 360
                if gap < 0:
                    ok = False
                    moves[-1 if not extra_feed else -2]["WARN"] = "ROTATION COLLIDES — no clearing feed found"
            # the bend stroke itself (pin-vs-part is handled by interference.py's pin_part rule)
            moves.append(dict(kind="bend", deg=round(float(val), 1)))
    return dict(moves=moves, ok=ok)


def _fmt(m):
    k = m["kind"]
    if k == "feed":
        return f"  FEED   {m['mm']:+7.1f} mm   {m.get('note','')}"
    if k == "rotate":
        w = "  ⚠ " + m["WARN"] if "WARN" in m else ""
        return f"  ROTATE {m['deg']:+7.1f}°  -> {m['to']:5.1f}°  ({m['note']}){w}"
    if k == "bend":
        return f"  BEND   {m['deg']:+7.1f}°"
    return str(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("program", nargs="?", default="staple")
    ap.add_argument("--springback", type=float, default=0.0)
    args = ap.parse_args()
    prog = bm.EXAMPLES.get(args.program)
    if prog is None:
        raise SystemExit(f"unknown example {args.program!r} ({', '.join(bm.EXAMPLES)})")
    res = plan(prog, springback=args.springback)
    print(f"plan for '{args.program}' ({'OK' if res['ok'] else 'HAS COLLISIONS'}):")
    for m in res["moves"]:
        print(_fmt(m))


if __name__ == "__main__":
    main()
