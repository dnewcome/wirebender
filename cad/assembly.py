"""assembly.py — the FULL machine as one build123d assembly (base + head +
feeder + feed tube), placed with the SAME world transforms the sim uses
(sim/make_mjcf.py). Until now the only assembled view was the MJCF; this is the
build123d equivalent, exportable as a single STEP for CAD / interference work.

World frame (mm): floor at z=0; the 6mm base deck sits on the floor (top at +6);
the wire axis runs along X at z=41 (= 6 deck + 35 AXIS_Z); the feeder centre is
at x=151. Each sub-assembly is built in its own local frame (base about the wire
axis AXIS_Z above its deck; head about the wire axis at its origin) and lifted
into the world here — exactly what make_mjcf.py does to the STLs.

Exports:
    build/assembly.step   native printable solids (base + head pieces + tube) — clean B-rep
    build/assembly.stl    everything incl. motor/cycloid ghosts + feeder (faceted, for viewing)

Run:  py/bin/python cad/assembly.py
"""
import os
from build123d import *

import base as B
import rothead as H
from base import AXIS_Z, BASE_TH, FEEDER_X, TUBE_BORE

# ── world placement (mm) — mirrors sim/make_mjcf.py ─────────────────
DECK_TOP_Z = BASE_TH                 # base deck bottom on the floor -> deck top at +6
AXIS_WORLD_Z = DECK_TOP_Z + AXIS_Z   # wire axis in the world: 6 + 35 = 41
TUBE_OD = 8.0                        # 1/4" feed tube (sim uses Ø8)
TUBE_X0, TUBE_X1 = -30.0, 95.0       # head gap -> into the feeder (matches the sim tube)
FEEDER_STL = "build/feeder_body.stl"


def _head(with_motors):
    """Head sub-assembly in its local frame (wire axis at the origin)."""
    children = [H.rot_piece(), H.bend_piece()]
    if with_motors:                                   # cycloid base + pancakes + pinion
        children += list(H.ghosts().values())
    return Compound(children=children)


def full_assembly(with_motors=True, with_feeder=True):
    """Everything placed in the world frame. with_motors/with_feeder add the
    faceted ghosts (good for STL/viewing, heavy/ugly in STEP)."""
    parts = [
        Pos(0, 0, DECK_TOP_Z) * B.build_base(),       # base: deck bottom on the floor
        Pos(0, 0, AXIS_WORLD_Z) * _head(with_motors),  # head: lifted onto the wire axis
        B.xcyl(TUBE_OD, TUBE_X0, TUBE_X1, 0, AXIS_WORLD_Z),  # passive feed tube
    ]
    if with_feeder and os.path.exists(FEEDER_STL):    # imported rough primitive, centred
        parts.append(Pos(FEEDER_X, 0, AXIS_WORLD_Z) * import_stl(FEEDER_STL))
    return Compound(children=parts)


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    # STEP: native B-rep solids only (base + head pieces + tube) — clean for CAD
    native = full_assembly(with_motors=False, with_feeder=False)
    export_step(native, "build/assembly.step")
    # STL: the full picture incl. motor/cycloid ghosts + feeder
    full = full_assembly(with_motors=True, with_feeder=True)
    export_stl(full, "build/assembly.stl")
    bb = full.bounding_box()
    print("assembly.step (native) + assembly.stl (full) | full bbox",
          [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
          "| wire axis z =", AXIS_WORLD_Z)
