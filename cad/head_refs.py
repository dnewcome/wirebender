"""head_refs.py — REFERENCE geometry for the sim only: the purchased parts around
the head (the cycloid drive body + the two NEMA17 motor bodies), positioned in the
head frame. These are NOT printed — they're bought — so they're generated here,
separately from the clean printable part scripts, and never fused into a printable
STL. The placements come from rothead.ghosts() (the head design authority).

    py/bin/python cad/head_refs.py   ->  build/head_refs.stl   (gitignored; cyclo is vendor)
"""
import os
from build123d import *
from rothead import ghosts

if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    refs = ghosts(motors=True, pinion_part=False)   # cyclo body + bend/rot NEMA bodies
    asm = Compound(children=list(refs.values()))
    export_stl(asm, "build/head_refs.stl")
    print("head_refs (reference, not printed):", ", ".join(refs))
