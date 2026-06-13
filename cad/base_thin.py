"""base_thin.py — a thin slice of the base plate for a layout test print.

Just the bottom few layers of base_plate() (footprint + every hole: feeder bolt
pattern, motor/nose boss bores, benchtop holes). Slices the plate ALONE, not the
full assembly, so the cantilevered fixed gear (which dips below the deck) doesn't
leave cross-section blobs in the slice. Print it flat, set the real feeder on top,
and check the layout before committing to the full base.

    py/bin/python cad/base_thin.py [THICKNESS_MM]   ->  build/base_thin.stl
"""
import os
import sys
from build123d import *
from base import base_plate, BASE_TH, BASE_X0, BASE_X1, BASE_W

THIN = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0   # ~5 layers at 0.2mm

if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    slab = (Pos((BASE_X0 + BASE_X1) / 2, 0, -BASE_TH)
            * Box(BASE_X1 - BASE_X0 + 20, BASE_W + 20, THIN,
                  align=(Align.CENTER, Align.CENTER, Align.MIN)))
    thin = base_plate() & slab
    export_stl(thin, "build/base_thin.stl")
    bb = thin.bounding_box()
    print(f"base_thin {THIN}mm  bbox", [round(v, 1) for v in (bb.size.X, bb.size.Y, bb.size.Z)])
