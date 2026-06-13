"""sleeve.py — printed adapter: 1/4" feed tube -> 608 (skateboard) bearing bore.

The stainless feed tube is 1/4" (6.35mm) OD; the 608 bearings have an 8mm bore.
This thin sleeve slips over the tube and seats in the bearing inner race, with a
small flange that locates it against the bearing face. Print TWO (one per bearing).

Wall is only ~0.83mm — print STANDING (axis vertical) so the perimeters run
concentric (hoop strength) instead of delaminating radially under the press fit.
Tune SLEEVE_ID/SLEEVE_OD to your printer: the tube should slide in snug and the
sleeve should press into the bearing (a dab of adhesive on the tube is fine).

    py/bin/python cad/sleeve.py   ->  build/sleeve.stl
"""
import os
from build123d import *
from base import BRG_BORE, BRG_W

TUBE_OD = 6.35                     # 1/4" stainless tube
ID_CLEAR = 0.15                    # printed-hole clearance so the tube slides in
SLEEVE_ID = TUBE_OD + ID_CLEAR     # ~6.50  (bore on the tube)
SLEEVE_OD = BRG_BORE               # 8.0    -> bearing inner race
SLEEVE_L = BRG_W + 1.0             # 8.0    span the 7mm bearing + 1mm proud
FLANGE_OD = SLEEVE_OD + 3.0        # 11.0   locating collar against the bearing face
FLANGE_T = 1.2
CHAMFER = 0.5                      # lead-in to start it into the race


def sleeve():
    body = Cylinder(SLEEVE_OD / 2, SLEEVE_L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    flange = Cylinder(FLANGE_OD / 2, FLANGE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = body + flange
    bore = (Pos(0, 0, -1) * Cylinder(SLEEVE_ID / 2, SLEEVE_L + FLANGE_T + 2,
                                     align=(Align.CENTER, Align.CENTER, Align.MIN)))
    part = part - bore
    # chamfer the leading (top, OD) rim so it starts into the bearing
    top_outer = part.edges().group_by(Axis.Z)[-1].sort_by(SortBy.RADIUS)[-1]
    part = chamfer(top_outer, CHAMFER)
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    s = sleeve()
    export_stl(s, "build/sleeve.stl")
    bb = s.bounding_box()
    print(f"sleeve  ID {SLEEVE_ID}  OD {SLEEVE_OD}  L {SLEEVE_L}  flange Ø{FLANGE_OD}x{FLANGE_T}"
          f"  bbox {[round(v,2) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}")
