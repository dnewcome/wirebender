"""housing.py — cyclo Housing_v1 (the rotating internal-gear ring), rebuilt parametric.

Fully parametric build123d reconstruction of the Sweep Dynamics micro-cycloidal Housing,
with NO dependency on the vendor STEP/STL. The geometry was read off the vendor B-rep once
(face types + exact radii / z-planes) and is reproduced here as named features:

  * Ø42 outer shell, 18.2 tall, with a 0.5mm chamfer on the top & bottom OD edges
    (r21 -> r20.5).
  * Ø37 bore (r18.5) with a 0.5mm chamfer at each mouth (r18.5 -> r19.0).
  * The internal "gear": 20 teeth, 18° apart, spanning z = 4.0 .. 14.2. Each tooth is a
    3-arc cycloidal-pin profile (verified by sampling the vendor bore + reading its B-rep
    arc centres off the circular edges): a convex Ø3 crest pin on an 18.5 pitch radius
    (crest at r17.0) blended into the Ø37 bore by two concave r0.5 root fillets. The
    surfaces are all circular arcs (cylinders), not a NURBS cycloid, so this is exact.

arbor_mount.py fuses the 4-bolt post pattern onto this body.

    py/bin/python cad/housing.py   ->  build/housing.stl
"""
import math
import os

from build123d import (Axis, BuildLine, BuildPart, BuildSketch, Circle, Mode, Plane,
                        PolarLocations, Polyline, add, export_stl, extrude, fillet,
                        make_face, revolve)

OD = 42.0
HEIGHT = 18.2
EDGE_CH = 0.5                       # chamfer on the OD edges and the bore mouths
R_OUT = OD / 2                      # 21.0 main OD
R_OUT_CH = R_OUT - EDGE_CH          # 20.5 at the chamfered top/bottom edges
R_BORE = 18.5                       # Ø37 bore
R_BORE_CH = R_BORE + EDGE_CH        # 19.0 at the chamfered bore mouths

N_PINS = 20
PIN_R = 1.5                         # Ø3 crest pins (convex)
PIN_PITCH = 18.5                    # pin-axis pitch radius (crest at r17.0, into the bore)
PIN_Z0, PIN_Z1 = 4.0, 14.2         # axial extent of the toothed section
ROOT_FILLET = 0.5                   # concave r0.5 fillet at each pin root (2 per tooth)
# The vendor also has a 0.25mm chamfer on each pin-crest end; it's omitted here — it is below
# FDM resolution and OCC's chamfer op is unreliable on the fused toothed solid (square pin ends).

# closed (radius, z) cross-section of the shell, revolved about Z
SHELL = [
    (R_OUT_CH, 0.0), (R_OUT, EDGE_CH), (R_OUT, HEIGHT - EDGE_CH), (R_OUT_CH, HEIGHT),
    (R_BORE_CH, HEIGHT), (R_BORE, HEIGHT - EDGE_CH), (R_BORE, EDGE_CH), (R_BORE_CH, 0.0),
]


def housing():
    with BuildPart() as bp:
        # shell: plain bore + OD/bore-mouth chamfers
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(*SHELL, close=True)
            make_face()
        revolve(axis=Axis.Z)
        # toothed band fused into the shell. Cross-section = outer disk minus the toothed bore
        # "air" (= bore disk minus the 20 pins); filleting the air's 40 corners rounds the pin
        # roots into the r0.5 concave fillets. Extruded over the pin section.
        with BuildSketch(Plane.XY.offset(PIN_Z0)):
            Circle(R_OUT)
            with BuildSketch(mode=Mode.PRIVATE) as air:
                Circle(R_BORE)
                with PolarLocations(PIN_PITCH, N_PINS):
                    Circle(PIN_R, mode=Mode.SUBTRACT)
                fillet(air.vertices(), ROOT_FILLET)
            add(air.sketch, mode=Mode.SUBTRACT)
        extrude(amount=PIN_Z1 - PIN_Z0)
    return bp.part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    export_stl(housing(), "build/housing.stl")
    import trimesh
    m = trimesh.load("build/housing.stl")
    print(f"housing: parametric cyclo ring  Ø{OD} x {HEIGHT}  {N_PINS} teeth (Ø{2*PIN_R} crest @ r{PIN_PITCH} "
          f"+ r{ROOT_FILLET} root fillets)  bbox {[round(v,1) for v in (m.bounds[1]-m.bounds[0])]}  "
          f"bodies:{len(m.split(only_watertight=False))}  watertight:{m.is_watertight}")
