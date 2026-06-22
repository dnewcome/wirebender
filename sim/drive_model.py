#!/usr/bin/env python3
"""drive_model.py — "what stock can THIS machine bend?" (the design explorer).

Turns the bend-drive parameters in machine.py into the two limits that decide a
bending job, so you can size a machine for a job (or a job for a machine):

  1. TORQUE to generate the bend (motor side):
       required = process_factor · σy · Z_plastic(section)         [grows as d³]
       available = motor_torque · usable_frac · DRIVE_RATIO · eff
     A stepper has no overload margin — if required > available it stalls and skips
     (exactly the 0.150" failure: pancake NEMA17 at the ceiling).

  2. CAPACITY of the printed disc (structural side):
       the output torque reacts through the 6 carrier rollers; the roller→disc-hole
       bearing stress must stay under the print material's allowable, or the holes
       crush/ovalise. Face width (DISC_FACE_W · N_DISCS) is the linear lever here.

The machine can bend a given stock only if it clears BOTH. `max_bendable()` returns
the larger of the two binding diameters so you can see which one to fix.

Section model: solid round wire  Z = d³/6;  tube  Z = (do³−di³)/6  (kind='tube',
wall=…). NOTE for tube the torque model is necessary-but-not-sufficient — tube also
needs a mandrel/wiper against wrinkling + ovalisation, which this does NOT model yet.

    python drive_model.py                 # capability table for the current machine
    python drive_model.py --material pa-cf
"""
from __future__ import annotations

import argparse

from machine import (DRIVE_RATIO, DRIVE_EFF, MOTOR_HOLDING_TORQUE, MOTOR_USABLE_FRAC,
                     BEND_PROCESS_FACTOR, CARRIER_R, CARRIER_ROLLER_D, DISC_FACE_W,
                     N_DISCS, N_CARRIER)

# ── design references ────────────────────────────────────────────────────────
# Printable-material BEARING allowable (sustained, derated for creep/heat), MPa.
# Order-of-magnitude from the bend-cell discussion; calibrate against real parts.
MATERIALS = {"pla": 15, "petg": 20, "abs": 22, "nylon": 28, "pc": 38, "pa-cf": 50}

# Common stock: (label, Ø mm, yield σy MPa). σy varies a lot with temper — mild ~400,
# hard-drawn / music wire 1000-1800; these defaults are mid-grade steel.
STOCK = [("16 ga wire", 1.29, 400), ("14 ga wire", 1.63, 400),
         ("0.150\" wire", 3.81, 400), ("1/4\" rod", 6.35, 400)]

# Poor load-sharing in a PRINTED cycloid: assume the worst carrier roller takes this
# fraction of the total tangential force (tolerances let only ~2 rollers engage).
PEAK_CARRIER_SHARE = 0.5


# ── section / torque ─────────────────────────────────────────────────────────
def plastic_modulus(d, kind="wire", wall=None):
    """Plastic section modulus Z (mm³). Solid round d³/6; tube (do³−di³)/6."""
    if kind == "tube":
        di = d - 2 * wall
        return (d ** 3 - di ** 3) / 6.0
    return d ** 3 / 6.0


def bend_torque(d, sigma_y, kind="wire", wall=None, process=BEND_PROCESS_FACTOR):
    """Output torque (N·m) to fully bend the section: process · σy · Z."""
    z = plastic_modulus(d, kind, wall)                    # mm³
    return process * sigma_y * z / 1000.0                 # MPa·mm³ = N·mm -> N·m


def drive_output_torque(motor_holding=MOTOR_HOLDING_TORQUE):
    """Deliverable output torque (N·m) = usable motor torque · ratio · efficiency."""
    return motor_holding * MOTOR_USABLE_FRAC * DRIVE_RATIO * DRIVE_EFF


# ── printed-disc capacity ────────────────────────────────────────────────────
def carrier_bearing_stress(out_torque):
    """Worst carrier roller→disc-hole bearing stress (MPa) for a given output torque."""
    total_tangential = out_torque / (CARRIER_R / 1000.0)      # N at the carrier circle
    peak = total_tangential * PEAK_CARRIER_SHARE
    area = CARRIER_ROLLER_D * DISC_FACE_W * N_DISCS           # mm²
    return peak / area                                        # N/mm² = MPa


def disc_torque_limit(material):
    """Output torque (N·m) at which the carrier bearing hits the material allowable."""
    allow = MATERIALS[material]
    area = CARRIER_ROLLER_D * DISC_FACE_W * N_DISCS
    peak = allow * area
    total = peak / PEAK_CARRIER_SHARE
    return total * (CARRIER_R / 1000.0)


# ── capability ───────────────────────────────────────────────────────────────
def _invert_d(t_out, sigma_y, kind, wall, process=BEND_PROCESS_FACTOR):
    """Largest Ø whose bend_torque == t_out (solid only; tube wall fixed)."""
    if kind == "tube":                                   # solve (do³-(do-2w)³) numerically
        lo, hi = 2 * wall + 0.1, 100.0
        for _ in range(60):
            mid = (lo + hi) / 2
            (lo, hi) = (mid, hi) if bend_torque(mid, sigma_y, kind, wall, process) < t_out else (lo, mid)
        return (lo + hi) / 2
    return (6.0 * t_out * 1000.0 / (process * sigma_y)) ** (1 / 3.0)


def max_bendable(sigma_y, material, motor_holding=MOTOR_HOLDING_TORQUE, kind="wire", wall=None):
    """Largest Ø this machine can bend, and which limit binds."""
    t_motor = drive_output_torque(motor_holding)
    t_disc = disc_torque_limit(material)
    d_motor = _invert_d(t_motor, sigma_y, kind, wall)
    d_disc = _invert_d(t_disc, sigma_y, kind, wall)
    binding = "torque" if d_motor <= d_disc else "disc"
    return min(d_motor, d_disc), binding, d_motor, d_disc


def capability(d, sigma_y, material, motor_holding=MOTOR_HOLDING_TORQUE, kind="wire", wall=None):
    req = bend_torque(d, sigma_y, kind, wall)
    avail = drive_output_torque(motor_holding)
    stress = carrier_bearing_stress(req)
    allow = MATERIALS[material]
    ok = req <= avail and stress <= allow
    return dict(req=req, avail=avail, t_margin=avail / req, stress=stress,
                allow=allow, s_margin=allow / stress, ok=ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--material", default="pa-cf", choices=MATERIALS)
    ap.add_argument("--motor", type=float, default=MOTOR_HOLDING_TORQUE, help="bend-motor holding torque (N·m)")
    a = ap.parse_args()
    t_out = drive_output_torque(a.motor)
    print(f"Drive: {DRIVE_RATIO:.0f}:1, motor {a.motor} N·m · {MOTOR_USABLE_FRAC} usable · {DRIVE_EFF} eff "
          f"-> {t_out:.1f} N·m output")
    print(f"Disc:  {N_DISCS}×{DISC_FACE_W}mm {a.material} discs, {N_CARRIER}× Ø{CARRIER_ROLLER_D} rollers @ r{CARRIER_R} "
          f"-> capacity {disc_torque_limit(a.material):.1f} N·m output\n")
    print(f"{'stock':>13} {'Ø':>5} {'σy':>5} {'need':>6} {'have':>6} {'torq×':>6} {'discMPa':>8} {'disc×':>6}  verdict")
    for label, d, sy in STOCK:
        c = capability(d, sy, a.material, a.motor)
        v = "OK" if c["ok"] else ("STALL" if c["req"] > c["avail"] else "DISC")
        print(f"{label:>13} {d:5.2f} {sy:5.0f} {c['req']:6.1f} {c['avail']:6.1f} {c['t_margin']:6.2f} "
              f"{c['stress']:8.1f} {c['s_margin']:6.2f}  {v}")
    dmax, bind, dm, dd = max_bendable(400, a.material, a.motor)
    print(f"\nMax bendable (σy=400, {a.material}): Ø{dmax:.2f} mm  (binds on {bind}; "
          f"torque-limit Ø{dm:.2f}, disc-limit Ø{dd:.2f})")


if __name__ == "__main__":
    main()
