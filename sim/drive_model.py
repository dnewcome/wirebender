#!/usr/bin/env python3
"""drive_model.py — "what stock can THIS machine bend?" (the design explorer).

Turns the bend-drive parameters in machine.py into the two limits that decide a
bending job, so you can size a machine for a job (or a job for a machine):

  1. TORQUE to generate the bend (motor side):
       required = process_factor · σy · Z_plastic(section)         [grows as d³]
       available = (rated_torque/rated_current · set_current) · usable_frac · RATIO · eff
     Stepper torque is set by PHASE CURRENT, so the model works back from a target wire to
     the phase current it needs — hence the DRIVER class to spec per machine size. A stepper
     has no overload margin: if you under-set the current you stall (the 0.150" bench stall =
     driver set below the ~1.6A that wire needs). If the required current exceeds the motor's
     rated current, no driver setting helps — it's a bigger-motor / more-ratio problem.

  2. CAPACITY of the printed disc (structural side):
       the output torque reacts through the 6 carrier rollers; the roller→disc-hole
       bearing stress must stay under the print material's allowable, or the holes
       crush/ovalise. Face width (DISC_FACE_W · N_DISCS) is the linear lever here.

The machine can bend a given stock only if it clears BOTH. `max_bendable()` returns
the larger of the two binding diameters so you can see which one to fix.

Section model: solid round wire  Z = d³/6;  tube  Z = (do³−di³)/6  (kind='tube',
wall=…). NOTE for tube the torque model is necessary-but-not-sufficient — tube also
needs a mandrel/wiper against wrinkling + ovalisation, which this does NOT model yet.

    python drive_model.py                              # current machine + driver-current table
    python drive_model.py --current 1.5                # what a 1.5A driver setting can bend
    python drive_model.py --rated-torque 3 --rated-current 4   # spec a NEMA23 for a bigger machine
"""
from __future__ import annotations

import argparse

from machine import (DRIVE_RATIO, DRIVE_EFF, MOTOR_RATED_TORQUE, MOTOR_RATED_CURRENT,
                     MOTOR_SET_CURRENT, MOTOR_USABLE_FRAC, BEND_PROCESS_FACTOR,
                     CARRIER_R, CARRIER_ROLLER_D, DISC_FACE_W, N_DISCS, N_CARRIER)

# Stepper-driver classes by usable continuous phase current (A) — for the "what driver" hint.
DRIVERS = [("A4988", 1.0), ("DRV8825", 1.7), ("TMC2209", 1.7), ("TB6600", 3.5),
           ("DM542 / TMC5160", 4.0), ("DM860 / closed-loop", 6.0)]

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


def motor_torque(set_current=MOTOR_SET_CURRENT, rated_torque=MOTOR_RATED_TORQUE,
                 rated_current=MOTOR_RATED_CURRENT):
    """Usable motor torque (N·m) at a driver current setting. Torque ~ current up to the
    rated current; beyond rated it saturates (clamped) and just makes heat."""
    return (rated_torque / rated_current) * min(set_current, rated_current) * MOTOR_USABLE_FRAC


def drive_output_torque(set_current=MOTOR_SET_CURRENT, **mk):
    """Deliverable output torque (N·m) = usable motor torque · ratio · efficiency."""
    return motor_torque(set_current, **mk) * DRIVE_RATIO * DRIVE_EFF


def current_for_torque(out_torque, rated_torque=MOTOR_RATED_TORQUE, rated_current=MOTOR_RATED_CURRENT):
    """Phase current (A) needed to deliver `out_torque` at the output — the DRIVER requirement.
    = (motor torque needed) / (torque-per-amp). May exceed the motor's rated current, in which
    case this motor can't do it at any driver setting (need a bigger motor or more ratio)."""
    motor_need = out_torque / (DRIVE_RATIO * DRIVE_EFF * MOTOR_USABLE_FRAC)
    return motor_need / (rated_torque / rated_current)


def current_for_stock(d, sigma_y, kind="wire", wall=None, **mk):
    return current_for_torque(bend_torque(d, sigma_y, kind, wall), **mk)


def driver_for(current):
    """Smallest driver class that can source `current`."""
    for name, cap in DRIVERS:
        if current <= cap:
            return name
    return ">6A (industrial)"


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


def max_bendable(sigma_y, material, set_current=MOTOR_SET_CURRENT, kind="wire", wall=None,
                 rated_torque=MOTOR_RATED_TORQUE, rated_current=MOTOR_RATED_CURRENT):
    """Largest Ø this machine can bend, and which limit binds."""
    t_motor = drive_output_torque(set_current, rated_torque=rated_torque, rated_current=rated_current)
    t_disc = disc_torque_limit(material)
    d_motor = _invert_d(t_motor, sigma_y, kind, wall)
    d_disc = _invert_d(t_disc, sigma_y, kind, wall)
    binding = "torque" if d_motor <= d_disc else "disc"
    return min(d_motor, d_disc), binding, d_motor, d_disc


def capability(d, sigma_y, material, set_current=MOTOR_SET_CURRENT, kind="wire", wall=None,
               rated_torque=MOTOR_RATED_TORQUE, rated_current=MOTOR_RATED_CURRENT):
    req = bend_torque(d, sigma_y, kind, wall)
    avail = drive_output_torque(set_current, rated_torque=rated_torque, rated_current=rated_current)
    stress = carrier_bearing_stress(req)
    allow = MATERIALS[material]
    req_i = current_for_stock(d, sigma_y, kind, wall, rated_torque=rated_torque, rated_current=rated_current)
    ok = req <= avail and stress <= allow
    return dict(req=req, avail=avail, t_margin=avail / req, stress=stress, allow=allow,
                s_margin=allow / stress, ok=ok, req_current=req_i)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--material", default="pa-cf", choices=MATERIALS)
    ap.add_argument("--current", type=float, default=MOTOR_SET_CURRENT, help="driver phase-current SET point (A)")
    ap.add_argument("--rated-torque", type=float, default=MOTOR_RATED_TORQUE, help="motor holding torque at rated current (N·m)")
    ap.add_argument("--rated-current", type=float, default=MOTOR_RATED_CURRENT, help="motor rated phase current (A)")
    a = ap.parse_args()
    mk = dict(rated_torque=a.rated_torque, rated_current=a.rated_current)
    t_out = drive_output_torque(a.current, **mk)
    tpa = a.rated_torque / a.rated_current
    print(f"Motor: {a.rated_torque} N·m @ {a.rated_current} A  ({tpa:.3f} N·m/A) · set {a.current} A · "
          f"{MOTOR_USABLE_FRAC} usable")
    print(f"Drive: {DRIVE_RATIO:.0f}:1 · {DRIVE_EFF} eff  ->  {t_out:.1f} N·m output at the set current")
    print(f"Disc:  {N_DISCS}×{DISC_FACE_W}mm {a.material}, {N_CARRIER}× Ø{CARRIER_ROLLER_D} rollers @ r{CARRIER_R}"
          f"  -> capacity {disc_torque_limit(a.material):.1f} N·m\n")
    print(f"{'stock':>13} {'Ø':>5} {'need':>6} {'req A':>6} {'driver':>16} {'disc×':>6}  verdict")
    for label, d, sy in STOCK:
        c = capability(d, sy, a.material, a.current, **mk)
        ri = c["req_current"]
        if ri <= a.current:
            v = "OK"
        elif ri <= a.rated_current:
            v = f"raise I to {ri:.1f}A"
        else:
            v = "MOTOR (exceeds rated)"
        if c["s_margin"] < 1:
            v = "DISC"
        print(f"{label:>13} {d:5.2f} {c['req']:6.1f} {ri:6.2f} {driver_for(ri):>16} {c['s_margin']:6.2f}  {v}")
    for lbl, cur in (("set current", a.current), ("rated current", a.rated_current)):
        dmax, bind, dm, dd = max_bendable(400, a.material, cur, **mk)
        print(f"\nMax bendable @ {lbl} ({cur}A, σy=400, {a.material}): Ø{dmax:.2f} mm  (binds on {bind}; "
              f"torque Ø{dm:.2f}, disc Ø{dd:.2f})")


if __name__ == "__main__":
    main()
