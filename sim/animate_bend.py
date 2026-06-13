"""animate_bend.py — animate a bend program in the assembled machine.

The wire feeds out of the head's bend point and bends as the program runs: the
head rotates about the wire axis (Axis 2 / tube_rot) and the cycloidal bend die
sweeps (Axis 3 / bend), while the already-formed wire grows out the front and
swings about the bending point — like the real machine pushing wire past the die.

Wire-forming geometry comes from bend_model (the forward model); the machine comes
from the generated wirebender.xml. Each frame re-injects the formed wire and sets
the two joints.

    cd sim && ../py/bin/python animate_bend.py staple      # -> preview/bending_staple.gif
    cd sim && ../py/bin/python animate_bend.py square --view
"""
import math
import argparse
from pathlib import Path
import numpy as np
import mujoco
from bend_model import EXAMPLES, BEND_RADIUS

HERE = Path(__file__).resolve().parent
XML = HERE / "wirebender.xml"
OUT = HERE / "preview"

# ── machine frame (mm): wire axis = X at (y=0, z=35); bend point at the head front ──
ZAXIS_MM = 21.0
B = np.array([-30.0, 0.0, ZAXIS_MM])     # bend point (mandrel), fixed in world
E = np.array([-1.0, 0.0, 0.0])           # formed wire trails -X (out the front)
B0 = np.array([0.0, -1.0, 0.0])          # deflect in -Y -> bend axis = +Z (the cycloidal
                                          # axis), so the wire bends in the horizontal plane
ROT_AXIS = np.array([1.0, 0.0, 0.0])     # Axis 2 (head rotation) = the wire axis
FEED_BACK_X = 90.0                        # unformed stub runs back to here (+X)
WIRE_R_MM = 1.4                           # exaggerated for visibility (real wire ~Ø1.6)
C_FORMED = "0.95 0.25 0.15 1"             # bright red so it reads against the head
C_STOCK = "0.55 0.57 0.62 1"


def _rot(v, axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def _rot_about(p, center, axis, ang):
    return center + _rot(p - center, axis, ang)


def frames_for(program, springback=0.0, feed_steps=6, bend_steps=8, rot_steps=8,
               bend_radius=None):
    """Yield (tube_deg, bend_deg, W) per frame; W = formed-wire polyline (mm, world)."""
    r = BEND_RADIUS if bend_radius is None else float(bend_radius)
    W = [B.copy()]
    tube = 0.0
    out = [(tube, 0.0, np.array(W))]
    for op, val in program:
        if op == "feed":
            d = float(val)
            base = [p.copy() for p in W]
            for s in range(1, feed_steps + 1):
                t = s / feed_steps
                W = [B.copy()] + [p + E * d * t for p in base]
                out.append((tube, 0.0, np.array(W)))
        elif op == "rotate":
            t0, t1 = tube, tube + float(val)
            for s in range(1, rot_steps + 1):
                tube = t0 + (t1 - t0) * s / rot_steps
                out.append((tube, 0.0, np.array(W)))
            tube = t1
        elif op == "bend":
            a = math.radians(float(val) * (1.0 - springback))
            b = _rot(B0, ROT_AXIS, math.radians(tube))
            axis = np.cross(E, b)
            C = B + b * r
            base = [p.copy() for p in W]

            def build(ang):
                if r > 1e-9 and abs(ang) > 1e-9:
                    n = max(1, int(abs(math.degrees(ang)) / 8))
                    arc = [_rot_about(B, C, axis, ang * k / n) for k in range(n + 1)]
                else:
                    arc = [B.copy()]
                return arc + [_rot_about(p, C, axis, ang) for p in base[1:]]

            for s in range(1, bend_steps + 1):
                ang = a * s / bend_steps
                W = build(ang)
                out.append((tube, math.degrees(ang), np.array(W)))
            held = [p.copy() for p in W]
            for s in range(bend_steps - 1, -1, -1):
                out.append((tube, math.degrees(a * s / bend_steps), np.array(held)))
        else:
            raise ValueError(f"unknown op: {op!r}")
    return out


def _capsule(p0, p1, r_mm, rgba):
    a, b = np.asarray(p0) * 0.001, np.asarray(p1) * 0.001
    return (f'      <geom type="capsule" fromto="{a[0]:.5f} {a[1]:.5f} {a[2]:.5f} '
            f'{b[0]:.5f} {b[1]:.5f} {b[2]:.5f}" size="{r_mm*0.001:.5f}" rgba="{rgba}" '
            f'contype="0" conaffinity="0"/>')


def _wire_body(W):
    g = [_capsule(B, [FEED_BACK_X, 0, ZAXIS_MM], WIRE_R_MM, C_STOCK)]
    for i in range(len(W) - 1):
        if np.linalg.norm(W[i + 1] - W[i]) > 1e-6:
            g.append(_capsule(W[i], W[i + 1], WIRE_R_MM, C_FORMED))
    return '    <body name="formed_wire">\n' + "\n".join(g) + "\n    </body>\n  </worldbody>"


def _inject(xml, W):
    return xml.replace("  </worldbody>", _wire_body(W), 1)


def _set_joint(model, data, name, val_rad):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[model.jnt_qposadr[jid]] = val_rad


def _rgba(s):
    return np.array([float(x) for x in s.split()], dtype=np.float32)


def _add_capsule_scn(scn, p0, p1, r_mm, rgba):
    """Append a capsule (mm endpoints) to a live MjvScene."""
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        np.zeros(3), np.zeros(3), np.zeros(9), rgba)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, r_mm * 0.001,
                         np.asarray(p0, float) * 0.001, np.asarray(p1, float) * 0.001)
    scn.ngeom += 1


def _draw_wire_scn(scn, W):
    """Redraw the formed wire + stock stub into the viewer's user scene."""
    scn.ngeom = 0
    _add_capsule_scn(scn, B, [FEED_BACK_X, 0, ZAXIS_MM], WIRE_R_MM, _rgba(C_STOCK))
    rgba = _rgba(C_FORMED)
    for i in range(len(W) - 1):
        if np.linalg.norm(W[i + 1] - W[i]) > 1e-6:
            _add_capsule_scn(scn, W[i], W[i + 1], WIRE_R_MM, rgba)


def view_live(name, program, springback=0.0, fps=20):
    """Play the bend program live in the interactive MuJoCo viewer (needs a display).
    The machine model loads once; the wire is drawn as user-scene capsules each frame."""
    import time
    import mujoco.viewer
    frames = frames_for(program, springback=springback)
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    cam = _fixed_camera(frames)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth, viewer.cam.elevation = cam.azimuth, cam.elevation
        viewer.cam.distance = cam.distance
        viewer.cam.lookat[:] = cam.lookat
        print(f"playing '{name}' — orbit with the mouse · close the window to exit")
        while viewer.is_running():
            for tube_deg, bend_deg, W in frames:
                if not viewer.is_running():
                    break
                _set_joint(model, data, "tube_rot", math.radians(tube_deg))
                _set_joint(model, data, "bend", math.radians(bend_deg))
                mujoco.mj_forward(model, data)
                _draw_wire_scn(viewer.user_scn, W)
                viewer.sync()
                time.sleep(1.0 / fps)
            time.sleep(0.6)        # hold the finished part, then loop


def _fixed_camera(frames):
    allW = np.vstack([f[2] for f in frames])     # mm
    # the formed wire trails -X in front of the machine; view from the -X/side so the
    # head doesn't hide it. Centre on the wire (+ a little of the bend point/head).
    lo, hi = allW.min(0), allW.max(0)
    lo = np.minimum(lo, B - 5); hi = np.maximum(hi, B + 5)
    cam = mujoco.MjvCamera()
    # centre on the formed wire (it trails -X in front of the machine)
    cam.lookat[:] = np.array([allW[:, 0].mean(), allW[:, 1].mean(), ZAXIS_MM]) * 0.001
    cam.distance = max(float(np.max(hi - lo)) * 0.001, 0.05) * 2.3
    cam.azimuth, cam.elevation = 180, -40        # from the front (-X), looking back + down
    return cam


def render_gif(name, program, springback=0.0):
    from PIL import Image
    frames = frames_for(program, springback=springback)
    base_xml = XML.read_text()
    tmp = HERE / "_anim_tmp.xml"
    cam = _fixed_camera(frames)
    imgs = []
    for tube_deg, bend_deg, W in frames:
        tmp.write_text(_inject(base_xml, W))
        model = mujoco.MjModel.from_xml_path(str(tmp))   # geom count changes as the wire grows
        data = mujoco.MjData(model)
        _set_joint(model, data, "tube_rot", math.radians(tube_deg))
        _set_joint(model, data, "bend", math.radians(bend_deg))
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 640, 900)      # fresh per frame (model differs)
        renderer.update_scene(data, cam)
        imgs.append(Image.fromarray(renderer.render().copy()))
        renderer.close()
    OUT.mkdir(exist_ok=True)
    path = OUT / f"bending_{name}.gif"
    pal = [im.convert("P", palette=Image.ADAPTIVE) for im in imgs]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=55, loop=0,
                optimize=False, disposal=2)
    tmp.unlink(missing_ok=True)
    print(f"wrote {path}  ({len(imgs)} frames)")


def _resolve_program(source, piece, tol):
    """source -> (display_name, program). Accepts a built-in example name, a
    .gcode file (parsed back), or any slicer input (path file / SVG / mesh)."""
    if source in EXAMPLES:
        return source, EXAMPLES[source]
    p = Path(source)
    if not p.exists():
        raise SystemExit(f"{source!r} is not an example ({', '.join(EXAMPLES)}) or a file")
    import slicer
    if p.suffix.lower() == ".gcode":
        programs = slicer.parse_gcode(p.read_text())
    else:                                   # slice it the same way slicer.py would
        _, paths = slicer.extract(str(p))
        programs = [slicer.slice_path(P, simplify_tol=tol) for P in paths]
    if not programs:
        raise SystemExit(f"no bend program found in {source!r}")
    if len(programs) > 1:
        print(f"# {len(programs)} pieces; animating piece {piece+1} "
              f"(use --piece N for others)")
    return f"{p.stem}_p{piece+1}" if len(programs) > 1 else p.stem, programs[piece]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("program", nargs="?", default="staple",
                    help=f"example ({', '.join(EXAMPLES)}), a .gcode file, or a path/SVG/mesh")
    ap.add_argument("--piece", type=int, default=0, help="which piece (multi-piece input)")
    ap.add_argument("--tol", type=float, default=0.4, help="simplify tol mm (when slicing a file)")
    ap.add_argument("--springback", type=float, default=0.0)
    ap.add_argument("--view", action="store_true",
                    help="play live in the interactive MuJoCo viewer (needs a display)")
    args = ap.parse_args()
    name, program = _resolve_program(args.program, args.piece, args.tol)
    if args.view:
        view_live(name, program, springback=args.springback)
    else:
        render_gif(name, program, springback=args.springback)


if __name__ == "__main__":
    main()
