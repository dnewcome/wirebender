# Learnings

Notes on what's worked well while building this machine, so we don't relearn it.

## Read the STEP, then re-create vendor geometry parametrically in build123d

When working with a purchased part supplied as a STEP file (e.g. the Sweep Dynamics
`20-1 Micro Cycloidal.step` assembly), it pays to **inspect the B-rep directly** and then
**re-create the geometry as a parametric build123d model** rather than carrying the vendor
mesh around.

**Inspecting the STEP is genuinely useful.** build123d can import the assembly, list its
named children (`Housing_v1`, `End_Cap`, `Base_-_Nema_17`, …), and enumerate each solid's
edges and faces with their *semantic types* — `CYLINDER`, `CONE`, `PLANE`, `CIRCLE`,
`BSPLINE` — plus exact radii, centers, and z-planes. That turns "stare at an STL" into
"query the actual design intent." Two payoffs we hit directly:

- **It exposes what a shape really is.** The cyclo Housing's internal "gear" *looked*
  epicycloidal in a render, but the faces were 20 `CYLINDER`s (Ø3 pins on an 18.39 pitch
  radius, 18° apart) + `CONE` chamfers, with **zero B-spline faces** — the 80 B-spline
  *edges* were just the intersection curves where round pins meet the bore. Measuring the
  faces settled "cylindrical-with-chamfers vs. epicycloidal" in seconds; eyeballing the mesh
  could not.
- **It gives exact dimensions to rebuild from.** The `End_Cap` was reconstructed as a clean
  revolve + 6 counterbored holes using radii/z-planes read straight off its circular edges.

**Re-creating it in build123d recovers semantic information the mesh has thrown away.**
An STL is just triangles — no notion of "this is a Ø40 flange," "these are 6 M3 counterbores
on a r10.125 circle," "this is the turn-down amount." Once the part is a parametric script,
those concepts are *named variables and operations*, so later edits are trivial and
intention-revealing: turning the End_Cap OD down 2mm is `R_FLANGE = (42 - OD_TURNDOWN)/2`;
the bolt pattern is a loop. Modifying a mesh to achieve the same thing is fragile guesswork.

**Reading the code beats re-probing the geometry over and over.** A parametric `.py` is the
single, legible source of truth for a part. You read the dimensions and features once, in
context, instead of repeatedly loading the STL, slicing sections, and ray-probing to
re-discover what something is. It also makes the part vendor-independent and instant to
rebuild.

Caveats:
- The vendor tessellation can be unreliable — the `End_Cap` STEP had 8 degenerate
  ("null triangulation") faces that OpenCASCADE couldn't mesh at any tolerance, so 4 of its
  6 holes dropped into detached, non-watertight shells and a slicer only printed 2 holes.
  A parametric rebuild sidesteps broken vendor tessellation entirely.
- When the vendor solid *does* tessellate cleanly (the `Housing_v1` mesh is watertight) and
  its surface is intricate, a faithful STEP extraction is fine to keep — but even then,
  reading the B-rep tells you the geometry is simple enough (20 plain pins) that a parametric
  version would be just as exact if you later need to modify it.
