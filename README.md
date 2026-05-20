# step2svg — Convert 3D STEP models to 2D SVG

Extract top-down 2D SVG silhouettes from KiCad 3D STEP models, with
colour preservation.  The resulting SVGs are useful as:

- **PCBdraw** component library entries (`--pcbdraw` mode)
- **Fritzing** part SVG creation (icon / breadboard views)
- **Datasheet-style** technical illustrations

Uses **XCAF tessellation + painter's algorithm** — loads the STEP file
via OpenCascade's XCAF document layer (preserving per-face colours),
tessellates into triangles, groups them by colour, culls back-faces,
and paints back-to-front as filled SVG `<path>` elements.

---

## Requirements

Python 3.10+ with `cadquery-ocp` (provides the `OCP` OpenCascade bindings):

```bash
pip install cadquery-ocp
```

---

## Quick start

```bash
# Render a STEP model from above (default)
python3 step2svg.py part.step -o part.svg

# PCBdraw-ready output (keeps origin at 0,0, Y matches KiCad PCB coords)
python3 step2svg.py part.step --pcbdraw -o part.svg

# Finer mesh for curved parts
python3 step2svg.py part.step --deflection 0.02 --angle 0.2

# View from below
python3 step2svg.py part.step --bottom -o part_bottom.svg

# Include back-facing triangles (full silhouette, bigger SVG)
python3 step2svg.py part.step --all-faces

# Add thin outline strokes
python3 step2svg.py part.step --stroke "#222" --stroke-width 0.05
```

### `--pcbdraw` mode

A shorthand for three flags that together produce an SVG PCBdraw can
consume as a component template:

```
--keep-origin    Keep STEP (0, 0) as the SVG origin (don't centre)
--kicad-y        Negate Y so the SVG uses KiCad's Y-down convention
--origin-marker  Emit a <rect id="origin"> at (0, 0) for PCBdraw
```

---

## How it works

1. **Load STEP via XCAF** — `STEPCAFControl_Reader` loads the model into
   an XCAF document so per-face colours survive the import.
2. **Tessellate** — `BRepMesh_IncrementalMesh` converts the shape into
   triangles (deflection and angle tolerances control mesh density).
3. **Group by colour** — each face is queried for its XCAF colour
   (face → parent solid → default gray). Triangles from faces sharing
   the same colour are grouped together.
4. **Cull back-faces** — triangles whose normal points away from the
   camera are dropped (override with `--all-faces`).
5. **Paint back-to-front** — colour groups are sorted by max-Z (or
   min-Z for bottom view) and emitted as filled SVG `<path>` elements
   — the classic painter's algorithm ensures correct occlusion.

Output coordinates are in **millimetres** (1 SVG user unit = 1 mm).

---

## Coordinate Systems

| System | X | Y | Z |
|--------|---|---|---|
| KiCad PCB | right | **down** | out of board |
| STEP (KiCad export) | right | up | out of board |
| SVG | right | down | — |

KiCad maps `(X_kicad, Y_kicad)` → `(X_step, –Y_step)` when writing
the STEP file.  `--kicad-y` reverses this flip so the SVG ends up
Y-down, matching both SVG and KiCad PCB conventions.

For bottom views (`--bottom`), X is mirrored in addition to the Y flip,
so the result matches what a camera below the part actually sees.

---

## Origin marker

PCBdraw uses the `<rect id="origin">` element to locate the component's
reference point (usually pad 1 / footprint origin).  The rect's
**top-left corner** (`x`, `y` attributes) is treated as the origin
position — PCBdraw's `element_position()` does not compute the centre
from width/height.

The origin rect is therefore placed at `x="0" y="0"` so the top-left
corner coincides with shape (0, 0).  See the `--origin-marker` /
`--pcbdraw` flags.

---

## `build_pcbdraw_lib.py` usage

A companion script for batch-converting a list of footprints into a
PCBdraw library lives in the
[p4hil](https://github.com/tannewt/p4hil) project.  It calls
`step2svg.py --pcbdraw` for each entry, generates front + back SVGs,
and falls back to synthetic outlines for footprints without STEP
models.
