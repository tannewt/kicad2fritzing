#!/usr/bin/env python3
"""
step2svg2 — Convert 3D STEP files to top-down 2D SVGs with color.

Loads STEP files via OpenCascade's XCAF document layer so that per-face
colors are preserved.  The shape is tessellated, upward-facing triangles
(those visible from a top-down camera) are grouped by color, and each
group is emitted as a single filled SVG <path>.  Groups are painted in
ascending max-Z order so taller parts render on top of shorter ones —
the classic painter's algorithm.

The output is in millimeters (1 user unit = 1 mm), which matches
Fritzing's native breadboard scale and is easy for pcbdraw to consume.

Requires:
    pip install cadquery-ocp     # provides the OCP module

Usage:
    python step2svg2.py part.step                       # writes part.svg
    python step2svg2.py part.step -o icon.svg
    python step2svg2.py part.step --deflection 0.02     # finer mesh
    python step2svg2.py part.step --all-faces           # incl. downward
    python step2svg2.py part.step --no-flatten          # keep paths layered
    python step2svg2.py part.step --bottom              # view from below
    python step2svg2.py part.step --stroke "#222" --stroke-width 0.05
"""

import argparse
import math
import sys
from collections import defaultdict
from pathlib import Path

# ── OCP imports ──────────────────────────────────────────────────────────────
try:
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
    from OCP.TDF import TDF_LabelSequence
    from OCP.Quantity import Quantity_ColorRGBA
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
except ImportError as e:
    sys.stderr.write(
        "Error: OCP (OpenCascade Python) is required.\n"
        "  pip install cadquery-ocp\n"
        f"Import error: {e}\n"
    )
    sys.exit(1)


# ── Defaults ──────────────────────────────────────────────────────────────────

DEFAULT_COLOR = (0.72, 0.72, 0.74, 1.0)   # neutral gray
COLOR_KEY_PRECISION = 3                    # round RGB(A) for grouping


# ── STEP loading with colors ──────────────────────────────────────────────────

def load_step_with_colors(path):
    """Load a STEP file via XCAF.  Returns (shapes, color_tool, doc).

    The document is returned so the caller keeps it alive — the
    XCAFDoc_ColorTool holds only a back-reference and lookups silently
    fail once the document is garbage-collected.
    """
    doc = TDocStd_Document(TCollection_ExtendedString("step-doc"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    status = reader.ReadFile(str(path))
    if int(status) != 1:
        raise RuntimeError(f"STEP read failed (status={int(status)}): {path}")
    if not reader.Transfer(doc):
        raise RuntimeError(f"STEP transfer failed: {path}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    free = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free)
    if free.Length() == 0:
        raise RuntimeError("No top-level shapes found in STEP file")

    shapes = [shape_tool.GetShape_s(free.Value(i)) for i in range(1, free.Length() + 1)]
    return shapes, color_tool, doc


def get_face_color(color_tool, face):
    """Return (r, g, b, a) in 0..1, or None if no color is assigned.

    XCAF raises Standard_NullObject when the shape isn't tracked by the
    document; treat that as "no colour".
    """
    c = Quantity_ColorRGBA()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf,
              XCAFDoc_ColorType.XCAFDoc_ColorGen,
              XCAFDoc_ColorType.XCAFDoc_ColorCurv):
        try:
            ok = color_tool.GetColor(face, t, c)
        except Exception:
            ok = False
        if ok:
            rgb = c.GetRGB()
            return (rgb.Red(), rgb.Green(), rgb.Blue(), c.Alpha())
    return None


# ── Tessellation & triangle extraction ────────────────────────────────────────

def tessellate(shape, linear_deflection=0.05, angular_deflection=0.5):
    """In-place mesh of the shape with the given tolerances (mm / rad)."""
    BRepMesh_IncrementalMesh(shape, linear_deflection, False,
                             angular_deflection, True).Perform()


def color_key(col):
    """Quantize a color so similar values group together."""
    r, g, b, a = col
    p = COLOR_KEY_PRECISION
    return (round(r, p), round(g, p), round(b, p), round(a, p))


def extract_triangles_by_color(shapes, color_tool, default_color,
                               linear_def, angular_def, upward_only,
                               flip_z, kicad_y=False):
    """Return dict {color_key: list[(p1, p2, p3)]} and dict {color_key: col_rgba}.

    Triangles are 3D points (x, y, z) in shape coordinates.  When upward_only
    is True we skip triangles whose normal points away from the camera.
    If flip_z is True (bottom view) the camera is along -Z instead of +Z.
    """
    triangles = defaultdict(list)
    colors = {}
    cam_dir_z = -1.0 if flip_z else +1.0

    for shape in shapes:
        tessellate(shape, linear_def, angular_def)

        # Pre-compute a colour for each solid so faces can inherit it.
        solid_colors = []  # list of (solid_shape, rgba)
        sexp = TopExp_Explorer(shape, TopAbs_SOLID)
        while sexp.More():
            sraw = sexp.Current()
            sc = get_face_color(color_tool, sraw)
            if sc is not None:
                solid_colors.append((sraw, sc))
            sexp.Next()

        def inherited_color(face_raw):
            for sraw, sc in solid_colors:
                # IsSame: shares the underlying TShape (same topology).
                # We check if the face belongs to this solid by walking
                # its faces — cheap enough because solids are few.
                fe = TopExp_Explorer(sraw, TopAbs_FACE)
                while fe.More():
                    if fe.Current().IsSame(face_raw):
                        return sc
                    fe.Next()
            return None

        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            # Look up colour against the *un-cast* shape — XCAF identifies
            # shapes by their underlying TShape pointer, and casting through
            # TopoDS.Face() may create a wrapper XCAF doesn't recognise.
            raw = exp.Current()
            col = (get_face_color(color_tool, raw)
                   or (inherited_color(raw) if solid_colors else None)
                   or default_color)
            face = TopoDS.Face(raw)
            key = color_key(col)
            if key not in colors:
                colors[key] = col

            reversed_ = (face.Orientation() == TopAbs_REVERSED)
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is None:
                exp.Next()
                continue

            trsf = loc.Transformation()
            nb_nodes = tri.NbNodes()
            nb_tri = tri.NbTriangles()

            # Cache transformed node coordinates
            nodes = [None] * (nb_nodes + 1)
            for i in range(1, nb_nodes + 1):
                p = tri.Node(i)
                p.Transform(trsf)
                nodes[i] = (p.X(), p.Y(), p.Z())

            for i in range(1, nb_tri + 1):
                t = tri.Triangle(i)
                n1, n2, n3 = t.Get()
                if reversed_:
                    n2, n3 = n3, n2
                a = nodes[n1]
                b = nodes[n2]
                c = nodes[n3]

                if kicad_y:
                    # STEP/KiCad export uses Y-up; KiCad PCB coords are
                    # Y-down.  Flip Y in shape coords so the resulting
                    # 2D image has +Y going down (matches kicad_mod).
                    a = (a[0], -a[1], a[2])
                    b = (b[0], -b[1], b[2])
                    c = (c[0], -c[1], c[2])

                if upward_only:
                    # Compute Z component of (b-a) × (c-a)
                    ux, uy, _uz = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
                    vx, vy, _vz = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
                    nz = ux * vy - uy * vx
                    if nz * cam_dir_z <= 0.0:
                        continue

                triangles[key].append((a, b, c))
            exp.Next()

    return triangles, colors


# ── 2D helpers ────────────────────────────────────────────────────────────────

def compute_bbox(triangles_by_key):
    xs, ys = [], []
    for tris in triangles_by_key.values():
        for tri in tris:
            for p in tri:
                xs.append(p[0]); ys.append(p[1])
    if not xs:
        return (-10.0, -10.0, 10.0, 10.0)
    return (min(xs), min(ys), max(xs), max(ys))


def group_max_z(tris):
    m = -math.inf
    for tri in tris:
        for p in tri:
            if p[2] > m:
                m = p[2]
    return m


def group_min_z(tris):
    m = math.inf
    for tri in tris:
        for p in tri:
            if p[2] < m:
                m = p[2]
    return m


def rgba_to_hex(col):
    r, g, b, a = col
    clamp = lambda v: max(0, min(255, int(round(v * 255))))
    return "#{:02x}{:02x}{:02x}".format(clamp(r), clamp(g), clamp(b)), a


# ── SVG writer ────────────────────────────────────────────────────────────────

def write_svg(path, triangles_by_key, colors_by_key,
              margin=1.0, stroke=None, stroke_width=0.0,
              flip_z=False, decimals=3, comment="",
              keep_origin=False, y_down=False, add_origin_marker=False):
    """Emit an SVG sized in millimetres.

    Each color group becomes one <path> made of many "M…L…L…Z" subpaths
    (one per triangle).  The triangles overlap their neighbours and the
    fill-rule=nonzero rule renders them as a single filled region.
    """
    xmin, ymin, xmax, ymax = compute_bbox(triangles_by_key)

    if keep_origin:
        # Expand the bbox to include the shape origin (0,0) so the SVG
        # viewBox covers everything plus the origin marker.
        xmin = min(xmin, 0.0)
        ymin = min(ymin, 0.0)
        xmax = max(xmax, 0.0)
        ymax = max(ymax, 0.0)

    w = (xmax - xmin) + 2 * margin
    h = (ymax - ymin) + 2 * margin
    if w <= 0 or h <= 0:
        raise RuntimeError("Empty geometry — nothing to draw.")

    # Painter's order: smaller max-Z first, larger max-Z last (on top).
    # For a bottom view we invert (smaller min-Z = closer to camera last).
    if flip_z:
        order_key = lambda kv: -group_min_z(kv[1])
    else:
        order_key = lambda kv:  group_max_z(kv[1])

    sorted_items = sorted(triangles_by_key.items(), key=order_key)

    fmt = f"%.{decimals}f"
    def f(x): return fmt % x

    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8" standalone="no"?>')
    if comment:
        safe = comment.replace("--", "- -")
        lines.append(f"<!-- {safe} -->")
    lines.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{f(w)}mm" height="{f(h)}mm" '
        f'viewBox="0 0 {f(w)} {f(h)}">'
    )

    # Map shape (x, y) → SVG (x, y) with Y flipped.  In the bottom view
    # we additionally mirror X so the resulting picture matches what a
    # camera below the part actually sees.
    tx = -xmin + margin
    if flip_z:
        # Bottom view: mirror X so the image matches what a camera below
        # the part would actually see, then flip Y as for the top view.
        lines.append(
            f' <g transform="translate({f(margin + (xmax - xmin))} '
            f'{f(margin + (ymax - ymin))}) scale(-1 -1) '
            f'translate({f(-xmin)} {f(-ymin)})">'
        )
    elif y_down:
        # Don't flip Y — the source data is already in a Y-down system
        # (e.g. supplied via --kicad-y).  Just translate to viewBox origin.
        lines.append(
            f' <g transform="translate({f(-xmin + margin)} {f(-ymin + margin)})">'
        )
    else:
        ty = ymax + margin
        lines.append(f' <g transform="translate({f(tx)} {f(ty)}) scale(1 -1)">')

    for key, tris in sorted_items:
        col = colors_by_key[key]
        hex_col, alpha = rgba_to_hex(col)

        attrs = [f'fill="{hex_col}"', 'fill-rule="nonzero"']
        if alpha < 0.999:
            attrs.append(f'fill-opacity="{alpha:.3f}"')
        if stroke is not None and stroke_width > 0:
            attrs += [f'stroke="{stroke}"',
                      f'stroke-width="{f(stroke_width)}"',
                      'stroke-linejoin="round"']
        else:
            attrs.append('stroke="none"')

        # Build a single d= string of M..L..L..Z triangles
        chunks = []
        for tri in tris:
            (x1, y1, _), (x2, y2, _), (x3, y3, _) = tri
            chunks.append(
                f"M{f(x1)} {f(y1)}L{f(x2)} {f(y2)}L{f(x3)} {f(y3)}Z"
            )
        d = "".join(chunks)
        lines.append(f'  <path {" ".join(attrs)} d="{d}"/>')

    if add_origin_marker:
        # 1×1 mm red rectangle centred on the shape origin (0, 0).
        # PCBdraw's element_position() returns the centre of this rect
        # via its bounding box, so we place it offset by -0.5 mm.
        # PCBdraw's element_position() uses the rect's top-left corner (x,y)
        # as the component's reference point.  The top-left corner must be at
        # shape (0,0) so it maps to the correct viewBox position after transforms.
        lines.append(
            f'  <rect id="origin" x="0" y="0" '
            f'width="1" height="1" fill="#ff0000" fill-opacity="0" '
            f'stroke="none"/>'
        )

    lines.append(' </g>')
    lines.append('</svg>')

    with open(path, "w") as fp:
        fp.write("\n".join(lines))
        fp.write("\n")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Convert a 3D STEP file to a top-down 2D SVG "
                    "(coloured, fillable, mm units).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("input", type=str, help="Input STEP file (.step / .stp)")
    p.add_argument("-o", "--output", type=str, default=None,
                   help="Output SVG file (default: <input>.svg)")

    p.add_argument("--bottom", action="store_true",
                   help="View from below (-Z) instead of from above (+Z)")
    p.add_argument("--all-faces", action="store_true",
                   help="Include downward-facing triangles too (slower, bigger SVG)")

    p.add_argument("--keep-origin", action="store_true",
                   help="Don't centre the shape — keep STEP (0,0) as the SVG origin")
    p.add_argument("--kicad-y", action="store_true",
                   help="Flip Y so the SVG is in KiCad's Y-down PCB coordinate "
                        "system (use with --keep-origin and --origin-marker for pcbdraw)")
    p.add_argument("--origin-marker", action="store_true",
                   help='Emit a <rect id="origin"> marker at (0,0) for pcbdraw')
    p.add_argument("--pcbdraw", action="store_true",
                   help="Shorthand for --keep-origin --kicad-y --origin-marker")

    p.add_argument("--deflection", type=float, default=0.05,
                   help="Mesh linear deflection in mm (default 0.05; smaller = finer)")
    p.add_argument("--angle", type=float, default=0.4,
                   help="Mesh angular deflection in radians (default 0.4)")

    p.add_argument("--margin", type=float, default=1.0,
                   help="Margin around drawing in mm (default 1.0)")
    p.add_argument("--decimals", type=int, default=3,
                   help="Decimal places in SVG coordinates (default 3)")

    p.add_argument("--stroke", type=str, default=None,
                   help="Optional stroke colour for outlines (e.g. '#222')")
    p.add_argument("--stroke-width", type=float, default=0.05,
                   help="Stroke width in mm (default 0.05)")

    p.add_argument("--default-color", type=str, default=None,
                   help="Hex colour used for faces with no XCAF colour "
                        "(default: light gray)")

    p.add_argument("--quiet", action="store_true", help="Suppress progress output")

    args = p.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        sys.exit(f"Error: input file not found: {in_path}")

    out_path = Path(args.output) if args.output else in_path.with_suffix(".svg")

    default_color = DEFAULT_COLOR
    if args.default_color:
        h = args.default_color.lstrip("#")
        if len(h) == 6:
            default_color = (int(h[0:2], 16) / 255.0,
                             int(h[2:4], 16) / 255.0,
                             int(h[4:6], 16) / 255.0,
                             1.0)
        else:
            sys.exit("--default-color must be #rrggbb")

    log = (lambda *a: None) if args.quiet else (lambda *a: print(*a))

    log(f"step2svg2  {in_path}  →  {out_path}")
    log("  loading STEP (with XCAF colours)…")
    shapes, color_tool, _doc = load_step_with_colors(in_path)  # keep doc alive
    log(f"  top-level shapes: {len(shapes)}")

    if args.pcbdraw:
        args.keep_origin = True
        args.kicad_y = True
        args.origin_marker = True

    log(f"  tessellating  (deflection={args.deflection} mm, angle={args.angle} rad)…")
    upward_only = not args.all_faces
    tris, cols = extract_triangles_by_color(
        shapes, color_tool, default_color,
        linear_def=args.deflection,
        angular_def=args.angle,
        upward_only=upward_only,
        flip_z=args.bottom,
        kicad_y=args.kicad_y,
    )

    if not tris:
        sys.exit("No triangles produced — is the model empty or all back-facing?")

    total = sum(len(v) for v in tris.values())
    log(f"  colour groups: {len(tris)}   total triangles: {total}")
    for key, ts in sorted(tris.items(), key=lambda kv: -len(kv[1])):
        hexcol, _ = rgba_to_hex(cols[key])
        log(f"    {hexcol}  {len(ts):>6} triangles")

    log("  writing SVG…")
    write_svg(
        out_path, tris, cols,
        margin=args.margin,
        stroke=args.stroke,
        stroke_width=args.stroke_width,
        flip_z=args.bottom,
        decimals=args.decimals,
        comment=f"Generated by step2svg2 from {in_path.name}",
        keep_origin=args.keep_origin,
        y_down=args.kicad_y,
        add_origin_marker=args.origin_marker,
    )

    log(f"  done.   wrote {out_path}  ({out_path.stat().st_size/1024:.1f} KB)")


if __name__ == "__main__":
    main()
