#!/usr/bin/env python3
"""
step2svg — Convert KiCad 3D STEP files to 2D SVGs.

Uses OCP (Open Cascade Python) to load STEP files and perform
Hidden Line Removal (HLR) projection for 2D SVG output.

Output SVGs are suitable for:
  • pcbdraw component outlines / footprints
  • Fritzing part SVG creation (icon view, breadboard view)
  • General 2D technical illustration of electronic components

Requires:
    pip install ocp-vscode

Usage:
    # Top-down view (default, looking along -Z axis)
    python step2svg.py input.step -o output.svg

    # Side/front views
    python step2svg.py input.step -o front.svg --view front

    # Isometric view
    python step2svg.py input.step -o iso.svg --view isometric

    # Only outer outline (using HLR outline edges, no internal detail)
    python step2svg.py input.step -o outline.svg --outline

    # Minimal view: only visible outline + sharp edges (no smooth interior)
    python step2svg.py input.step -o minimal.svg --minimal

    # Adjust scale and line appearance
    python step2svg.py input.step -o output.svg --scale 2.0 --line-width 1.0 --color "#333333"
"""

import argparse
import math
import sys
from pathlib import Path
from typing import List, Tuple, Set

# ── OCP imports ──────────────────────────────────────────────────────────────

try:
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopoDS import TopoDS
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Trsf, gp_Vec
    from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
    from OCP.HLRAlgo import HLRAlgo_Projector
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepBndLib import BRepBndLib
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.GCPnts import GCPnts_UniformAbscissa
    from OCP.GeomAbs import GeomAbs_CurveType

    OCP_AVAILABLE = True
except ImportError as e:
    OCP_AVAILABLE = False
    _ocp_import_error = e


# ── SVG helpers ───────────────────────────────────────────────────────────────

SVG_HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg xmlns="http://www.w3.org/2000/svg"
     xmlns:xlink="http://www.w3.org/1999/xlink"
     version="1.1"
     width="{width:.0f}px" height="{height:.0f}px"
     viewBox="{vx:.1f} {vy:.1f} {vw:.1f} {vh:.1f}">
  <g transform="translate({tx:.1f}, {ty:.1f}) scale(1, -1)">
    <g stroke="{color}" stroke-width="{lw}" fill="none" stroke-linecap="round" stroke-linejoin="round">
"""

SVG_FOOTER = """    </g>
  </g>
</svg>
"""


def write_svg(
    path: str,
    polylines: List[List[Tuple[float, float]]],
    bbox: Tuple[float, float, float, float],
    margin: float = 10,
    scale: float = 1.0,
    line_width: float = 0.5,
    color: str = "#000000",
) -> None:
    """Write polylines as SVG. Uses Y-up coordinates (via scale(1,-1))."""
    xmin, ymin, xmax, ymax = bbox

    # Apply scale
    sx, sy, sw, sh = xmin * scale, ymin * scale, xmax * scale, ymax * scale

    w = sw - sx
    h = sh - sy

    vx = sx - margin
    vy = sy - margin
    vw = w + 2 * margin
    vh = h + 2 * margin

    tx = -sx + margin
    ty = -sy + margin

    lines = []
    for poly in polylines:
        if len(poly) < 2:
            continue
        pts = " ".join(f"{x * scale:.4f},{y * scale:.4f}" for x, y in poly)
        lines.append(f'      <polyline points="{pts}"/>')

    svg_content = (
        SVG_HEADER.format(
            width=vw, height=vh, vx=vx, vy=vy, vw=vw, vh=vh,
            tx=tx, ty=ty, color=color, lw=line_width,
        )
        + "\n".join(lines)
        + "\n"
        + SVG_FOOTER
    )

    with open(path, "w") as f:
        f.write(svg_content)

    n_polylines = len(polylines)
    n_points = sum(len(p) for p in polylines)
    print(f"  Wrote {n_polylines} polylines ({n_points} points) to {path}")


# ── STEP loading ──────────────────────────────────────────────────────────────


def load_step(path: str) -> TopoDS_Shape:
    """Load a STEP file and return the shape."""
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != 1:
        raise RuntimeError(f"Failed to read STEP file: {path} (status={status})")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError("No shape found in STEP file")
    return shape


def get_bounding_box(shape: TopoDS_Shape) -> Tuple[float, float, float, float, float, float]:
    """Return (xmin, ymin, zmin, xmax, ymax, zmax)."""
    bbox = Bnd_Box()
    BRepBndLib.Add_s(shape, bbox)
    return bbox.Get()


def center_shape(shape: TopoDS_Shape) -> Tuple[TopoDS_Shape, Tuple[float, float, float]]:
    """Translate shape so its bounding-box center is at the origin."""
    xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(shape)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    cz = (zmin + zmax) / 2.0

    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(-cx, -cy, -cz))
    transform = BRepBuilderAPI_Transform(shape, trsf)
    return transform.Shape(), (cx, cy, cz)


# ── Edge discretization & deduplication ───────────────────────────────────────


def discretize_edge(edge) -> List[Tuple[float, float, float]]:
    """Sample 3D points along an edge using equal arc-length spacing."""
    try:
        adapt = BRepAdaptor_Curve(edge)
    except Exception:
        return []

    fp = adapt.FirstParameter()
    lp = adapt.LastParameter()

    if abs(lp - fp) < 1e-8:
        p = gp_Pnt()
        adapt.D0(fp, p)
        return [(p.X(), p.Y(), p.Z())]

    # Determine sample count based on curve type
    curve_type = adapt.GetType()
    if curve_type == GeomAbs_CurveType.GeomAbs_Line:
        num_pts = 2
    elif curve_type == GeomAbs_CurveType.GeomAbs_Circle:
        angle = abs(lp - fp)
        num_pts = max(16, int(angle * 6))
    elif curve_type == GeomAbs_CurveType.GeomAbs_Ellipse:
        angle = abs(lp - fp)
        num_pts = max(20, int(angle * 8))
    else:
        # B-spline, Bezier, etc.
        dx, dy, dz = _estimate_curve_length(adapt)
        num_pts = max(4, int(dx * 2))
        num_pts = min(num_pts, 200)

    num_pts = max(2, min(num_pts, 200))

    # Uniform abscissa sampling (equal arc-length)
    try:
        sampler = GCPnts_UniformAbscissa(adapt, num_pts, fp, lp)
    except Exception:
        sampler = GCPnts_UniformAbscissa()
        sampler.Initialize(adapt, num_pts, fp, lp)

    if not sampler.IsDone():
        # Fallback: uniform parameter sampling
        return _uniform_param_sampling(adapt, fp, lp, num_pts)

    n = sampler.NbPoints()
    if n < 2:
        return _uniform_param_sampling(adapt, fp, lp, 2)

    points = []
    p = gp_Pnt()
    for i in range(1, n + 1):
        try:
            t = sampler.Parameter(i)
        except Exception:
            t = fp + (lp - fp) * (i - 1) / (n - 1)
        adapt.D0(t, p)
        points.append((p.X(), p.Y(), p.Z()))

    return points


def _estimate_curve_length(adapt) -> Tuple[float, float, float]:
    """Rough estimate of curve extent by sampling a few points."""
    p = gp_Pnt()
    adapt.D0(adapt.FirstParameter(), p)
    x0, y0, z0 = p.X(), p.Y(), p.Z()
    adapt.D0(adapt.LastParameter(), p)
    x1, y1, z1 = p.X(), p.Y(), p.Z()
    return (abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def _uniform_param_sampling(adapt, fp, lp, n) -> List[Tuple[float, float, float]]:
    """Sample curve uniformly in parameter space (fallback)."""
    points = []
    p = gp_Pnt()
    for i in range(n):
        t = fp + (lp - fp) * i / (n - 1)
        adapt.D0(t, p)
        points.append((p.X(), p.Y(), p.Z()))
    return points


def edge_fingerprint(pts: List[Tuple[float, float, float]]) -> Tuple:
    """Create a fingerprint for deduplication based on start/end positions.

    Two edges are considered duplicates if they have the same start and end
    points (within tolerance) regardless of direction.
    """
    if len(pts) < 2:
        return None

    x0, y0, z0 = pts[0]
    x1, y1, z1 = pts[-1]

    # Round to 0.01 (approx 0.1 mm at standard Fritzing scale)
    tol = 2  # rounding to 2 decimal places
    k0 = (round(x0, tol), round(y0, tol), round(z0, tol))
    k1 = (round(x1, tol), round(y1, tol), round(z1, tol))

    # Normalize: always order the endpoints so (A,B) == (B,A)
    if k0 < k1:
        return (k0, k1, len(pts))
    else:
        return (k1, k0, len(pts))


# ── View direction definitions ────────────────────────────────────────────────

VIEW_DIRECTIONS = {
    "top":        gp_Dir(0, 0, 1),
    "bottom":     gp_Dir(0, 0, -1),
    "front":      gp_Dir(0, -1, 0),
    "back":       gp_Dir(0, 1, 0),
    "left":       gp_Dir(-1, 0, 0),
    "right":      gp_Dir(1, 0, 0),
    "isometric":  gp_Dir(1, 1, 1),
    "isometric2": gp_Dir(-1, -1, 1),
}


# ── HLR-based projection ──────────────────────────────────────────────────────


def hlr_project(
    shape: TopoDS_Shape,
    view: str = "top",
    mode: str = "detailed",
) -> List[List[Tuple[float, float]]]:
    """Project a 3D shape to 2D using HLR (Hidden Line Removal).

    Modes:
      "detailed"  : VCompound (all visible edges) + Rg1LineVCompound (sharp edges)
      "minimal"   : VCompound (all visible edges) only
      "outline"   : OutLineVCompound3d (projected outline edges) only
    """
    if view not in VIEW_DIRECTIONS:
        raise ValueError(f"Unknown view '{view}'. Choose from: {list(VIEW_DIRECTIONS.keys())}")

    direction = VIEW_DIRECTIONS[view]

    algo = HLRBRep_Algo()
    algo.Add(shape)
    projector = HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), direction))
    algo.Projector(projector)
    algo.Update()

    hlr_shapes = HLRBRep_HLRToShape(algo)

    sources = []
    if mode == "outline":
        # Projected outline edges
        oc = hlr_shapes.OutLineVCompound3d()
        if not oc.IsNull():
            sources.append(oc)
        # Also get the 2D VCompound outline
        vc = hlr_shapes.VCompound()
        if not vc.IsNull():
            sources.append(vc)
    elif mode == "minimal":
        # Visible edges only (no Rg1)
        vc = hlr_shapes.VCompound()
        if not vc.IsNull():
            sources.append(vc)
    else:  # detailed
        vc = hlr_shapes.VCompound()
        if not vc.IsNull():
            sources.append(vc)
        rg1 = hlr_shapes.Rg1LineVCompound()
        if not rg1.IsNull():
            sources.append(rg1)

    # Discretize all edges, deduplicate
    seen_fingerprints: Set[Tuple] = set()
    polylines_2d: List[List[Tuple[float, float]]] = []

    for compound in sources:
        exp = TopExp_Explorer(compound, TopAbs_EDGE)
        while exp.More():
            edge = TopoDS.Edge(exp.Current())
            pts_3d = discretize_edge(edge)
            if len(pts_3d) < 2:
                exp.Next()
                continue

            fp = edge_fingerprint(pts_3d)
            if fp is not None and fp in seen_fingerprints:
                exp.Next()
                continue
            if fp is not None:
                seen_fingerprints.add(fp)

            # Drop Z (HLR edges are on the projection plane, Z≈0)
            pts_2d = [(x, y) for x, y, z in pts_3d]
            polylines_2d.append(pts_2d)
            exp.Next()

    return polylines_2d


# ── 2D utilities ──────────────────────────────────────────────────────────────


def compute_2d_bbox(
    polylines: List[List[Tuple[float, float]]],
) -> Tuple[float, float, float, float]:
    """Compute (xmin, ymin, xmax, ymax) from 2D polylines."""
    if not polylines:
        return (-10, -10, 10, 10)

    all_x = [x for poly in polylines for x, y in poly]
    all_y = [y for poly in polylines for x, y in poly]

    if not all_x:
        return (-10, -10, 10, 10)

    return (min(all_x), min(all_y), max(all_x), max(all_y))


def simplify_polyline(
    pts: List[Tuple[float, float]], min_dist: float = 0.01
) -> List[Tuple[float, float]]:
    """Remove near-collinear points using perpendicular-distance check."""
    if len(pts) <= 2:
        return pts

    result = [pts[0]]
    for i in range(1, len(pts) - 1):
        p0 = result[-1]
        p1 = pts[i]
        p2 = pts[i + 1]

        dx = p2[0] - p0[0]
        dy = p2[1] - p0[1]
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 1e-10:
            continue

        # Projected distance of p1 from line p0-p2
        t = ((p1[0] - p0[0]) * dx + (p1[1] - p0[1]) * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        proj_x = p0[0] + t * dx
        proj_y = p0[1] + t * dy
        dist = math.hypot(p1[0] - proj_x, p1[1] - proj_y)

        if dist > min_dist:
            result.append(p1)

    result.append(pts[-1])
    return result


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Convert 3D STEP files to 2D SVGs using Open Cascade (OCP)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("input", type=str, help="Input STEP file (.step, .stp)")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output SVG file (default: <input_stem>_<view>.svg)")
    parser.add_argument("--view", choices=list(VIEW_DIRECTIONS.keys()),
                        default="top",
                        help="View direction (default: top)")

    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--outline", action="store_true",
                            help="Output only projected outline edges (no internal detail)")
    mode_group.add_argument("--minimal", action="store_true",
                            help="Output visible edges only (no internal sharp edges)")

    parser.add_argument("--no-center", action="store_true",
                        help="Don't center the model")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Output scale factor (default: 1.0)")
    parser.add_argument("--margin", type=float, default=10,
                        help="Margin in SVG units (default: 10)")
    parser.add_argument("--line-width", type=float, default=0.5,
                        help="Stroke width in SVG units (default: 0.5)")
    parser.add_argument("--color", type=str, default="#000000",
                        help="Stroke color (default: #000000)")
    parser.add_argument("--tolerance", type=float, default=0.01,
                        help="Polyline simplification tolerance (default: 0.01)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print detailed progress")

    args = parser.parse_args()

    if not OCP_AVAILABLE:
        print("Error: OCP (Open Cascade Python) is not installed.", file=sys.stderr)
        print("Install with: pip install ocp-vscode", file=sys.stderr)
        print(f"Import error: {_ocp_import_error}", file=sys.stderr)
        sys.exit(1)

    input_path = args.input
    if not Path(input_path).exists():
        print(f"Error: input file not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    # Determine mode string
    if args.outline:
        mode = "outline"
    elif args.minimal:
        mode = "minimal"
    else:
        mode = "detailed"

    # Default output name
    if args.output is None:
        stem = Path(input_path).stem
        args.output = f"{stem}_{args.view}_{mode}.svg"

    def log(msg):
        if args.verbose:
            print(msg)

    print(f"STEP → SVG  |  {input_path}")
    print(f"  view: {args.view}  mode: {mode}")

    print("Loading STEP file...")
    shape = load_step(input_path)

    if not args.no_center:
        print("  Centering shape...")
        try:
            shape, offset = center_shape(shape)
        except Exception as e:
            print(f"  Warning: centering failed ({e})")

    try:
        bbox_3d = get_bounding_box(shape)
        print(f"  BBox: x=[{bbox_3d[0]:.2f}, {bbox_3d[3]:.2f}] "
              f"y=[{bbox_3d[1]:.2f}, {bbox_3d[4]:.2f}] "
              f"z=[{bbox_3d[2]:.2f}, {bbox_3d[5]:.2f}]")
    except Exception:
        pass

    try:
        exp = TopExp_Explorer(shape, TopAbs_EDGE)
        total_edges = sum(1 for _ in exp)
        print(f"  Edges in shape: {total_edges}")
    except Exception:
        pass

    print(f"Projecting ({mode})...")
    try:
        polylines_2d = hlr_project(shape, view=args.view, mode=mode)
    except Exception as e:
        print(f"  HLR failed: {e}", file=sys.stderr)
        print("  Falling back to direct edge projection (no hidden-line removal)")
        polylines_2d = _project_all_edges(shape)

    if not polylines_2d:
        print("Warning: no visible edges found")
        bbox_2d = (-10, -10, 10, 10)
        write_svg(args.output, [], bbox_2d, margin=args.margin,
                  scale=args.scale, line_width=args.line_width, color=args.color)
        return

    log(f"  Raw polylines: {len(polylines_2d)}")

    # Simplify
    before = sum(len(p) for p in polylines_2d)
    polylines_2d = [simplify_polyline(p, min_dist=args.tolerance) for p in polylines_2d]
    after = sum(len(p) for p in polylines_2d)
    if before > 0:
        reduction = 100 * (before - after) / before
        log(f"  Points: {before} → {after} ({reduction:.0f}% reduction)")

    # Compute 2D bbox
    bbox_2d = compute_2d_bbox(polylines_2d)
    print(f"  2D BBox: x=[{bbox_2d[0]:.2f}, {bbox_2d[2]:.2f}] "
          f"y=[{bbox_2d[1]:.2f}, {bbox_2d[3]:.2f}]")

    # Write SVG
    write_svg(
        args.output,
        polylines_2d,
        bbox_2d,
        margin=args.margin,
        scale=args.scale,
        line_width=args.line_width,
        color=args.color,
    )

    print("Done.")


def _project_all_edges(shape: TopoDS_Shape) -> List[List[Tuple[float, float]]]:
    """Fallback: project all edges by flattening Z to 0."""
    polylines = []
    seen: Set[Tuple] = set()
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge(exp.Current())
        pts_3d = discretize_edge(edge)
        if len(pts_3d) >= 2:
            fp = edge_fingerprint(pts_3d)
            if fp is not None and fp in seen:
                exp.Next()
                continue
            if fp is not None:
                seen.add(fp)
            pts_2d = [(x, y) for x, y, z in pts_3d]
            polylines.append(pts_2d)
        exp.Next()
    return polylines


if __name__ == "__main__":
    main()
