#!/usr/bin/env python3
"""
build_kicad3d_lib.py — Convert all KiCad-bundled 3D STEP models into
a PCBdraw component library.

Scans /usr/share/kicad/3dmodels/*.3dshapes/ for .step files and runs
step2svg.py --pcbdraw on each one, producing front and back SVGs in a
directory tree that PCBdraw can consume via --libs.

Output layout:

    <out_dir>/<LibName>/<ModelName>.svg        (front, top-down view)
    <out_dir>/<LibName>/<ModelName>.back.svg   (back, Y-mirrored)

LibName is the 3dshapes directory name without the ".3dshapes" suffix
(e.g. "Connector_PinHeader_2.54mm"), matching the KiCad footprint
library nicknames that PCBdraw resolves.

Usage:
    # Full build (will take a while — 7000+ models)
    python3 build_kicad3d_lib.py

    # Quick test with a single library
    python3 build_kicad3d_lib.py --filter "Connector_PinHeader_2.54mm"

    # Resume a partial build (skips existing files)
    python3 build_kicad3d_lib.py --resume

    # Parallel conversion (default: 4 workers)
    python3 build_kicad3d_lib.py --workers 8

    # Then point PCBdraw at the output:
    #   pcbdraw plot --libs KiCAD-base,kicad-3d board.kicad_pcb out.svg
    # (run from the parent of <out_dir>, or use --libs-path)
"""

import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

# ── Paths ────────────────────────────────────────────────────────────────────

SCRIPT_DIR = Path(__file__).resolve().parent
STEP2SVG   = SCRIPT_DIR / "step2svg.py"
KICAD_3D   = Path("/usr/share/kicad/3dmodels")
DEFAULT_OUT = SCRIPT_DIR / "kicad-3d"

# ── STEP discovery ───────────────────────────────────────────────────────────

def discover_step_files(kicad_3d: Path, filter_str: str | None = None) -> list[tuple[str, str, Path]]:
    """Return list of (lib_name, model_name, step_path) for all .step files.

    Lib name strips the ".3dshapes" suffix from the directory name.
    """
    results = []
    pattern = "*.step"
    for d in sorted(kicad_3d.iterdir()):
        if not d.is_dir() or not d.name.endswith(".3dshapes"):
            continue
        lib_name = d.name.removesuffix(".3dshapes")
        if filter_str and filter_str not in lib_name:
            continue
        for step in sorted(d.glob(pattern)):
            model_name = step.stem  # filename without .step
            results.append((lib_name, model_name, step))
    return results


# ── Conversion ────────────────────────────────────────────────────────────────

def make_back_variant(front_svg: Path) -> None:
    """Y-mirror a front SVG to produce a .back.svg.

    KiCad flips footprints to the back side by mirroring across the
    X-axis (negating Y).  We apply the same transform to the SVG.
    """
    import re
    text = front_svg.read_text()
    m = re.search(r'viewBox="([^"]+)"', text)
    if not m:
        return
    vb = m.group(1).split()
    vb_h = float(vb[3])
    new = re.sub(
        r'(<svg\b[^>]*>)',
        rf'\1<g transform="translate(0 {vb_h}) scale(1 -1)">',
        text, count=1,
    )
    new = new.replace("</svg>", "</g></svg>", 1)
    back_path = front_svg.with_name(front_svg.stem + ".back.svg")
    back_path.write_text(new)


def convert_one(lib_name: str, model_name: str, step_path: Path,
                out_dir: Path, deflection: float, angle: float,
                quiet: bool) -> tuple[str, str, bool]:
    """Convert a single STEP file to front+back SVGs.

    Returns (lib_name, model_name, success).
    """
    lib_out = out_dir / lib_name
    lib_out.mkdir(parents=True, exist_ok=True)
    out_svg = lib_out / f"{model_name}.svg"

    # Skip if already exists (for resume)
    if out_svg.exists():
        return (lib_name, model_name, True)

    cmd = [
        sys.executable, str(STEP2SVG),
        str(step_path),
        "-o", str(out_svg),
        "--pcbdraw",
        "--deflection", str(deflection),
        "--angle", str(angle),
        "--margin", "0.5",
    ]
    if quiet:
        cmd.append("--quiet")

    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        if not quiet:
            print(f"  TIMEOUT  {lib_name}/{model_name}", file=sys.stderr)
        return (lib_name, model_name, False)
    except Exception as e:
        if not quiet:
            print(f"  ERROR    {lib_name}/{model_name} — {e}", file=sys.stderr)
        return (lib_name, model_name, False)

    if res.returncode != 0:
        if not quiet:
            err = res.stderr.strip()[:200] if res.stderr else "unknown error"
            print(f"  FAIL     {lib_name}/{model_name} — {err}", file=sys.stderr)
        # Clean up partial output
        if out_svg.exists():
            out_svg.unlink()
        return (lib_name, model_name, False)

    # Generate back-side variant
    make_back_variant(out_svg)
    return (lib_name, model_name, True)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT,
                   help=f"Output directory (default: {DEFAULT_OUT})")
    p.add_argument("--filter", type=str, default=None,
                   help="Only process libraries whose name contains this string")
    p.add_argument("--workers", type=int, default=4,
                   help="Number of parallel conversions (default: 4)")
    p.add_argument("--deflection", type=float, default=0.05,
                   help="Mesh linear deflection in mm (default: 0.05)")
    p.add_argument("--angle", type=float, default=0.4,
                   help="Mesh angular deflection in rad (default: 0.4)")
    p.add_argument("--resume", action="store_true",
                   help="Skip files that already exist")
    p.add_argument("--quiet", action="store_true",
                   help="Only print errors and summary")
    p.add_argument("--dry-run", action="store_true",
                   help="List files that would be converted, then exit")
    args = p.parse_args()

    if not KICAD_3D.exists():
        sys.exit(f"KiCad 3D models directory not found: {KICAD_3D}")
    if not STEP2SVG.exists():
        sys.exit(f"step2svg.py not found at: {STEP2SVG}")

    models = discover_step_files(KICAD_3D, args.filter)
    if not models:
        print("No STEP files found.")

        if args.filter:
            print(f"  (filter '{args.filter}' matched nothing)")
        print(f"  searched: {KICAD_3D}")
        return

    # Optionally filter already-converted files for resume
    if args.resume:
        remaining = []
        for lib, name, path in models:
            out_svg = args.out_dir / lib / f"{name}.svg"
            if not out_svg.exists():
                remaining.append((lib, name, path))
        skipped = len(models) - len(remaining)
        models = remaining
        if skipped:
            print(f"Skipping {skipped} already-converted files")

    print(f"Converting {len(models)} STEP files from {KICAD_3D}")
    print(f"Output:  {args.out_dir}")
    print(f"Workers: {args.workers}")
    if args.dry_run:
        print()
        for lib, name, _ in models[:20]:
            print(f"  {lib}/{name}")
        if len(models) > 20:
            print(f"  ... and {len(models) - 20} more")
        return

    args.out_dir.mkdir(parents=True, exist_ok=True)
    start = time.time()
    ok = fail = 0

    if args.workers <= 1:
        # Sequential
        for lib, name, step_path in models:
            _, _, success = convert_one(
                lib, name, step_path, args.out_dir,
                args.deflection, args.angle, args.quiet,
            )
            if success:
                ok += 1
            else:
                fail += 1
            if not args.quiet and (ok + fail) % 50 == 0:
                elapsed = time.time() - start
                print(f"  [{ok+fail}/{len(models)}]  {ok} ok, {fail} failed  "
                      f"({elapsed:.0f}s)")
    else:
        # Parallel
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {
                pool.submit(
                    convert_one, lib, name, step_path, args.out_dir,
                    args.deflection, args.angle, args.quiet,
                ): (lib, name)
                for lib, name, step_path in models
            }
            done = 0
            for future in as_completed(futures):
                done += 1
                _, _, success = future.result()
                if success:
                    ok += 1
                else:
                    fail += 1
                if not args.quiet and done % 100 == 0:
                    elapsed = time.time() - start
                    rate = done / elapsed if elapsed > 0 else 0
                    eta = (len(models) - done) / rate if rate > 0 else 0
                    print(f"  [{done}/{len(models)}]  {ok} ok, {fail} failed  "
                          f"({rate:.1f}/s, ETA {eta:.0f}s)")

    elapsed = time.time() - start
    print()
    print(f"Done in {elapsed:.0f}s — {ok} succeeded, {fail} failed")
    print(f"Library at: {args.out_dir}")
    print()
    print("Usage:")
    print(f"  cd {args.out_dir.parent}")
    print(f"  pcbdraw plot --libs KiCAD-base,{args.out_dir.name} "
          f"board.kicad_pcb out.svg")


if __name__ == "__main__":
    main()
