#!/usr/bin/env python3
"""
kicad2fritzing.py — Convert a KiCad PCB into a Fritzing part (FZP + SVGs)

Takes a KiCad .kicad_pcb file and produces a Fritzing-compatible part
using PcbDraw for the board rendering.  PcbDraw generates a beautiful
board SVG with copper traces, silkscreen, and 3D-rendered component
bodies.  The script then wraps it into Fritzing's layer format, adds
connector pin markers for every connector pad, generates schematic
and icon views, and packages everything into a .fzpz archive.

Usage:
    # Basic (requires pcbdraw + pre-built step2svg library)
    python3 kicad2fritzing.py p4hil.kicad_pcb ./fritzing_part/

    # Convert with a custom library
    python3 kicad2fritzing.py myboard.kicad_pcb ./output/ \\
        --libs KiCAD-base,kicad-3d

    # Rebuild the 3D library first, then convert
    python3 kicad2fritzing.py myboard.kicad_pcb ./output/ --rebuild-lib

    # Pick a style and pass extra options to pcbdraw
    python3 kicad2fritzing.py myboard.kicad_pcb ./output/ \\
        --style oshpark-purple -- --no-tag

Pipeline:
    1. Parse the KiCad PCB to extract footprints, connectors, board outline.
    2. Run `pcbdraw plot --side front` to produce the board SVG.
    3. Post-process the raw SVG:
       - Wrap content in Fritzing layer groups (copper1, copper0, silkscreen).
       - Add connector pin markers (hidden rects with id="connector<N>pin").
    4. Generate schematic view (simplified pin diagram) and icon SVG.
    5. Write the FZP part descriptor with connector definitions.
    6. Package into a .fzpz archive ready for Fritzing import.

Requirements:
    - pcbdraw (pip install pcbdraw, needs KiCad's pcbnew module)
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path


# ── Project paths (relative to step2svg repo) ─────────────────────────────

SCRIPT_DIR     = Path(__file__).resolve().parent
DEFAULT_PCBDRAW = "/tmp/pcbdraw-venv/bin/pcbdraw"
STEP2SVG       = SCRIPT_DIR / "step2svg.py"
BUILD_LIB      = SCRIPT_DIR / "build_kicad3d_lib.py"
KICAD3D_LIB    = SCRIPT_DIR / "kicad-3d"           # PCBdraw library dir

# Fritzing coordinate system: 100 units = 1 inch = 25.4 mm
FRITZING_SCALE  = 100.0 / 25.4


# ══════════════════════════════════════════════════════════════════════════
#  KiCad PCB S-expression parser
# ══════════════════════════════════════════════════════════════════════════

def tokenize_sexp(text):
    tokens = []
    i = 0
    while i < len(text):
        c = text[i]
        if c in '()':
            tokens.append(c)
            i += 1
        elif c.isspace():
            i += 1
        elif c == '"':
            j = i + 1
            while j < len(text):
                if text[j] == '"' and text[j-1] != '\\':
                    break
                j += 1
            tokens.append(text[i+1:j])
            i = j + 1
        else:
            j = i
            while j < len(text) and not text[j].isspace() and text[j] not in '()':
                j += 1
            tokens.append(text[i:j])
            i = j
    return tokens


def parse_sexp(tokens, idx=0):
    result = []
    while idx < len(tokens):
        t = tokens[idx]
        if t == '(':
            sub, idx = parse_sexp(tokens, idx + 1)
            result.append(sub)
        elif t == ')':
            return result, idx + 1
        else:
            result.append(t)
            idx += 1
    return result, idx


def load_kicad_pcb(path):
    with open(path, 'r') as f:
        text = f.read()
    tokens = tokenize_sexp(text)
    parsed, _ = parse_sexp(tokens, 0)
    return parsed


def find_first(node, tag):
    for child in node:
        if isinstance(child, list) and len(child) > 0 and child[0] == tag:
            return child
    return None


def find_all(node, tag):
    return [child for child in node if isinstance(child, list) and len(child) > 0 and child[0] == tag]


def get_float(node, idx, default=0.0):
    if idx < len(node):
        try:
            return float(node[idx])
        except (ValueError, TypeError):
            pass
    return default


def get_str(node, idx, default=''):
    if idx < len(node):
        return str(node[idx])
    return default


def get_xyz(node):
    x = get_float(node, 1, 0.0)
    y = get_float(node, 2, 0.0)
    rot = get_float(node, 3, 0.0)
    return x, y, rot


class PCBParser:
    """Parses a KiCad PCB S-expression tree into footprint/connector data."""

    def __init__(self, parsed):
        self.parsed = parsed
        self.board = parsed[0] if parsed else []
        self.footprints = []
        self.board_w = 0
        self.board_h = 0
        self.board_x = 0
        self.board_y = 0
        self._parse()

    def _parse(self):
        for child in self.board:
            if not isinstance(child, list) or len(child) == 0:
                continue
            tag = child[0]

            if tag in ('gr_rect', 'gr_line', 'gr_poly'):
                self._parse_board_outline(child)

            elif tag == 'footprint':
                self._parse_footprint(child)

    def _parse_board_outline(self, item):
        tag = item[0]
        layer_node = find_first(item, 'layer')
        layer = get_str(layer_node, 1, '') if layer_node else ''
        if layer != 'Edge.Cuts':
            return

        if tag == 'gr_rect':
            start = find_first(item, 'start')
            end = find_first(item, 'end')
            if start and end:
                x1, y1 = get_float(start, 1), get_float(start, 2)
                x2, y2 = get_float(end, 1), get_float(end, 2)
                self.board_x = min(x1, x2)
                self.board_y = min(y1, y2)
                self.board_w = abs(x2 - x1)
                self.board_h = abs(y2 - y1)

        elif tag == 'gr_line':
            start = find_first(item, 'start')
            end = find_first(item, 'end')
            if start and end:
                x1, y1 = get_float(start, 1), get_float(start, 2)
                x2, y2 = get_float(end, 1), get_float(end, 2)
                xs, ys = min(x1, x2), min(y1, y2)
                xe, ye = max(x1, x2), max(y1, y2)
                if self.board_w == 0:
                    self.board_x, self.board_y = xs, ys
                    self.board_w, self.board_h = xe - xs, ye - ys
                else:
                    xmin = min(self.board_x, xs)
                    ymin = min(self.board_y, ys)
                    xmax = max(self.board_x + self.board_w, xe)
                    ymax = max(self.board_y + self.board_h, ye)
                    self.board_x, self.board_y = xmin, ymin
                    self.board_w, self.board_h = xmax - xmin, ymax - ymin

    def _parse_footprint(self, fp):
        name = get_str(fp, 1, '')
        layer_node = find_first(fp, 'layer')
        layer = get_str(layer_node, 1, '')
        at_node = find_first(fp, 'at')
        x, y, rot = get_xyz(at_node) if at_node else (0, 0, 0)

        ref_node = None
        val_node = None
        for prop in find_all(fp, 'property'):
            prop_name = get_str(prop, 1, '')
            if prop_name == 'Reference':
                ref_node = prop
            elif prop_name == 'Value':
                val_node = prop

        ref = get_str(ref_node, 2, '?') if ref_node else '?'
        val = get_str(val_node, 2, '?') if val_node else '?'

        pads = []
        for pad_node in find_all(fp, 'pad'):
            pad_name = get_str(pad_node, 1, '')
            pad_type = get_str(pad_node, 2, '')
            pad_shape = get_str(pad_node, 3, '')
            pad_at = find_first(pad_node, 'at')
            pad_size = find_first(pad_node, 'size')

            if pad_at:
                px, py, prot = get_xyz(pad_at)
            else:
                px, py, prot = 0, 0, 0

            if pad_size:
                sx = get_float(pad_size, 1, 1)
                sy = get_float(pad_size, 2, 1)
            else:
                sx, sy = 1, 1

            drill_node = find_first(pad_node, 'drill')
            drill = get_float(drill_node, 1, 0) if drill_node else 0

            layers_node = find_first(pad_node, 'layers')
            pad_layers = []
            if layers_node:
                pad_layers = [get_str(layers_node, i, '') for i in range(1, len(layers_node))]

            # Extract net name (e.g. ['net', 'GND'] or ['net', '42', 'VCC'])
            net_node = find_first(pad_node, 'net')
            net = ''
            if net_node:
                # KiCad 8+ uses (net <num> <name>), older uses (net <name>)
                if len(net_node) >= 3:
                    net = get_str(net_node, 2, '')
                else:
                    net = get_str(net_node, 1, '')

            pads.append({
                'name': pad_name,
                'type': pad_type,
                'shape': pad_shape,
                'x': px, 'y': py, 'rot': prot,
                'sx': sx, 'sy': sy,
                'drill': drill,
                'layers': pad_layers,
                'net': net,
            })

        self.footprints.append({
            'name': name,
            'ref': ref,
            'value': val,
            'layer': layer,
            'x': x, 'y': y, 'rot': rot,
            'pads': pads,
        })


def is_connector(fp):
    """Check if a footprint is a connector that should have exposed pins."""
    name = fp['name']
    ref = fp['ref']
    if ref.startswith('H') and 'MountingHole' in name:
        return False
    connector_keywords = [
        'PinHeader', 'Connector', 'BarrelJack', 'USB', 'HDMI', 'DSUB',
        'TerminalBlock', 'ScrewTerminal', 'AudioJack', 'RJ',
        'Button', 'Switch', 'Jumper', 'TestPoint',
    ]
    for kw in connector_keywords:
        if kw in name:
            return True
    return False


# ══════════════════════════════════════════════════════════════════════════
#  SVG helpers
# ══════════════════════════════════════════════════════════════════════════

def svg_header(width_in, height_in, viewbox=None):
    """width/height in inches."""
    if viewbox is None:
        viewbox = f"0 0 {width_in * 100:.4f} {height_in * 100:.4f}"
    return (f'<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<svg xmlns="http://www.w3.org/2000/svg"\n'
            f'     xmlns:xlink="http://www.w3.org/1999/xlink"\n'
            f'     width="{width_in:.4f}in" height="{height_in:.4f}in"\n'
            f'     viewBox="{viewbox}">\n')


def svg_footer():
    return '</svg>\n'


def make_svg_rect(x, y, w, h, fill='none', stroke='#000', sw=0.1, rx=0):
    attrs = f'x="{x:.4f}" y="{y:.4f}" width="{w:.4f}" height="{h:.4f}"'
    if rx:
        attrs += f' rx="{rx:.4f}"'
    attrs += f' fill="{fill}" stroke="{stroke}" stroke-width="{sw:.4f}"'
    return f'  <rect {attrs}/>\n'


def make_svg_line(x1, y1, x2, y2, stroke='#000', sw=0.1):
    return (f'  <line x1="{x1:.4f}" y1="{y1:.4f}" '
            f'x2="{x2:.4f}" y2="{y2:.4f}" '
            f'stroke="{stroke}" stroke-width="{sw:.4f}"/>\n')


def make_svg_text(x, y, text, size=1, fill='#000', anchor='start'):
    return (f'  <text x="{x:.4f}" y="{y:.4f}" font-size="{size:.4f}" '
            f'fill="{fill}" text-anchor="{anchor}" '
            f'font-family="sans-serif">{text}</text>\n')


# ══════════════════════════════════════════════════════════════════════════
#  Post-process PcbDraw SVG → Fritzing part SVGs
# ══════════════════════════════════════════════════════════════════════════

def wrap_pcbdraw_svg_for_fritzing(pcbdraw_svg_path: Path, output_path: Path,
                                   pcb: PCBParser) -> int:
    """Post-process a raw PcbDraw SVG into a Fritzing PCB-layer SVG.

    Wraps the content in Fritzing layer groups (copper1, copper0,
    silkscreen) and adds connector pin markers at each connector pad.

    Returns the number of connector pins added.
    """
    with open(pcbdraw_svg_path, 'r') as f:
        svg = f.read()

    # Extract the inner content between <svg>…</svg>
    m = re.search(r'<svg[^>]*>(.*)</svg>', svg, re.DOTALL)
    inner = m.group(1) if m else svg

    # Parse viewBox
    vb_match = re.search(r'viewBox="([^"]*)"', svg)
    if vb_match:
        parts = vb_match.group(1).strip().split()
        if len(parts) == 4:
            vb_x, vb_y, vb_w, vb_h = map(float, parts)
        else:
            vb_x = vb_y = 0.0
            vb_w = vb_h = 100.0
    else:
        vb_x = vb_y = 0.0
        vb_w = vb_h = 100.0

    scale = FRITZING_SCALE
    margin_mm = 5
    # Compute viewBox origin in Fritzing units (includes margin)
    off_x = (vb_x - margin_mm) * scale
    off_y = (vb_y - margin_mm) * scale
    svg_w = vb_w * scale + 2 * margin_mm * scale
    svg_h = vb_h * scale + 2 * margin_mm * scale

    # Normalized viewBox starting at (0, 0) — Fritzing units, 100 = 1 inch
    result = svg_header(svg_w / 100, svg_h / 100,
                        viewbox=f"0 0 {svg_w:.4f} {svg_h:.4f}")

    # Board fill (in Fritzing units, shifted to viewBox origin)
    board_x_f = pcb.board_x * scale - off_x
    board_y_f = pcb.board_y * scale - off_y
    board_w_f = pcb.board_w * scale
    board_h_f = pcb.board_h * scale
    result += f'  <g id="boardFill">\n'
    result += f'    <rect x="{board_x_f:.4f}" y="{board_y_f:.4f}" '
    result += f'width="{board_w_f:.4f}" height="{board_h_f:.4f}" '
    result += f'fill="#2b5f82" stroke="none"/>\n'
    result += f'  </g>\n'

    # Copper1 — PcbDraw content, transform from mm to Fritzing units
    result += f'  <g id="copper1" transform="translate({-off_x:.4f}, {-off_y:.4f}) scale({scale:.6f})">\n'
    result += inner
    result += f'  </g>\n'

    # Copper0, Silkscreen — empty layers at same scale
    result += f'  <g id="copper0" transform="translate({-off_x:.4f}, {-off_y:.4f}) scale({scale:.6f})"/>\n'
    result += f'  <g id="silkscreen" transform="translate({-off_x:.4f}, {-off_y:.4f}) scale({scale:.6f})"/>\n'

    # Connector pin markers (in Fritzing units, shifted to viewBox origin)
    conn_idx = 0
    result += f'  <g id="connectors">\n'
    for fp in pcb.footprints:
        if not is_connector(fp):
            continue
        for pad in fp['pads']:
            angle_rad = math.radians(-fp.get('rot', 0))
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            px_mm = fp['x'] + pad['x'] * cos_a - pad['y'] * sin_a
            py_mm = fp['y'] + pad['x'] * sin_a + pad['y'] * cos_a
            px = px_mm * scale - off_x
            py = py_mm * scale - off_y
            result += (f'    <rect id="connector{conn_idx}pin" '
                       f'x="{px - 0.2:.4f}" y="{py - 0.2:.4f}" '
                       f'width="0.4" height="0.4" '
                       f'fill="none" stroke="none" '
                       f'visibility="hidden"/>\n')
            conn_idx += 1
    result += f'  </g>\n'

    result += svg_footer()

    with open(output_path, 'w') as f:
        f.write(result)

    return conn_idx


def generate_schematic_svg(pcb: PCBParser, output_path: Path,
                           board_w_mm: float, board_h_mm: float):
    """Generate a schematic view SVG — simplified pin diagram."""
    board_w = 120
    board_h = 80
    margin = 5

    svg = svg_header((board_w + 2 * margin) / 100.0,
                     (board_h + 2 * margin) / 100.0,
                     viewbox=f"{-margin} {-margin} {board_w + 2 * margin} {board_h + 2 * margin}")

    # Board box
    svg += make_svg_rect(0, 0, board_w, board_h,
                         fill='#f0f0f0', stroke='#222', sw=0.5, rx=2)
    svg += make_svg_text(board_w / 2, -1, f'{board_w_mm:.0f}×{board_h_mm:.0f} mm Board',
                         size=3, fill='#222', anchor='middle')

    # Connector pins
    connectors = [fp for fp in pcb.footprints if is_connector(fp)]
    connectors.sort(key=lambda fp: (fp['y'], fp['x']))

    pin_count = 0
    max_pins = 40

    for conn in connectors:
        pins = [p for p in conn['pads'] if p['type'] != 'np_thru_hole']
        for pad in pins:
            if pin_count >= max_pins:
                break
            if pin_count < max_pins // 2:
                px, py = 0, 10 + pin_count * 3.5
                svg += make_svg_line(px, py, px - 3, py, stroke='#888', sw=0.3)
                svg += make_svg_text(px - 3.2, py + 0.5,
                                     f"{conn['ref']}.{pad['name']}",
                                     size=1.2, fill='#666', anchor='end')
            else:
                py = 10 + (pin_count - max_pins // 2) * 3.5
                svg += make_svg_line(board_w, py, board_w + 3, py,
                                     stroke='#888', sw=0.3)
                svg += make_svg_text(board_w + 0.5, py + 0.5,
                                     f"{conn['ref']}.{pad['name']}",
                                     size=1.2, fill='#666', anchor='start')
            pin_count += 1

    svg += svg_footer()
    with open(output_path, 'w') as f:
        f.write(svg)


def generate_icon_svg(output_path: Path):
    """Generate a simple icon view SVG."""
    svg = svg_header(64.0 / 100, 64.0 / 100, viewbox="0 0 64 64")
    svg += make_svg_rect(4, 4, 56, 56, fill='#2a7a2a', stroke='#1a5a1a', sw=1, rx=3)
    svg += make_svg_text(7, 30, 'PCB', size=8, fill='#fff')
    svg += make_svg_text(7, 44, 'Board', size=6, fill='#cfc')
    svg += svg_footer()
    with open(output_path, 'w') as f:
        f.write(svg)


# ══════════════════════════════════════════════════════════════════════════
#  FZP generator
# ══════════════════════════════════════════════════════════════════════════

def generate_fzp(pcb: PCBParser, output_dir: Path, name: str,
                 svg_bb: str, svg_sch: str,
                 svg_ico: str, svg_pcb: str) -> str:
    """Generate the Fritzing FZP part descriptor XML."""
    module_id = f"{name}_Board_v1"

    # Build connector list and net-to-connector mapping
    connectors = []
    net_connectors = {}  # net_name -> list of connector ids
    conn_idx = 0
    for fp in pcb.footprints:
        if not is_connector(fp):
            continue
        for pad in fp['pads']:
            label = f"{fp['ref']}.{pad['name']}"
            cid = f'connector{conn_idx}'
            net = pad.get('net', '')
            connectors.append({
                'id': cid,
                'name': label,
                'type': 'male',
                'description': f'{fp["ref"]} pin {pad["name"]} ({fp["value"]})',
                'svg_id': f'{cid}pin',
                'net': net,
            })
            if net:
                net_connectors.setdefault(net, []).append(cid)
            conn_idx += 1

    title = name.replace('_', ' ').replace('-', ' ').title()
    fzp = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
           f'<module moduleId="{module_id}" fritzingVersion="0.9.9" version="1">\n'
           f'  <title>{title}</title>\n'
           f'  <description>{name} — KiCad board converted via PcbDraw</description>\n'
           f'  <author>step2svg</author>\n'
           f'  <label>{name.upper()}</label>\n'
           f'  <tags>\n'
           f'    <tag>PCB</tag>\n'
           f'    <tag>{name}</tag>\n'
           f'    <tag>Custom</tag>\n'
           f'  </tags>\n'
           f'  <properties>\n'
           f'    <property name="Board Width" value="{pcb.board_w:.1f}mm"/>\n'
           f'    <property name="Board Height" value="{pcb.board_h:.1f}mm"/>\n'
           f'  </properties>\n'
           f'  <views>\n'
           f'    <breadboardView>\n'
           f'      <layers image="{svg_bb}">\n'
           f'        <layer layerId="breadboard"/>\n'
           f'      </layers>\n'
           f'    </breadboardView>\n'
           f'    <schematicView>\n'
           f'      <layers image="{svg_sch}">\n'
           f'        <layer layerId="schematic"/>\n'
           f'      </layers>\n'
           f'    </schematicView>\n'
           f'    <iconView>\n'
           f'      <layers image="{svg_ico}">\n'
           f'        <layer layerId="icon"/>\n'
           f'      </layers>\n'
           f'    </iconView>\n'
           f'    <pcbView>\n'
           f'      <layers image="{svg_pcb}">\n'
           f'        <layer layerId="copper1"/>\n'
           f'        <layer layerId="copper0"/>\n'
           f'        <layer layerId="silkscreen"/>\n'
           f'      </layers>\n'
           f'    </pcbView>\n'
           f'  </views>\n'
           f'  <connectors>\n')

    for conn in connectors:
        fzp += (f'    <connector id="{conn["id"]}" type="{conn["type"]}">\n'
                f'      <title>{conn["name"]}</title>\n'
                f'      <description>{conn["description"]}</description>\n'
                f'      <views>\n'
                f'        <breadboardView>\n'
                f'          <p layer="breadboard" svgId="{conn["svg_id"]}"/>\n'
                f'        </breadboardView>\n'
                f'        <schematicView>\n'
                f'          <p layer="schematic" svgId="{conn["svg_id"]}"/>\n'
                f'        </schematicView>\n'
                f'        <pcbView>\n'
                f'          <p layer="copper1" svgId="{conn["svg_id"]}"/>\n'
                f'        </pcbView>\n'
                f'      </views>\n'
                f'    </connector>\n')

    fzp += '  </connectors>\n'

    # ── Buses: group connectors on the same net ──
    # Only emit buses for nets that connect 2+ connector pins
    multi_net_buses = {net: cids for net, cids in net_connectors.items()
                       if len(cids) >= 2}
    if multi_net_buses:
        fzp += '  <buses>\n'
        for net_name in sorted(multi_net_buses):
            cids = multi_net_buses[net_name]
            # Sanitise net name for use as an XML id
            bus_id = re.sub(r'[^a-zA-Z0-9_.-]', '_', net_name.lstrip('/'))
            fzp += f'    <bus id="{bus_id}">\n'
            for cid in cids:
                fzp += f'      <member connectorId="{cid}"/>\n'
            fzp += f'    </bus>\n'
        fzp += '  </buses>\n'

    fzp += ('  <subparts/>\n'
            '  <modules/>\n'
            '</module>\n')

    fzp_path = output_dir / f'part.{module_id}.fzp'
    with open(fzp_path, 'w') as f:
        f.write(fzp)

    return module_id


# ══════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('kicad_pcb', type=Path,
                   help='Input KiCad PCB file (.kicad_pcb)')
    p.add_argument('output_dir', type=Path, nargs='?',
                   default=Path('./fritzing_part/'),
                   help='Output directory (default: ./fritzing_part/)')
    p.add_argument('--pcbdraw', default=DEFAULT_PCBDRAW,
                   help='Path to the pcbdraw executable')
    p.add_argument('--libs', default=f'KiCAD-base,kicad-3d',
                   help='Comma-separated PCBdraw libs (default: KiCAD-base,kicad-3d)')
    p.add_argument('--style', default=None,
                   help='PCBdraw --style argument (e.g. "oshpark-purple")')
    p.add_argument('--rebuild-lib', action='store_true',
                   help='Run build_kicad3d_lib.py before plotting')
    p.add_argument('--keep-raw', action='store_true',
                   help='Keep the raw (un-wrapped) PcbDraw SVG')
    p.add_argument('--name', default=None,
                   help='Base filename for outputs (default: board stem)')
    p.add_argument('--temp-dir', type=Path, default=None,
                   help='Temporary working directory (default: output_dir/tmp)')
    p.add_argument('--raw-svg', type=Path, default=None,
                   help='Use existing raw PcbDraw SVG instead of running pcbdraw')
    p.add_argument('--extra', nargs=argparse.REMAINDER, default=[],
                   help='Extra args passed through to pcbdraw plot '
                        '(everything after "--" on the command line)')
    args = p.parse_args()

    kicad_file = args.kicad_pcb.resolve()
    if not kicad_file.exists():
        sys.exit(f"Error: KiCad PCB file not found: {kicad_file}")

    base = args.name or kicad_file.stem  # e.g. "p4hil" from "p4hil.kicad_pcb"

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    temp_dir = (args.temp_dir or output_dir / 'tmp').resolve()
    temp_dir.mkdir(parents=True, exist_ok=True)

    # ── Step 1: Parse KiCad PCB ────────────────────────────────────────
    print(f"Loading KiCad PCB: {kicad_file}")
    parsed = load_kicad_pcb(kicad_file)
    print("Parsing board data...")

    pcb = PCBParser(parsed)
    print(f"  Board: {pcb.board_w:.1f} x {pcb.board_h:.1f} mm")
    print(f"  Footprints: {len(pcb.footprints)}")

    connectors = [fp for fp in pcb.footprints if is_connector(fp)]
    print(f"  Connector footprints: {len(connectors)}")

    # ── Step 2: (Re)build pcbdraw library ──────────────────────────────
    if args.rebuild_lib:
        if not BUILD_LIB.exists():
            sys.exit(f"build script not found: {BUILD_LIB}")
        print("\n→ Rebuilding PcbDraw component library...")
        subprocess.run([sys.executable, str(BUILD_LIB)],
                       cwd=str(SCRIPT_DIR), check=True)

    # ── Step 3: Generate board SVG via PcbDraw (or use existing) ────
    if args.raw_svg:
        raw_svg = args.raw_svg.resolve()
        if not raw_svg.exists():
            sys.exit(f"Error: raw SVG not found: {raw_svg}")
        print(f"  Using existing raw SVG: {raw_svg}")
    else:
        raw_svg = temp_dir / f'{base}_front.raw.svg'
        print(f"\n→ Running PcbDraw plot (front side)...")
        cmd = [args.pcbdraw, 'plot', '--libs', args.libs]
        if args.style:
            cmd += ['--style', args.style]
        extra = list(args.extra)
        if extra and extra[0] == '--':
            extra = extra[1:]
        cmd += extra
        cmd += [str(kicad_file), str(raw_svg)]

        try:
            subprocess.run(cmd, cwd=str(SCRIPT_DIR), check=True,
                           capture_output=True, text=True)
            print(f"  PcbDraw SVG: {raw_svg}")
        except subprocess.CalledProcessError as e:
            sys.exit(f"PcbDraw failed (exit {e.returncode}):\n{e.stderr}")
        except FileNotFoundError:
            sys.exit(f"pcbdraw not found at {args.pcbdraw}")

    # ── Step 4: Build Fritzing part SVGs ───────────────────────────────
    print(f"\n→ Generating Fritzing part in: {output_dir}")

    module_id = f'{base}_Board_v1'

    # Fritzing FZP convention: image="<viewType>/<moduleId>_<name>.svg"
    # FZPZ archive convention: "svg.<viewType>.<moduleId>_<name>.svg"
    svg_pcb = f'svg.pcb.{module_id}_pcb.svg'
    svg_bb  = f'svg.breadboard.{module_id}_breadboard.svg'
    svg_sch = f'svg.schematic.{module_id}_schematic.svg'
    svg_ico = f'svg.icon.{module_id}_icon.svg'
    # FZP image references use slash-separated view paths
    img_pcb = f'pcb/{module_id}_pcb.svg'
    img_bb  = f'breadboard/{module_id}_breadboard.svg'
    img_sch = f'schematic/{module_id}_schematic.svg'
    img_ico = f'icon/{module_id}_icon.svg'
    fzp_file = f'part.{module_id}.fzp'

    # Wrap PcbDraw SVG into Fritzing format with connector markers
    print(f"  Wrapping PcbDraw SVG as PCB view...")
    conn_count = wrap_pcbdraw_svg_for_fritzing(
        raw_svg, output_dir / svg_pcb, pcb)
    print(f"    Added {conn_count} connector markers")

    # Breadboard view — same as PCB for a custom board
    print(f"  Copying to breadboard view...")
    shutil.copy(output_dir / svg_pcb, output_dir / svg_bb)

    # Keep raw if requested
    if args.keep_raw:
        shutil.copy(raw_svg, output_dir / 'part_pcbdraw_raw.svg')
        print(f"  Kept raw PcbDraw SVG")


    # Schematic and icon
    print(f"  Generating schematic view...")
    generate_schematic_svg(pcb, output_dir / svg_sch,
                           pcb.board_w, pcb.board_h)

    print(f"  Generating icon...")
    generate_icon_svg(output_dir / svg_ico)

    # ── Step 5: Generate FZP ───────────────────────────────────────────
    print(f"  Generating FZP part descriptor...")
    generate_fzp(pcb, output_dir, base,
                 img_bb, img_sch, img_ico, img_pcb)

    # ── Step 6: Create .fzpz archive ───────────────────────────────────
    fzpz_path = output_dir / f'{base}.fzpz'
    with zipfile.ZipFile(fzpz_path, 'w') as zf:
        zf.write(output_dir / fzp_file, fzp_file)
        for svg in (svg_pcb, svg_bb, svg_sch, svg_ico):
            zf.write(output_dir / svg, svg)
    print(f"  Created FZPZ archive: {fzpz_path}")

    # ── Summary ────────────────────────────────────────────────────────
    print(f"\n✓ Fritzing part created in: {output_dir}")
    for f in (fzp_file, svg_pcb, svg_bb, svg_sch, svg_ico, f'{base}.fzpz'):
        fpath = output_dir / f
        if fpath.exists():
            size = fpath.stat().st_size
            print(f"    {f}  ({size/1024:.1f} KB)" if size < 1e6
                  else f"    {f}  ({size/1e6:.1f} MB)")

    print(f"\nTo use in Fritzing:")
    print(f"  1. Open Fritzing")
    print(f"  2. Parts > Import... > {fzpz_path}")
    print(f"  Or unzip: cd {output_dir} && unzip {base}.fzpz")


if __name__ == '__main__':
    main()
