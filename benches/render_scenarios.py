#!/usr/bin/env python3
"""Render benchmark scenarios to SVG using the msagl-rust Python bindings.

Outputs one SVG per scenario into benches/renders/.
Each SVG shows:
  - obstacles (filled rectangles or polygons) in light blue
  - padded obstacle outlines in dashed grey
  - routed edge paths in colour, with arrows
  - source ports as green circles, target ports as red circles

Usage:
    cd msagl-rust
    python benches/render_scenarios.py           # render small/medium/large
    python benches/render_scenarios.py --no-route  # layout only, no routing
"""

import json
import sys
import os
import math
import subprocess
import struct
from pathlib import Path

REPO = Path(__file__).parent.parent
SCENARIOS_DIR = REPO / "benches" / "scenarios"
OUTPUT_DIR = REPO / "benches" / "renders"

PADDING = 20  # SVG canvas margin

# Colour palette for edges (cycles)
EDGE_COLOURS = [
    "#e74c3c", "#3498db", "#2ecc71", "#f39c12",
    "#9b59b6", "#1abc9c", "#e67e22", "#34495e",
    "#e91e63", "#00bcd4", "#8bc34a", "#ff5722",
]

# ── SVG helpers ──────────────────────────────────────────────────────────────

def svg_header(w, h):
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{w}" height="{h}" '
        f'style="font-family:monospace">\n'
        f'<rect width="{w}" height="{h}" fill="#f8f9fa"/>\n'
        '<defs>\n'
        '  <marker id="arr" markerWidth="6" markerHeight="6" '
        '          refX="5" refY="3" orient="auto">\n'
        '    <path d="M0,0 L6,3 L0,6 Z" fill="context-stroke"/>\n'
        '  </marker>\n'
        '</defs>\n'
    )

def svg_rect(x, y, w, h, fill, stroke, stroke_w=1, dash="", opacity=1.0, rx=2):
    d = f'stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{stroke_w}" '
        f'opacity="{opacity}" {d}/>\n'
    )

def svg_polygon(points, fill, stroke, stroke_w=1, dash="", opacity=1.0):
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    d = f'stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" '
        f'stroke-width="{stroke_w}" opacity="{opacity}" {d}/>\n'
    )

def svg_polyline(points, colour, width=1.5, arrow=True):
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    arr = 'marker-end="url(#arr)"' if arrow else ""
    return (
        f'<polyline points="{pts}" fill="none" stroke="{colour}" '
        f'stroke-width="{width}" stroke-linejoin="round" {arr}/>\n'
    )

def svg_circle(cx, cy, r, fill, stroke="#fff", stroke_w=1):
    return (
        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{stroke_w}"/>\n'
    )

def svg_text(x, y, text, size=9, colour="#333"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
        f'fill="{colour}" text-anchor="middle">{text}</text>\n'
    )

# ── Geometry helpers ─────────────────────────────────────────────────────────

def bbox_of_points(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)

def offset_polygon(points, padding):
    """Rough outward offset: push each point away from centroid by padding."""
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    result = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        dist = math.hypot(dx, dy) or 1.0
        result.append((x + dx / dist * padding, y + dy / dist * padding))
    return result

# ── Try to import msagl_rust Python bindings ─────────────────────────────────

def try_import_msagl():
    """Return the msagl_rust module or None if bindings aren't built."""
    # Try the wheel installed in the current environment first
    try:
        import msagl_rust  # noqa: F401
        return msagl_rust
    except ImportError:
        pass
    # Try the maturin dev build location
    for candidate in (
        REPO / "target" / "wheels",
        REPO,
    ):
        sys.path.insert(0, str(candidate))
        try:
            import msagl_rust  # noqa: F401
            return msagl_rust
        except ImportError:
            sys.path.pop(0)
    return None

# ── Route scenarios via msagl_rust Python bindings ───────────────────────────

def route_scenario_python(scenario, msagl):
    """Route using Python bindings. Returns list of edge point lists or None."""
    try:
        router = msagl.RectilinearEdgeRouter(
            [(o["x"], o["y"], o["width"], o["height"])
             for o in scenario["obstacles"]],
            padding=scenario["padding"],
            edge_separation=scenario["edgeSeparation"],
        )
        for e in scenario["edges"]:
            router.add_edge(
                e["sourceObstacle"], (e["source"]["x"], e["source"]["y"]),
                e["targetObstacle"], (e["target"]["x"], e["target"]["y"]),
            )
        result = router.run()
        return [list(map(tuple, edge_pts)) for edge_pts in result]
    except Exception as exc:
        print(f"  Python binding routing failed: {exc}", file=sys.stderr)
        return None

# ── Route scenarios via Rust binary (cargo run --example) ────────────────────

def route_via_cargo(scenario_name):
    """
    Route via `cargo run --example route_and_dump -- <name>`.
    Builds the example if needed. Returns list of edge point lists or None.
    """
    r = subprocess.run(
        ["cargo", "run", "--example", "route_and_dump", "--quiet", "--", scenario_name],
        cwd=REPO, capture_output=True, text=True,
    )
    if r.returncode != 0:
        print(f"  cargo run failed: {r.stderr[:200]}", file=sys.stderr)
        return None
    try:
        data = json.loads(r.stdout.strip())
        return [list(map(tuple, edge)) for edge in data["edges"]]
    except (json.JSONDecodeError, KeyError) as exc:
        print(f"  JSON parse failed: {exc}", file=sys.stderr)
        return None

# ── Main render function ──────────────────────────────────────────────────────

def render_scenario(name, scenario, routed_edges, output_path):
    obstacles = scenario["obstacles"]
    raw_edges = scenario["edges"]
    padding_val = scenario.get("padding", 4.0)

    # Determine world bounds
    all_x, all_y = [], []
    for o in obstacles:
        if "points" in o:
            for px, py in o["points"]:
                all_x.append(px)
                all_y.append(py)
        else:
            all_x += [o["x"], o["x"] + o["width"]]
            all_y += [o["y"], o["y"] + o["height"]]
    for e in raw_edges:
        all_x += [e["source"]["x"], e["target"]["x"]]
        all_y += [e["source"]["y"], e["target"]["y"]]
    if routed_edges:
        for pts in routed_edges:
            for px, py in pts:
                all_x.append(px)
                all_y.append(py)

    min_x = min(all_x) - PADDING
    min_y = min(all_y) - PADDING
    max_x = max(all_x) + PADDING
    max_y = max(all_y) + PADDING
    W = max_x - min_x
    H = max_y - min_y

    def tx(x): return x - min_x
    def ty(y): return y - min_y

    svg = svg_header(W, H)

    # Grid
    grid = 50
    gx = math.floor(min_x / grid) * grid
    while gx < max_x:
        x = tx(gx)
        svg += f'<line x1="{x:.0f}" y1="0" x2="{x:.0f}" y2="{H:.0f}" stroke="#e0e0e0" stroke-width="0.5"/>\n'
        gx += grid
    gy = math.floor(min_y / grid) * grid
    while gy < max_y:
        y = ty(gy)
        svg += f'<line x1="0" y1="{y:.0f}" x2="{W:.0f}" y2="{y:.0f}" stroke="#e0e0e0" stroke-width="0.5"/>\n'
        gy += grid

    # Routed edges or fallback lines (drawn BEFORE obstacles so boxes cover wire stubs inside them)
    for ei, e in enumerate(raw_edges):
        colour = EDGE_COLOURS[ei % len(EDGE_COLOURS)]
        if routed_edges and ei < len(routed_edges) and routed_edges[ei]:
            pts = [(tx(px), ty(py)) for px, py in routed_edges[ei]]
            svg += svg_polyline(pts, colour, width=1.8, arrow=True)
        else:
            # Unrouted: straight dashed line
            sx, sy = tx(e["source"]["x"]), ty(e["source"]["y"])
            ex2, ey2 = tx(e["target"]["x"]), ty(e["target"]["y"])
            svg += (
                f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{ex2:.1f}" y2="{ey2:.1f}" '
                f'stroke="{colour}" stroke-width="1.2" stroke-dasharray="4,3" '
                f'marker-end="url(#arr)"/>\n'
            )

    # Obstacles drawn ON TOP of edges so boxes visually cover the wire stubs inside them
    for i, o in enumerate(obstacles):
        if "points" in o:
            pts = [(tx(px), ty(py)) for px, py in o["points"]]
            padded = offset_polygon(pts, padding_val)
            svg += svg_polygon(padded, "#deeaf7", "#7fb3d3", dash="3,2", opacity=0.6)
            svg += svg_polygon(pts, "#d0e4f7", "#5b9bd5", stroke_w=1.5, opacity=1.0)
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
        else:
            x, y, w, h = tx(o["x"]), ty(o["y"]), o["width"], o["height"]
            svg += svg_rect(x - padding_val, y - padding_val,
                            w + 2 * padding_val, h + 2 * padding_val,
                            "none", "#7fb3d3", dash="3,2", opacity=0.5)
            svg += svg_rect(x, y, w, h, "#d0e4f7", "#5b9bd5", stroke_w=1.5, opacity=1.0)
            cx, cy = x + w / 2, y + h / 2
        svg += svg_text(cx, cy + 3, str(i), size=8, colour="#1a5276")

    # Port dots
    for ei, e in enumerate(raw_edges):
        colour = EDGE_COLOURS[ei % len(EDGE_COLOURS)]
        svg += svg_circle(tx(e["source"]["x"]), ty(e["source"]["y"]), 3.5, colour)
        svg += svg_circle(tx(e["target"]["x"]), ty(e["target"]["y"]), 3.5, colour,
                          stroke=colour, stroke_w=1.5)

    # Legend
    label = f"{name}: {len(obstacles)} obstacles, {len(raw_edges)} edges"
    if routed_edges:
        label += " [routed]"
    else:
        label += " [layout only]"
    svg += (
        f'<text x="6" y="14" font-size="11" fill="#555" font-family="monospace">'
        f'{label}</text>\n'
    )

    svg += "</svg>\n"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(svg)
    print(f"  Wrote {output_path.relative_to(REPO)}")

# ── Polygon scenario (defined in Rust bench, mirrored here) ──────────────────

def polygon_scenario():
    """Mirror of the bench_polygon scenario in benches/routing.rs."""
    obstacles = []

    # 12 rectangles in 4×3 grid
    for col in range(4):
        for row in range(3):
            obstacles.append({
                "x": col * 120.0,
                "y": row * 100.0,
                "width": 60.0,
                "height": 40.0,
            })

    # 8 polygon obstacles
    poly_defs = [
        [(490.0, 30.0), (460.0, 70.0), (520.0, 70.0)],
        [(490.0, 130.0), (460.0, 170.0), (520.0, 170.0)],
        [(490.0, 230.0), (460.0, 270.0), (520.0, 270.0)],
        [(490.0, 330.0), (460.0, 370.0), (520.0, 370.0)],
        [
            (620.0, 50.0), (640.0, 30.0), (670.0, 30.0),
            (690.0, 50.0), (670.0, 70.0), (640.0, 70.0),
        ],
        [
            (620.0, 150.0), (640.0, 130.0), (670.0, 130.0),
            (690.0, 150.0), (670.0, 170.0), (640.0, 170.0),
        ],
        [
            (620.0, 250.0), (640.0, 230.0), (670.0, 230.0),
            (690.0, 250.0), (670.0, 270.0), (640.0, 270.0),
        ],
        [
            (620.0, 350.0), (640.0, 330.0), (670.0, 330.0),
            (690.0, 350.0), (670.0, 370.0), (640.0, 370.0),
        ],
    ]
    for pts in poly_defs:
        obstacles.append({"points": pts})

    n_rect, n_poly = 12, 8
    edges = []
    shapes = obstacles

    def center(o):
        if "points" in o:
            xs = [p[0] for p in o["points"]]
            ys = [p[1] for p in o["points"]]
            return sum(xs) / len(xs), sum(ys) / len(ys)
        return o["x"] + o["width"] / 2, o["y"] + o["height"] / 2

    # rect↔rect ring (12 edges)
    for i in range(n_rect):
        j = (i + 4) % n_rect
        sx, sy = center(shapes[i])
        tx2, ty2 = center(shapes[j])
        edges.append({"source": {"x": sx, "y": sy}, "target": {"x": tx2, "y": ty2},
                      "sourceObstacle": i, "targetObstacle": j})

    # rect↔poly (16 edges)
    for k in range(n_poly * 2):
        r = (k * 3) % n_rect
        p = n_rect + (k % n_poly)
        sx, sy = center(shapes[r])
        tx2, ty2 = center(shapes[p])
        edges.append({"source": {"x": sx, "y": sy}, "target": {"x": tx2, "y": ty2},
                      "sourceObstacle": r, "targetObstacle": p})

    # poly↔poly ring (8 edges)
    for k in range(n_poly):
        p1 = n_rect + k
        p2 = n_rect + (k + 1) % n_poly
        sx, sy = center(shapes[p1])
        tx2, ty2 = center(shapes[p2])
        edges.append({"source": {"x": sx, "y": sy}, "target": {"x": tx2, "y": ty2},
                      "sourceObstacle": p1, "targetObstacle": p2})

    return {
        "obstacles": obstacles,
        "edges": edges,
        "padding": 3.0,
        "edgeSeparation": 2.0,
    }

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    no_route = "--no-route" in sys.argv

    msagl = None
    if not no_route:
        msagl = try_import_msagl()
        if msagl is None:
            print(
                "msagl_rust Python bindings not found — rendering layout only.\n"
                "Build them with:  maturin develop  (or pip install msagl-rust)\n",
                file=sys.stderr,
            )

    # JSON-based scenarios
    for name in ("small", "medium", "large"):
        path = SCENARIOS_DIR / f"{name}.json"
        if not path.exists():
            print(f"Skipping {name}: {path} not found")
            continue
        print(f"Rendering {name}...")
        scenario = json.loads(path.read_text())
        routed = None
        if not no_route:
            if msagl:
                routed = route_scenario_python(scenario, msagl)
            else:
                routed = route_via_cargo(name)
        render_scenario(name, scenario, routed, OUTPUT_DIR / f"{name}.svg")

    # Polygon scenario (layout mirrored from Rust bench)
    print("Rendering polygon...")
    scenario = polygon_scenario()
    routed_poly = None
    if not no_route:
        routed_poly = route_via_cargo("polygon")
    render_scenario("polygon", scenario, routed_poly, OUTPUT_DIR / "polygon.svg")

    # Convert SVGs to PNGs using rsvg-convert (2× scale)
    rsvg = None
    for candidate in ("rsvg-convert",):
        try:
            subprocess.run([candidate, "--version"], capture_output=True, check=True)
            rsvg = candidate
            break
        except (FileNotFoundError, subprocess.CalledProcessError):
            pass

    if rsvg:
        print("\nConverting to PNG...")
        for svg_path in sorted(OUTPUT_DIR.glob("*.svg")):
            png_path = svg_path.with_suffix(".png")
            r = subprocess.run(
                [rsvg, "-z", "2", str(svg_path), "-o", str(png_path)],
                capture_output=True,
            )
            if r.returncode == 0:
                print(f"  Wrote {png_path.relative_to(REPO)}")
            else:
                print(f"  FAILED: {png_path.name}: {r.stderr.decode()[:100]}")
    else:
        print("\nrsvg-convert not found — skipping PNG conversion")

    print(f"\nDone. Files in {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
