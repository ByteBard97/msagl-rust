"""Generate deterministic benchmark scenario JSON files.

Produces grid-based layouts with obstacles and edges connecting
nearby obstacles. Uses no randomness — purely deterministic.
"""

import json
import math
import sys
from pathlib import Path


def generate_scenario(cols, rows, edges_per_obstacle):
    """Generate a grid of obstacles with edges connecting nearby pairs.

    Args:
        cols: Number of columns in the grid.
        rows: Number of rows in the grid.
        edges_per_obstacle: Approximate number of outgoing edges per obstacle.

    Returns:
        dict with obstacles, edges, padding, edgeSeparation.
    """
    obstacle_width = 100
    obstacle_height = 50
    h_spacing = 150  # horizontal gap between obstacles
    v_spacing = 100  # vertical gap between obstacles

    obstacles = []
    for row in range(rows):
        for col in range(cols):
            x = col * (obstacle_width + h_spacing)
            y = row * (obstacle_height + v_spacing)
            obstacles.append({
                "x": x,
                "y": y,
                "width": obstacle_width,
                "height": obstacle_height,
            })

    num_obstacles = len(obstacles)
    edges = []
    edge_set = set()

    for i in range(num_obstacles):
        row_i = i // cols
        col_i = i % cols

        # Connect to right neighbor
        if col_i + 1 < cols:
            j = row_i * cols + (col_i + 1)
            key = (min(i, j), max(i, j), 0)
            if key not in edge_set:
                edge_set.add(key)
                edges.append(_make_edge(obstacles, i, j, "right"))

        # Connect to bottom neighbor
        if row_i + 1 < rows:
            j = (row_i + 1) * cols + col_i
            key = (min(i, j), max(i, j), 0)
            if key not in edge_set:
                edge_set.add(key)
                edges.append(_make_edge(obstacles, i, j, "down"))

        # Connect diagonals for extra density
        if edges_per_obstacle > 2:
            if col_i + 1 < cols and row_i + 1 < rows:
                j = (row_i + 1) * cols + (col_i + 1)
                key = (min(i, j), max(i, j), 0)
                if key not in edge_set:
                    edge_set.add(key)
                    edges.append(_make_edge(obstacles, i, j, "diag_right"))

        if edges_per_obstacle > 3:
            if col_i - 1 >= 0 and row_i + 1 < rows:
                j = (row_i + 1) * cols + (col_i - 1)
                key = (min(i, j), max(i, j), 0)
                if key not in edge_set:
                    edge_set.add(key)
                    edges.append(_make_edge(obstacles, i, j, "diag_left"))

        # Skip-one connections for even more density
        if edges_per_obstacle > 4:
            if col_i + 2 < cols:
                j = row_i * cols + (col_i + 2)
                key = (min(i, j), max(i, j), 0)
                if key not in edge_set:
                    edge_set.add(key)
                    edges.append(_make_edge(obstacles, i, j, "skip_right"))

    return {
        "obstacles": obstacles,
        "edges": edges,
        "padding": 4.0,
        "edgeSeparation": 8.0,
    }


def _make_edge(obstacles, src_idx, tgt_idx, direction):
    """Create an edge connecting two obstacles with ports on appropriate sides."""
    src = obstacles[src_idx]
    tgt = obstacles[tgt_idx]

    # Source port: right side for rightward, bottom for downward, etc.
    if direction in ("right", "skip_right"):
        sx = src["x"] + src["width"]
        sy = src["y"] + src["height"] / 2
        tx = tgt["x"]
        ty = tgt["y"] + tgt["height"] / 2
    elif direction == "down":
        sx = src["x"] + src["width"] / 2
        sy = src["y"] + src["height"]
        tx = tgt["x"] + tgt["width"] / 2
        ty = tgt["y"]
    elif direction == "diag_right":
        sx = src["x"] + src["width"]
        sy = src["y"] + src["height"]
        tx = tgt["x"]
        ty = tgt["y"]
    elif direction == "diag_left":
        sx = src["x"]
        sy = src["y"] + src["height"]
        tx = tgt["x"] + tgt["width"]
        ty = tgt["y"]
    else:
        sx = src["x"] + src["width"] / 2
        sy = src["y"] + src["height"] / 2
        tx = tgt["x"] + tgt["width"] / 2
        ty = tgt["y"] + tgt["height"] / 2

    return {
        "source": {"x": sx, "y": sy},
        "target": {"x": tx, "y": ty},
        "sourceObstacle": src_idx,
        "targetObstacle": tgt_idx,
    }


def main():
    out_dir = Path(__file__).parent / "scenarios"

    # Medium: ~50 obstacles, ~80 edges → 7×7 grid = 49 obstacles
    medium = generate_scenario(cols=7, rows=7, edges_per_obstacle=2)
    assert len(medium["obstacles"]) >= 49
    # Trim edges to ~80
    medium["edges"] = medium["edges"][:80]
    with open(out_dir / "medium.json", "w") as f:
        json.dump(medium, f, indent=2)
    print(f"medium.json: {len(medium['obstacles'])} obstacles, {len(medium['edges'])} edges")

    # Large: ~200 obstacles, ~500 edges → 14×15 grid = 210 obstacles
    large = generate_scenario(cols=14, rows=15, edges_per_obstacle=4)
    assert len(large["obstacles"]) >= 200
    # Trim edges to 500
    large["edges"] = large["edges"][:500]
    with open(out_dir / "large.json", "w") as f:
        json.dump(large, f, indent=2)
    print(f"large.json: {len(large['obstacles'])} obstacles, {len(large['edges'])} edges")


if __name__ == "__main__":
    main()
