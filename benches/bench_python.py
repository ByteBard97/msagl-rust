"""Python benchmark script for msagl-rust rectilinear edge routing.

Imports the `msagl` native module (built via maturin) and runs each
scenario (small/medium/large) using timeit, reporting mean +/- stddev.

Usage: python benches/bench_python.py
"""

import json
import math
import sys
import time
from pathlib import Path

ITERATIONS = 100
SCENARIO_NAMES = ["small", "medium", "large"]
SCENARIOS_DIR = Path(__file__).parent / "scenarios"


def load_scenario(name: str) -> dict:
    """Load a scenario JSON file."""
    path = SCENARIOS_DIR / f"{name}.json"
    with open(path) as f:
        return json.load(f)


def run_scenario(msagl, scenario: dict):
    """Run the router on a scenario and return the result."""
    obstacles = [
        msagl.Obstacle(o["x"], o["y"], o["width"], o["height"])
        for o in scenario["obstacles"]
    ]

    router = msagl.Router(
        obstacles,
        padding=scenario["padding"],
        edge_separation=scenario["edgeSeparation"],
    )

    for edge in scenario["edges"]:
        src_obs = obstacles[edge["sourceObstacle"]]
        tgt_obs = obstacles[edge["targetObstacle"]]
        src_port = src_obs.port(edge["source"]["x"], edge["source"]["y"])
        tgt_port = tgt_obs.port(edge["target"]["x"], edge["target"]["y"])
        router.add_edge(src_port, tgt_port)

    return router.route()


def mean(values: list[float]) -> float:
    """Compute the arithmetic mean."""
    return sum(values) / len(values)


def stddev(values: list[float]) -> float:
    """Compute the population standard deviation."""
    m = mean(values)
    variance = sum((v - m) ** 2 for v in values) / len(values)
    return math.sqrt(variance)


def main():
    try:
        import msagl
    except ImportError:
        print("ERROR: Could not import msagl module.", file=sys.stderr)
        print(
            "Build it first: cd crates/msagl-python && maturin develop --release",
            file=sys.stderr,
        )
        sys.exit(1)

    print("Python Benchmark Results")
    print("=" * 60)
    print()

    for name in SCENARIO_NAMES:
        scenario = load_scenario(name)
        num_obstacles = len(scenario["obstacles"])
        num_edges = len(scenario["edges"])

        # Correctness check (single run)
        result = run_scenario(msagl, scenario)
        if len(result.paths) != num_edges:
            print(
                f"ERROR: {name} — expected {num_edges} paths, "
                f"got {len(result.paths)}",
                file=sys.stderr,
            )
            sys.exit(1)

        fallbacks = sum(1 for p in result.paths if len(p.points) <= 2)
        if fallbacks > 0:
            print(
                f"  WARN: {name} — {fallbacks}/{num_edges} edges have "
                f"<= 2 waypoints (fallback paths)"
            )

        # Benchmark: run ITERATIONS times
        timings = []
        for _ in range(ITERATIONS):
            start = time.perf_counter()
            run_scenario(msagl, scenario)
            elapsed = (time.perf_counter() - start) * 1000  # convert to ms
            timings.append(elapsed)

        m = mean(timings)
        sd = stddev(timings)

        label = f"{name:<8} ({num_obstacles} obstacles, {num_edges} edges)"
        print(f"{label}: {m:.3f} ± {sd:.3f} ms  ({ITERATIONS} iterations)")

    print()


if __name__ == "__main__":
    main()
