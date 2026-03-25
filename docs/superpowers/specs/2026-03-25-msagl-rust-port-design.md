# msagl-rust: Rectilinear Edge Router Port Design

**Date:** 2026-03-25
**Status:** Draft
**Crate name:** msagl-rust
**License:** MIT (same as MSAGL source)

## Purpose

Port Microsoft's MSAGL RectilinearEdgeRouter from C#/TypeScript to Rust. This will be the first and only Rust crate for orthogonal edge routing with guaranteed edge separation. Published as a standalone open-source library on crates.io.

## Why This Port

The existing SignalCanvasRouter uses a custom PathFinder negotiation algorithm (Rust) with a 22-stage TypeScript post-processing pipeline. It works but requires extensive parameter tuning. MSAGL's rectilinear router produces clean, separated orthogonal routes out of the box with minimal configuration — demonstrated by running the C# version on broadcast signal flow test scenarios (2026-03-25).

No Rust crate exists for orthogonal edge routing. The `petgraph` ecosystem (3M+ monthly downloads) has graph algorithms but nothing for visual edge routing.

## Source Material

- **Primary porting source:** TypeScript port at `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/`
- **Authoritative reference:** C# source at `MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/`
- **Strategy:** Hybrid — TS for module structure and design patterns, C# for algorithm logic and correctness
- **Test fixtures:** 86 projection solver data files from C# test suite + 242 procedural routing test methods in `RectilinearTests.cs`

## Public API

```rust
use msagl_rust::{RectilinearEdgeRouter, Shape, FloatingPort, EdgeGeometry, Point};

// Create obstacles from node rectangles
let shapes = vec![
    Shape::rectangle(0.0, 100.0, 120.0, 60.0),
    Shape::rectangle(400.0, 100.0, 120.0, 60.0),
];

// Create router with configuration
let mut router = RectilinearEdgeRouter::new(&shapes)
    .padding(4.0)
    .corner_fit_radius(3.0)
    .edge_separation(8.0);

// Add edges to route
router.add_edge(EdgeGeometry::new(
    FloatingPort::new(0, Point::new(120.0, 130.0)),
    FloatingPort::new(1, Point::new(400.0, 130.0)),
));

// Route all edges
let result = router.run();

// Read routed paths
for edge in &result.edges {
    for segment in edge.curve.segments() {
        // LineSegment or Arc
    }
}
```

## Algorithm Pipeline (6 stages)

1. **Obstacle Tree Construction** — Pad node rectangles by `padding` distance, insert into R-tree (`rstar`) for spatial queries. Detect overlapping obstacles and merge into clumps.

2. **Visibility Graph Construction** — Two-pass sweep-line (horizontal then vertical). Each pass maintains an RB-tree of active obstacle sides, extends scan segments from obstacle corners, and records where edges can legally travel. The two sets of scan segments are intersected to produce the visibility graph vertices and edges.

3. **Port Visibility Splicing** — For each edge to route, temporarily connect source and target ports into the static visibility graph along their horizontal and vertical projections.

4. **Shortest Path Search** — Direction-aware A* on the visibility graph. Each vertex tracks 4 entry directions (N/S/E/W). Cost = `length_importance * length + bends_importance * bend_count`. Heuristic is Manhattan distance + estimated bends.

5. **Nudging (Edge Separation)** — The key differentiator. Sub-stages:
   - **PathRefiner:** Find intersections between all routed paths, insert shared vertices
   - **PathMerger:** Remove situations where two paths cross more than once, eliminate self-loops
   - **CombinatorialNudger:** Build a DAG of axis edges, topologically sort to determine segment ordering
   - **FreeSpaceFinder:** Sweep-line to compute how far each segment can move before hitting an obstacle
   - **LongestNudgedSegment:** Group consecutive collinear segments into maximal straight runs
   - **QPSC Solver:** 1D constrained optimization — position segments to respect minimum separation while minimizing deviation from ideal positions
   - **StaircaseRemover:** Collapse zig-zag patterns into straight segments

6. **Finalization** — Fit arcs (`kurbo::Arc`) into path corners using `corner_fit_radius`. Trim arrowheads. Output compound curves.

## Crate Structure

```
msagl-rust/
├── Cargo.toml
├── LICENSE-MIT
├── src/
│   ├── lib.rs                    # Public API
│   ├── geometry/
│   │   ├── mod.rs
│   │   ├── point.rs              # Custom Point (OrderedFloat for hashing)
│   │   ├── rectangle.rs
│   │   ├── polyline.rs           # SlotMap-backed doubly-linked PolylinePoints
│   │   ├── curve.rs              # Compound curve (kurbo::Line + kurbo::Arc)
│   │   └── point_comparer.rs     # Epsilon comparison + rounding to 6 decimals
│   ├── data_structures/
│   │   ├── mod.rs
│   │   └── rb_tree.rs            # intrusive-collections RBTree wrapper
│   ├── projection_solver/        # Full QPSC port
│   │   ├── mod.rs
│   │   ├── solver.rs             # Main solver driver (Block merge/split, constraint processing)
│   │   ├── qpsc.rs               # Gradient-projection quadratic programming
│   │   ├── block.rs              # Block data structure + DfDvNode traversal logic
│   │   ├── variable.rs
│   │   ├── constraint.rs
│   │   ├── violation_cache.rs    # Constraint violation caching (performance critical)
│   │   ├── parameters.rs         # Solver configuration
│   │   ├── solver_shell.rs       # High-level wrapper
│   │   └── uniform_solver.rs     # UniformOneDimensionalSolver (nudger entry point)
│   ├── visibility/
│   │   ├── mod.rs
│   │   ├── graph.rs              # HashMap<(OrderedFloat, OrderedFloat), VertexKey>
│   │   ├── vertex.rs             # Includes VertexEntries[4] (no subclass needed)
│   │   └── edge.rs
│   ├── routing/
│   │   ├── mod.rs
│   │   ├── rectilinear_edge_router.rs
│   │   ├── obstacle.rs
│   │   ├── obstacle_tree.rs      # rstar::RTree
│   │   ├── shape.rs
│   │   ├── port.rs               # FloatingPort
│   │   ├── edge_geometry.rs
│   │   ├── port_manager.rs       # Port visibility splicing + TransientGraphUtility
│   │   ├── static_graph_utility.rs  # Shared graph helpers (edge direction, vertex finding)
│   │   ├── splice_utility.rs     # Port splice helpers
│   │   ├── visibility_graph_generator.rs  # FullVisibilityGraphGenerator
│   │   ├── scan_line.rs          # RectilinearScanLine + NeighborSides
│   │   ├── scan_segment.rs       # ScanSegment + ScanSegmentTree + ScanSegmentVector
│   │   ├── segment_intersector.rs  # Intersects H/V scan segments to build visibility graph
│   │   ├── sweep_events.rs       # SweepEvent enum (LeftVertex, RightVertex, Low/HighBend, etc.)
│   │   ├── obstacle_sides.rs     # LeftObstacleSide, RightObstacleSide
│   │   ├── path_search.rs        # SsstRectilinearPath + MsmtRectilinearPath + VertexEntry
│   │   └── nudging/
│   │       ├── mod.rs
│   │       ├── nudger.rs
│   │       ├── path_refiner.rs
│   │       ├── path_merger.rs    # Removes redundant crossings, self-loops
│   │       ├── linked_point.rs   # LinkedPoint + LinkedPointSplitter
│   │       ├── combinatorial_nudger.rs
│   │       ├── free_space_finder.rs  # + AxisEdgesContainer, sweep events
│   │       ├── longest_nudged_segment.rs
│   │       ├── axis_edge.rs      # SlotMap-backed shared references
│   │       ├── path_edge.rs
│   │       └── staircase_remover.rs
│   └── arenas.rs                 # Typed SlotMap arenas
└── tests/
    ├── projection_solver/        # 86 fixture-driven tests
    ├── rectilinear/              # 159 fixture-driven tests
    └── fixtures/                 # JSON-converted C# test data
```

## Dependencies

| Crate | Version | Purpose | Downloads | Notes |
|---|---|---|---|---|
| `ordered-float` | 5.x | Hashable/orderable f64 | 278M | Eliminates string-key PointMap hack |
| `slotmap` | 1.x | Arena allocation with typed keys | 60M | Polyline nodes, AxisEdge shared refs |
| `rstar` | 0.12 | R-tree spatial indexing | 20M | Replaces custom RectangleNode |
| `kurbo` | 0.13 | Arc/curve geometry | 18M | Arc fitting, line segments |
| `intrusive-collections` | 0.10 | RBTree with cursor navigation | 10M | Scan line, free space finder |

## Rust-Specific Design Decisions

| Problem | C#/TS Approach | Rust Solution |
|---|---|---|
| Polyline doubly-linked list | Object refs with next/prev pointers | `SlotMap<PolylinePointKey, PolylinePointData>` with next/prev as keys |
| Shared mutable AxisEdge | GC/JS object references | `SlotMap<AxisEdgeKey, AxisEdgeData>`, all references are keys |
| Float-keyed HashMaps | PointMap with "x,y" string keys (TS) | `HashMap<(OrderedFloat<f64>, OrderedFloat<f64>), V>` |
| Abstract class hierarchies | `abstract class` + `extends` | Rust enums: `SweepEvent::LeftVertex(...)`, `Port::Floating(...)` |
| RBTree with node navigation | Custom RBTree with next/prev | `intrusive-collections` RBTree with cursor API (fallback: `BTreeMap` with `range()` if intrusive API proves too complex) |
| R-Tree spatial queries | Custom RectangleNode | `rstar::RTree<ObstacleEnvelope>` |
| Arc/curve geometry | Custom Ellipse + LineSegment | `kurbo::Arc`, `kurbo::Line` |
| Point as HashMap key | Rounding to 6 decimals + string key | `OrderedFloat` wrapping + pre-rounding on input |

## Build Order

Bottom-up — each layer is testable independently before the next begins.

### Phase 1: Foundation
1. `geometry/` — Point, Rectangle, Polyline (SlotMap-backed), Curve, PointComparer
2. `data_structures/` — RBTree wrapper around intrusive-collections

### Phase 2: Solver
3. `projection_solver/` — Full QPSC port (Variable, Constraint, Block, Solver, SolverShell)
   - Validated against 86 C# fixture files

### Phase 3: Visibility
4. `visibility/` — VisibilityGraph, VisibilityVertex, VisibilityEdge
5. `routing/obstacle` — Shape, Obstacle, ObstacleTree (rstar)
6. `routing/scan_line` — ScanSegment, ScanSegmentTree, EventQueue, VisibilityGraphGenerator

### Phase 4: Path Search
7. `routing/path_search` — SsstRectilinearPath, MsmtRectilinearPath, VertexEntry
8. `routing/port_manager` — PortManager, TransientGraphUtility (port splicing)

### Phase 5: Nudging
9. `routing/nudging/` — Full pipeline: PathRefiner, PathMerger, CombinatorialNudger, FreeSpaceFinder, LongestNudgedSegment, StaircaseRemover, Nudger orchestrator

### Phase 6: Integration
10. `routing/rectilinear_edge_router` — Top-level orchestrator wiring all stages
11. Integration tests — ported from C# `RectilinearTests.cs` + golden baseline JSON dumps
12. SVG output for visual verification

## Test Strategy

- **Projection solver fixtures:** 86 C# data files with variable definitions, constraints, and expected solutions. Write a C# script to dump these to JSON for Rust test consumption.
- **Routing tests:** `RectilinearTests.cs` contains 242 procedural test methods that construct obstacle layouts in code (not external fixture files). These must be ported as Rust `#[test]` functions. Additionally, write a C# dump harness that runs each test and serializes input (obstacles, edges) + output (routed paths) to JSON for golden baseline comparison.
- **Unit tests per module:** Each phase has its own tests validated before moving to the next.
- **Cross-validation:** Run identical inputs through C# demo, TS version, and Rust — compare results.
- **SVG visual tests:** Port the SVG writer for human inspection of routing quality.
- **Invariant assertions:** No node violations, minimum separation respected, all edges routed.
- **Error handling:** Router returns `Result<RoutingResult, RoutingError>`. Errors for: no path found, degenerate inputs (zero-size obstacles, coincident ports). Debug assertions for internal invariants during development.

## Scope Exclusions

- **WASM target:** Not day-one. Added after API stabilizes (just a compile target + wasm-bindgen layer)
- **SparseVisibilityGraphGenerator:** Only the full generator is ported
- **Group/cluster routing:** No hierarchical Shape parent/child support
- **Interactive re-routing (RectilinearInteractiveEditor):** Batch routing only
- **Non-floating ports:** CurvePort, RelativeFloatingPort, HookUpAnywhereFromInsidePort excluded
- **SignalCanvas-specific features:** No bus bundles, fan-out, or domain concepts

## Rounding Policy

All `Point` values are rounded to 6 decimal places on construction (`Point::new()` rounds both coordinates). This ensures consistent `OrderedFloat` hashing and matches C#'s `ApproximateComparer.Round()` behavior. Epsilon comparisons use `distance_epsilon = 1e-6` and `difference_epsilon = distance_epsilon / 2`.

## Estimated Scope

~12,000-14,000 lines of Rust library code + ~3,000-4,000 lines of tests. The in-scope C# source is ~18,000 lines; Rust is typically 0.65-0.75x the size of equivalent C# for algorithmic code (no boilerplate properties, less verbose generics, but explicit lifetime/ownership annotations add some).

## Attribution

README and crate metadata will credit Microsoft's MSAGL as the original source and link to the MSAGL repository. License file will include the original MIT copyright notice.

## Source References

- C# MSAGL: `MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/`
- TS MSAGL: `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/`
- C# test fixtures: `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Resources/`
- C# routing tests: `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs`
- Demo project: `MSAGL-Reference/RouterDemo/`
