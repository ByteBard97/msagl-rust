# msagl-rust Algorithmic Parity Audit — 2026-03-26

Full file-by-file comparison of Rust implementation against C# and TypeScript MSAGL reference.

## Summary: 60 files audited

| Status | Count | Files |
|--------|-------|-------|
| **FAITHFUL** | 5 | variable.rs, constraint.rs, scan_direction.rs, segment_intersector.rs, linked_point_splitter.rs |
| **MOSTLY_FAITHFUL** | 18 | rectangle.rs, obstacle_side.rs, event_queue.rs, block.rs, qpsc.rs, solver/mod.rs, dfdv.rs, qpsc_solve.rs, obstacle.rs, linked_point.rs, axis_edge.rs, longest_nudged_segment.rs, path_edge.rs, path_refiner.rs, transient_graph_utility.rs, vertex_entry.rs, visibility/graph.rs, visibility/edge.rs |
| **SIMPLIFIED** | 14 | point.rs, polyline.rs, curve.rs, obstacle_tree.rs, scan_segment.rs, visibility_graph_generator.rs, static_graph_utility.rs, splice_utility.rs, compass_direction.rs, combinatorial_nudger.rs, free_space_finder.rs, staircase_remover.rs, path_merger.rs, rectilinear_edge_router.rs |
| **DIVERGENT** | 6 | point_comparer.rs, convex_hull.rs, scan_line.rs, path_search.rs, port_manager.rs, port.rs |
| **DEAD CODE** | 2 | vg_event_processing.rs, msmt_path.rs |

---

## Critical Issues (blocks correct/performant routing)

### 1. path_search.rs — DIVERGENT (complete rewrite)
**C# ref:** `SsstRectilinearPath.cs` (523 lines)

The Rust path search is a complete rewrite that doesn't match C#/TS:
- Wrong cost model: uses `length + distance * percentage * bends` vs C#'s `LengthImportance * length + BendsImportance * bends`
- No edge weight support — all edges cost Manhattan distance, ignoring `SegmentWeight::Overlapped` (100000x)
- No `IsPassable` edge filtering for group boundaries
- No compound direction heuristic (NE, NW, SE, SW) — bend estimation wrong for diagonal targets
- No NextNeighbor[3] direction ordering (straight/right/left preference)
- No decrease-key on priority queue — uses stale entry detection
- 14+ methods missing

### 2. free_space_finder.rs — SIMPLIFIED (brute-force)
**C# ref:** `FreeSpaceFinder.cs` (527 lines)

Completely missing the sweep-line algorithm. Uses O(E*O) brute-force obstacle bounding instead of O((E+V) log n) sweep. Types exist in `free_space_finder_types.rs` but are unused. This is the primary performance bottleneck on large graphs.

### 3. scan_line.rs — DIVERGENT (static keys)
**C# ref:** `RectilinearScanLine.cs` (204 lines)

Uses static `SideKey` from `side.start()` instead of C#'s dynamic `ScanLineIntersectSide` at current sweep position. Works for rectangles, fails for angled sides. Missing `linePositionAtLastInsertOrRemove`.

### 4. point_comparer.rs — DIVERGENT (epsilon mismatch)
**C# ref:** `PointComparer.cs`

`compare()` uses `DISTANCE_EPSILON` (1e-6) where C# uses `DifferenceEpsilon` (5e-7) — 2x more tolerant. Affects precision of all routing comparisons.

### 5. convex_hull.rs — DIVERGENT (wrong algorithm)
**C# ref:** `ConvexHull.cs`

Uses Andrew's monotone chain instead of C#'s O'Rourke pivot-sort-scan. Different vertex ordering, no collinear point handling, no `BackSwitchOverPivot` edge case.

### 6. visibility_graph_generator.rs — SIMPLIFIED (2 of 6 events)
**C# ref:** `FullVisibilityGraphGenerator.cs` (1013 lines)

Only processes OpenVertex and CloseVertex. Missing: LowBend, HighBend, LowReflection, HighReflection events. No lookahead scan, no reflection staircase. `SkipToNeighbor` present but missing `IntersectionAtSideIsInsideAnotherObstacle`.

---

## High Priority Issues

### 7. port_manager.rs — DIVERGENT (600-line class → single function)
Replaces entire ObstaclePort lifecycle, FreePoint management, group ancestors, port entrances, edge chain extension with a single `splice_port` function.

### 8. port.rs — DIVERGENT (2 classes → 3-field struct)
`ObstaclePort` (port entrances, visibility rectangle, center vertex) and `FreePoint` (OOB handling, edge chain extension) both replaced by `FloatingPort { obstacle_index, location }`.

### 9. combinatorial_nudger.rs — SIMPLIFIED
Missing topological-order DAG traversal. Uses perpendicular-offset heuristic instead of recursive walk-ahead comparison with fork detection.

### 10. path_merger.rs — SIMPLIFIED (self-cycles only)
Missing cross-path merging entirely. C#/TS detects when two paths cross each other more than once and collapses the loop.

### 11. scan_segment.rs — MOSTLY_FAITHFUL but key divergence
Creates **bidirectional** edges where C# creates **unidirectional ascending**. Doubles graph edge count.

### 12. point.rs — SIMPLIFIED + constructor rounding
Rounds coordinates on construction (C#/TS does not). Missing 15+ geometric methods used throughout routing.

---

## Medium Priority Issues

### 13. obstacle_tree.rs — SIMPLIFIED
Missing full hierarchy calculation, connected-component clump creation, convex hull accretion. `graph_box()` is O(n) instead of O(1).

### 14. staircase_remover.rs — SIMPLIFIED
Missing segment R-tree for cross-path crossing detection. Collapse removes 1 point instead of 2.

### 15. rectilinear_edge_router.rs — SIMPLIFIED
Missing dynamic obstacle add/remove/update, SparseVG option, MsmtRectilinearPath, group support, self-edge routing, proper arrowhead calculation. Default padding 4.0 vs C#'s 1.0.

### 16. nudger.rs — DIVERGENT details
Missing `MapAxisEdgesToTheirObstacles`, `CalculateIdealPositionsForLongestSegs`, `SetWidthsOfArrowheads`. Has Rust-only safety nets (`restore_if_crossing`) compensating for simplified sub-algorithms.

### 17. qpsc_solve.rs — MOSTLY_FAITHFUL
Missing `foundViolation` in loop termination (C# checks `!foundSplit && !foundViolation`). Could cause premature termination.

### 18. solver cloning overhead
Extensive `Vec::clone()` for borrow checker workarounds. C# iterates directly.

---

## Dead Code

### 19. vg_event_processing.rs
Not in `mod.rs`. Contains more complete event handlers (all 6 types) with `SweepState` but never compiled. Reflection handlers are empty stubs.

### 20. msmt_path.rs
Not in `mod.rs`. Imports non-existent symbols. Would fail to compile.

---

## Low Priority / Acceptable Deviations

| File | Status | Note |
|------|--------|------|
| polyline.rs | SIMPLIFIED | Missing ICurve interface — not needed for rectilinear routing |
| curve.rs | SIMPLIFIED | Segment storage only — parametric eval not needed |
| static_graph_utility.rs | SIMPLIFIED | 7 of 25 methods — others needed as features expand |
| splice_utility.rs | SIMPLIFIED | Directional clamping differs but functional |
| vertex_entry.rs | SIMPLIFIED | Direction auto-computation from previous entry missing |
| compass_direction.rs | SIMPLIFIED | No compound directions — blocks better heuristic |
| visibility/edge.rs | MOSTLY_FAITHFUL | Missing IsPassable callback — blocks group routing |
| visibility/graph.rs | MOSTLY_FAITHFUL | Missing VertexFactory, prevEdge Dijkstra support |
| linked_point.rs | MOSTLY_FAITHFUL | Arena adaptation is correct |

---

## Performance Impact Estimate

| Issue | Current | Faithful | Impact on Large Bench |
|-------|---------|----------|-----------------------|
| free_space_finder sweep-line | O(E*O) | O((E+V) log n) | ~150ms savings |
| path_search edge weights | ignores 100000x weight | respects weight | correctness fix |
| scan_segment bidirectional edges | 2x edges | 1x edges | ~20% graph size reduction |
| combinatorial_nudger walk-ahead | O(E*k) heuristic | O(E*d) recursive | better path ordering |
| solver Vec cloning | extra allocs | direct iteration | ~10-20ms savings |
| obstacle_tree graph_box | O(n) | O(1) | minor |

---

## Recommended Remediation Order

### Phase 1: Performance (gets large bench competitive with C#)
1. `free_space_finder.rs` — full sweep-line (types already exist)
2. `scan_segment.rs` — unidirectional edges
3. Solver Vec cloning reduction

### Phase 2: Correctness (fixes remaining test gaps)
4. `path_search.rs` — faithful port with edge weights + direction ordering
5. `point_comparer.rs` — fix epsilon to match C#
6. `convex_hull.rs` — port C# algorithm
7. `point.rs` — remove constructor rounding

### Phase 3: Feature completeness
8. `visibility_graph_generator.rs` — all 6 event types
9. `port_manager.rs` + `port.rs` — full ObstaclePort/FreePoint lifecycle
10. `combinatorial_nudger.rs` — topological DAG + walk-ahead
11. `path_merger.rs` — cross-path merging
12. `scan_line.rs` — dynamic intersection keys
