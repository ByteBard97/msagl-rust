# msagl-rust Code Review — 2026-03-25

Audit of `src/` against `ClaudeCodeRules.md`. Excludes `port_manager.rs` and `rectilinear_edge_router.rs` (actively being modified).

---

## Rule 2: File Sizes — CLEAN

All 61 files under 500 lines. Largest is `nudger.rs` at 483. No violations.

---

## Rule 1: Separate Concerns

| File | Severity | Issue |
|------|----------|-------|
| `routing/nudging/nudger.rs` | warning | Mixes pipeline orchestration with path cleanup logic (`remove_switchbacks`, `is_collinear`, `is_switchback`, `rectilinearise`). Extract path cleanup to a separate module. |
| `routing/rectilinear_edge_router.rs` | note | `points_to_curve` with corner-fitting arc logic (lines 171-224) is a distinct concern from routing orchestration. Could be extracted. |
| `projection_solver/solver/mod.rs` | note | `Solver::solve()` handles parameter defaulting, constraint setup, equality merging, cache config, algorithm dispatch, and post-solve stats in one 70-line method. |

---

## Rule 4: DRY

### Violations

| Location | Issue | Fix |
|----------|-------|-----|
| `routing/visibility_graph_generator.rs:344-351` | `side_scan_coord()` and `side_scan_coord_from_side()` are identical functions with identical bodies. | Delete one, alias or rename the other. |
| `routing/nudging/staircase_remover.rs:130` `seg_direction`, `routing/nudging/axis_edge.rs:60` `direction_from_points`, `routing/nudging/combinatorial_nudger.rs:100` `canonicalize`, `routing/nudging/longest_nudged_segment.rs:100` `direction_from_points_loose` | Four separate functions across four files all compute direction from two points using `dx.abs() > dy.abs()`. | Extract a single shared `direction_from_points() -> Direction` helper into `scan_direction.rs` or a new `direction_utils.rs`. |

### Warnings

| Location | Issue | Fix |
|----------|-------|-----|
| `routing/nudging/nudger.rs` (`get_left_bound` / `get_right_bound`) | Structurally identical, differ only in which field and whether they use `max` vs `min`. | Parameterize into a single function with a closure or enum. |
| `routing/visibility_graph_generator.rs:16` and `routing/obstacle_tree.rs:121` | `SENTINEL_OFFSET = 1.0` defined in two files. | Define once in a shared location. |
| `routing/scan_direction.rs` `Direction` enum vs `routing/compass_direction.rs` `CompassDirection` enum | Two direction enums where `Direction {North, East}` is a strict subset of `CompassDirection {N, E, S, W}`. | Nudging code could use `CompassDirection` directly, eliminating the parallel type. |
| `routing/segment_intersector.rs:116-130` and `:154-176` | H/V branches are structurally identical, differ only in which axis is fixed vs ranged. | Factor through `ScanDirection.coord()`/`perp_coord()`. |
| `geometry/polyline.rs:203-245` | `PointIter` and `PolylinePointIter` are nearly identical iterator implementations. | Unify with a generic iterator or `.map()`. |

### Notes

| Location | Issue |
|----------|-------|
| `rectilinear_edge_router.rs` `axis_distance` vs `path_search.rs` `manhattan_distance` | Both compute Manhattan distance identically. Router could call `manhattan_distance`. |

---

## Rule 7: YAGNI — Dead Code

> These modules may be scaffolding for the ongoing faithful port. Confirm with the porting agent before removing.

### Warnings (entire modules unused)

| Module | Issue |
|--------|-------|
| `routing/neighbor_sides.rs` | `NeighborSides` struct never imported or used anywhere in `src/`. |
| `routing/lookahead_scan.rs` | `LookaheadScan` struct never imported or used anywhere in `src/`. |
| `routing/splice_utility.rs` | `SpliceUtility` methods never called anywhere in `src/`. |
| `routing/vertex_entry.rs` | `VertexEntry` struct never constructed. Path search uses `SearchEntry` instead. `VertexEntryIndex` is referenced in `visibility/graph.rs` but no entries are ever created. |

### Notes (dead fields/methods)

| Location | Issue |
|----------|-------|
| `routing/nudging/path_edge.rs:21` | `width` field is `#[allow(dead_code)]` — never read. |
| `routing/nudging/longest_nudged_segment.rs:92` | `max_width()` is `#[allow(dead_code)]`, always returns `0.0`, never called. |
| `routing/nudging/linked_point_splitter.rs:49` | `Event::is_vertical` field is `#[allow(dead_code)]` — stored but never read. |
| `visibility/edge.rs` | `VisEdge::toll_free()` constructor never called; `length_multiplier` always `1.0`, never read. |
| `routing/nudging/nudger.rs:328` | `_segments` parameter on `shift_point` is unused. |
| `routing/nudging/nudger.rs:387` | `opposite_dir` is a no-op stub (returns the same value it receives). |

---

## Rule 8: Error Handling

### Violations (crash risk)

| Location | Issue | Fix |
|----------|-------|-----|
| `routing/nudging/free_space_finder.rs:91` | `pa.partial_cmp(&pb).unwrap()` — panics on NaN. | Use `.unwrap_or(Ordering::Equal)` or `OrderedFloat`. |
| `routing/nudging/linked_point_splitter.rs:241` | `b.0.partial_cmp(&a.0).unwrap()` in sort closure — panics on NaN. | Same fix. |

### Warnings

| Location | Issue | Fix |
|----------|-------|-----|
| `routing/visibility_graph_generator.rs:196` | `_ => {}` silently ignores unhandled event types. | Add `debug_assert!` or enumerate explicitly. |
| `routing/path_search.rs:271` | `.unwrap()` on `filtered.last()` — no message. | Use `.expect("filtered must be non-empty after initial push")`. |
| `routing/path_search.rs:282` | `.unwrap()` on `points.last()` — no message. | Add `.expect()`. |
| `geometry/curve.rs:85` | `.unwrap()` on `points.last()` — guarded but no message. | Add `.expect()`. |
| `projection_solver/solver/mod.rs:291` | `self.blocks_order.last().unwrap()` — no visible guard. | Add `.expect()` or guard. |
| `projection_solver/solver/dfdv.rs:143,163` | `done_vi.is_none() \|\| right != done_vi.unwrap()` — safe via short-circuit but non-idiomatic. | Use `done_vi.map_or(true, \|v\| right != v)`. |
| `projection_solver/violation_cache.rs:56` | `best.unwrap().1` after manual `is_none()` check. | Use `best.map_or(true, \|(_, v)\| ...)`. |

---

## Rule 9: Magic Numbers

### Violations

| Location | Value | Used for | Fix |
|----------|-------|----------|-----|
| `routing/path_search.rs:301,306-309,312-313` | `1e-9` (x8) | Geometric tolerance | Use `GeomConstants::TOLERANCE` (1e-8) or define `const COLLINEAR_EPSILON: f64 = 1e-9` |
| `routing/obstacle_side.rs:29,34,81,86` | `1e-10` (x4) | Slope/intersection zero-check | Define `const SLOPE_EPSILON: f64 = 1e-10` or use `GeomConstants` |
| `routing/static_graph_utility.rs:62` | `1e-10` | Collinearity tolerance | Same — use a named constant |
| `routing/nudging/free_space_finder.rs:63,68,73,103,126` | `1e-6` (x5) | Geometric epsilon | Use `GeomConstants::DISTANCE_EPSILON` |
| `routing/nudging/nudger.rs:183` | `1e6` | Fixed variable weight | Define `const FIXED_VARIABLE_WEIGHT: f64 = 1e6` |
| `projection_solver/uniform_solver.rs:36,42` | `1e8` | Anchor weight for bounds | Define `const ANCHOR_WEIGHT: f64 = 1e8` |

### Warnings

| Location | Value | Fix |
|----------|-------|-----|
| `routing/rectilinear_edge_router.rs:200` | `1e-6` | Use `GeomConstants::DISTANCE_EPSILON` |
| `routing/rectilinear_edge_router.rs:236` | `1e-12` | Use `GeomConstants::SQUARE_OF_DISTANCE_EPSILON` |
| `routing/compass_direction.rs:61` | `1e-12` | Use `GeomConstants::SQUARE_OF_DISTANCE_EPSILON` |
| `routing/scan_segment.rs:20-21` | `5`, `500` | Define `const REFLECTION_WEIGHT` and `OVERLAPPED_WEIGHT` |
| `projection_solver/solver/mod.rs:141` | `100` | Define `const ITERATION_LIMIT_MULTIPLIER` |
| `routing/nudging/linked_point_splitter.rs:131,138,150` | `0`, `1`, `2` | Define `const EVENT_ORDER_VERT_LOW`, `EVENT_ORDER_HORIZ`, `EVENT_ORDER_VERT_HIGH` |

---

## Rule 6: Coupling — Acceptable

Three orchestrator files import 7-9 internal modules each. Expected for top-level coordinators:

| File | Internal imports |
|------|-----------------|
| `visibility_graph_generator.rs` | 9 modules |
| `nudger.rs` | 8 modules |
| `rectilinear_edge_router.rs` | 7 modules |

One coupling smell: `port_manager.rs:64-94` iterates raw vertex indices (`0..graph.vertex_count()`) instead of using a graph API method. Should be a `VisibilityGraph::find_nearest_aligned_vertex()` method.

---

## Summary by Priority

| Priority | Category | Count | Effort |
|----------|----------|-------|--------|
| 1 | Magic numbers (Rule 9) | 6 violations, 6 warnings | Low — define constants, find-replace |
| 2 | NaN panics (Rule 8) | 2 violations | Low — one-line fixes |
| 3 | DRY: direction_from_points (Rule 4) | 1 violation (4 files) | Medium — extract helper, update call sites |
| 4 | DRY: duplicate functions (Rule 4) | 1 violation + 5 warnings | Low-Medium |
| 5 | Dead code audit (Rule 7) | 4 unused modules | Needs port status check first |
| 6 | Concerns: nudger cleanup (Rule 1) | 1 warning | Medium — extract path cleanup module |
| 7 | Error handling style (Rule 8) | 7 warnings | Low — add `.expect()` messages |
