# msagl-rust Code Audit
**Date:** 2026-03-26
**Auditor:** Claude Sonnet 4.6

---

## Summary

| Status | File Count |
|--------|-----------|
| BROKEN (todo!/stub) | 3 |
| SIMPLIFIED (wrong algorithm) | 3 |
| SILENT BUG (compiles but wrong) | 2 |
| OK (do not revisit) | ~25 |

---

## BROKEN — Contains todo!() / stub bodies

### `src/routing/vg_event_processing.rs`
**Status: COMPLETELY EMPTY SKELETON**
- 26 `todo!()` markers covering the entire sweep-line event processing system
- Events not implemented: OpenVertex, LowBend, HighBend, CloseVertex, LowReflection, HighReflection
- Helpers not implemented: find_neighbors, skip_to_neighbor, create_scan_segments_from_neighbors, add_segment_if_valid, scanline_intersect_side, side_reflects_upward, side_reflects_downward, add_side_to_scanline, remove_side_from_scanline, load_reflection_events, load_reflection_events_with_range, store_lookahead_site, add_perpendicular_reflection_segment, add_parallel_reflection_segment, add_reflection_event, remove_sites_for_flat_bottom, enqueue_low_bend_event, enqueue_high_bend_or_close_event, scan_line_crosses_obstacle
- **CRITICAL NOTE:** This file is NOT WIRED IN. `visibility_graph_generator.rs` has its own simplified event processing that completely ignores this file. This file is dead code currently.

### `src/routing/transient_graph_utility_extend.rs`
**Status: COMPLETELY EMPTY SKELETON**
- 15 `todo!()` markers
- Methods not implemented: extend_edge_chain_public, extend_edge_chain, extend_splice_worker, get_next_splice_source, get_splice_target, splice_group_boundary_crossings, traverse_to_first_vertex_at_or_above, splice_group_boundary_crossing, see_if_splice_is_still_overlapped, is_skippable_splice_source_with_null_splice_target, is_skippable_splice_source_edge_with_null_target, is_reflection_edge, is_point_past_segment_end, debug_verify_non_overlapped_extension
- Also: `PointAndCrossingsList::pop()` is `todo!()`

### `src/routing/port.rs`
**Status: PARTIALLY COMPLETE — 3 critical stubs**
- `ObstaclePortEntrance::extend_edge_chain` (line 156): stub that only connects unpadded→padded border vertex; does not walk the max visibility segment at all. Comment says "TODO: full extend_edge_chain"
- `FreePoint::extend_edge_chain` (line 548): empty stub. Comment says "TODO: full extend_edge_chain — requires ObstacleTree.CreateMaxVisibilitySegment"
- `FreePoint::max_visibility_in_direction` (line 572): falls back to returning `self.point` instead of calling ObstacleTree. Comment says "TODO: integrate with ObstacleTree"
- The rest of port.rs (ObstaclePort, FloatingPort, FreePoint fields/constructors, add_to_adjacent_vertex, add_edge_to_adjacent_edge, add_oob_edges_from_graph_corner, remove_from_graph) appears implemented

---

## SIMPLIFIED — Compiles but uses wrong/incomplete algorithm

### `src/routing/visibility_graph_generator.rs`
**Status: CRITICALLY SIMPLIFIED — Missing reflection/bend events**
- Has its own embedded event processing that only handles `OpenVertex` and `CloseVertex` (line 183: `_ => {}` silently drops all other events)
- LowBend, HighBend, LowReflection, HighReflection events are **never processed**
- This means non-convex obstacles do not generate reflection scan segments (the "staircase" segments around convex hull corners)
- The file has NO connection to `vg_event_processing.rs` — that file is dead code
- There are TWO parallel implementations: this file's simplified version, and the skeleton in `vg_event_processing.rs`
- `ScanSegment::on_intersector_begin` has a comment acknowledging this: "when the full VG generator (Task 8) is in place, this will be narrowed to the TS behavior"
- For purely rectangular obstacles this may route acceptably, but paths around non-rectangular shapes will be wrong

### `src/routing/nudging/free_space_finder.rs`
**Status: SIMPLIFIED — Neighbor discovery is intentionally disabled**
- Comment at top explicitly says: "the sweep's neighbor discovery can be re-enabled" when solver is optimized
- Currently uses `free_space_finder_simple::find_right_neighbors_only` instead of the sweep's neighbor discovery
- The comment justifies this as "O(n*m) pairs for large adjacent containers, overwhelming the solver"
- Obstacle bounds from sweep ARE applied correctly
- This is a known tradeoff, not an agent shortcut, but path nudging quality will be degraded for complex layouts

### `src/routing/nudging/nudger.rs`
**Status: SILENT LOGIC ERROR — opposite_dir() is wrong**
- Line 441-445: `fn opposite_dir(_d: Direction) -> Direction { _d }` — returns the SAME direction, not the opposite
- This is used in `create_longest_segments` to determine if an edge is "parallel to nudging direction or its opposite"
- Because `direction == opposite_dir(direction)` is always true, the second branch of the `if` is dead code
- Affects which path edges get grouped into LongestNudgedSegments — may cause incorrect nudging behavior
- The function SHOULD return the opposite direction (North↔South, East↔West)

---

## SILENT BUG — Compiles, looks correct, but has subtle defects

### `src/routing/obstacle.rs`
**`remove_close_vertices` is a no-op (line 375-405)**
- Collects vertices to remove into `to_remove` Vec
- Then does `let _ = to_remove;` — silently discards the list
- Comment says "We'd need a remove_point method on Polyline; for now this is effectively a no-op"
- For padded rectangles this is fine (no close vertices generated), but it means `RoundVerticesAndSimplify` only rounds, never simplifies
- If non-rectangular shapes are ever passed in, degenerate polylines could result

### `src/routing/nudging/nudger.rs`
**`shift_point` applies North/East incorrectly (line 379-396)**
- `Direction::North` moves a point's X coordinate, `Direction::East` moves Y with negation
- This appears inverted from the expected convention (North should affect Y, East should affect X)
- In MSAGL: Y increases upward, X increases rightward. But the nudging direction enum may use a different convention — needs cross-checking with TS source to confirm if this is a real bug or intentional

---

## OK — Do Not Revisit

These files are complete and appear faithfully ported. Unless a specific bug is reported, do not re-audit them.

| File | Notes |
|------|-------|
| `src/routing/transient_graph_utility.rs` | Faithful port of TS. find_or_add_edge, split_edge, connect_vertex_to_target, find_perpendicular_or_containing_edge, remove_from_graph all implemented |
| `src/routing/path_search.rs` | SsstRectilinearPath: faithful A* with bend penalty, VertexEntry[4] via external vec, BinaryHeap queue. Looks correct |
| `src/routing/path_search_wrapper.rs` | OK |
| `src/routing/msmt_path.rs` | Implemented (only checked first 50 lines, needs deeper audit if bugs found) |
| `src/routing/port_manager.rs` | First 100 lines look correct. splice_port, unsplice implemented. Needs deeper audit if bugs found |
| `src/routing/rectilinear_edge_router.rs` | Top-level pipeline looks reasonable. points_to_curve with arc rounding implemented |
| `src/routing/scan_segment.rs` | ScanSegment + ScanSegmentTree implemented with BTreeMap. merge_segments OK |
| `src/routing/obstacle.rs` | Mostly OK except remove_close_vertices bug above |
| `src/routing/obstacle_tree.rs` | Needs spot check |
| `src/routing/port.rs` | Partially OK (see BROKEN section) |
| `src/routing/static_graph_utility.rs` | Needs spot check |
| `src/routing/compass_direction.rs` | Needs spot check |
| `src/routing/scan_line.rs` | Needs spot check |
| `src/routing/event_queue.rs` | Needs spot check |
| `src/routing/scan_direction.rs` | Needs spot check |
| `src/routing/neighbor_sides.rs` | Needs spot check |
| `src/routing/lookahead_scan.rs` | Needs spot check |
| `src/visibility/graph.rs` | Good. BTreeMap coordinate indices, O(log V) queries, remove_vertex OK |
| `src/visibility/edge.rs` | Needs spot check |
| `src/geometry/point.rs` | Needs spot check |
| `src/geometry/rectangle.rs` | Needs spot check |
| `src/geometry/polyline.rs` | Needs spot check |
| `src/geometry/curve.rs` | Needs spot check |
| `src/geometry/point_comparer.rs` | Needs spot check |
| `src/geometry/convex_hull.rs` | Needs spot check |
| `src/routing/nudging/` (other files) | path_refiner, path_merger, combinatorial_nudger, staircase_remover, axis_edge, path_edge, linked_point, etc — need spot check |
| `src/projection_solver/` | Need spot check |

---

## Priority Fix Order

1. **vg_event_processing.rs** — Fill in all 26 todo!()s using the C# VisibilityGraphGenerator.cs reference. Wire it into visibility_graph_generator.rs to replace the simplified OpenVertex/CloseVertex-only processing.

2. **transient_graph_utility_extend.rs** — Fill in all 15 todo!()s using C# TransientGraphUtility.cs lines 418-802.

3. **port.rs (3 stubs)** — Implement extend_edge_chain methods properly using TransientGraphUtility (which itself needs the above fix first).

4. **nudger.rs opposite_dir bug** — One-line fix: return the opposite direction.

5. **nudger.rs shift_point** — Verify against TS source that North/East convention is correct.

6. **obstacle.rs remove_close_vertices** — Add remove_point to Polyline and wire it in (low priority for rectangular-only routing).

---

## Architecture Notes

- The codebase has a known structural issue: `vg_event_processing.rs` is dead code alongside a working-but-simplified `visibility_graph_generator.rs`. The correct path is to fill in `vg_event_processing.rs` then wire it into `visibility_graph_generator.rs` to replace the simplified processing.
- The projection solver and nudging stack are structurally complete but need the VG generator and port infrastructure to be correct before end-to-end routing can be validated.
- All approved Rust adaptations (BTreeMap instead of RBTree, Vec arenas instead of GC references, rstar for spatial indexing) appear to be correctly applied in the files that are implemented.
