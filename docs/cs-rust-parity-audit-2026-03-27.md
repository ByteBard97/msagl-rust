# C# → Rust Parity Audit
**Date:** 2026-03-27
**Scope:** All 47 C# classes in `Routing/Rectilinear/` + `Nudging/`
**Method:** 5 parallel explorer agents reading both C# and Rust source

Verdicts: **MATCH** = faithful port | **PARTIAL** = skeleton + gaps | **DIVERGED** = different architecture | **STATELESS** = C# class state not carried over | **MISSING** = no Rust equivalent

---

## Routing Core

| C# Class | Rust File(s) | Verdict | Notes |
|---|---|---|---|
| RectilinearEdgeRouter.cs | rectilinear_edge_router.rs | **DIVERGED** | C# is stateful OOP with `Add/Update/RemoveObstacle()` for incremental updates; Rust is a one-shot builder (`new()` → `run()`). GraphGenerator, PortManager, ShapeToObstacleMap, AncestorSets are created locally per run, not owned. |
| PortManager.cs | port_manager.rs, port_splice.rs | **PARTIAL** | FullPortManager struct exists with correct fields but is never instantiated — router calls stateless unit struct. ~40% of methods missing: `CreateObstaclePortEntrancesFromPoints()` (300+ lines of boundary intersection), all group/ancestor management. |
| TransientGraphUtility.cs | transient_graph_utility.rs, _helpers.rs, _extend.rs | **PARTIAL** | Core operations (add/find/remove vertex+edge) present. Missing: `ExtendEdgeChain()` (100+ lines, deferred per comments), `FindNearestPerpendicularOrContainingEdge()`, `SpliceGroupBoundaryCrossings()`. GraphGenerator not owned — passed as parameter. |
| ObstacleTree.cs | obstacle_tree.rs, _queries.rs, _overlap.rs | **STATELESS** | C# is a persistent stateful hierarchy (`RectangleNode Root`, `AncestorSets` dictionary, `overlapPairs` accumulator) supporting incremental updates. Rust is an immutable query wrapper (`Vec<Obstacle>` + R-tree), built once and queried. Clump/convex hull/group-growth methods entirely absent from the struct (delegated to `resolve_overlaps()`). |
| Obstacle.cs | obstacle.rs | **MATCH** | All fields and methods present. Option<> for null, Vec<usize> for clump indices (arena). `remove_close_vertices` bug previously fixed. |

---

## Visibility Graph Generation

| C# Class | Rust File(s) | Verdict | Notes |
|---|---|---|---|
| VisibilityGraphGenerator.cs | visibility_graph_generator.rs | **DIVERGED** | C# class owns ScanLine, EventQueue, ObstacleTree, H/V ScanSegmentTrees, VisibilityGraph as instance fields. Rust is a free function — all state created locally and dropped. PortManager can never access HScanSegments/VScanSegments. |
| FullVisibilityGraphGenerator.cs | visibility_graph_generator.rs | **MERGED** | Merged into same file, no subclass distinction, `hintScanSegment` merge optimization not present. |
| RectilinearScanLine.cs | scan_line.rs | **PARTIAL** | BTreeMap-based; correct. Missing `linePositionAtLastInsertOrRemove` (debug-only, low priority). |
| ScanSegment.cs | scan_segment.rs | **PARTIAL** | Core fields present. Missing `NeedStartOverlapVertex`/`NeedEndOverlapVertex` (needed for group handling), `NextSegment` (sparse VG). |
| ScanSegmentTree.cs | scan_segment.rs | **MATCH** | Elegant nested BTreeMap design. |
| SegmentIntersector.cs | segment_intersector.rs | **MATCH** | Faithful sweep-line with BTreeMap for active verticals. 13 tests. |
| EventQueue.cs | event_queue.rs | **MATCH** | Perfect Ord trait port of C# comparator. |
| LookaheadScan.cs | lookahead_scan.rs | **PARTIAL** | Missing `staleSites` deferred-removal optimization (batch removal). |
| NeighborSides.cs | neighbor_sides.rs | **PARTIAL** | Missing `GroupSideInterveningBeforeLowNeighbor`/`BeforeHighNeighbor` (groups deferred). |
| ScanDirection.cs | scan_direction.rs | **MATCH** | Idiomatic Rust improvement (Copy struct). |
| SparseVisibilityGraphGenerator.cs | — | **MISSING** | Explicitly deferred per CLAUDE.md. |

---

## Port & Obstacle Side Infrastructure

| C# Class | Rust File(s) | Verdict | Notes |
|---|---|---|---|
| ObstaclePort.cs | port.rs (ObstaclePort) | **MATCH** | All fields and methods; index-based references. |
| ObstaclePortEntrance.cs | port.rs (ObstaclePortEntrance) | **PARTIAL** | Core methods present. Constructor simplified — C# calls `ObstacleTree.CreateMaxVisibilitySegment()`; Rust assigns zeros and expects caller to set visibility fields. Group fields deferred. |
| FreePoint.cs | free_point.rs | **MATCH** | All fields and methods including `extend_edge_chain`, `get_segment_and_crossings`, `add_oob_edges`. |
| BasicObstacleSide.cs + Low/High | obstacle_side.rs | **MATCH** | C# class hierarchy → Rust SideType enum. Slope computation faithful. |
| GroupBoundaryCrossing.cs | group_boundary_crossing.rs | **MATCH** | |
| GroupBoundaryCrossingMap.cs | group_boundary_crossing.rs | **MATCH** | |
| PointAndCrossings.cs | group_boundary_crossing.rs | **MATCH** | |
| PointAndCrossingsList.cs | group_boundary_crossing.rs | **MATCH** | All methods including pop, merge_from, trim, to_crossing_array. |
| OverlapConvexHull.cs | overlap_convex_hull.rs | **MATCH** | Obstacle refs → indices. |
| SpliceUtility.cs | splice_utility.rs | **MATCH** | |
| StaticGraphUtility.cs | static_graph_utility.rs | **PARTIAL** | C# is 16K+ lines; Rust has ~7 core methods. Full audit would require reading everything. Core methods (EdgeDirection, FindAdjacentVertex, FindAdjacentEdge, etc.) are faithful. |
| VertexEntry.cs | vertex_entry.rs | **MATCH** | Arena indices. All fields. |
| PointComparer.cs | point_comparer.rs (GeomConstants) | **PARTIAL** | Core comparison logic present. Direction utilities (GetDirections, IsPureDirection) moved to CompassDirection elsewhere. |

---

## Path Search & Events

| C# Class | Rust File(s) | Verdict | Notes |
|---|---|---|---|
| SsstRectilinearPath.cs | path_search.rs | **MATCH** | Arena-based SearchEntry, BinaryHeap, faithful A*. |
| MsmtRectilinearPath.cs | msmt_path.rs | **MATCH** | Multi-source/target routing preserved. |
| 8 vertex/reflection event classes | event_queue.rs | **MATCH** | C# class hierarchy → Rust enum (approved pattern). All fields present. |
| VisibilityVertexRectilinear.cs | visibility/graph.rs (VertexData) | **PARTIAL** | `vertex_entries: [Option<VertexEntryIndex>; 4]` present in VertexData. Methods inlined at call sites. |
| ScanSegmentVector.cs | — | **MISSING** | Sparse VG, deferred. |
| ScanSegmentVectorItem.cs | — | **MISSING** | Sparse VG, deferred. |
| AxisCoordinateEvent.cs | — | **MISSING** | Not in event_queue.rs. Likely deferred. |
| Clump.cs | — | **MISSING** | Overlap management deferred. |

---

## Nudging

| C# Class | Rust File(s) | Verdict | Notes |
|---|---|---|---|
| Nudger.cs | nudger.rs | **MATCH** | Functional refactor; algorithm line-for-line identical. |
| AxisEdge.cs | axis_edge.rs | **MATCH** | Index-based. Missing `setOfLongestSegs` field (reconstructed during grouping). |
| AxisEdgesContainer.cs | — | **DIVERGED** | Merged into sweep-line; no standalone struct. |
| PathEdge.cs | path_edge.rs | **MATCH** | All fields including linked list next/prev as indices. |
| LongestNudgedSegment.cs | longest_nudged_segment.rs | **MATCH** | Faithful algorithm. ID-based edge refs. |
| CombinatorialNudger.cs | combinatorial_nudger.rs | **MATCH** | Functional refactor; walk-ahead algorithm faithful. |
| FreeSpaceFinder.cs | free_space_finder.rs + _sweep.rs | **PARTIAL** | Bounds computation: faithful sweep-line. Neighbor discovery: simplified O(n·m) fallback (known limitation, per CLAUDE.md). |
| PathMerger.cs | path_merger.rs | **MATCH** | Arena-based; algorithm faithful. |
| PathRefiner.cs | path_refiner.rs | **MATCH** | BTreeMap for perpendicular buckets; faithful. |
| StaircaseRemover.cs | staircase_remover.rs | **MATCH** | R-tree preserved; algorithm faithful. |
| LinkedPoint.cs | linked_point.rs | **MATCH** | Arena-based singly-linked list. |
| LinkedPointSplitter.cs | linked_point_splitter.rs | **MATCH** | BTreeMap for active segments; sweep faithful. |
| Path.cs | — | **ADAPTED** | Simplified to `Vec<Point>` throughout. Acceptable. |
| SegWithIndex.cs | — | **ADAPTED** | Inlined into staircase_remover.rs. |

---

## Summary by Category

### Fully Faithful (MATCH) — 25 classes
Nudging pipeline, path search, event system, obstacle side/port/group infrastructure, scan segments, vertex entry. These are well-ported.

### Partial — 11 classes
Missing group-related fields (deferred), some complex algorithms deferred with comments. StaticGraphUtility vastly incomplete (C# is 16K lines). ObstaclePortEntrance constructor simplified. LookaheadScan missing batch optimization.

### Architecturally Diverged / Stateless — 3 classes
| Class | Problem |
|---|---|
| RectilinearEdgeRouter | Stateful OOP → one-shot builder. Incremental obstacle updates impossible. |
| VisibilityGraphGenerator | Class with owned state → free function. H/V ScanSegmentTrees dropped. PortManager blind. |
| ObstacleTree | Stateful hierarchy with ancestor tracking → immutable query wrapper. Clump/hull/group-growth logic detached. |

### Missing (Deferred) — 6 classes
SparseVisibilityGraphGenerator, ScanSegmentVector, ScanSegmentVectorItem, AxisCoordinateEvent, Clump, AxisEdgesContainer.

---

## Critical Gaps (Non-Deferred)

These are gaps that affect current functionality, not deferred features:

1. **PortManager never instantiated** — `FullPortManager` has the right fields but the router calls the stateless unit struct. The ~300 lines of port entrance creation logic (`CreateObstaclePortEntrancesFromPoints`) is simply absent.

2. **VisibilityGraphGenerator drops H/V scan segment trees** — After `generate_visibility_graph()` returns, the segment trees are gone. PortManager cannot access them for splice operations. This is the root of the parameter-threading cascade.

3. **ObstaclePortEntrance constructor doesn't call CreateMaxVisibilitySegment** — The max visibility segment fields are set to zero; the caller is expected to fill them in, but this path isn't clearly established.

4. **TransientGraphUtility missing ExtendEdgeChain** — Marked deferred in comments but needed for port visibility rays to connect to the VG. This is directly related to the 2 failing tests (C5 in the bug tracker).

5. **StaticGraphUtility ~16K lines → ~100 lines** — It's unclear how much of the C# utility has been ported. Likely the missing methods are called from elsewhere and silently fail or are never reached.
