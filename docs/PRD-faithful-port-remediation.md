# PRD: msagl-rust Faithful Port Remediation

## Background

The msagl-rust crate was intended to be a **full faithful port** of Microsoft's MSAGL RectilinearEdgeRouter from C#/TypeScript to Rust. The initial implementation (~6,300 lines, 261 tests) cut corners — simplified algorithms were substituted for the real ones in multiple places. This document specifies exactly what needs to be fixed.

## Guiding Principles

1. **Port the algorithms faithfully from the TypeScript source.** Do not simplify, rewrite, or invent alternatives. If the TS does something, the Rust does the same thing.
2. **Where a well-tested Rust crate already solves a problem** (e.g., `rstar` for R-trees, `ordered-float` for hashable floats), use it instead of porting custom data structures.
3. **Where Rust idioms conflict with C#/TS patterns** (e.g., GC'd object references → index-based arenas, class hierarchies → enums, LinkedList → SlotMap), use idiomatic Rust that preserves the algorithm's behavior.
4. **Port the existing test suites.** The C# repo has 242 procedural routing test methods in `RectilinearTests.cs` and 86 projection solver fixture files. These are the acceptance criteria.

## Reference Sources

- **TypeScript (primary structure):** `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/`
- **C# (authoritative algorithms):** `MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/`
- **C# test suite:** `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs` (5,544 lines, 242 test methods)
- **C# solver fixtures:** `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Resources/Constraints/ProjectionSolver/Data/` (86 files)

---

## Defect List

### CRITICAL — Wrong algorithm, must be rewritten

#### 1. Visibility Graph Generator (REWRITE)

**Current:** `src/routing/visibility_graph_generator.rs` (232 lines) — simplified ray-cast approach. Collects unique coordinates, finds gaps between obstacles, creates segments.

**Should be:** Full sweep-line algorithm from `VisibilityGraphGenerator.ts` (1,013 lines). Two-pass event-driven sweep with:
- EventQueue processing obstacle corner events in sweep order
- RectilinearScanLine maintaining active obstacle sides
- NeighborSides tracking for visibility segment creation
- Reflection event generation for concave visibility regions
- LookaheadScan for perpendicular reflection events
- Proper scan segment creation from obstacle sides and neighbors

**Also rewrite:** `src/routing/segment_intersector.rs` (218 lines) — replace with faithful port of TS `SegmentIntersector.ts`.

**Missing TS files to port:**
- `VisibilityGraphGenerator.ts` (1,013 lines)
- `OpenVertexEvent.ts`, `BasicVertexEvent.ts`, `MiscVertexEvents.ts`
- `BasicReflectionEvent.ts`, `HighReflectionEvent.ts`, `LowReflectionEvent.ts`
- `LookaheadScan.ts`
- `NeighborSides.ts`
- `BasicObstacleSide.ts` (full version with slope tracking)

#### 2. Port Manager / Transient Graph Utility (REWRITE)

**Current:** `src/routing/port_manager.rs` (98 lines) — finds nearest axis-aligned vertex, adds edges.

**Should be:** Full port splicing from `PortManager.ts` (1,052 lines) + `TransientGraphUtility.ts`. This includes:
- ObstaclePort and FreePoint handling
- ObstaclePortEntrance creation
- Edge splitting when a transient vertex falls on an existing edge
- Visibility intersection shortcuts (collinear port detection)
- Full cleanup/restore of transient modifications

**Missing TS files to port:**
- `PortManager.ts` (1,052 lines)
- `TransientGraphUtility.ts`
- `ObstaclePort.ts`
- `FreePoint.ts`
- `ObstaclePortEntrance.ts`
- `SpliceUtility.ts`
- `StaticGraphUtility.ts`

#### 3. VisibilityVertex with VertexEntry[4] (REWRITE)

**Current:** `src/visibility/graph.rs` — VertexData stores Point + flat edge lists. No direction-aware entry tracking.

**Should be:** Each vertex stores `VertexEntries[4]` (one per compass direction N/E/S/W) as in `VisibilityVertexRectilinear.ts`. This is critical for the A* path search to work correctly — it allows tracking the cost of entering a vertex from different directions independently.

**Missing TS files to port:**
- `VisibilityVertexRectilinear.ts`
- `VertexEntry.ts`

#### 4. Path Search — SsstRectilinearPath (REWRITE)

**Current:** `src/routing/path_search.rs` (352 lines) — basic A* with bend counting.

**Should be:** Full direction-aware A* from `SsstRectilinearPath.ts` (523 lines) that:
- Uses VertexEntry[4] per vertex for direction tracking
- Has NextNeighbor[3] optimization (straight, preferred bend, other bend)
- Computes heuristic with estimated bends to target
- Supports multi-source/multi-target via `MsmtRectilinearPath.ts`
- Properly handles cost adjustments for source/target sets

**Missing TS files to port:**
- `SsstRectilinearPath.ts` (523 lines)
- `MsmtRectilinearPath.ts`

---

### IMPORTANT — Missing components that affect quality

#### 5. PathMerger (MISSING)

**Current:** Not implemented.

**Should be:** Port `PathMerger.ts` (160 lines). Removes situations where two paths cross each other more than once and eliminates self-loops. Sits between PathRefiner and CombinatorialNudger in the nudging pipeline.

#### 6. LinkedPoint + LinkedPointSplitter (MISSING)

**Current:** Paths are `Vec<Point>`. Insertion requires shifting.

**Should be:** Port `LinkedPoint.ts` (47 lines) and `LinkedPointSplitter.ts` (152 lines). LinkedPoint is a singly-linked list for path vertices that allows O(1) insertion. LinkedPointSplitter uses an RBTree to efficiently find and insert crossing points between path segments.

**Rust adaptation:** Could use a `Vec<Point>` with insert-by-index if performance is acceptable, OR use a linked list. The key is that `LinkedPointSplitter` must be ported — it handles the intersection logic.

#### 7. FreeSpaceFinder — Full Sweep-Line (EXPAND)

**Current:** `src/routing/nudging/free_space_finder.rs` (180 lines) — direct bounding box computation.

**Should be:** Full sweep-line algorithm from `FreeSpaceFinder.ts` (498 lines) with:
- AxisEdgesContainer in an RBTree for active segment tracking
- AxisEdgeLowPointEvent / AxisEdgeHighPointEvent for sweep events
- Neighbor relationship tracking between axis edges
- Proper obstacle boundary constraint computation during sweep

**Missing TS files to port:**
- `AxisEdgesContainer.ts`
- `AxisEdgeLowPointEvent.ts`
- `AxisEdgeHighPointEvent.ts`

#### 8. ObstacleTree — Full Implementation (EXPAND)

**Current:** `src/routing/obstacle_tree.rs` (87 lines) — basic rstar wrapper with point/rect queries.

**Should be:** Full port of `ObstacleTree.ts` (823 lines) including:
- Overlap detection and ConvexHull merging
- Ancestor set tracking for group hierarchy
- InsideHitTest for determining obstacle containment
- Clump detection and handling
- Full query API used by VisibilityGraphGenerator

**Note:** Group/cluster routing can be deferred, but overlap detection and the full query API are needed.

#### 9. Obstacle Sides — Full Hierarchy (EXPAND)

**Current:** `src/routing/obstacle_side.rs` (15 lines) — simple struct with start/end points.

**Should be:** Full port of `BasicObstacleSide.ts` with:
- LowObstacleSide and HighObstacleSide subtypes
- Slope and SlopeInverse computation
- Proper polyline traversal (clockwise/counter-clockwise depending on side type)
- Connection to obstacle for back-references

#### 10. ScanSegmentTree — RBTree-based (EXPAND)

**Current:** `src/routing/scan_segment.rs` — BTreeMap keyed by perpendicular coordinate.

**Should be:** Port of `ScanSegmentTree.ts` (311 lines) with:
- RBTree (or BTreeMap) with proper segment ordering
- `FindLowestIntersector` / `FindHighestIntersector` range queries
- `MergeSegments` for combining adjacent segments
- Segment next/prev navigation

---

### MODERATE — Incomplete but partially working

#### 11. Projection Solver — 6 Ignored Fixtures (FIX)

**Current:** 80/86 fixture tests pass, 6 ignored.

**Fix:** Debug the 6 failures:
- 4 cycle-handling fixtures (unsatisfiable constraint cycles with equality constraints)
- 2 QPSC convergence fixtures (extreme weight ratios, large variable counts)

Compare algorithm behavior against C# step-by-step to find divergence.

#### 12. EventQueue — Full Event Types (EXPAND)

**Current:** `src/routing/event_queue.rs` — simple Open/Close events.

**Should be:** Full event hierarchy from TS:
- `OpenVertexEvent` (obstacle entering sweep)
- `CloseVertexEvent` (obstacle leaving sweep)
- `HighBendVertexEvent`, `LowBendVertexEvent` (obstacle corner bends)
- `BasicReflectionEvent`, `HighReflectionEvent`, `LowReflectionEvent`
- Proper priority ordering: perpendicular coord → event type → scan coord

#### 13. RectilinearScanLine — Full API (EXPAND)

**Current:** `src/routing/scan_line.rs` (94 lines) — BTreeMap with low/high neighbor queries.

**Should be:** Full port of `RectilinearScanLine.ts` (204 lines) with:
- Side comparison using `ScanLineIntersectSide` (dynamic intersection point)
- Proper neighbor traversal skipping group boundaries
- Position tracking for insert/remove operations

---

### LOW — Missing but can be deferred

#### 14. Group/Cluster Routing (DEFER)

`GroupBoundaryCrossing.ts`, `GroupBoundaryCrossingMap.ts`, ancestor set handling. Spec already excludes this.

#### 15. SparseVisibilityGraphGenerator (DEFER)

Alternative VG generation strategy. Spec already excludes this.

#### 16. RectilinearInteractiveEditor (DEFER)

Incremental re-routing. Spec already excludes this.

#### 17. Non-Floating Ports (DEFER)

`CurvePort`, `RelativeFloatingPort`, `ClusterBoundaryPort`. Spec already excludes these.

---

## Acceptance Criteria

1. **All 86 projection solver fixture tests pass** (currently 80/86)
2. **Port the 242 procedural routing tests** from `RectilinearTests.cs` as Rust `#[test]` functions. Build a C# dump harness to serialize inputs/outputs as JSON golden baselines.
3. **Visibility graph matches TS output** for identical obstacle layouts (write comparison tests)
4. **Path search produces same paths as TS** for identical inputs
5. **Nudged paths match TS output** within floating-point tolerance
6. **All TS files in `routing/rectilinear/` have a Rust equivalent** (with the exceptions listed in "DEFER" above)
7. `cargo test` passes, `cargo clippy -- -D warnings` is clean

## File Mapping

Every TS file below should have a corresponding Rust implementation (unless marked DEFER):

```
TS routing/rectilinear/                    → Rust src/routing/
  RectilinearEdgeRouter.ts                 → rectilinear_edge_router.rs ✓ (EXPAND)
  VisibilityGraphGenerator.ts              → visibility_graph_generator.rs (REWRITE)
  obstacle.ts                              → obstacle.rs (EXPAND)
  ObstacleTree.ts                          → obstacle_tree.rs (EXPAND)
  BasicObstacleSide.ts                     → obstacle_side.rs (EXPAND)
  ScanSegment.ts                           → scan_segment.rs (EXPAND)
  ScanSegmentTree.ts                       → scan_segment.rs (EXPAND)
  ScanDirection.ts                         → scan_direction.rs ✓
  RectilinearScanLine.ts                   → scan_line.rs (EXPAND)
  EventQueue.ts                            → event_queue.rs (EXPAND)
  OpenVertexEvent.ts                       → event_queue.rs (EXPAND)
  BasicVertexEvent.ts                      → event_queue.rs (EXPAND)
  MiscVertexEvents.ts                      → event_queue.rs (EXPAND)
  BasicReflectionEvent.ts                  → event_queue.rs (ADD)
  HighReflectionEvent.ts                   → event_queue.rs (ADD)
  LowReflectionEvent.ts                    → event_queue.rs (ADD)
  LookaheadScan.ts                         → lookahead_scan.rs (ADD)
  NeighborSides.ts                         → neighbor_sides.rs (ADD)
  PortManager.ts                           → port_manager.rs (REWRITE)
  TransientGraphUtility.ts                 → transient_graph_utility.rs (ADD)
  SpliceUtility.ts                         → splice_utility.rs (ADD)
  StaticGraphUtility.ts                    → static_graph_utility.rs (ADD)
  SsstRectilinearPath.ts                   → path_search.rs (REWRITE)
  MsmtRectilinearPath.ts                   → path_search.rs (ADD)
  VertexEntry.ts                           → vertex_entry.rs (ADD)
  FreePoint.ts                             → port.rs (EXPAND)
  ObstaclePort.ts                          → port.rs (EXPAND)
  ObstaclePortEntrance.ts                  → port.rs (ADD)
  OverlapConvexHull.ts                     → obstacle.rs (ADD)
  SegmentIntersector.ts                    → segment_intersector.rs (REWRITE)
  PointComparer.ts                         → (use existing point_comparer.rs)

TS routing/rectilinear/nudging/            → Rust src/routing/nudging/
  Nudger.ts                                → nudger.rs (EXPAND)
  PathRefiner.ts                           → path_refiner.rs ✓ (MINOR FIXES)
  CombinatorialNudger.ts                   → combinatorial_nudger.rs ✓ (MINOR FIXES)
  FreeSpaceFinder.ts                       → free_space_finder.rs (EXPAND)
  LongestNudgedSegment.ts                  → longest_nudged_segment.rs ✓
  AxisEdge.ts                              → axis_edge.rs ✓
  PathEdge.ts                              → path_edge.rs ✓
  Path.ts                                  → (use Vec<Point> — idiomatic Rust)
  StaircaseRemover.ts                      → staircase_remover.rs ✓
  PathMerger.ts                            → path_merger.rs (ADD)
  LinkedPoint.ts                           → linked_point.rs (ADD)
  LinkedPointSplitter.ts                   → linked_point_splitter.rs (ADD)
  AxisEdgesContainer.ts                    → free_space_finder.rs (ADD)
  AxisEdgeLowPointEvent.ts                 → free_space_finder.rs (ADD)
  AxisEdgeHighPointEvent.ts                → free_space_finder.rs (ADD)

TS routing/visibility/                     → Rust src/visibility/
  VisibilityGraph.ts                       → graph.rs (EXPAND)
  VisibilityVertex.ts                      → graph.rs (EXPAND)
  VisibilityVertexRectilinear.ts           → graph.rs (ADD VertexEntry[4])
  VisibilityEdge.ts                        → edge.rs ✓

DEFER (per spec exclusions):
  SparseVisibilityGraphGenerator.ts        → DEFER
  RectilinearInteractiveEditor.ts          → DEFER
  GroupBoundaryCrossing.ts                 → DEFER
  GroupBoundaryCrossingMap.ts              → DEFER
  ClusterBoundaryPort handling             → DEFER
```

## Rust Idiom Adaptations (Approved Deviations)

These are NOT simplifications — they're language adaptations that preserve algorithmic behavior:

| TS/C# Pattern | Rust Adaptation | Why |
|---------------|-----------------|-----|
| Object references with GC | Index-based arenas (`Vec<T>` + `usize` indices) | No GC in Rust |
| Class hierarchies (`extends`) | Enums with variants OR trait objects | Rust has no inheritance |
| `LinkedList<T>` with node refs | `SlotMap` or `Vec` with index links | Ownership model |
| `RBTree<T>` with cursor nav | `BTreeMap` with `range()` queries | stdlib equivalent |
| `Dictionary<Point, V>` | `HashMap<Point, V>` with `OrderedFloat` | Need hashable floats |
| `abstract class` + subclasses | Enum dispatch or trait + impl | No abstract classes |
| `null` references | `Option<T>` | Rust idiom |
| Mutable shared state | `&mut self` methods on owning struct | Borrow checker |

## Estimated Scope

The faithful port should be ~12,000-14,000 lines of Rust (matching the original spec estimate). Current: ~6,300 lines. Delta: ~6,000-8,000 lines of new/rewritten code.
