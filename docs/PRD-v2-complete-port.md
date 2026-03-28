# PRD v2: msagl-rust Complete Port

**Supersedes:** `docs/PRD-faithful-port-remediation.md`
**Date:** 2026-03-28
**Trigger:** Full audit of C# public API and all 242 RectilinearTests.cs test methods revealed
two entire gap categories the original PRD missed.

---

## What Changed Since the Original PRD

The original PRD focused on internal algorithm correctness (VG sweep-line, path search, nudging,
solver). It never audited:

1. **The public API surface of `RectilinearEdgeRouter`** — 10 methods/properties in C# are missing
   from Rust, including `UpdateObstacle`, `RemoveEdgeGeometryToRoute`, and three config flags.
2. **The WASM binding** — written as a one-shot stateless function; C# and TS both expose a
   long-lived stateful object.
3. **All 242 C# test methods by category** — enumerating them reveals 43 group tests, 10 waypoint
   tests, and 13 port-type tests that require features we have not implemented at all.

---

## C# Test Inventory (242 Tests)

| Category | Count | Key Features Required |
|----------|-------|-----------------------|
| BASIC_ROUTING | 83 | Floating ports, absolute ports, padding, bend penalty |
| STRESS | 91 | Reflections, overlaps, convex hulls, staircases, splicing |
| GROUPS | 43 | Group hierarchy, boundary crossings, ancestor tracking |
| PORT_TYPE | 13 | Absolute/relative ports, free points, `RemoveEdgeGeometryToRoute` |
| WAYPOINTS | 10 | Waypoint constraints threaded through path search |
| STATEFUL_UPDATE | 2 | `UpdateObstacle`, `RemoveObstacle`, incremental VG rebuild |

**Current Rust port can pass at most:** BASIC_ROUTING (partially) and STRESS (partially).
**Blocked entirely:** GROUPS (43), WAYPOINTS (10), STATEFUL_UPDATE (2), PORT_TYPE partially.

---

## Defect Inventory

### A. STATEFUL PUBLIC API (NEW — not in original PRD)

**Severity: CRITICAL** — blocks 8 C# tests, blocks interactive drag-routing in browser.

Missing from `RectilinearEdgeRouter`:

| Missing Item | C# Method/Property | Blocks Tests |
|---|---|---|
| Move obstacle | `UpdateObstacle()` / `UpdateObstacles()` | MoveOneObstacle_* × 4 |
| Remove edge from queue | `RemoveEdgeGeometryToRoute()` | Update_FreePort, MoveOneObstacle_* |
| Batch add/remove | `AddObstacles()`, `RemoveObstacles()` | Performance only |
| Deferred VG rebuild | `AddObstacleWithoutRebuild()` + `RebuildTreeAndGraph()` | Performance only |
| Remove all | `RemoveAllObstacles()`, `Clear()` | Reset use case |
| Route to center | `RouteToCenterOfObstacles` property | Group tests, verifier |
| Limit splice rect | `LimitPortVisibilitySpliceToEndpointBoundingBox` | GroupTest_Simple_NoGroup_PortSplice_LimitRect |
| Staircase toggle | `RemoveStaircases` property | 4 staircase stress tests |
| Obstacle introspection | `Obstacles` property | Test verifier infrastructure |
| Edge introspection | `EdgeGeometriesToRoute` property | Test verifier infrastructure |

**Architecture gap:** Rust rebuilds the VG on every `add_shape`/`remove_shape` call.
C# batches via `*WithoutRebuild` + one `RebuildTreeAndGraph()`. For N obstacle additions
this is O(N) rebuilds vs O(1).

### B. WASM STATEFUL ROUTER (NEW — not in original PRD)

**Severity: HIGH** — blocks real-time drag-routing in browser.

Current WASM API is one-shot stateless:
```rust
pub fn route_edges(input: RoutingInput) -> Result<RoutingOutput, JsError>
// Router is created, used, and dropped on every call.
```

C# and TS both expose a long-lived object. Required WASM API:
```rust
#[wasm_bindgen]
pub struct Router { /* RouterSession survives between calls */ }

impl Router {
    pub fn new(padding: f64, edge_separation: f64) -> Router
    pub fn add_obstacle(x, y, w, h) -> usize       // returns handle
    pub fn move_obstacle(id: usize, x, y, w, h)
    pub fn remove_obstacle(id: usize)
    pub fn add_edge(src_obstacle: usize, tgt_obstacle: usize) -> usize
    pub fn remove_edge(id: usize)
    pub fn route()                                  // routes all pending edges
    pub fn route_affected(obstacle_id: usize)       // incremental re-route
    pub fn get_paths() -> JsValue
}
```

Keeping `route_edges()` as a one-shot convenience function is fine; the stateful
`Router` must exist alongside it.

### C. ALGORITHM GAPS (from original PRD — updated status)

**C1. VisibilityVertex VertexEntry[4] — NOT DONE**
Each vertex must store `VertexEntries[4]` (N/E/S/W), one entry per compass direction.
Path search tracks best cost entering from each direction independently.
Without this the path search explores incorrect directions.
- File: `src/visibility/graph.rs`
- C# reference: `VisibilityVertexRectilinear.cs`

**C2. VG Generator — reflection events incomplete**
Sweep-line infrastructure is now present. Reflection event generation
(`BasicReflectionEvent`, `HighReflectionEvent`, `LowReflectionEvent`) is skeletal.
`LookaheadScan` uses linear iteration instead of RBTree range queries.
- File: `src/routing/visibility_graph_generator.rs`, `src/routing/lookahead_scan.rs`
- Blocks: 20+ reflection stress tests, any concave obstacle configuration.

**C3. Path Search — different algorithm**
`src/routing/msmt_path.rs` uses standard A* with bend counting.
C# uses direction-aware SSST (Single Source Shortest Tree) with `VertexEntry[4]`,
`NextNeighbor[3]` optimization, and multi-stage cost bounds.
Without VertexEntry[4] (C1), this cannot be fixed.
- File: `src/routing/msmt_path.rs`
- C# reference: `SsstRectilinearPath.cs`, `MsmtRectilinearPath.cs`

**C4. Groups — entirely missing**
43 C# tests require group/cluster routing. Not started.
- Missing files: `group_boundary_crossing_map.rs`, group hierarchy in `obstacle_tree.rs`
- C# reference: `GroupBoundaryCrossing.cs`, `GroupBoundaryCrossingMap.cs`
- Deferred in original PRD; now quantified as 43/242 tests (18%).

**C5. Waypoints — entirely missing**
10 C# tests pass explicit waypoint constraints that the router must route through.
No waypoint concept exists in Rust structs or API.
- Deferred in original PRD; now quantified as 10/242 tests (4%).

**C6. Non-rectangular obstacles — incomplete**
`ObstaclePort` class missing entirely. Port entrance logic for non-rectangular
shapes is scattered and incomplete. Affects reflected routes and non-axis-aligned walls.
- File: `src/routing/port_manager_entrances.rs` (stub functions present, not called)

**C7. SparseVisibilityGraphGenerator — missing**
Memory optimization for large graphs. Not a correctness issue; deferred.

**C8. RectilinearInteractiveEditor — missing**
UI helper methods. Out of scope until groups and waypoints done.

**C9. Projection Solver — 6 ignored fixtures**
6/86 fixture tests still ignored:
- 4 cycle fixtures: unsatisfiable equality constraint cycles
- 2 QPSC fixtures: extreme weight ratio convergence (1 to 1e6)

---

## Priority Order

Work in this sequence. Each item unblocks the next.

### Priority 1 — Stateful Public API (unblocks 8 C# tests + WASM)
1a. Add `UpdateObstacle`, `RemoveEdgeGeometryToRoute`, `Clear()`, `RouteToCenterOfObstacles`,
    `LimitPortVisibilitySpliceToEndpointBoundingBox`, `RemoveStaircases` to `RectilinearEdgeRouter`
1b. Add `*WithoutRebuild` variants + explicit `rebuild()` for deferred VG reconstruction
1c. Port the 7 C# stateful tests as Rust `#[test]` functions

### Priority 2 — WASM Stateful Router (unblocks browser drag-routing)
2a. Add `Router` struct to `crates/msagl-wasm/src/lib.rs` wrapping `RouterSession`
2b. Expose `add_obstacle`, `move_obstacle`, `remove_obstacle`, `add_edge`, `remove_edge`,
    `route`, `route_affected`, `get_paths`
2c. Keep existing `route_edges` one-shot function

### Priority 3 — VertexEntry[4] (unblocks path search and 83+ tests)
3a. Add `vertex_entries: [Option<VertexEntry>; 4]` to `VisibilityVertex`
3b. Rewrite path search to use directional entries (C3)
3c. Run benchmark comparison before/after

### Priority 4 — Reflection events + LookaheadScan (unblocks 20+ stress tests)
4a. Complete `LookaheadScan` with BTreeMap range queries
4b. Port `BasicReflectionEvent`, `HighReflectionEvent`, `LowReflectionEvent`
4c. Wire into VG event processing sweep

### Priority 5 — 83 BASIC_ROUTING + 91 STRESS tests (port from C#)
Port all non-groups, non-waypoints C# tests as Rust `#[test]` functions.
These are the acceptance gate for priorities 1–4.

### Priority 6 — Waypoints (10 tests)
Add waypoint constraint threading through `EdgeGeometry` → path search → nudger.

### Priority 7 — Groups (43 tests)
Largest single block. Requires: group hierarchy in obstacle tree, boundary crossing map,
ancestor set tracking in VG generator, group-aware path search.

### Priority 8 — Projection solver 6 fixtures
Fix cycle-handling and QPSC convergence edge cases.

---

## Acceptance Criteria

1. `cargo test` passes with zero failures, zero ignored.
2. `cargo clippy -- -D warnings` is clean.
3. All 86 projection solver fixture tests pass (currently 80).
4. All 242 C# procedural routing tests ported and passing as Rust `#[test]`.
5. WASM `Router` object exposes stateful add/move/remove/route API.
6. `route_edges` one-shot function continues to pass all bench scenarios with zero
   genuine routing failures (`is_fallback == false` for all edges).
7. Large benchmark stays under 30ms (currently 23.8ms native, 29.5ms WASM).

---

## What the Original PRD Got Right

The original PRD correctly identified C1–C4 as critical. Work completed since then:
- Solver: zero-allocation visited tracking + bound deduplication (3.6× faster than C#)
- FreeSpaceFinder: sweep-line from C# reference
- StaircaseRemover: R-tree spatial indexing
- ObstacleTree: rstar-based spatial queries
- RouterSession: unified session struct
- TGU: per-edge cleanup, port splicing
- `is_fallback` field on `RoutedEdge` / `PathOutput`

The original PRD missed the stateful API (Defect A) and the WASM binding (Defect B)
entirely. It also did not enumerate the 242 C# tests by category, which is why the
groups/waypoints/port-type gaps were not visible.
