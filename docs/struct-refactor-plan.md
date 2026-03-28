# Struct Refactor Plan
**Date:** 2026-03-27

## Rust Nomenclature

In C# you write a `class` with fields and methods. In Rust the equivalent is:

```rust
// C#:                          // Rust:
class Foo {                     struct Foo {
    Bar bar;                        bar: Bar,
    void DoThing() {}           }
}                               impl Foo {
                                    fn do_thing(&mut self) {}
                                }
```

When this document says "refactor into a proper struct," it means: give the type owned fields that it keeps across method calls, rather than being a stateless unit struct (`struct Foo;`) or a free function. Methods that mutate owned state take `&mut self`; methods that only read take `&self`.

---

## The Root Problem

The C# design has a shared ownership chain:

```
RectilinearEdgeRouter
  ├── PortManager
  │     └── TransientGraphUtility
  │                └─ (via graphGenerator) ─┐
  └── VisibilityGraphGenerator ◄────────────┘
          ├── VisibilityGraph          (shared)
          ├── ObstacleTree             (shared)
          ├── HScanSegments            (shared)
          └── VScanSegments            (shared)
```

Multiple structs need simultaneous access to the same shared data. In C# this is free (GC). In Rust, the clean solution is a **session struct** that owns all shared data, and the other structs become method collections that take `&mut RouterSession`.

---

## Refactor 1 — Create `RouterSession` (New Struct)

**What it is:** A struct that owns all data shared between PortManager, VGG, and TGU. Equivalent to what C#'s graphGenerator reference gives access to.

**Target struct:**
```rust
pub struct RouterSession {
    pub vis_graph: VisibilityGraph,
    pub obstacle_tree: ObstacleTree,
    pub h_scan_segments: ScanSegmentTree,
    pub v_scan_segments: ScanSegmentTree,
    pub obstacles: Vec<Obstacle>,
    pub padding: f64,
}
```

**Why no lifetimes needed:** Other structs don't store a reference to RouterSession — they just receive `&mut RouterSession` as a method parameter. Rust allows this without any lifetime annotation.

**Files affected:** New file `src/routing/router_session.rs`. Referenced by rectilinear_edge_router.rs, visibility_graph_generator.rs, port_manager.rs, transient_graph_utility.rs.

---

## Refactor 2 — `VisibilityGraphGenerator` (Free function → Struct with impl)

**Current state:** `generate_visibility_graph()` is a free function. It creates H/V scan segment trees locally and drops them when it returns.

**Problem:** PortManager needs HScanSegments and VScanSegments to do O(log n) edge lookups during port splicing. They're currently inaccessible.

**Target struct:**
```rust
pub struct VisibilityGraphGenerator {
    scan_direction: ScanDirection,
    // SweepState (scan_line, event_queue, seg_tree, lookahead_scan) stays local to generate()
    // — it's transient per sweep pass, not persistent
}

impl VisibilityGraphGenerator {
    /// Runs both sweep passes and populates session.vis_graph,
    /// session.h_scan_segments, session.v_scan_segments.
    pub fn generate(&self, session: &mut RouterSession) { ... }
}
```

**Key insight:** The sweep state (scan line, event queue, lookahead scan) is still transient — created per sweep pass and dropped. Only the *output* (vis_graph, h/v scan segments) needs to persist on the session.

**Files affected:** `visibility_graph_generator.rs`. Callers: `rectilinear_edge_router.rs`.

**Compiler-guided approach:** Change the function signature, run `cargo check`, fix each error.

---

## Refactor 3 — `PortManager` (Unit struct → Struct with impl, or methods on RouterSession)

**Current state:** `pub struct PortManager;` — no fields. All methods are static (no `&self`). `FullPortManager` with the correct fields exists but is never used.

**Problem:** No access to HScanSegments/VScanSegments. No persistent port entrance cache. No shared TransientGraphUtility lifecycle.

**Two options:**

**Option A — Proper owned struct:**
```rust
pub struct PortManager {
    obstacle_port_map: HashMap<usize, ObstaclePort>,
    free_point_map: HashMap<Point, FreePoint>,
    obstacle_ports_in_graph: Vec<usize>,
    free_points_in_graph: Vec<Point>,
    route_to_center: bool,
}

impl PortManager {
    pub fn begin_route_edges(&mut self, session: &mut RouterSession) { ... }
    pub fn splice_port(&mut self, session: &mut RouterSession, location: Point) { ... }
}
```

**Option B — Module of functions on RouterSession:**
Port entrance creation and splicing become methods directly on `RouterSession`. PortManager fields move onto `RouterSession`. This avoids a second struct entirely.

**Recommendation:** Option A, promoted to the `RectilinearEdgeRouter`. `FullPortManager` is already the right shape — it just needs to be wired in and the stateless `PortManager::splice_port` removed.

**Files affected:** `port_manager.rs`, `port_splice.rs`, `rectilinear_edge_router.rs`, `port.rs` (call sites).

---

## Refactor 4 — `TransientGraphUtility` (Parameter threading → Takes `&mut RouterSession`)

**Current state:** Every method takes `graph: &mut VisibilityGraph, obstacle_tree: &mut ObstacleTree` as separate parameters. Callers must thread these through.

**Problem:** Awkward API. Missing `LimitPortVisibilitySpliceToEndpointBoundingBox` field. No reference to HScanSegments (needed for `FindNearestPerpendicularOrContainingEdge`).

**Target:**
```rust
impl TransientGraphUtility {
    pub fn find_or_add_vertex(&mut self, session: &mut RouterSession, point: Point) -> VertexId { ... }
    pub fn extend_edge_chain(&mut self, session: &mut RouterSession, ...) { ... }
}
```

**What TGU owns (persists across calls):**
```rust
pub struct TransientGraphUtility {
    pub added_vertices: Vec<VertexId>,
    pub added_edges: Vec<(VertexId, VertexId)>,
    pub edges_to_restore: Vec<(VertexId, VertexId, f64, bool)>,
    pub limit_splice_to_bbox: bool,
}
```

Everything else comes from `session`.

**Files affected:** `transient_graph_utility.rs`, `transient_graph_utility_helpers.rs`, `transient_graph_utility_extend.rs`. All callers of TGU methods.

---

## Refactor 5 — `RectilinearEdgeRouter` (Builder → Stateful struct)

**Current state:** One-shot builder — `new()` → `run()`. GraphGenerator, PortManager, ObstacleTree created locally inside `run()`.

**C# capability we're missing:** Dynamic obstacle management (`AddObstacle`, `UpdateObstacle`, `RemoveObstacle`) and selective graph regeneration. Needed for interactive routing.

**Target:**
```rust
pub struct RectilinearEdgeRouter {
    session: RouterSession,           // owns all shared data
    port_manager: PortManager,        // persistent port cache
    vg_generator: VisibilityGraphGenerator,
    edges: Vec<EdgeGeometry>,
    padding: f64,
    edge_separation: f64,
    bend_penalty: f64,
}

impl RectilinearEdgeRouter {
    pub fn add_shape(&mut self, shape: Shape) { ... }
    pub fn remove_shape(&mut self, shape: Shape) { ... }
    pub fn add_edge(&mut self, edge: EdgeGeometry) { ... }
    pub fn run(&mut self) -> Vec<Vec<Point>> { ... }
}
```

**Note:** For the current one-pass use case, this is not strictly needed — the builder pattern works. This refactor matters when you want to wire the router into an interactive editor (which C# `RectilinearInteractiveEditor.cs` does). Flag as lower priority than Refactors 1–4.

**Files affected:** `rectilinear_edge_router.rs` and its callers.

---

## Refactor 6 — `ObstacleTree` (Immutable query → Stateful with persistent hierarchy)

**Current state:** `Vec<Obstacle>` + R-tree. Built once, queried but never updated. No ancestor tracking.

**C# capability we're missing:** Incremental `AdjustSpatialAncestors()`, group hierarchy, ancestor sets for port management.

**Target:** This is the hardest refactor because it requires porting the C# `RectangleNode` hierarchy building (clump/convex hull creation, group growth) that's currently absent. The R-tree handles spatial queries fine; what's missing is the *metadata* C# builds during construction:

```rust
pub struct ObstacleTree {
    obstacles: Vec<Obstacle>,
    rtree: RTree<ObstacleEnvelope>,
    ancestor_sets: HashMap<usize, HashSet<usize>>,  // NEW
    group_boundary_crossing_map: GroupBoundaryCrossingMap,
}
```

**Note:** Ancestor sets are only needed for group routing. For rectangular non-group obstacles (current scope), this refactor is low priority.

---

## Execution Strategy

### Phase 1 — Foundation (unblocks everything else)
1. Create `RouterSession` struct in new file
2. Refactor `VisibilityGraphGenerator` to populate session fields
3. Change `TransientGraphUtility` methods to take `&mut RouterSession`

Use the compiler as the guide:
```bash
# After each type/signature change:
cargo check 2>&1 | grep "error\[" | head -30
```
The compiler enumerates every broken call site. Fix each one. Repeat until clean.

### Phase 2 — PortManager
4. Activate `FullPortManager` — wire it into `RectilinearEdgeRouter`
5. Remove stateless `PortManager` unit struct
6. Port `CreateObstaclePortEntrancesFromPoints` (300+ lines, C# reference)

### Phase 3 — Advanced (when needed)
7. `RectilinearEdgeRouter` stateful struct (for interactive editor)
8. `ObstacleTree` ancestor sets (for groups)

### rust-analyzer Refactoring Support

| Operation | rust-analyzer Support |
|---|---|
| Rename a struct/field/method | ✅ Full workspace rename (F2 in VS Code) |
| Add a field to a struct | ✅ Compiler finds all places that construct the struct |
| Change a method signature | ⚠️ Partial — compiler finds call sites; rust-analyzer doesn't auto-rewrite them |
| Move a method from one impl to another | Manual |
| Convert free function to method | Manual + compiler guidance |

**Most effective workflow:** Make one change at a time, `cargo check`, fix all errors, commit. Do not batch multiple structural changes — the error messages become harder to interpret.

### Estimated Call Site Counts (from audit)
- `generate_visibility_graph` callers: ~2
- TGU method callers across all files: ~30–40
- `PortManager::splice_port` callers: ~3
- `RectilinearEdgeRouter::run` callers: ~1 (test harness)

Total affected call sites: ~40–50. Manageable in a single focused session with compiler guidance.
