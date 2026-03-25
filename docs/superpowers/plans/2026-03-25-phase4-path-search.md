# Phase 4: Path Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement direction-aware A* pathfinding on the visibility graph with port splicing to connect edge endpoints.

**Architecture:** Direction-aware A* using VertexEntry[4] per vertex (one per compass direction N/E/S/W). Cost = length_importance * path_length + bends_importance * bend_count. Ports are temporarily spliced into the visibility graph via horizontal/vertical ray extensions, the path is found, then transient vertices/edges are removed.

**Tech Stack:** Rust, existing visibility graph and geometry from Phases 1+3.

**Porting sources:**
- TS: `routing/rectilinear/SsstRectilinearPath.ts`, `VertexEntry.ts`, `PortManager.ts`, `TransientGraphUtility.ts`

---

## File Structure

```
msagl-rust/src/routing/
├── port.rs                # FloatingPort type
├── edge_geometry.rs       # EdgeGeometry (source port, target port)
├── path_search.rs         # Direction-aware A* (SsstRectilinearPath)
└── port_manager.rs        # Port splicing into/out of visibility graph
```

---

### Task 1: Port + EdgeGeometry Types

**Files:**
- Create: `src/routing/port.rs`
- Create: `src/routing/edge_geometry.rs`
- Modify: `src/routing/mod.rs`

FloatingPort represents a connection point on an obstacle. EdgeGeometry defines an edge to route.

```rust
// src/routing/port.rs
use crate::geometry::point::Point;

/// A floating port on an obstacle boundary.
pub struct FloatingPort {
    /// Index of the obstacle this port is on.
    pub obstacle_index: usize,
    /// Location of the port.
    pub location: Point,
}

impl FloatingPort {
    pub fn new(obstacle_index: usize, location: Point) -> Self {
        Self { obstacle_index, location }
    }
}
```

```rust
// src/routing/edge_geometry.rs
use super::port::FloatingPort;

/// An edge to be routed between two ports.
pub struct EdgeGeometry {
    pub source: FloatingPort,
    pub target: FloatingPort,
}

impl EdgeGeometry {
    pub fn new(source: FloatingPort, target: FloatingPort) -> Self {
        Self { source, target }
    }
}
```

Tests: basic construction tests.

Commit: `feat(msagl-rust): add FloatingPort and EdgeGeometry types`

---

### Task 2: Direction-Aware A* Path Search

**Files:**
- Create: `src/routing/path_search.rs`
- Modify: `src/visibility/graph.rs` (add VertexEntry storage)

**Porting from:** TS: `SsstRectilinearPath.ts` (~523 lines), `VertexEntry.ts`

The A* tracks 4 entry directions per vertex. Cost = length * length_importance + bends * bends_importance. Heuristic = Manhattan distance + estimated bends to target.

```rust
// Key types in path_search.rs

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CompassDirection { North, East, South, West }

/// Entry to a vertex from a specific direction.
struct VertexEntry {
    vertex: VertexId,
    direction: CompassDirection,
    prev_entry: Option<usize>,  // Index into entries vec
    length: f64,
    bends: u32,
    cost: f64,
    is_closed: bool,
}

/// A* pathfinder on the visibility graph.
pub struct PathSearch {
    length_importance: f64,
    bends_importance: f64,
}

impl PathSearch {
    pub fn find_path(
        &self,
        graph: &VisibilityGraph,
        source: Point,
        target: Point,
    ) -> Option<Vec<Point>> { ... }
}
```

The algorithm:
1. Initialize: create entries for source vertex in all 4 directions
2. Priority queue ordered by cost + heuristic
3. Dequeue lowest cost entry
4. If at target vertex, reconstruct path
5. For each out-edge from current vertex:
   - Compute new direction from edge
   - Count bends (direction change from entry direction)
   - Calculate cost = length * l_imp + bends * b_imp
   - If better than existing entry for (vertex, direction), update and enqueue
6. Reconstruct path by walking prev_entry chain, collecting non-collinear points

Tests:
```rust
#[test]
fn path_search_straight_line() {
    // Two vertices connected by a single edge
    let mut graph = VisibilityGraph::new();
    let v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let v2 = graph.add_vertex(Point::new(10.0, 0.0));
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);

    let search = PathSearch::new(1.0, 0.001);
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0));
    assert!(path.is_some());
    let pts = path.unwrap();
    assert_eq!(pts.len(), 2);
}

#[test]
fn path_search_with_bend() {
    // L-shaped path: (0,0) → (10,0) → (10,10)
    let mut graph = VisibilityGraph::new();
    let v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let v2 = graph.add_vertex(Point::new(10.0, 0.0));
    let v3 = graph.add_vertex(Point::new(10.0, 10.0));
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);
    graph.add_edge(v2, v3, 1.0);
    graph.add_edge(v3, v2, 1.0);

    let search = PathSearch::new(1.0, 0.001);
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 10.0));
    assert!(path.is_some());
    let pts = path.unwrap();
    assert!(pts.len() >= 2);
}

#[test]
fn path_search_prefers_fewer_bends() {
    // Two paths: direct with 2 bends, or longer with 0 bends
    // With high bend penalty, should prefer the straight path
}

#[test]
fn path_search_no_path_returns_none() {
    let mut graph = VisibilityGraph::new();
    let _v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let _v2 = graph.add_vertex(Point::new(10.0, 0.0));
    // No edge connecting them
    let search = PathSearch::new(1.0, 0.001);
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0));
    assert!(path.is_none());
}
```

Commit: `feat(msagl-rust): add direction-aware A* path search with bend penalty`

---

### Task 3: Port Splicing (Port Manager)

**Files:**
- Create: `src/routing/port_manager.rs`

Port splicing temporarily connects a port location to the visibility graph by extending horizontal and vertical rays from the port until they hit existing graph edges.

```rust
pub struct PortManager;

impl PortManager {
    /// Splice a point into the visibility graph by extending H/V rays.
    /// Returns the VertexId of the spliced point.
    /// Tracks added vertices/edges for later removal.
    pub fn splice_port(
        graph: &mut VisibilityGraph,
        port_location: Point,
        added_vertices: &mut Vec<VertexId>,
        added_edges: &mut Vec<(VertexId, VertexId)>,
    ) -> VertexId { ... }

    /// Remove all transient vertices and edges added by splice_port.
    pub fn remove_spliced_ports(
        graph: &mut VisibilityGraph,
        added_vertices: &[VertexId],
        added_edges: &[(VertexId, VertexId)],
    ) { ... }
}
```

The splice algorithm:
1. Add vertex at port location
2. For each of 4 directions (N/E/S/W):
   - Walk along existing graph edges in that direction
   - Find the nearest existing vertex that's collinear with the port
   - Add an edge connecting port vertex to that vertex
3. Track all added vertices/edges for cleanup

Tests:
```rust
#[test]
fn splice_port_into_graph() {
    // Graph with a horizontal edge from (0,5) to (20,5)
    // Splice port at (10, 5) — should split the edge
    let mut graph = VisibilityGraph::new();
    let v1 = graph.add_vertex(Point::new(0.0, 5.0));
    let v2 = graph.add_vertex(Point::new(20.0, 5.0));
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);

    let mut added_v = Vec::new();
    let mut added_e = Vec::new();
    let port_v = PortManager::splice_port(
        &mut graph, Point::new(10.0, 5.0), &mut added_v, &mut added_e
    );
    assert!(graph.out_degree(port_v) > 0);
}
```

Commit: `feat(msagl-rust): add PortManager for port splicing into visibility graph`

---

### Task 4: End-to-End Route Path Test

**Files:**
- Test: `tests/routing/path_search_tests.rs`

Integration test: create obstacles, build visibility graph, splice ports, find path.

```rust
#[test]
fn route_between_two_obstacles() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(30.0, 0.0, 10.0, 10.0),
    ];
    let mut graph = generate_visibility_graph(&shapes, 2.0);

    // Source port on right side of obstacle 0, target on left of obstacle 1
    let source = Point::new(12.0, 5.0);  // Just outside padded right edge
    let target = Point::new(28.0, 5.0);  // Just outside padded left edge

    let mut added_v = Vec::new();
    let mut added_e = Vec::new();
    PortManager::splice_port(&mut graph, source, &mut added_v, &mut added_e);
    PortManager::splice_port(&mut graph, target, &mut added_v, &mut added_e);

    let search = PathSearch::new(1.0, 0.001);
    let path = search.find_path(&graph, source, target);
    assert!(path.is_some(), "should find a path between obstacles");
    let pts = path.unwrap();
    assert!(pts.len() >= 2);
    assert_eq!(pts[0], source);
    assert_eq!(*pts.last().unwrap(), target);
}
```

Commit: `test(msagl-rust): add end-to-end route path integration test`

---

### Task 5: Final Integration

- [ ] Run: `cargo test && cargo clippy -- -D warnings`
- [ ] Fix any issues
- [ ] Commit and push

```bash
git push
```

---

## Summary

| Task | What | Files | Tests |
|------|------|-------|-------|
| 1 | Port + EdgeGeometry types | port.rs, edge_geometry.rs | ~2 |
| 2 | Direction-aware A* | path_search.rs | ~4 |
| 3 | Port splicing | port_manager.rs | ~2 |
| 4 | E2E integration test | tests only | ~1 |
| 5 | Integration | clippy + push | 0 |
| **Total** | | **4 source files** | **~9 tests** |

## What Comes Next

**Phase 5: Nudging** — the QPSC-based edge separation pipeline.
