# Phase 3: Visibility Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the visibility graph — the spatial data structure that defines all legal routing corridors between rectangular obstacles.

**Architecture:** Two-pass sweep-line algorithm (horizontal then vertical). Each pass processes obstacle corner events in order, maintaining an active scanline of obstacle sides, and creates visibility segments where edges can legally travel. The two sets of segments are intersected to produce the visibility graph. All data structures use index-based arenas or BTreeMap for sorted collections.

**Tech Stack:** Rust (2021 edition), rstar 0.12 (R-tree spatial index), existing geometry types from Phase 1.

**Spec:** `msagl-rust/docs/superpowers/specs/2026-03-25-msagl-rust-port-design.md`

**Porting sources:**
- TypeScript: `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/`
- TypeScript: `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/visibility/`
- C#: `MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/`

**Scope simplifications (per spec):**
- Only rectangular obstacles (no arbitrary polyline boundaries)
- No groups/clusters
- Only FullVisibilityGraphGenerator (no sparse variant)
- No overlap/clump handling (obstacles must not overlap)

---

## Rust Ownership Design

The **VisibilityGraph** owns all vertices in a `Vec<VertexData>` indexed by `VertexId`. A `HashMap<Point, VertexId>` provides O(1) point-to-vertex lookup. Edges are stored inline in each vertex's `out_edges: BTreeSet<EdgeTarget>` sorted by target point for neighbor traversal.

The **ObstacleTree** uses `rstar::RTree` for spatial queries.

The **RectilinearScanLine** uses `BTreeMap<SideKey, ObstacleSideId>` for sorted obstacle side tracking with O(log n) neighbor queries.

---

## File Structure

```
msagl-rust/src/
├── visibility/
│   ├── mod.rs
│   ├── graph.rs              # VisibilityGraph, VertexId, VertexData
│   └── edge.rs               # Edge representation
├── routing/
│   ├── mod.rs
│   ├── shape.rs              # Shape (user input — rectangle dimensions)
│   ├── obstacle.rs           # Obstacle (padded shape + sides)
│   ├── obstacle_side.rs      # LowObstacleSide, HighObstacleSide
│   ├── obstacle_tree.rs      # rstar R-tree wrapper
│   ├── scan_direction.rs     # Coordinate system abstraction
│   ├── scan_segment.rs       # ScanSegment + ScanSegmentTree
│   ├── scan_line.rs          # RectilinearScanLine (BTreeMap-based)
│   ├── event_queue.rs        # EventQueue + SweepEvent types
│   └── visibility_graph_generator.rs  # Main sweep algorithm
└── lib.rs                    # Add visibility + routing modules

msagl-rust/tests/
├── visibility/
│   ├── mod.rs
│   └── graph_tests.rs
└── routing/
    ├── mod.rs
    ├── obstacle_tests.rs
    ├── scan_direction_tests.rs
    └── visibility_gen_tests.rs
```

---

### Task 1: Module Scaffolding + Dependencies

**Files:**
- Modify: `Cargo.toml` (add rstar dependency)
- Modify: `src/lib.rs`
- Create: `src/visibility/mod.rs`, `src/visibility/graph.rs`, `src/visibility/edge.rs`
- Create: `src/routing/mod.rs` + all routing source files as placeholders
- Create: test files

- [ ] **Step 1: Add rstar dependency to Cargo.toml**

```toml
[dependencies]
ordered-float = "5"
slotmap = "1"
kurbo = "0.13"
rstar = "0.12"
```

- [ ] **Step 2: Create visibility and routing module structures**

`src/visibility/mod.rs`:
```rust
pub mod graph;
pub mod edge;
```

`src/routing/mod.rs`:
```rust
pub mod shape;
pub mod obstacle;
pub mod obstacle_side;
pub mod obstacle_tree;
pub mod scan_direction;
pub mod scan_segment;
pub mod scan_line;
pub mod event_queue;
pub mod visibility_graph_generator;
```

- [ ] **Step 3: Add modules to lib.rs**

```rust
pub mod visibility;
pub mod routing;
```

- [ ] **Step 4: Create placeholder files, verify compilation**

Each file gets a minimal placeholder struct.

- [ ] **Step 5: Create test structure**

- [ ] **Step 6: Verify cargo check, commit**

```bash
git commit -m "feat(msagl-rust): scaffold visibility and routing modules"
```

---

### Task 2: ScanDirection (Coordinate System Abstraction)

**Files:**
- Create: `src/routing/scan_direction.rs`
- Test: `tests/routing/scan_direction_tests.rs`

**Porting from:** TS: `ScanDirection.ts` (111 lines)

ScanDirection encapsulates whether we're sweeping vertically (processing horizontal segments) or horizontally (processing vertical segments). It makes the sweep algorithm direction-agnostic by providing coordinate projection and comparison.

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn horizontal_scan_projects_x_as_primary() {
    let sd = ScanDirection::horizontal();
    let p = Point::new(3.0, 7.0);
    assert_eq!(sd.coord(p), 3.0);        // Primary = X
    assert_eq!(sd.perp_coord(p), 7.0);   // Perpendicular = Y
}

#[test]
fn vertical_scan_projects_y_as_primary() {
    let sd = ScanDirection::vertical();
    let p = Point::new(3.0, 7.0);
    assert_eq!(sd.coord(p), 7.0);        // Primary = Y
    assert_eq!(sd.perp_coord(p), 3.0);   // Perpendicular = X
}

#[test]
fn compare_by_perp_then_primary() {
    let sd = ScanDirection::horizontal();
    let a = Point::new(1.0, 5.0);
    let b = Point::new(3.0, 5.0);
    let c = Point::new(1.0, 10.0);
    // Same perp (y=5), different primary: a < b
    assert!(sd.compare(a, b) == std::cmp::Ordering::Less);
    // Different perp: a < c (y=5 < y=10)
    assert!(sd.compare(a, c) == std::cmp::Ordering::Less);
}

#[test]
fn is_flat() {
    let sd = ScanDirection::horizontal();
    // Flat = no change in perpendicular direction (same Y)
    assert!(sd.is_flat(Point::new(0.0, 5.0), Point::new(10.0, 5.0)));
    assert!(!sd.is_flat(Point::new(0.0, 5.0), Point::new(10.0, 7.0)));
}

#[test]
fn is_perpendicular() {
    let sd = ScanDirection::horizontal();
    // Perpendicular = no change in primary direction (same X)
    assert!(sd.is_perpendicular(Point::new(5.0, 0.0), Point::new(5.0, 10.0)));
    assert!(!sd.is_perpendicular(Point::new(5.0, 0.0), Point::new(7.0, 10.0)));
}
```

- [ ] **Step 2: Implement ScanDirection**

```rust
// src/routing/scan_direction.rs
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use std::cmp::Ordering;

/// Direction of the sweep line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    North, // Sweep bottom-to-top (processing horizontal segments)
    East,  // Sweep left-to-right (processing vertical segments)
}

/// Encapsulates coordinate system logic for direction-agnostic sweep.
///
/// For horizontal sweep (Direction::East):
///   - Primary axis = X, Perpendicular axis = Y
///   - Sweep moves left-to-right
///
/// For vertical sweep (Direction::North):
///   - Primary axis = Y, Perpendicular axis = X
///   - Sweep moves bottom-to-top
#[derive(Clone, Copy, Debug)]
pub struct ScanDirection {
    pub dir: Direction,
}

impl ScanDirection {
    pub fn horizontal() -> Self { Self { dir: Direction::East } }
    pub fn vertical() -> Self { Self { dir: Direction::North } }

    /// The coordinate along the primary (sweep) axis.
    #[inline]
    pub fn coord(&self, p: Point) -> f64 {
        match self.dir {
            Direction::East => p.x(),
            Direction::North => p.y(),
        }
    }

    /// The coordinate along the perpendicular axis.
    #[inline]
    pub fn perp_coord(&self, p: Point) -> f64 {
        match self.dir {
            Direction::East => p.y(),
            Direction::North => p.x(),
        }
    }

    /// Compare two points: perpendicular coordinate first, then primary.
    pub fn compare(&self, a: Point, b: Point) -> Ordering {
        let perp = GeomConstants::compare(self.perp_coord(a), self.perp_coord(b));
        if perp != Ordering::Equal { return perp; }
        GeomConstants::compare(self.coord(a), self.coord(b))
    }

    /// Compare only the perpendicular coordinates.
    pub fn compare_perp(&self, a: Point, b: Point) -> Ordering {
        GeomConstants::compare(self.perp_coord(a), self.perp_coord(b))
    }

    /// Compare only the primary (sweep) coordinates.
    pub fn compare_scan(&self, a: Point, b: Point) -> Ordering {
        GeomConstants::compare(self.coord(a), self.coord(b))
    }

    /// True if start and end have the same perpendicular coordinate (horizontal/vertical segment).
    pub fn is_flat(&self, start: Point, end: Point) -> bool {
        GeomConstants::close(self.perp_coord(start), self.perp_coord(end))
    }

    /// True if start and end have the same primary coordinate (perpendicular segment).
    pub fn is_perpendicular(&self, start: Point, end: Point) -> bool {
        GeomConstants::close(self.coord(start), self.coord(end))
    }

    /// Create a point from (primary, perpendicular) coordinates.
    pub fn make_point(&self, coord: f64, perp_coord: f64) -> Point {
        match self.dir {
            Direction::East => Point::new(coord, perp_coord),
            Direction::North => Point::new(perp_coord, coord),
        }
    }
}
```

- [ ] **Step 3: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add ScanDirection coordinate system abstraction"
```

---

### Task 3: Visibility Graph Data Structure

**Files:**
- Create: `src/visibility/graph.rs`
- Create: `src/visibility/edge.rs`
- Test: `tests/visibility/graph_tests.rs`

**Porting from:** TS: `VisibilityGraph.ts` (317 lines), `VisibilityVertex.ts` (128 lines), `VisibilityEdge.ts` (58 lines)

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::visibility::graph::{VisibilityGraph, VertexId};
use msagl_rust::Point;

#[test]
fn add_and_find_vertex() {
    let mut g = VisibilityGraph::new();
    let v = g.add_vertex(Point::new(1.0, 2.0));
    assert_eq!(g.find_vertex(Point::new(1.0, 2.0)), Some(v));
    assert_eq!(g.find_vertex(Point::new(3.0, 4.0)), None);
}

#[test]
fn add_edge() {
    let mut g = VisibilityGraph::new();
    let v1 = g.add_vertex(Point::new(0.0, 0.0));
    let v2 = g.add_vertex(Point::new(5.0, 0.0));
    g.add_edge(v1, v2, 1.0);
    assert_eq!(g.out_degree(v1), 1);
    assert_eq!(g.in_degree(v2), 1);
}

#[test]
fn find_or_add_vertex_idempotent() {
    let mut g = VisibilityGraph::new();
    let v1 = g.find_or_add_vertex(Point::new(1.0, 2.0));
    let v2 = g.find_or_add_vertex(Point::new(1.0, 2.0));
    assert_eq!(v1, v2);
    assert_eq!(g.vertex_count(), 1);
}

#[test]
fn remove_edge() {
    let mut g = VisibilityGraph::new();
    let v1 = g.add_vertex(Point::new(0.0, 0.0));
    let v2 = g.add_vertex(Point::new(5.0, 0.0));
    g.add_edge(v1, v2, 1.0);
    g.remove_edge(v1, v2);
    assert_eq!(g.out_degree(v1), 0);
}

#[test]
fn vertex_point() {
    let mut g = VisibilityGraph::new();
    let v = g.add_vertex(Point::new(3.0, 4.0));
    assert_eq!(g.point(v), Point::new(3.0, 4.0));
}

#[test]
fn out_edges_iteration() {
    let mut g = VisibilityGraph::new();
    let v1 = g.add_vertex(Point::new(0.0, 0.0));
    let v2 = g.add_vertex(Point::new(5.0, 0.0));
    let v3 = g.add_vertex(Point::new(0.0, 5.0));
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v1, v3, 1.0);
    let targets: Vec<VertexId> = g.out_edges(v1).map(|e| e.target).collect();
    assert_eq!(targets.len(), 2);
}
```

- [ ] **Step 2: Implement VisibilityGraph**

```rust
// src/visibility/graph.rs
use std::collections::{BTreeSet, HashMap};
use crate::geometry::point::Point;
use super::edge::VisEdge;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

/// Data for a single visibility vertex.
pub struct VertexData {
    pub point: Point,
    pub out_edges: BTreeSet<VisEdge>,
    pub in_edges: Vec<VertexId>,   // Sources of incoming edges
    // Pathfinding state (mutable during search)
    pub distance: f64,
    pub prev_vertex: Option<VertexId>,
    pub is_terminal: bool,
}

/// The visibility graph: vertices keyed by Point, edges stored per-vertex.
pub struct VisibilityGraph {
    vertices: Vec<VertexData>,
    point_to_vertex: HashMap<Point, VertexId>,
}

impl VisibilityGraph {
    pub fn new() -> Self { ... }
    pub fn add_vertex(&mut self, point: Point) -> VertexId { ... }
    pub fn find_vertex(&self, point: Point) -> Option<VertexId> { ... }
    pub fn find_or_add_vertex(&mut self, point: Point) -> VertexId { ... }
    pub fn add_edge(&mut self, source: VertexId, target: VertexId, weight: f64) { ... }
    pub fn remove_edge(&mut self, source: VertexId, target: VertexId) { ... }
    pub fn point(&self, v: VertexId) -> Point { ... }
    pub fn vertex_count(&self) -> usize { ... }
    pub fn out_degree(&self, v: VertexId) -> usize { ... }
    pub fn in_degree(&self, v: VertexId) -> usize { ... }
    pub fn out_edges(&self, v: VertexId) -> impl Iterator<Item = &VisEdge> { ... }
}
```

```rust
// src/visibility/edge.rs
use super::graph::VertexId;

/// A directed edge in the visibility graph.
#[derive(Clone, Debug)]
pub struct VisEdge {
    pub target: VertexId,
    pub weight: f64,
    pub length_multiplier: f64,
}

// Ord/Eq by target VertexId for BTreeSet storage
impl Ord for VisEdge { ... }
impl PartialOrd for VisEdge { ... }
impl PartialEq for VisEdge { ... }
impl Eq for VisEdge {}
```

- [ ] **Step 3: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add VisibilityGraph with vertex/edge management"
```

---

### Task 4: Shape + Obstacle + ObstacleSides

**Files:**
- Create: `src/routing/shape.rs`
- Create: `src/routing/obstacle.rs`
- Create: `src/routing/obstacle_side.rs`
- Test: `tests/routing/obstacle_tests.rs`

**Porting from:** TS: `obstacle.ts` (238 lines), `BasicObstacleSide.ts` (50 lines)

Shape is the user-facing input (a rectangle). Obstacle is the internal representation (padded polyline + sides).

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::routing::shape::Shape;
use msagl_rust::routing::obstacle::Obstacle;
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn shape_rectangle_creation() {
    let s = Shape::rectangle(10.0, 20.0, 100.0, 50.0); // x, y, width, height
    assert_eq!(s.bounding_box().left(), 10.0);
    assert_eq!(s.bounding_box().right(), 110.0);
    assert_eq!(s.bounding_box().bottom(), 20.0);
    assert_eq!(s.bounding_box().top(), 70.0);
}

#[test]
fn obstacle_from_shape_with_padding() {
    let s = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&s, 4.0, 0);
    // Padded by 4 on each side
    let bb = obs.padded_bounding_box();
    assert_eq!(bb.left(), 6.0);
    assert_eq!(bb.right(), 114.0);
    assert_eq!(bb.bottom(), 16.0);
    assert_eq!(bb.top(), 74.0);
}

#[test]
fn obstacle_creates_four_sides() {
    let s = Shape::rectangle(0.0, 0.0, 10.0, 10.0);
    let obs = Obstacle::from_shape(&s, 0.0, 0);
    assert_eq!(obs.padded_polyline().count(), 4);
}
```

- [ ] **Step 2: Implement Shape, Obstacle, ObstacleSide**

Shape holds the original rectangle. Obstacle creates a padded polyline (4-point closed polyline for rectangles) and can create sides for sweep processing.

ObstacleSides represent the left/right/top/bottom edges of obstacle polylines:
```rust
// src/routing/obstacle_side.rs
pub struct ObstacleSide {
    pub obstacle_index: usize,
    pub start: Point,
    pub end: Point,
    pub is_low_side: bool,  // Low = left/bottom; High = right/top
}
```

- [ ] **Step 3: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add Shape, Obstacle, and ObstacleSide types"
```

---

### Task 5: ObstacleTree (R-tree Spatial Index)

**Files:**
- Create: `src/routing/obstacle_tree.rs`
- Test: `tests/routing/obstacle_tests.rs` (append)

**Porting from:** TS: `ObstacleTree.ts` (823 lines) — heavily simplified since we don't support groups/overlaps

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::routing::obstacle_tree::ObstacleTree;
use msagl_rust::routing::shape::Shape;
use msagl_rust::Point;

#[test]
fn obstacle_tree_query_point() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(20.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 4.0);
    // Point inside first obstacle's padded box
    let hits = tree.query_point(Point::new(5.0, 5.0));
    assert_eq!(hits.len(), 1);
}

#[test]
fn obstacle_tree_query_rect() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(20.0, 0.0, 10.0, 10.0),
        Shape::rectangle(50.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 4.0);
    let hits = tree.query_rect(&Rectangle::new(-5.0, -5.0, 35.0, 15.0));
    assert_eq!(hits.len(), 2); // first two obstacles
}
```

- [ ] **Step 2: Implement ObstacleTree using rstar**

```rust
// src/routing/obstacle_tree.rs
use rstar::{RTree, RTreeObject, AABB, PointDistance};
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use super::obstacle::Obstacle;
use super::shape::Shape;

struct ObstacleEnvelope {
    obstacle_index: usize,
    bbox: Rectangle,
}

impl RTreeObject for ObstacleEnvelope {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [self.bbox.left(), self.bbox.bottom()],
            [self.bbox.right(), self.bbox.top()],
        )
    }
}

pub struct ObstacleTree {
    obstacles: Vec<Obstacle>,
    rtree: RTree<ObstacleEnvelope>,
}
```

- [ ] **Step 3: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add ObstacleTree with rstar R-tree spatial index"
```

---

### Task 6: ScanSegment + ScanSegmentTree

**Files:**
- Create: `src/routing/scan_segment.rs`
- Test: `tests/routing/visibility_gen_tests.rs`

**Porting from:** TS: `ScanSegment.ts` (499 lines), `ScanSegmentTree.ts` (311 lines)

ScanSegments represent the visibility edges being traced during the sweep. The ScanSegmentTree stores them in a BTreeMap for intersection queries.

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::routing::scan_segment::{ScanSegment, ScanSegmentTree};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn scan_segment_creation() {
    let seg = ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0));
    assert_eq!(seg.start, Point::new(0.0, 5.0));
    assert_eq!(seg.end, Point::new(10.0, 5.0));
}

#[test]
fn scan_segment_tree_insert_and_find() {
    let sd = ScanDirection::horizontal();
    let mut tree = ScanSegmentTree::new(sd);
    let seg = ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0));
    tree.insert(seg);
    let found = tree.find_containing_point(Point::new(5.0, 5.0));
    assert!(found.is_some());
}
```

- [ ] **Step 2: Implement ScanSegment and ScanSegmentTree**

ScanSegment is a simple struct with start/end points and weight. ScanSegmentTree uses a BTreeMap keyed by the perpendicular coordinate for fast lookup.

- [ ] **Step 3: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add ScanSegment and ScanSegmentTree"
```

---

### Task 7: EventQueue + SweepEvents

**Files:**
- Create: `src/routing/event_queue.rs`

**Porting from:** TS: `EventQueue.ts` (63 lines), various event types

Events represent obstacle corners encountered during the sweep. The queue orders them by perpendicular coordinate (sweep position).

- [ ] **Step 1: Implement event types and queue**

```rust
// src/routing/event_queue.rs
use std::collections::BinaryHeap;
use std::cmp::{Ordering, Reverse};
use crate::geometry::point::Point;
use super::scan_direction::ScanDirection;

/// Sweep events for rectangular obstacle corners.
/// For rectangular obstacles, each corner generates specific event types
/// depending on its position relative to the sweep direction.
/// The TS source uses: OpenVertexEvent, CloseVertexEvent, plus bend events.
/// For rectangles, corners are always axis-aligned so we use these 4 types.
#[derive(Clone, Debug)]
pub enum SweepEvent {
    /// Bottom-left corner (horizontal sweep) or left-bottom (vertical).
    /// Opens both low and high sides of the obstacle.
    OpenLow { point: Point, obstacle_index: usize },
    /// Bottom-right or right-bottom corner. Closes the low side.
    CloseLow { point: Point, obstacle_index: usize },
    /// Top-left or left-top corner. Closes the high side.
    CloseHigh { point: Point, obstacle_index: usize },
    /// Top-right or right-top corner. Closes remaining sides.
    OpenHigh { point: Point, obstacle_index: usize },
}

impl SweepEvent {
    pub fn point(&self) -> Point {
        match self {
            SweepEvent::LowVertex { point, .. } => *point,
            SweepEvent::HighVertex { point, .. } => *point,
        }
    }
}

pub struct EventQueue {
    scan_direction: ScanDirection,
    events: BinaryHeap<Reverse<OrderedEvent>>,
}
```

Events are ordered by perpendicular coordinate (sweep position), then by event type priority.

**Note:** These event types are the starting set for rectangular obstacles. If the implementer finds during Task 9 that the TS source requires additional event variants (e.g., reflection events, bend events), extend the enum then. The TS `VisibilityGraphGenerator.ts` processes a richer event hierarchy, but for axis-aligned rectangles, many reduce to the four corner types above.

- [ ] **Step 2: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add EventQueue with sweep event ordering"
```

---

### Task 8: RectilinearScanLine

**Files:**
- Create: `src/routing/scan_line.rs`

**Porting from:** TS: `RectilinearScanLine.ts` (204 lines), `NeighborSides.ts` (69 lines)

The scan line maintains the active obstacle sides sorted by their intersection with the current sweep position. Uses BTreeMap for O(log n) insertion, removal, and neighbor queries.

- [ ] **Step 1: Write tests**

```rust
#[test]
fn scan_line_insert_and_neighbors() {
    // Create two obstacles, insert their sides, query neighbors
    let sd = ScanDirection::horizontal();
    let mut sl = RectilinearScanLine::new(sd);
    // Insert sides at y=0..10 and y=20..30
    // Query neighbors at y=15 should find top of first, bottom of second
}
```

- [ ] **Step 2: Implement RectilinearScanLine**

```rust
// src/routing/scan_line.rs
use std::collections::BTreeMap;
use crate::geometry::point::Point;
use super::scan_direction::ScanDirection;
use super::obstacle_side::ObstacleSide;

/// Key for ordering obstacle sides along the scan line.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct SideKey {
    perp_coord: ordered_float::OrderedFloat<f64>,
    obstacle_ordinal: usize,
}

pub struct RectilinearScanLine {
    scan_direction: ScanDirection,
    sides: BTreeMap<SideKey, ObstacleSide>,
}
```

- [ ] **Step 3: Run tests, commit**

```bash
git commit -m "feat(msagl-rust): add RectilinearScanLine with BTreeMap-based side tracking"
```

---

### Task 9: VisibilityGraphGenerator (Main Sweep Algorithm)

**Files:**
- Create: `src/routing/visibility_graph_generator.rs`
- Test: `tests/routing/visibility_gen_tests.rs`

**Porting from:** TS: `VisibilityGraphGenerator.ts` (1013 lines)

This is the largest and most complex task. The implementer should read the TS source carefully.

**Simplified algorithm for rectangular obstacles:**

1. For each scan direction (horizontal, vertical):
   a. Create event queue with all obstacle corner events
   b. Create empty scan line
   c. Process events in order:
      - **Low vertex:** Insert obstacle sides into scan line, create scan segments from obstacle edges
      - **High vertex:** Remove obstacle sides from scan line, finalize segments
   d. For each event, find low/high neighbor sides and extend visibility segments between them

2. After both passes, intersect horizontal and vertical scan segments to create visibility graph vertices at intersections.

- [ ] **Step 1: Write integration tests**

```rust
use msagl_rust::routing::visibility_graph_generator::generate_visibility_graph;
use msagl_rust::routing::shape::Shape;

#[test]
fn two_obstacles_visibility_graph() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(30.0, 0.0, 10.0, 10.0),
    ];
    let graph = generate_visibility_graph(&shapes, 2.0);
    // Should have vertices at padded obstacle corners + intersection points
    assert!(graph.vertex_count() > 0);
    // Should have edges between visible corners
}

#[test]
fn single_obstacle_visibility_graph() {
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let graph = generate_visibility_graph(&shapes, 2.0);
    // Padded obstacle has 4 corners, each with horizontal + vertical visibility
    assert!(graph.vertex_count() >= 4);
}

#[test]
fn three_obstacles_L_shape() {
    // Obstacles forming an L-shape — tests that visibility is blocked
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),   // bottom-left
        Shape::rectangle(0.0, 20.0, 10.0, 10.0),   // top-left
        Shape::rectangle(20.0, 0.0, 10.0, 10.0),   // bottom-right
    ];
    let graph = generate_visibility_graph(&shapes, 2.0);
    assert!(graph.vertex_count() > 0);
    // Verify no direct edge between top-left and bottom-right
    // (blocked by bottom-left obstacle)
}
```

- [ ] **Step 2: Implement the sweep algorithm (single direction)**

The implementer should read `VisibilityGraphGenerator.ts` and port the event processing loop. For rectangular obstacles, many of the complex reflection/overlap paths can be simplified.

The sweep is a 3-part process:
1. **Event creation:** Generate corner events from all obstacles for the sweep direction
2. **Event processing:** Process events in sweep order, maintaining the scan line and creating scan segments
3. **Segment intersection:** After both passes, intersect H and V segments to produce visibility graph vertices and edges

```rust
pub fn generate_visibility_graph(shapes: &[Shape], padding: f64) -> VisibilityGraph {
    let mut graph = VisibilityGraph::new();
    let obstacle_tree = ObstacleTree::new(shapes, padding);

    // Pass 1: Horizontal sweep → creates horizontal scan segments
    let h_segments = sweep(&obstacle_tree, ScanDirection::horizontal());
    // Pass 2: Vertical sweep → creates vertical scan segments
    let v_segments = sweep(&obstacle_tree, ScanDirection::vertical());
    // Pass 3: Intersect to build graph
    intersect_segments(&mut graph, &h_segments, &v_segments);

    graph
}
```

- [ ] **Step 3: Implement segment intersection**

This is a distinct post-processing step (TS: `SegmentIntersector`, ~200 lines). For each horizontal segment, find all vertical segments that cross it (and vice versa). At each intersection point, create a visibility vertex and connect it via edges to its neighbors along both the horizontal and vertical segments.

```rust
/// Intersect horizontal and vertical scan segments to produce
/// visibility graph vertices at crossing points and edges along segments.
fn intersect_segments(
    graph: &mut VisibilityGraph,
    h_segments: &ScanSegmentTree,
    v_segments: &ScanSegmentTree,
) {
    // For each horizontal segment at y=k from x=a to x=b:
    //   Find all vertical segments whose x-range includes [a..b]
    //   and whose y-range includes k
    //   Create vertex at each intersection, add edges along segments
}
```

The implementer should read the TS `SegmentIntersector.ts` for the sweep-based approach, or implement a simpler nested-loop approach if the obstacle count is small enough.

- [ ] **Step 4: Debug and verify all tests pass**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(msagl-rust): add VisibilityGraphGenerator sweep-line algorithm"
```

---

### Task 10: Final Integration

- [ ] **Step 1: Run full test suite**

Run: `cd msagl-rust && cargo test`

- [ ] **Step 2: Run clippy**

Run: `cd msagl-rust && cargo clippy -- -D warnings`

- [ ] **Step 3: Fix any issues, commit**

```bash
git commit -m "chore(msagl-rust): Phase 3 complete — visibility graph construction"
```

- [ ] **Step 4: Push to GitHub**

```bash
git push
```

---

## Summary

| Task | What | Key Files | Tests |
|------|------|-----------|-------|
| 1 | Module scaffolding | mod.rs, placeholders | 0 |
| 2 | ScanDirection | scan_direction.rs | ~5 |
| 3 | VisibilityGraph | graph.rs, edge.rs | ~6 |
| 4 | Shape + Obstacle + Sides | shape.rs, obstacle.rs, obstacle_side.rs | ~3 |
| 5 | ObstacleTree | obstacle_tree.rs | ~2 |
| 6 | ScanSegment + Tree | scan_segment.rs | ~2 |
| 7 | EventQueue | event_queue.rs | ~2 |
| 8 | RectilinearScanLine | scan_line.rs | ~2 |
| 9 | VisibilityGraphGenerator | visibility_graph_generator.rs (largest) | ~3 |
| 10 | Integration | clippy + push | 0 |
| **Total** | | **~12 source files** | **~25 tests** |

## What Comes Next

**Phase 4: Path Search** — A* pathfinding on the visibility graph + port splicing. Plan will be written separately.
