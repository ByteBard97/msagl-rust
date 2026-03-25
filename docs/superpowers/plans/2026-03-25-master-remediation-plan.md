# MSAGL-Rust Faithful Port: Master Remediation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **SUPERSEDES:** phase1-foundation.md, phase2-projection-solver.md, phase3-visibility.md, phase4-path-search.md, phase5-nudging.md — those plans followed the PRD's original top-down ordering, which was revised after Socratic debate analysis.

**Goal:** Replace all simplified algorithms in msagl-rust with faithful ports from the C#/TypeScript MSAGL RectilinearEdgeRouter, growing the crate from ~6,300 to ~12,000-14,000 lines.

**Architecture:** Bottom-up layered build. Each layer compiles and passes tests against real dependencies below it. No mocks, no stubs. Golden baseline JSON dumps from C# validate fidelity at every layer. Arena-indexed ownership model throughout.

**Tech Stack:** Rust 2021, ordered-float 5, slotmap 1, kurbo 0.13, rstar 0.12, approx 0.5 (dev)

**Reference Sources:**
- **TS (primary structure):** `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/`
- **C# (authoritative algorithms):** `MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/`
- **C# tests:** `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs` (242 methods)
- **C# solver fixtures:** `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Resources/Constraints/ProjectionSolver/Data/` (86 files)

**Approved Rust Idiom Adaptations:**

| TS/C# Pattern | Rust Adaptation |
|---|---|
| Object references with GC | Index-based arenas (`Vec<T>` + `usize` indices) |
| Class hierarchies | Enums with variants |
| `RBTree<T>` with cursor nav | `BTreeMap` with `range()` queries |
| `Dictionary<Point, V>` | `HashMap<Point, V>` with `OrderedFloat` |
| `abstract class` | Enum dispatch or trait + impl |
| `null` references | `Option<T>` |
| Custom spatial index | `rstar` crate |
| Custom linked list | `Vec` with index links or `SlotMap` |

**If a pattern not in this table needs adaptation, STOP AND ASK.**

---

## Layer Map

| Layer | What | Est. Lines | Depends On |
|---|---|---|---|
| 0 | Foundation: CompassDirection, VertexId, C# golden baselines, ObstacleSides, Obstacle expand, ScanDirection | ~500 new/rewritten | existing geometry |
| 0.5 | Solver failure triage (investigation only) | 0 new code | Layer 0 |
| 1 | Event system: EventQueue, ScanLine, NeighborSides, LookaheadScan | ~600 | Layer 0 |
| 2 | VG Generator rewrite + ScanSegmentTree + SegmentIntersector | ~1,800 | Layer 1 |
| 2.5 | End-to-end smoke test (existing path search + new VG) | ~100 | Layer 2 |
| 3 | VertexEntry[4] + VisibilityVertexRectilinear | ~200 | Layer 2 |
| 4 | SsstRectilinearPath + MsmtRectilinearPath rewrite | ~700 | Layer 3 |
| 5 | PortManager + TransientGraphUtility + TollFreeEdge + StaticGraphUtility + SpliceUtility | ~1,250 | Layer 4 |
| 6 | Nudging: PathMerger, LinkedPointSplitter, FreeSpaceFinder expand | ~800 | Layer 5 |
| 7 | Solver fixture fixes (6 failures) | ~100 | Layer 6 |
| 8 | Port 242 C# routing tests | ~2,000 | Layer 7 |
| | **Total new/rewritten** | **~7,900** | |

---

## File Structure (Final State)

```
msagl-rust/src/
├── lib.rs
├── geometry/
│   ├── mod.rs
│   ├── point.rs              # ✓ exists (186 lines)
│   ├── rectangle.rs          # ✓ exists (132 lines)
│   ├── polyline.rs           # ✓ exists (245 lines)
│   ├── curve.rs              # ✓ exists (93 lines)
│   └── point_comparer.rs     # ✓ exists (46 lines)
├── projection_solver/        # ✓ exists (~1,200 lines) — fix 6 fixtures in Layer 7
│   └── ...
├── visibility/
│   ├── mod.rs                # ✓ exists
│   ├── graph.rs              # EXPAND: add VertexEntry[4] per vertex (Layer 3)
│   └── edge.rs               # EXPAND: add TollFreeVisibilityEdge variant (Layer 5)
└── routing/
    ├── mod.rs                # ✓ exists
    ├── shape.rs              # ✓ exists (31 lines)
    ├── port.rs               # EXPAND: add ObstaclePort, FreePoint, ObstaclePortEntrance (Layer 5)
    ├── edge_geometry.rs      # ✓ exists (14 lines)
    ├── obstacle.rs           # EXPAND: add ConvexHull, sentinel support, sides (Layer 0)
    ├── obstacle_side.rs      # REWRITE: LowObstacleSide, HighObstacleSide, slope (Layer 0)
    ├── obstacle_tree.rs      # EXPAND: overlap detection, full query API (Layer 0)
    ├── scan_direction.rs     # EXPAND: add comparison ops, coord extraction (Layer 0)
    ├── scan_segment.rs       # REWRITE: weights, group crossings, ScanSegmentTree (Layer 2)
    ├── scan_segment_tree.rs  # NEW: RBTree-based segment tree (Layer 2)
    ├── scan_line.rs          # REWRITE: full RectilinearScanLine (Layer 1)
    ├── event_queue.rs        # REWRITE: full event hierarchy (Layer 1)
    ├── neighbor_sides.rs     # NEW: NeighborSides tracking (Layer 1)
    ├── lookahead_scan.rs     # NEW: reflection lookahead (Layer 1)
    ├── visibility_graph_generator.rs  # REWRITE: full sweep-line (Layer 2)
    ├── segment_intersector.rs # REWRITE: faithful port (Layer 2)
    ├── vertex_entry.rs       # NEW: VertexEntry for direction-aware A* (Layer 3)
    ├── path_search.rs        # REWRITE: SsstRectilinearPath (Layer 4)
    ├── msmt_path.rs          # NEW: MsmtRectilinearPath (Layer 4)
    ├── port_manager.rs       # REWRITE: full port splicing (Layer 5)
    ├── transient_graph_utility.rs  # NEW: edge splitting, bracket logic (Layer 5)
    ├── static_graph_utility.rs     # NEW: geometry utilities (Layer 5)
    ├── splice_utility.rs     # NEW: intersection munging (Layer 5)
    ├── rectilinear_edge_router.rs  # EXPAND: full orchestration (Layer 5)
    └── nudging/
        ├── mod.rs            # ✓ exists
        ├── nudger.rs         # EXPAND: two-pass orchestration (Layer 6)
        ├── axis_edge.rs      # ✓ exists (71 lines) — minor expand
        ├── path_edge.rs      # ✓ exists (45 lines) — minor expand
        ├── path.rs           # ✓ exists (implicit in nudger) — extract
        ├── longest_nudged_segment.rs  # ✓ exists (108 lines)
        ├── combinatorial_nudger.rs    # ✓ exists (220 lines) — minor fixes
        ├── path_refiner.rs   # EXPAND: LinkedPointSplitter integration (Layer 6)
        ├── free_space_finder.rs  # REWRITE: full sweep-line (Layer 6)
        ├── staircase_remover.rs  # ✓ exists (177 lines)
        ├── path_merger.rs    # NEW: loop/self-cycle removal (Layer 6)
        ├── linked_point.rs   # NEW: linked-list path vertices (Layer 6)
        └── linked_point_splitter.rs  # NEW: H/V intersection sweep (Layer 6)

msagl-rust/tests/
├── routing_tests.rs          # ✓ exists — expand with golden baselines
├── projection_solver/        # ✓ exists — fix 6 fixtures
├── visibility/               # ✓ exists — expand
├── golden_baselines/         # NEW: JSON dumps from C# (Layer 2)
├── test_harness/             # NEW: Rust test builder (Layer 8)
│   ├── mod.rs
│   ├── scenario_builder.rs   # Shape/port/edge creation helpers
│   └── verifier.rs           # Hard invariant assertions
└── rectilinear/              # NEW: 242 ported C# tests (Layer 8)
    ├── mod.rs
    ├── diamond_tests.rs
    ├── free_port_tests.rs
    ├── reflection_tests.rs
    ├── overlap_tests.rs
    └── ...
```

---

## Layer 0: Foundation Data Structures

### Task 0.0a: Create CompassDirection Enum

**Files:**
- Create: `src/routing/compass_direction.rs` (~60 lines)
- Modify: `src/routing/mod.rs`
- Test: inline `#[cfg(test)]`

The existing `Direction` enum in `scan_direction.rs` only has North/East (scan directions). Layers 3-5 need a full 4-direction compass enum for VertexEntry[4] and path search direction tracking. Create it now so all layers can use it.

- [ ] **Step 1: Write tests**

```rust
#[test]
fn compass_direction_has_four_variants() {
    let all = CompassDirection::all();
    assert_eq!(all.len(), 4);
}

#[test]
fn compass_direction_opposite() {
    assert_eq!(CompassDirection::North.opposite(), CompassDirection::South);
    assert_eq!(CompassDirection::East.opposite(), CompassDirection::West);
}

#[test]
fn compass_direction_to_index() {
    // Each direction maps to a unique index 0-3 for VertexEntry[4]
    let indices: Vec<usize> = CompassDirection::all().iter().map(|d| d.index()).collect();
    assert_eq!(indices, vec![0, 1, 2, 3]);
}

#[test]
fn compass_direction_left_right_turns() {
    assert_eq!(CompassDirection::North.left(), CompassDirection::West);
    assert_eq!(CompassDirection::North.right(), CompassDirection::East);
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement CompassDirection**

```rust
// src/routing/compass_direction.rs

/// Four compass directions for VertexEntry[4] indexing and path search.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CompassDirection {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
}

impl CompassDirection {
    pub fn all() -> [Self; 4] {
        [Self::North, Self::East, Self::South, Self::West]
    }
    pub fn index(self) -> usize { self as usize }
    pub fn opposite(self) -> Self {
        match self {
            Self::North => Self::South,
            Self::East => Self::West,
            Self::South => Self::North,
            Self::West => Self::East,
        }
    }
    pub fn left(self) -> Self {
        match self {
            Self::North => Self::West,
            Self::East => Self::North,
            Self::South => Self::East,
            Self::West => Self::South,
        }
    }
    pub fn right(self) -> Self {
        match self {
            Self::North => Self::East,
            Self::East => Self::South,
            Self::South => Self::West,
            Self::West => Self::North,
        }
    }
}
```

- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 0.0b: Create VertexId Newtype

**Files:**
- Modify: `src/visibility/graph.rs` (add newtype)
- Test: inline `#[cfg(test)]`

The plan uses `VertexId(usize)` throughout all layers. The current codebase uses raw `usize`. Create the newtype now to avoid churn later.

- [ ] **Step 1: Add VertexId newtype to graph.rs**

```rust
/// Type-safe vertex identifier for the visibility graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);
```

- [ ] **Step 2: Update all existing code that uses raw `usize` for vertices to use `VertexId`**
- [ ] **Step 3: Run `cargo test` — all 261 tests pass**
- [ ] **Step 4: Commit**

---

### Task 0.0c: Build C# Golden Baseline Dump Tool

**Files:**
- Create: `tools/golden_dump/Program.cs` (~200 lines)
- Create: `tools/golden_dump/golden_dump.csproj`
- Output: `tests/golden_baselines/*.json`

**Goal:** Build a small C# console app that uses the MSAGL reference library to generate visibility graphs and routed paths for test scenarios, then serializes the results as JSON. This provides ground-truth comparison targets for every layer.

**Scenarios to dump:**
1. Two boxes side by side (simplest)
2. Three boxes in triangle (Diamond3 from RectilinearTests.cs)
3. Box with obstacle in between (reflection test)
4. Four boxes in grid (multi-path)

**JSON format per scenario:**
```json
{
  "scenario": "two_boxes",
  "obstacles": [{"left": 0, "bottom": 0, "right": 100, "top": 50}, ...],
  "padding": 4.0,
  "visibility_graph": {
    "vertices": [{"x": -5, "y": -5}, ...],
    "edges": [{"source": 0, "target": 1, "weight": 1}, ...]
  },
  "paths": [
    {"source": {"x": 50, "y": 25}, "target": {"x": 250, "y": 25}, "waypoints": [...]}
  ]
}
```

- [ ] **Step 1: Create C# project referencing MSAGL-Reference assemblies**
- [ ] **Step 2: Implement scenario setup (reuse RectilinearTests.cs patterns)**
- [ ] **Step 3: Run RectilinearEdgeRouter, serialize VG + paths to JSON**
- [ ] **Step 4: Save JSON files to `tests/golden_baselines/`**
- [ ] **Step 5: Commit**

**Note:** If building the C# project is blocked (missing .NET SDK, build issues), create the golden baselines manually by reading the TS source and computing expected outputs for simple scenarios. The C# dump is preferred but not blocking.

---

### Task 0.1: Expand ObstacleSide — Full Hierarchy

**Files:**
- Rewrite: `src/routing/obstacle_side.rs` (currently 18 lines → ~120 lines)
- Test: `tests/routing/obstacle_side_tests.rs`

**Port from:** `BasicObstacleSide.ts` (51 lines), C# `BasicObstacleSide.cs` (39 lines), `LowObstacleSide.cs` (22 lines), `HighObstacleSide.cs` (22 lines)

The current obstacle_side.rs is a 15-line struct with start/end points. The TS version has Low/High subtypes with slope tracking and clockwise/counter-clockwise polyline traversal.

- [ ] **Step 1: Write tests for obstacle side construction and slope computation**

```rust
// tests/routing/obstacle_side_tests.rs
use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
use msagl_rust::geometry::point::Point;

#[test]
fn low_side_has_correct_slope() {
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        0, // obstacle ordinal
    );
    assert_eq!(side.slope(), 0.0);
    assert_eq!(side.side_type(), SideType::Low);
}

#[test]
fn high_side_traverses_opposite_direction() {
    let low = ObstacleSide::new(SideType::Low, Point::new(0.0, 0.0), Point::new(10.0, 0.0), 0);
    let high = ObstacleSide::new(SideType::High, Point::new(0.0, 10.0), Point::new(10.0, 10.0), 0);
    // Low sides go in scan direction (left to right for horizontal)
    // High sides go opposite (right to left)
    assert!(low.start().x < low.end().x);
}

#[test]
fn vertical_side_slope_is_infinite() {
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(5.0, 0.0),
        Point::new(5.0, 10.0),
        0,
    );
    assert!(side.slope().is_infinite());
}
```

- [ ] **Step 2: Run tests — verify they fail (module doesn't exist yet)**

Run: `cargo test obstacle_side_tests -- --nocapture`
Expected: compilation error — `SideType` not defined

- [ ] **Step 3: Implement ObstacleSide with Low/High variants**

```rust
// src/routing/obstacle_side.rs
use crate::geometry::point::Point;

/// Whether this is a low (bottom/left) or high (top/right) side of an obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SideType {
    Low,
    High,
}

/// A side of an obstacle boundary, with slope tracking for scanline intersection.
///
/// Ported from BasicObstacleSide.ts + LowObstacleSide/HighObstacleSide.
#[derive(Debug, Clone)]
pub struct ObstacleSide {
    side_type: SideType,
    start: Point,
    end: Point,
    obstacle_ordinal: usize,
    slope: f64,
    slope_inverse: f64,
}

impl ObstacleSide {
    pub fn new(side_type: SideType, start: Point, end: Point, obstacle_ordinal: usize) -> Self {
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        let slope = if dx.abs() < 1e-10 { f64::INFINITY } else { dy / dx };
        let slope_inverse = if dy.abs() < 1e-10 { f64::INFINITY } else { dx / dy };
        Self { side_type, start, end, obstacle_ordinal, slope, slope_inverse }
    }

    pub fn side_type(&self) -> SideType { self.side_type }
    pub fn start(&self) -> Point { self.start }
    pub fn end(&self) -> Point { self.end }
    pub fn obstacle_ordinal(&self) -> usize { self.obstacle_ordinal }
    pub fn slope(&self) -> f64 { self.slope }
    pub fn slope_inverse(&self) -> f64 { self.slope_inverse }

    /// Compute the scanline intersection point at a given perpendicular coordinate.
    /// For horizontal scan: given a Y value, compute the X where this side crosses it.
    /// For vertical scan: given an X value, compute the Y where this side crosses it.
    pub fn scanline_intersection(&self, perp_coord: f64, is_horizontal_scan: bool) -> f64 {
        if is_horizontal_scan {
            // Given Y, find X
            if self.slope.is_infinite() {
                self.start.x
            } else {
                self.start.x + (perp_coord - self.start.y) * self.slope_inverse
            }
        } else {
            // Given X, find Y
            if self.slope == 0.0 {
                self.start.y
            } else if self.slope.is_infinite() {
                self.start.y + (perp_coord - self.start.x) * self.slope
            } else {
                self.start.y + (perp_coord - self.start.x) * self.slope
            }
        }
    }
}
```

- [ ] **Step 4: Run tests — verify they pass**

Run: `cargo test obstacle_side_tests -- --nocapture`
Expected: all 3 pass

- [ ] **Step 5: Commit**

```bash
git add src/routing/obstacle_side.rs tests/routing/obstacle_side_tests.rs
git commit -m "feat(msagl-rust): rewrite ObstacleSide with Low/High variants and slope tracking

Faithful port of BasicObstacleSide.ts + LowObstacleSide/HighObstacleSide.
Adds slope computation for scanline intersection during sweep."
```

---

### Task 0.2: Expand Obstacle — Sides, Ordinals, Sentinels

**Files:**
- Expand: `src/routing/obstacle.rs` (currently 66 lines → ~200 lines)
- Test: `tests/routing/obstacle_tests.rs` (existing — add tests)

**Port from:** `obstacle.ts` (239 lines), C# `Obstacle.cs` (272 lines)

Current obstacle.rs creates a padded bounding box. The TS version has:
- Ordinal numbering (for scanline tiebreaking)
- Active low/high sides (set during sweep)
- Sentinel obstacle creation (boundary markers)
- Polyline simplification (remove close/collinear vertices)
- Port tracking

- [ ] **Step 1: Write tests for obstacle ordinal assignment and sentinel creation**

```rust
#[test]
fn obstacles_get_sequential_ordinals() {
    let shapes = vec![
        Shape::new_rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::new_rectangle(200.0, 0.0, 100.0, 50.0),
    ];
    let obstacles: Vec<Obstacle> = shapes.iter().enumerate()
        .map(|(i, s)| Obstacle::new(s, 4.0, Obstacle::FIRST_NON_SENTINEL_ORDINAL + i))
        .collect();
    assert_eq!(obstacles[0].ordinal(), Obstacle::FIRST_NON_SENTINEL_ORDINAL);
    assert_eq!(obstacles[1].ordinal(), Obstacle::FIRST_NON_SENTINEL_ORDINAL + 1);
}

#[test]
fn sentinel_obstacle_has_sentinel_ordinal() {
    let sentinel = Obstacle::create_sentinel(
        Point::new(-1000.0, -1000.0),
        Point::new(1000.0, -1000.0),
        Obstacle::FIRST_SENTINEL_ORDINAL,
    );
    assert!(sentinel.is_sentinel());
    assert_eq!(sentinel.ordinal(), Obstacle::FIRST_SENTINEL_ORDINAL);
}

#[test]
fn obstacle_creates_sides_for_horizontal_scan() {
    let shape = Shape::new_rectangle(10.0, 20.0, 100.0, 50.0);
    let mut obs = Obstacle::new(&shape, 4.0, 10);
    obs.create_initial_sides(true); // horizontal scan
    let low = obs.active_low_side().unwrap();
    let high = obs.active_high_side().unwrap();
    // For horizontal scan, low side is the bottom, high side is the top
    assert!(low.start().y < high.start().y);
}
```

- [ ] **Step 2: Run tests — verify compilation fails**
- [ ] **Step 3: Implement expanded Obstacle**

Key additions to `src/routing/obstacle.rs`:
- `pub const FIRST_SENTINEL_ORDINAL: usize = 1`
- `pub const FIRST_NON_SENTINEL_ORDINAL: usize = 10`
- `ordinal: usize` field
- `active_low_side: Option<ObstacleSide>` and `active_high_side: Option<ObstacleSide>`
- `is_sentinel: bool` flag
- `create_initial_sides(&mut self, is_horizontal_scan: bool)` — creates Low/High sides from padded polyline corners
- `create_sentinel(a: Point, b: Point, ordinal: usize) -> Self`
- Keep existing `padded_bounding_box()` and `padded_polyline()` methods

- [ ] **Step 4: Run tests — verify they pass**
- [ ] **Step 5: Commit**

---

### Task 0.3: Expand ScanDirection — Full Comparison API

**Files:**
- Expand: `src/routing/scan_direction.rs` (currently 65 lines → ~130 lines)
- Test: `tests/routing/scan_direction_tests.rs` (existing — add tests)

**Port from:** `ScanDirection.ts` (112 lines)

Current scan_direction.rs has a Direction enum with coord/perp_coord methods. The TS version adds:
- Comparison operators for perpendicular and parallel coordinates
- `is_flat()` / `is_perpendicular()` checks
- `min()` / `max()` helpers
- Static `HorizontalInstance` / `VerticalInstance`

- [ ] **Step 1: Write tests for direction-aware comparison**

```rust
#[test]
fn horizontal_scan_perp_is_y() {
    let dir = ScanDirection::horizontal();
    assert_eq!(dir.perp_coord(Point::new(10.0, 20.0)), 20.0);
    assert_eq!(dir.coord(Point::new(10.0, 20.0)), 10.0);
}

#[test]
fn compare_perp_coords() {
    let dir = ScanDirection::horizontal();
    assert!(dir.compare_perp(10.0, 20.0).is_lt());
    assert!(dir.compare_perp(20.0, 10.0).is_gt());
}

#[test]
fn is_flat_checks_perpendicular_alignment() {
    let dir = ScanDirection::horizontal();
    let a = Point::new(0.0, 5.0);
    let b = Point::new(10.0, 5.0);
    assert!(dir.is_flat(a, b)); // same Y = flat for horizontal scan
}
```

- [ ] **Step 2: Run tests — verify fail**
- [ ] **Step 3: Implement expanded ScanDirection**

Add to `ScanDirection`:
- `pub fn compare_perp(&self, a: f64, b: f64) -> Ordering`
- `pub fn compare_scan(&self, a: f64, b: f64) -> Ordering`
- `pub fn is_flat(&self, a: Point, b: Point) -> bool` — same perp coord within epsilon
- `pub fn is_perpendicular(&self, a: Point, b: Point) -> bool` — same scan coord within epsilon
- `pub fn min_perp(&self, a: f64, b: f64) -> f64`
- `pub fn max_perp(&self, a: f64, b: f64) -> f64`
- `pub fn perpendicular(&self) -> ScanDirection` — returns the other direction
- `pub fn horizontal() -> Self` and `pub fn vertical() -> Self` constructors

- [ ] **Step 4: Run tests — verify pass**
- [ ] **Step 5: Commit**

---

### Task 0.4: Expand ObstacleTree — Full Query API

**Files:**
- Expand: `src/routing/obstacle_tree.rs` (currently 87 lines → ~250 lines)
- Test: `tests/routing/obstacle_tree_tests.rs`

**Port from:** `ObstacleTree.ts` (823 lines — but we defer groups/clusters, so ~300 lines relevant)

Current obstacle_tree.rs is a basic rstar wrapper. We need:
- `all_obstacles()` accessor
- `graph_box()` — bounding box of all obstacles with margin
- Overlap detection between obstacles
- `inside_hit_test(point)` — which obstacle contains a point
- Obstacle ordinal management
- Sentinel creation at graph boundaries

**Note:** Group/cluster routing is DEFERRED per spec. We only need the non-group query API.

- [ ] **Step 1: Write tests for graph box computation and inside-hit-test**

```rust
#[test]
fn graph_box_encompasses_all_obstacles_with_margin() {
    let shapes = vec![
        Shape::new_rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::new_rectangle(200.0, 100.0, 80.0, 60.0),
    ];
    let tree = ObstacleTree::new(&shapes, 4.0);
    let gb = tree.graph_box();
    // Graph box should contain all padded obstacles plus a sentinel offset
    assert!(gb.left < -4.0);
    assert!(gb.bottom < -4.0);
    assert!(gb.right > 280.0 + 4.0);
    assert!(gb.top > 160.0 + 4.0);
}

#[test]
fn inside_hit_test_finds_containing_obstacle() {
    let shapes = vec![
        Shape::new_rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::new_rectangle(200.0, 0.0, 100.0, 50.0),
    ];
    let tree = ObstacleTree::new(&shapes, 4.0);
    let hit = tree.inside_hit_test(Point::new(50.0, 25.0));
    assert!(hit.is_some());
    assert_eq!(hit.unwrap(), 0); // first obstacle
}

#[test]
fn inside_hit_test_returns_none_for_empty_space() {
    let shapes = vec![Shape::new_rectangle(0.0, 0.0, 100.0, 50.0)];
    let tree = ObstacleTree::new(&shapes, 4.0);
    assert!(tree.inside_hit_test(Point::new(500.0, 500.0)).is_none());
}
```

- [ ] **Step 2: Run tests — verify fail**
- [ ] **Step 3: Implement expanded ObstacleTree**
- [ ] **Step 4: Run tests — verify pass**
- [ ] **Step 5: Run `cargo clippy -- -D warnings` and fix any issues**
- [ ] **Step 6: Commit**

---

## Layer 0.5: Solver Failure Triage

### Task 0.5.1: Investigate 6 Failing Solver Fixtures (TRIAGE ONLY — DO NOT FIX)

**Files:**
- Read only: `tests/projection_solver/fixture_tests.rs`
- Read only: 6 fixture files in `tests/fixtures/projection_solver/`
- Write: `docs/solver-failure-triage.md` (investigation notes)

**Goal:** Spend ~2 hours reading the 6 failing fixtures and categorize root causes. Determine if any failures touch the constraint model architecture vs. being edge-case numerics.

The 6 ignored tests are:
1. `Cycles_Vars100_ConstraintsMax10_EqualityConstraints_PosMax1M_GapMax100K_WeightMax10K_Cycles10`
2. `Cycles_Vars100_ConstraintsMax10_PosMax1M_GapMax100K_WeightMax10K_Cycles10`
3. `Cycles_Vars500_ConstraintsMax10_EqualityConstraints_PosMax1M_GapMax100K_WeightMax10K_Cycles10`
4. `Cycles_Vars500_ConstraintsMax10_PosMax1M_GapMax100K_WeightMax10K_Cycles10`
5. `Neighbors_Vars1000_ConstraintsMax10_NeighborsMax10_NeighborWeightMax100_VarWeights_1_To_1E6_At_10_Percent`
6. `Neighbors_Vars400_ConstraintsMax10_NeighborsMax10_WeightMax100`

- [ ] **Step 1: Un-ignore each test one at a time, run it, capture the error output**

```bash
# For each test, temporarily remove #[ignore], run, capture output
cargo test cycles_vars100_constraintsmax10_equalityconstraints -- --nocapture 2>&1 | head -50
```

- [ ] **Step 2: For each failure, document: what assertion fails, by how much, and whether the issue is**
  - (a) Numerical convergence (solver gets close but not within tolerance)
  - (b) Cycle detection logic (fundamentally wrong handling of constraint cycles)
  - (c) Missing feature (equality constraints not fully implemented)

- [ ] **Step 3: Write triage document**

```markdown
<!-- docs/solver-failure-triage.md -->
# Solver Failure Triage

## Classification

| Fixture | Root Cause | Severity | Blocks Layers |
|---|---|---|---|
| Cycles_100_Equality | ? | ? | ? |
| ... | | | |

## Conclusion
[Does anything here block Layers 0-6? If yes, what needs to change?]
```

- [ ] **Step 4: Commit the triage document**

---

## Layer 1: Event System

### Task 1.1: Rewrite EventQueue — Full Event Hierarchy

**Files:**
- Rewrite: `src/routing/event_queue.rs` (currently 109 lines → ~200 lines)
- Test: `tests/routing/event_queue_tests.rs` (existing — expand)

**Port from:** `EventQueue.ts` (64 lines), `BasicVertexEvent.ts` (13 lines), `OpenVertexEvent.ts` (10 lines), `MiscVertexEvents.ts` (21 lines), `BasicReflectionEvent.ts` (41 lines), `HighReflectionEvent.ts` (13 lines), `LowReflectionEvent.ts` (13 lines)

Current event_queue.rs has only Open/Close event types. The TS has a full hierarchy:

```
SweepEvent
├── VertexEvent
│   └── BasicVertexEvent { obstacle_index }
│       ├── OpenVertexEvent
│       ├── CloseVertexEvent
│       ├── LowBendVertexEvent
│       └── HighBendVertexEvent
├── BasicReflectionEvent { initial_obstacle, reflecting_obstacle, site, prev_event }
│   ├── LowReflectionEvent { low_side }
│   └── HighReflectionEvent { high_side }
└── AxisCoordinateEvent { site } (for lookahead)
```

In Rust, this becomes a flat enum:

- [ ] **Step 1: Write tests for event ordering**

```rust
#[test]
fn events_ordered_by_perp_coord_first() {
    let mut queue = EventQueue::new(ScanDirection::horizontal());
    queue.enqueue(SweepEvent::open_vertex(Point::new(5.0, 20.0), 0));
    queue.enqueue(SweepEvent::open_vertex(Point::new(3.0, 10.0), 1));
    // Horizontal scan: perp coord is Y. Y=10 comes before Y=20
    let first = queue.dequeue().unwrap();
    assert_eq!(first.site().y, 10.0);
}

#[test]
fn reflection_events_before_vertex_events_at_same_coord() {
    let mut queue = EventQueue::new(ScanDirection::horizontal());
    queue.enqueue(SweepEvent::open_vertex(Point::new(5.0, 10.0), 0));
    queue.enqueue(SweepEvent::low_reflection(Point::new(3.0, 10.0), 0, 1, None));
    let first = queue.dequeue().unwrap();
    assert!(matches!(first, SweepEvent::LowReflection { .. }));
}
```

- [ ] **Step 2: Run tests — verify fail**
- [ ] **Step 3: Implement SweepEvent enum and EventQueue**

```rust
// src/routing/event_queue.rs
use std::collections::BinaryHeap;
use std::cmp::Ordering;
use crate::geometry::point::Point;
use super::scan_direction::ScanDirection;
use super::obstacle_side::ObstacleSide;

/// All sweep event types for visibility graph generation.
/// Ported from TS: BasicVertexEvent, OpenVertexEvent, MiscVertexEvents,
/// BasicReflectionEvent, HighReflectionEvent, LowReflectionEvent.
#[derive(Debug, Clone)]
pub enum SweepEvent {
    /// Obstacle corner entering sweep range
    OpenVertex { site: Point, obstacle_index: usize },
    /// Obstacle corner leaving sweep range
    CloseVertex { site: Point, obstacle_index: usize },
    /// Low bend at obstacle corner
    LowBend { site: Point, obstacle_index: usize },
    /// High bend at obstacle corner
    HighBend { site: Point, obstacle_index: usize },
    /// Reflection off a low obstacle side
    LowReflection {
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
        prev_event_index: Option<usize>,
    },
    /// Reflection off a high obstacle side
    HighReflection {
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
        prev_event_index: Option<usize>,
    },
}

impl SweepEvent {
    pub fn site(&self) -> Point { /* match on all variants, return site */ }

    /// Event type priority (lower = processed first at same coordinate).
    /// Reflection events before vertex events.
    fn type_priority(&self) -> u8 {
        match self {
            Self::LowReflection { .. } | Self::HighReflection { .. } => 0,
            Self::OpenVertex { .. } => 1,
            Self::LowBend { .. } | Self::HighBend { .. } => 2,
            Self::CloseVertex { .. } => 3,
        }
    }
}
```

EventQueue wraps `BinaryHeap` with a wrapper struct implementing `Ord` based on:
1. Perpendicular coordinate (primary)
2. Event type priority (secondary)
3. Scan coordinate (tertiary)

- [ ] **Step 4: Run tests — verify pass**
- [ ] **Step 5: Run existing tests to verify no regressions**

Run: `cargo test`
Expected: all 261 existing tests still pass

- [ ] **Step 6: Commit**

---

### Task 1.2: Rewrite RectilinearScanLine — Full API

**Files:**
- Rewrite: `src/routing/scan_line.rs` (currently 94 lines → ~200 lines)
- Test: `tests/routing/scan_line_tests.rs` (existing — expand)

**Port from:** `RectilinearScanLine.ts` (204 lines)

Current scan_line.rs uses BTreeMap with basic insert/remove/neighbor queries. The TS version adds:
- Side comparison using scanline intersection point (dynamic)
- Position tracking for insert/remove
- Proper low/high neighbor traversal

- [ ] **Step 1: Write tests for scanline intersection-based ordering**

```rust
#[test]
fn sides_ordered_by_scanline_intersection() {
    let mut scanline = RectilinearScanLine::new(ScanDirection::horizontal());
    // Two vertical sides at x=10 and x=30
    let side_a = ObstacleSide::new(SideType::Low, Point::new(10.0, 0.0), Point::new(10.0, 100.0), 0);
    let side_b = ObstacleSide::new(SideType::Low, Point::new(30.0, 0.0), Point::new(30.0, 100.0), 1);
    scanline.insert(side_a.clone());
    scanline.insert(side_b.clone());
    // At any Y position, side_a (x=10) should come before side_b (x=30)
    let low_neighbor = scanline.find_low_neighbor(15.0);
    assert_eq!(low_neighbor.unwrap().obstacle_ordinal(), 0);
}

#[test]
fn find_neighbors_returns_adjacent_sides() {
    let mut scanline = RectilinearScanLine::new(ScanDirection::horizontal());
    let side_a = ObstacleSide::new(SideType::Low, Point::new(10.0, 0.0), Point::new(10.0, 100.0), 0);
    let side_b = ObstacleSide::new(SideType::High, Point::new(30.0, 0.0), Point::new(30.0, 100.0), 1);
    scanline.insert(side_a);
    scanline.insert(side_b);
    // Query between the two sides
    let (low, high) = scanline.find_neighbors(20.0);
    assert_eq!(low.unwrap().obstacle_ordinal(), 0);
    assert_eq!(high.unwrap().obstacle_ordinal(), 1);
}
```

- [ ] **Step 2: Run tests — verify fail**
- [ ] **Step 3: Implement full RectilinearScanLine**

Use `BTreeMap<SideKey, ObstacleSide>` where `SideKey` orders by scanline intersection point, then side type (High before Low), then obstacle ordinal for tiebreaking.

- [ ] **Step 4: Run tests — verify pass**
- [ ] **Step 5: Commit**

---

### Task 1.3: Create NeighborSides

**Files:**
- Create: `src/routing/neighbor_sides.rs` (~80 lines)
- Test: inline `#[cfg(test)]` module

**Port from:** `NeighborSides.ts` (70 lines)

Simple data structure holding low and high neighbor sides during scanline traversal. In TS it has nullable LowNeighbor, HighNeighbor, and overlap-end tracking.

- [ ] **Step 1: Write tests**

```rust
#[test]
fn neighbor_sides_set_and_get() {
    let mut ns = NeighborSides::new();
    let low = ObstacleSide::new(SideType::Low, Point::new(0.0, 0.0), Point::new(0.0, 10.0), 0);
    let high = ObstacleSide::new(SideType::High, Point::new(20.0, 0.0), Point::new(20.0, 10.0), 1);
    ns.set_sides(Some(low.clone()), Some(high.clone()));
    assert_eq!(ns.low_neighbor().unwrap().obstacle_ordinal(), 0);
    assert_eq!(ns.high_neighbor().unwrap().obstacle_ordinal(), 1);
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement NeighborSides**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 1.4: Create LookaheadScan

**Files:**
- Create: `src/routing/lookahead_scan.rs` (~120 lines)
- Test: inline `#[cfg(test)]` module

**Port from:** `LookaheadScan.ts` (119 lines)

BTreeMap-based collection of reflection events for lookahead processing. Supports range queries, stale-site removal, and comparison by scan-parallel coordinate.

- [ ] **Step 1: Write tests**

```rust
#[test]
fn lookahead_stores_and_retrieves_reflection_events() {
    let mut scan = LookaheadScan::new(ScanDirection::horizontal());
    scan.add_site(Point::new(10.0, 20.0), 0, 1);
    let found = scan.find_first_in_range(5.0, 15.0);
    assert!(found.is_some());
}

#[test]
fn lookahead_removes_stale_sites() {
    let mut scan = LookaheadScan::new(ScanDirection::horizontal());
    scan.add_site(Point::new(10.0, 20.0), 0, 1);
    scan.remove_stale(Point::new(10.0, 20.0));
    assert!(scan.find_first_in_range(5.0, 15.0).is_none());
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement LookaheadScan using BTreeMap**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Update `src/routing/mod.rs` to export all new Layer 1 modules**
- [ ] **Step 6: Run full `cargo test` — verify all existing tests still pass**
- [ ] **Step 7: Commit**

---

## Layer 2: Visibility Graph Generator

### Task 2.1: Rewrite ScanSegment — Weights + Full API

**Files:**
- Rewrite: `src/routing/scan_segment.rs` (currently 103 lines → ~300 lines)
- Test: `tests/routing/scan_segment_tests.rs` (existing — expand)

**Port from:** `ScanSegment.ts` (499 lines)

Current scan_segment.rs has basic start/end/weight. The TS version adds:
- **Weight enum: Normal(1), Reflection(5), Overlapped(500)** — CRITICAL for path quality
- Lowest/highest visibility vertex tracking
- Overlap state tracking
- Subsumption logic for collinear segments
- Sparse vertex coordinate list

- [ ] **Step 1: Write tests for segment weight system**

```rust
#[test]
fn scan_segment_weight_ordering() {
    assert!(SegmentWeight::Normal < SegmentWeight::Reflection);
    assert!(SegmentWeight::Reflection < SegmentWeight::Overlapped);
}

#[test]
fn segment_weight_values() {
    assert_eq!(SegmentWeight::Normal.value(), 1);
    assert_eq!(SegmentWeight::Reflection.value(), 5);
    assert_eq!(SegmentWeight::Overlapped.value(), 500);
}

#[test]
fn segment_tracks_visibility_vertices() {
    let mut seg = ScanSegment::new(Point::new(0.0, 10.0), Point::new(100.0, 10.0), SegmentWeight::Normal, true);
    let v1 = VertexId(0);
    let v2 = VertexId(5);
    seg.set_lowest_vertex(v1);
    seg.set_highest_vertex(v2);
    assert_eq!(seg.lowest_vertex(), Some(v1));
    assert_eq!(seg.highest_vertex(), Some(v2));
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement expanded ScanSegment with weight system**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 2.2: Create ScanSegmentTree

**Files:**
- Create: `src/routing/scan_segment_tree.rs` (~250 lines)
- Test: `tests/routing/scan_segment_tree_tests.rs`

**Port from:** `ScanSegmentTree.ts` (311 lines)

BTreeMap-based tree of ScanSegments ordered by start coordinate. Supports:
- `find_lowest_intersector(perp_coord)` — first segment crossing a perpendicular line
- `find_highest_intersector(perp_coord)` — last segment crossing a perpendicular line
- `merge_segments()` — combine adjacent/overlapping collinear segments
- Insert with uniqueness check
- Segment next/prev navigation

- [ ] **Step 1: Write tests for segment tree operations**

```rust
#[test]
fn find_lowest_intersector_returns_first_crossing_segment() {
    let mut tree = ScanSegmentTree::new();
    tree.insert(ScanSegment::new(Point::new(0.0, 10.0), Point::new(50.0, 10.0), SegmentWeight::Normal, true));
    tree.insert(ScanSegment::new(Point::new(60.0, 10.0), Point::new(100.0, 10.0), SegmentWeight::Normal, true));
    // Perpendicular line at x=25 should hit first segment
    let found = tree.find_lowest_intersector(25.0);
    assert!(found.is_some());
    assert_eq!(found.unwrap().start().x, 0.0);
}

#[test]
fn merge_adjacent_segments() {
    let mut tree = ScanSegmentTree::new();
    tree.insert(ScanSegment::new(Point::new(0.0, 10.0), Point::new(50.0, 10.0), SegmentWeight::Normal, true));
    tree.insert(ScanSegment::new(Point::new(50.0, 10.0), Point::new(100.0, 10.0), SegmentWeight::Normal, true));
    tree.merge_segments();
    assert_eq!(tree.len(), 1);
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement ScanSegmentTree using BTreeMap**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 2.3: Rewrite VisibilityGraphGenerator — Full Sweep-Line

**Files:**
- Rewrite: `src/routing/visibility_graph_generator.rs` (currently 232 lines → ~500 lines)
- Test: `tests/routing/visibility_gen_tests.rs` (existing — expand significantly)

**Port from:** `VisibilityGraphGenerator.ts` (1,013 lines), C# `VisibilityGraphGenerator.cs` (996 lines)

This is the biggest single rewrite. The current simplified ray-cast must become a full two-pass event-driven sweep:

**Algorithm (from TS):**
1. Two perpendicular scans: horizontal (vertical sweep, horizontal segments) and vertical (horizontal sweep, vertical segments)
2. Initialize EventQueue with obstacle corner events
3. Add sentinels at graph boundaries
4. Process events in sweep order:
   - **OpenVertex:** Add obstacle's low/high sides to scanline, find neighbors, create scan segments in the gaps
   - **LowBend/HighBend:** Update obstacle sides on scanline
   - **CloseVertex:** Remove obstacle sides, create segments in newly opened gaps
   - **Reflection events:** Create weighted (×5) reflection segments for visibility around obstacles
5. Build two ScanSegmentTrees (horizontal + vertical)
6. Pass both trees to SegmentIntersector to create the visibility graph

- [ ] **Step 1: Write golden baseline tests**

Before implementing, create a test that compares Rust output against known-correct VG for simple scenarios:

```rust
#[test]
fn two_boxes_visibility_graph_vertex_count() {
    // Two 100x50 boxes separated by 100px gap
    let shapes = vec![
        Shape::new_rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::new_rectangle(200.0, 0.0, 100.0, 50.0),
    ];
    let graph = generate_visibility_graph(&shapes, 4.0);
    // Expected: padded corners (8 per box = 16) + sentinel corners + intersection points
    // The exact count depends on the sweep algorithm; we'll update after C# golden baseline
    assert!(graph.vertex_count() > 0);
    // Every vertex should have at least one edge
    for v in 0..graph.vertex_count() {
        let vid = VertexId(v);
        assert!(graph.out_degree(vid) + graph.in_degree(vid) > 0,
            "vertex {:?} at {:?} is disconnected", vid, graph.point(vid));
    }
}

#[test]
fn single_box_visibility_graph_is_grid() {
    let shapes = vec![Shape::new_rectangle(50.0, 50.0, 100.0, 60.0)];
    let graph = generate_visibility_graph(&shapes, 4.0);
    // Single obstacle: VG should form a grid around it
    // All edges should be axis-aligned
    for v in 0..graph.vertex_count() {
        let vid = VertexId(v);
        let p = graph.point(vid);
        for edge in graph.out_edges(vid) {
            let tp = graph.point(edge.target);
            assert!(
                (p.x - tp.x).abs() < 1e-10 || (p.y - tp.y).abs() < 1e-10,
                "non-rectilinear edge from {:?} to {:?}", p, tp
            );
        }
    }
}
```

- [ ] **Step 2: Run tests — verify they fail or produce incorrect results with current simplified generator**

- [ ] **Step 3: Implement the full sweep-line algorithm**

The generator struct:

```rust
pub struct VisibilityGraphGenerator {
    scan_direction: ScanDirection,
    event_queue: EventQueue,
    scan_line: RectilinearScanLine,
    h_scan_segments: ScanSegmentTree,
    v_scan_segments: ScanSegmentTree,
    obstacle_tree: ObstacleTree,
    graph: VisibilityGraph,
    lookahead_scan: LookaheadScan,
    neighbor_sides: NeighborSides,
}

impl VisibilityGraphGenerator {
    pub fn generate(shapes: &[Shape], padding: f64) -> VisibilityGraph {
        let mut gen = Self::new(shapes, padding);
        // Pass 1: horizontal scan (creates horizontal segments)
        gen.run_scan(ScanDirection::horizontal());
        // Pass 2: vertical scan (creates vertical segments)
        gen.run_scan(ScanDirection::vertical());
        // Intersect horizontal and vertical segments to build graph
        SegmentIntersector::intersect(
            &gen.h_scan_segments,
            &gen.v_scan_segments,
            &mut gen.graph,
        );
        gen.graph
    }

    fn run_scan(&mut self, direction: ScanDirection) {
        self.scan_direction = direction;
        self.initialize_event_queue();
        self.process_events();
    }

    fn initialize_event_queue(&mut self) { /* enqueue obstacle corner events */ }
    fn process_events(&mut self) { /* main sweep loop */ }
    fn process_open_vertex(&mut self, event: &SweepEvent) { /* add sides, find gaps */ }
    fn process_close_vertex(&mut self, event: &SweepEvent) { /* remove sides */ }
    fn process_low_bend(&mut self, event: &SweepEvent) { /* update sides */ }
    fn process_high_bend(&mut self, event: &SweepEvent) { /* update sides */ }
    fn process_reflection(&mut self, event: &SweepEvent) { /* create reflection segments */ }
    fn create_scan_segment(&mut self, start: Point, end: Point, weight: SegmentWeight) { /* add to tree */ }
}
```

**IMPORTANT:** Read VisibilityGraphGenerator.ts line-by-line while implementing. Do not simplify the event processing logic.

- [ ] **Step 4: Run tests — verify pass**
- [ ] **Step 5: Commit**

---

### Task 2.4: Rewrite SegmentIntersector

**Files:**
- Rewrite: `src/routing/segment_intersector.rs` (currently 218 lines → ~300 lines)
- Test: `tests/routing/segment_intersector_tests.rs`

**Port from:** `SegmentIntersector.ts` (307 lines)

Current implementation creates vertices at H/V segment crossings. The TS version additionally:
- Creates visibility edges with proper weights
- Handles segments without visibility (dead-ends)
- Uses event-based processing (VOpen, VClose, HOpen)
- Tracks segment-to-vertex associations

- [ ] **Step 1: Write tests for segment intersection with weights**

```rust
#[test]
fn intersecting_segments_create_vertex_with_correct_weight() {
    let mut h_tree = ScanSegmentTree::new();
    let mut v_tree = ScanSegmentTree::new();
    h_tree.insert(ScanSegment::new(
        Point::new(0.0, 50.0), Point::new(100.0, 50.0), SegmentWeight::Normal, true
    ));
    v_tree.insert(ScanSegment::new(
        Point::new(50.0, 0.0), Point::new(50.0, 100.0), SegmentWeight::Reflection, false
    ));
    let mut graph = VisibilityGraph::new();
    SegmentIntersector::intersect(&h_tree, &v_tree, &mut graph);
    // Should create vertex at (50, 50)
    let v = graph.find_vertex(Point::new(50.0, 50.0));
    assert!(v.is_some());
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement faithful SegmentIntersector**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Run full `cargo test`**
- [ ] **Step 6: Commit**

---

## Layer 2.5: End-to-End Smoke Test

### Task 2.5.1: Smoke Test — Route One Edge Through New VG

**Files:**
- Create: `tests/smoke_test.rs` (~100 lines)

**Goal:** Wire the new VG generator with the existing (simplified) path search and port manager. Route one edge between two boxes. Verify the path exists and is rectilinear. This is NOT a fidelity test — it's a structural validity test.

- [ ] **Step 1: Write the smoke test**

```rust
#[test]
fn smoke_route_one_edge_between_two_boxes() {
    let mut router = RectilinearEdgeRouter::new(&[
        Shape::new_rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::new_rectangle(300.0, 0.0, 100.0, 50.0),
    ]);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(50.0, 25.0)),
        FloatingPort::new(1, Point::new(350.0, 25.0)),
    ));
    let result = router.padding(4.0).edge_separation(8.0).run();
    assert_eq!(result.edges.len(), 1);
    let path = &result.edges[0].points;
    assert!(path.len() >= 2, "path should have at least 2 points");
    // Verify all segments are axis-aligned
    for w in path.windows(2) {
        assert!(
            (w[0].x - w[1].x).abs() < 1e-10 || (w[0].y - w[1].y).abs() < 1e-10,
            "non-rectilinear segment: {:?} -> {:?}", w[0], w[1]
        );
    }
    // Path should start near source and end near target
    assert!((path[0].x - 50.0).abs() < 20.0);
    assert!((path.last().unwrap().x - 350.0).abs() < 20.0);
}

#[test]
fn smoke_route_three_edges_between_four_boxes() {
    let shapes = vec![
        Shape::new_rectangle(0.0, 0.0, 80.0, 40.0),
        Shape::new_rectangle(200.0, 0.0, 80.0, 40.0),
        Shape::new_rectangle(0.0, 150.0, 80.0, 40.0),
        Shape::new_rectangle(200.0, 150.0, 80.0, 40.0),
    ];
    let mut router = RectilinearEdgeRouter::new(&shapes);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(40.0, 20.0)),
        FloatingPort::new(1, Point::new(240.0, 20.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(2, Point::new(40.0, 170.0)),
        FloatingPort::new(3, Point::new(240.0, 170.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(40.0, 20.0)),
        FloatingPort::new(3, Point::new(240.0, 170.0)),
    ));
    let result = router.padding(4.0).run();
    assert_eq!(result.edges.len(), 3);
    for edge in &result.edges {
        assert!(edge.points.len() >= 2);
    }
}
```

- [ ] **Step 2: Run smoke tests**

Run: `cargo test smoke_ -- --nocapture`
Expected: PASS — if they fail, debug the VG generator before proceeding

- [ ] **Step 3: Commit**

---

## Layer 3: VertexEntry[4] + Direction-Aware Vertex

### Task 3.1: Create VertexEntry

**Files:**
- Create: `src/routing/vertex_entry.rs` (~80 lines)
- Test: inline `#[cfg(test)]`

**Port from:** `VertexEntry.ts` (75 lines)

Each VertexEntry tracks: vertex, direction of entry, cost, length, bend count, previous entry (for path reconstruction), and closed flag.

- [ ] **Step 1: Write tests**

```rust
#[test]
fn vertex_entry_tracks_direction_and_cost() {
    let entry = VertexEntry::new(VertexId(0), CompassDirection::East, 10.0, 10.0, 0, None);
    assert_eq!(entry.direction(), CompassDirection::East);
    assert_eq!(entry.length(), 10.0);
    assert_eq!(entry.number_of_bends(), 0);
}

#[test]
fn vertex_entry_cost_combines_length_and_bends() {
    let entry = VertexEntry::new(VertexId(0), CompassDirection::East, 100.0, 100.0, 2, None);
    // cost should be length + bend_penalty * bends
    // Default bend penalty: 4% of distance → 100 * 0.04 * 2 = 8
    // Total: 100 + 8 = 108
    assert!((entry.cost() - 108.0).abs() < 1e-10);
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement VertexEntry**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 3.2: Expand VisibilityGraph — VertexEntry[4] Per Vertex

**Files:**
- Expand: `src/visibility/graph.rs` (currently 94 lines → ~160 lines)
- Test: `tests/visibility/graph_tests.rs` (existing — expand)

**Port from:** `VisibilityVertexRectilinear.ts` (23 lines), `VisibilityVertex.ts` (128 lines)

Add `vertex_entries: [Option<VertexEntryIndex>; 4]` to `VertexData`. Each slot corresponds to N/E/S/W entry direction. The path search will use these to track the cost of entering a vertex from each direction independently.

- [ ] **Step 1: Write tests**

```rust
#[test]
fn vertex_has_four_entry_slots() {
    let mut graph = VisibilityGraph::new();
    let v = graph.add_vertex(Point::new(10.0, 20.0));
    // All entry slots should start as None
    for dir in CompassDirection::all() {
        assert!(graph.vertex_entry(v, dir).is_none());
    }
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Add VertexEntry[4] to VertexData**
- [ ] **Step 4: Run — pass, plus all existing tests**
- [ ] **Step 5: Commit**

---

## Layer 4: Path Search Rewrite

### Task 4.1: Rewrite SsstRectilinearPath

**Files:**
- Rewrite: `src/routing/path_search.rs` (currently 352 lines → ~450 lines)
- Test: `tests/routing/path_search_tests.rs` (existing — expand significantly)

**Port from:** `SsstRectilinearPath.ts` (584 lines)

Current path_search.rs has a basic A* with bend counting. The TS version:
- Uses VertexEntry[4] per vertex for direction tracking
- Has NextNeighbor[3] optimization (straight, preferred bend, other bend)
- Computes heuristic with estimated bends to target
- Handles entry direction constraints at target

Key differences in the rewrite:
- Each vertex has 4 independent entries (one per direction)
- When expanding a vertex, neighbors are tried in order: straight, right turn, left turn
- Cost includes both length and bend penalty
- Default bend penalty: 4% of Manhattan distance between source and target

- [ ] **Step 1: Write tests for direction-aware pathfinding**

```rust
#[test]
fn path_search_prefers_straight_over_bends() {
    // Create an L-shaped graph where straight path exists
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(100.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 100.0));
    let d = graph.add_vertex(Point::new(0.0, 100.0));
    // Direct path: a → b (straight east)
    graph.add_edge(a, b, 1.0);
    // Bent path: a → d → c → b (north, east, south = 2 bends)
    graph.add_edge(a, d, 1.0);
    graph.add_edge(d, c, 1.0);
    graph.add_edge(c, b, 1.0);
    let search = SsstRectilinearPath::new(0.5); // bend penalty 0.5
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(100.0, 0.0));
    assert!(path.is_some());
    let points = path.unwrap();
    // Should take direct path (2 points) not bent path (4 points)
    assert_eq!(points.len(), 2);
}

#[test]
fn path_search_finds_path_around_obstacle() {
    // Source at left, target at right, obstacle in between
    // Graph has paths going above and below the obstacle
    let mut graph = VisibilityGraph::new();
    // ... (create vertices and edges for obstacle avoidance scenario)
    let search = SsstRectilinearPath::new(0.5);
    let path = search.find_path(&graph, source, target);
    assert!(path.is_some());
    // All segments should be rectilinear
    for w in path.unwrap().windows(2) {
        assert!((w[0].x - w[1].x).abs() < 1e-10 || (w[0].y - w[1].y).abs() < 1e-10);
    }
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement full SsstRectilinearPath**

Key structures:
```rust
pub struct SsstRectilinearPath {
    bend_penalty_as_percentage: f64,
}

struct NextNeighbor {
    vertex: VertexId,
    weight: f64,
}

impl SsstRectilinearPath {
    pub fn find_path(&self, graph: &mut VisibilityGraph, source: Point, target: Point) -> Option<Vec<Point>>;
    fn enqueue_initial_entries(&self, ...);
    fn extend_path_to_neighbor_vertex(&self, ...);
    fn get_entry_directions_to_target(&self, graph: &VisibilityGraph, target: VertexId) -> u8;
    fn combined_cost(&self, length: f64, bends: u32, source: Point, target: Point) -> f64;
    fn estimated_bends_to_target(direction: CompassDirection, point: Point, target: Point) -> u32;
}
```

- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Run smoke tests from Layer 2.5 to verify integration**
- [ ] **Step 6: Commit**

---

### Task 4.2: Create MsmtRectilinearPath

**Files:**
- Create: `src/routing/msmt_path.rs` (~120 lines)
- Test: inline `#[cfg(test)]`

**Port from:** `MsmtRectilinearPath.ts` (158 lines)

Multi-source, multi-target wrapper around SsstRectilinearPath. Finds the lowest-cost path across all source-target pairs.

- [ ] **Step 1: Write tests**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement MsmtRectilinearPath**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

## Layer 5: Port Manager + Transient Graph

### Task 5.1: Create StaticGraphUtility

**Files:**
- Create: `src/routing/static_graph_utility.rs` (~200 lines)
- Test: inline `#[cfg(test)]`

**Port from:** `StaticGraphUtility.ts` (238 lines)

Static utility functions for graph analysis: edge direction determination, adjacent vertex/edge finding, bend point calculations, segment intersection testing, collinearity checks.

- [ ] **Step 1: Write tests for key utilities**

```rust
#[test]
fn edge_direction_horizontal() {
    let dir = StaticGraphUtility::edge_direction(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
    );
    assert_eq!(dir, CompassDirection::East);
}

#[test]
fn find_adjacent_vertex() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(10.0, 0.0));
    let c = graph.add_vertex(Point::new(20.0, 0.0));
    graph.add_edge(a, b, 1.0);
    graph.add_edge(b, c, 1.0);
    let next = StaticGraphUtility::find_adjacent_vertex(&graph, b, CompassDirection::East);
    assert_eq!(next, Some(c));
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement StaticGraphUtility**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 5.2: Create SpliceUtility

**Files:**
- Create: `src/routing/splice_utility.rs` (~40 lines)
- Test: inline `#[cfg(test)]`

**Port from:** `SpliceUtility.ts` (32 lines)

Small utility for intersection point munging — adjusts intersection points to stay within object boundaries.

- [ ] **Step 1: Write tests**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 5.2b: Add TollFreeVisibilityEdge Variant

**Files:**
- Expand: `src/visibility/edge.rs` (currently 26 lines → ~50 lines)
- Test: inline `#[cfg(test)]`

**Port from:** `TollFreeVisibilityEdge` class in TS (extends VisibilityEdge)

TransientGraphUtility creates "toll-free" edges — temporary edges that should not incur path cost. These mark transient connections added during port splicing. After routing, they are removed.

- [ ] **Step 1: Write test for toll-free edge**

```rust
#[test]
fn toll_free_edge_has_zero_weight() {
    let edge = VisEdge::toll_free(VertexId(1));
    assert!(edge.is_toll_free);
    assert_eq!(edge.weight, 0.0);
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Add `is_toll_free: bool` field to `VisEdge` and `VisEdge::toll_free()` constructor**
- [ ] **Step 4: Run — pass, plus all existing tests**
- [ ] **Step 5: Commit**

---

### Task 5.3: Create TransientGraphUtility

**Files:**
- Create: `src/routing/transient_graph_utility.rs` (~400 lines)
- Test: `tests/routing/transient_graph_tests.rs`

**Port from:** `TransientGraphUtility.ts` (742 lines)

This is the core graph manipulation layer for port splicing. Key operations:
- `add_vertex(location)` — create temporary vertex
- `find_or_add_edge(source, target, weight)` — find existing edge or create new one with bracket detection
- `split_edge(edge, split_vertex)` — split an edge at an intermediate point
- `extend_edge_chain(vertex, direction, target_vertex)` — extend edges to visibility limits
- `remove_from_graph()` — cleanup all transient additions

**CRITICAL: Bracket detection.** When adding an edge from A to B, if there's already an edge from A to C where B is between A and C, we must split A→C into A→B + B→C instead of creating a duplicate.

- [ ] **Step 1: Write tests for bracket detection and edge splitting**

```rust
#[test]
fn find_or_add_edge_detects_bracket() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, c, 1.0);

    let mut tgu = TransientGraphUtility::new(&mut graph);
    let b = tgu.find_or_add_vertex(Point::new(50.0, 0.0));
    tgu.find_or_add_edge(a, b, 1.0);

    // Edge A→C should have been split into A→B + B→C
    assert!(graph.has_edge(a, b));
    assert!(graph.has_edge(b, c));
    assert!(!graph.has_edge(a, c)); // original edge removed
}

#[test]
fn transient_cleanup_removes_all_additions() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, b, 1.0);
    let initial_vertex_count = graph.vertex_count();

    let mut tgu = TransientGraphUtility::new(&mut graph);
    tgu.find_or_add_vertex(Point::new(50.0, 50.0));
    assert!(graph.vertex_count() > initial_vertex_count);

    tgu.remove_from_graph();
    // After cleanup, graph should be back to original state
    assert_eq!(graph.vertex_count(), initial_vertex_count);
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement TransientGraphUtility**

```rust
pub struct TransientGraphUtility {
    added_vertices: Vec<VertexId>,
    added_edges: Vec<(VertexId, VertexId)>,
    removed_edges: Vec<(VertexId, VertexId, f64)>, // for restore
}
```

- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 5.4: Expand Port Types — ObstaclePort, FreePoint, ObstaclePortEntrance

**Files:**
- Expand: `src/routing/port.rs` (currently 20 lines → ~250 lines)
- Test: `tests/routing/port_tests.rs`

**Port from:** `ObstaclePort.ts` (75 lines), `FreePoint.ts` (147 lines), `ObstaclePortEntrance.ts` (200 lines)

Current port.rs has only `FloatingPort`. We need:
- `ObstaclePort` — port attached to an obstacle with entrance points
- `FreePoint` — free-standing waypoint with visibility caching
- `ObstaclePortEntrance` — entry point on obstacle boundary with max visibility segment

- [ ] **Step 1: Write tests**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement port types**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 5.5: Rewrite PortManager — Full Port Splicing

**Files:**
- Rewrite: `src/routing/port_manager.rs` (currently 98 lines → ~400 lines)
- Test: `tests/routing/port_manager_tests.rs` (existing — expand)

**Port from:** `PortManager.ts` (1,052 lines — but we defer groups, so ~650 lines relevant)

Current port_manager.rs finds nearest axis-aligned vertex in 4 directions. The TS version:
- Creates ObstaclePort and FreePoint objects
- Computes ObstaclePortEntrance positions on obstacle boundaries
- Uses TransientGraphUtility for edge splitting and bracket detection
- Handles out-of-bounds ports at graph edges
- Routes-to-center-of-obstacle mode
- Full cleanup/restore cycle

- [ ] **Step 1: Write tests for port splicing with entrance points**

```rust
#[test]
fn port_splice_creates_entrance_on_obstacle_boundary() {
    let shapes = vec![Shape::new_rectangle(0.0, 0.0, 100.0, 50.0)];
    let graph = generate_visibility_graph(&shapes, 4.0);
    let mut pm = PortManager::new(&mut graph, &shapes, 4.0);
    let port = FloatingPort::new(0, Point::new(50.0, 25.0)); // center of obstacle

    pm.add_obstacle_port(port);
    pm.create_obstacle_port_entrances();

    // Port should have entrance vertices on the padded boundary
    let entrances = pm.get_entrances(0);
    assert!(!entrances.is_empty());
    // Each entrance should be on the padded boundary (within epsilon)
    for e in entrances {
        let p = graph.point(e);
        // Should be on padded boundary: x in {-4, 104} or y in {-4, 54}
        let on_boundary = (p.x + 4.0).abs() < 1e-6
            || (p.x - 104.0).abs() < 1e-6
            || (p.y + 4.0).abs() < 1e-6
            || (p.y - 54.0).abs() < 1e-6;
        assert!(on_boundary, "entrance at {:?} not on padded boundary", p);
    }
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement full PortManager**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Run full `cargo test` and smoke tests**
- [ ] **Step 6: Commit**

---

### Task 5.6: Expand RectilinearEdgeRouter — Full Orchestration

**Files:**
- Expand: `src/routing/rectilinear_edge_router.rs` (currently 331 lines → ~450 lines)
- Test: existing integration tests — expand

**Port from:** `RectilinearEdgeRouter.ts` (659 lines)

Wire the new components: VG generator, PortManager with TransientGraphUtility, SsstRectilinearPath, MsmtRectilinearPath, nudging pipeline.

- [ ] **Step 1: Write integration test**

```rust
#[test]
fn full_pipeline_routes_and_nudges_three_edges() {
    let shapes = vec![
        Shape::new_rectangle(0.0, 0.0, 80.0, 40.0),
        Shape::new_rectangle(200.0, 0.0, 80.0, 40.0),
        Shape::new_rectangle(100.0, 100.0, 80.0, 40.0),
    ];
    let mut router = RectilinearEdgeRouter::new(&shapes);
    // Route edges between all pairs
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(40.0, 20.0)),
        FloatingPort::new(1, Point::new(240.0, 20.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(40.0, 20.0)),
        FloatingPort::new(2, Point::new(140.0, 120.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, Point::new(240.0, 20.0)),
        FloatingPort::new(2, Point::new(140.0, 120.0)),
    ));
    let result = router.padding(4.0).edge_separation(8.0).run();
    assert_eq!(result.edges.len(), 3);
    for edge in &result.edges {
        assert!(edge.points.len() >= 2);
        // Verify rectilinear
        for w in edge.points.windows(2) {
            assert!((w[0].x - w[1].x).abs() < 1e-6 || (w[0].y - w[1].y).abs() < 1e-6);
        }
    }
}
```

- [ ] **Step 2: Run — likely fails until all Layer 5 components are wired**
- [ ] **Step 3: Wire new components into router pipeline**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

## Layer 6: Nudging Expansions

### Task 6.1: Create LinkedPoint

**Files:**
- Create: `src/routing/nudging/linked_point.rs` (~50 lines)
- Test: inline `#[cfg(test)]`

**Port from:** `LinkedPoint.ts` (47 lines)

Linked-list node for path points. In Rust, use `Vec<Point>` with a separate `next: Vec<Option<usize>>` index array, or a simple `Vec<LinkedPointNode>` where each node has `point: Point, next: Option<usize>`.

- [ ] **Step 1: Write tests**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 6.2: Create LinkedPointSplitter

**Files:**
- Create: `src/routing/nudging/linked_point_splitter.rs` (~150 lines)
- Test: `tests/routing/nudging/linked_point_splitter_tests.rs`

**Port from:** `LinkedPointSplitter.ts` (152 lines)

Event-driven plane sweep to find all H/V crossings between path segments. Uses a BTreeMap as the active vertical segment tree.

- [ ] **Step 1: Write tests for crossing detection**

```rust
#[test]
fn splitter_finds_hv_crossing() {
    // Horizontal segment from (0,50) to (100,50)
    // Vertical segment from (50,0) to (50,100)
    // Should find crossing at (50,50)
    let h_points = vec![Point::new(0.0, 50.0), Point::new(100.0, 50.0)];
    let v_points = vec![Point::new(50.0, 0.0), Point::new(50.0, 100.0)];
    let result = LinkedPointSplitter::split(&h_points, &v_points);
    // Both segments should now have the crossing point inserted
    assert!(result.h_points.contains(&Point::new(50.0, 50.0)));
    assert!(result.v_points.contains(&Point::new(50.0, 50.0)));
}
```

- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement LinkedPointSplitter**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 6.3: Create PathMerger

**Files:**
- Create: `src/routing/nudging/path_merger.rs` (~160 lines)
- Test: `tests/routing/nudging/path_merger_tests.rs`

**Port from:** `PathMerger.ts` (160 lines)

Detects and removes self-loops and multi-crossing patterns between paths. Maintains per-vertex tracking of which paths pass through; when a path "departs" and "returns" to a vertex, collapses the loop.

- [ ] **Step 1: Write tests**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement PathMerger**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 6.4: Expand FreeSpaceFinder — Full Sweep-Line

**Files:**
- Rewrite: `src/routing/nudging/free_space_finder.rs` (currently 180 lines → ~350 lines)
- Test: `tests/routing/nudging/free_space_finder_tests.rs`

**Port from:** `FreeSpaceFinder.ts` (499 lines)

Current free_space_finder.rs does direct bounding box computation. The TS version uses a sweep-line with:
- AxisEdgesContainer in a BTreeMap for active edge tracking
- AxisEdgeLowPointEvent / AxisEdgeHighPointEvent for sweep events
- Neighbor relationship tracking between axis edges
- Obstacle boundary constraint computation during sweep

- [ ] **Step 1: Write tests for sweep-line bounds computation**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement full sweep-line FreeSpaceFinder**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 6.5: Expand PathRefiner — LinkedPointSplitter Integration

**Files:**
- Expand: `src/routing/nudging/path_refiner.rs` (currently 223 lines → ~250 lines)

Wire LinkedPointSplitter into PathRefiner's `cross_vertical_and_horizontal_segs()` method.

- [ ] **Step 1: Write integration test**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Wire LinkedPointSplitter into PathRefiner**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Commit**

---

### Task 6.6: Expand Nudger — PathMerger Integration + Two-Pass

**Files:**
- Expand: `src/routing/nudging/nudger.rs` (currently 480 lines → ~500 lines)

Wire PathMerger into the nudging pipeline between PathRefiner and CombinatorialNudger. Ensure the two-pass orthogonal nudging (North then East) is correct per TS.

- [ ] **Step 1: Write integration test verifying PathMerger is called**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Wire PathMerger into nudger pipeline**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Run full `cargo test` — all tests including smoke tests**
- [ ] **Step 6: Run `cargo clippy -- -D warnings`**
- [ ] **Step 7: Commit**

---

## Layer 7: Solver Fixture Fixes

### Task 7.1: Fix 6 Failing Solver Fixtures

**Files:**
- Modify: `src/projection_solver/` (specific files depend on triage from Layer 0.5)
- Modify: `tests/projection_solver/fixture_tests.rs` (un-ignore tests)

**Approach:** Based on the triage document from Layer 0.5, fix each failure category:

1. **Cycle handling (4 fixtures):** Compare QPSC cycle detection logic against C# `Solver.cs` step-by-step. The C# solver handles cycles by detecting them during block merge and breaking the weakest constraint.

2. **Convergence (2 fixtures):** Compare convergence criteria and iteration limits against C#. May need to adjust tolerance or max iterations for extreme weight ratios.

- [ ] **Step 1: Un-ignore first cycle fixture, reproduce failure**
- [ ] **Step 2: Read C# solver cycle handling code**
- [ ] **Step 3: Fix Rust cycle handling to match C#**
- [ ] **Step 4: Un-ignore all 4 cycle fixtures, verify they pass**
- [ ] **Step 5: Un-ignore convergence fixtures, reproduce failure**
- [ ] **Step 6: Fix convergence issues**
- [ ] **Step 7: All 86/86 fixture tests pass**
- [ ] **Step 8: Commit**

---

## Layer 8: Port C# Routing Tests

### Task 8.1: Build Test Harness Infrastructure

**Files:**
- Create: `tests/test_harness/mod.rs`
- Create: `tests/test_harness/scenario_builder.rs` (~300 lines)
- Create: `tests/test_harness/verifier.rs` (~200 lines)

**Port from:** `RectilinearVerifier.cs` (760 lines), `RectilinearEdgeRouterWrapper.cs` (1,309 lines) — ported incrementally, not all 2,069 lines

The test harness provides:

**ScenarioBuilder** (port of RectilinearVerifier's shape creation helpers):
```rust
pub struct ScenarioBuilder {
    shapes: Vec<Shape>,
    edges: Vec<EdgeGeometry>,
    padding: f64,
    edge_separation: f64,
    bend_penalty: f64,
    want_nudger: bool,
}

impl ScenarioBuilder {
    pub fn new() -> Self;
    pub fn add_rectangle(&mut self, center: Point, width: f64, height: f64) -> usize;
    pub fn add_obstacle_port(&mut self, obstacle: usize, location: Point) -> FloatingPort;
    pub fn add_free_port(&mut self, location: Point) -> FloatingPort;
    pub fn route_between(&mut self, source: FloatingPort, target: FloatingPort);
    pub fn padding(mut self, p: f64) -> Self;
    pub fn edge_separation(mut self, s: f64) -> Self;
    pub fn run(&self) -> RoutingResult;
}
```

**Verifier** (port of wrapper's verification logic):
```rust
pub struct Verifier;

impl Verifier {
    /// Assert all paths are valid rectilinear routes.
    pub fn verify_all(result: &RoutingResult, shapes: &[Shape], padding: f64);
    /// No path segment crosses through an obstacle.
    pub fn assert_no_obstacle_crossings(result: &RoutingResult, shapes: &[Shape], padding: f64);
    /// All segments are axis-aligned.
    pub fn assert_rectilinear(result: &RoutingResult);
    /// Path starts within source obstacle and ends within target obstacle.
    pub fn assert_endpoints(result: &RoutingResult, edges: &[EdgeGeometry], shapes: &[Shape]);
}
```

- [ ] **Step 1: Write tests for ScenarioBuilder**
- [ ] **Step 2: Run — fail**
- [ ] **Step 3: Implement ScenarioBuilder**
- [ ] **Step 4: Run — pass**
- [ ] **Step 5: Write tests for Verifier**
- [ ] **Step 6: Implement Verifier**
- [ ] **Step 7: Commit**

---

### Task 8.2: Port Diamond Tests (First Batch — ~20 tests)

**Files:**
- Create: `tests/rectilinear/diamond_tests.rs`

**Port from:** `RectilinearTests.cs` — Diamond3, Diamond3_With_FreePorts, Diamond3_Square6_Overlap, etc.

```rust
#[test]
fn diamond3() {
    let mut builder = ScenarioBuilder::new();
    // Create 3 diamond-shaped obstacles (from C# Create_Diamond3())
    builder.add_rectangle(Point::new(200.0, 100.0), 100.0, 100.0);
    builder.add_rectangle(Point::new(100.0, 300.0), 100.0, 100.0);
    builder.add_rectangle(Point::new(300.0, 300.0), 100.0, 100.0);
    // Route between all pairs
    let p0 = builder.add_obstacle_port(0, Point::new(200.0, 100.0));
    let p1 = builder.add_obstacle_port(1, Point::new(100.0, 300.0));
    let p2 = builder.add_obstacle_port(2, Point::new(300.0, 300.0));
    builder.route_between(p0, p1);
    builder.route_between(p0, p2);
    builder.route_between(p1, p2);
    let result = builder.padding(1.0).edge_separation(1.0).run();
    Verifier::verify_all(&result, &builder.shapes(), 1.0);
}
```

- [ ] **Step 1: Port first 5 diamond tests**
- [ ] **Step 2: Run — verify pass**
- [ ] **Step 3: Port remaining 15 diamond/basic tests**
- [ ] **Step 4: Run — verify pass**
- [ ] **Step 5: Commit**

---

### Task 8.3: Port Free Port Tests (~40 tests)

**Files:**
- Create: `tests/rectilinear/free_port_tests.rs`

- [ ] **Step 1: Port first 10 free port tests**
- [ ] **Step 2: Run — verify pass**
- [ ] **Step 3: Port remaining free port tests in batches of 10**
- [ ] **Step 4: Commit after each batch**

---

### Task 8.4: Port Reflection Tests (~12 tests)

**Files:**
- Create: `tests/rectilinear/reflection_tests.rs`

- [ ] **Step 1: Port all 12 reflection tests**
- [ ] **Step 2: Run — verify pass**
- [ ] **Step 3: Commit**

---

### Task 8.5: Port Overlap Tests (~30 tests)

**Files:**
- Create: `tests/rectilinear/overlap_tests.rs`

- [ ] **Step 1: Port in batches of 10**
- [ ] **Step 2: Run after each batch**
- [ ] **Step 3: Commit after each batch**

---

### Task 8.6: Port Remaining Tests (~140 tests)

**Files:**
- Create: `tests/rectilinear/collinear_tests.rs`
- Create: `tests/rectilinear/visibility_tests.rs`
- Create: `tests/rectilinear/update_tests.rs`
- Create: `tests/rectilinear/advanced_tests.rs`

- [ ] **Step 1: Port in batches of 20**
- [ ] **Step 2: Run after each batch**
- [ ] **Step 3: Commit after each batch**

---

### Task 8.7: Final Validation

- [ ] **Step 1: Run full test suite**

```bash
cargo test 2>&1 | tail -5
# Expected: test result: ok. ~500+ passed; 0 failed; 0 ignored
```

- [ ] **Step 2: Run clippy**

```bash
cargo clippy -- -D warnings
# Expected: no warnings
```

- [ ] **Step 3: Count lines**

```bash
find src -name '*.rs' | xargs wc -l | tail -1
# Expected: 12,000-14,000 lines
```

- [ ] **Step 4: Verify file mapping completeness**

Check every TS file in `routing/rectilinear/` has a Rust equivalent (except DEFERRED items).

- [ ] **Step 5: Final commit + tag**

```bash
git tag v0.2.0-faithful-port
```

---

## Summary

| Layer | Tasks | Est. Lines | Key Risk |
|---|---|---|---|
| 0 | 4 tasks | ~400 | Low — data structures only |
| 0.5 | 1 task | 0 (triage doc) | Low — investigation only |
| 1 | 4 tasks | ~600 | Medium — event ordering correctness |
| 2 | 4 tasks | ~1,800 | **HIGH** — sweep-line is the hardest algorithm |
| 2.5 | 1 task | ~100 | Low — smoke test |
| 3 | 2 tasks | ~200 | Low — data structure addition |
| 4 | 2 tasks | ~700 | Medium — A* correctness |
| 5 | 6 tasks | ~1,200 | **HIGH** — bracket detection, edge splitting |
| 6 | 6 tasks | ~800 | Medium — sweep-line in nudging |
| 7 | 1 task | ~100 | Medium — numerical debugging |
| 8 | 7 tasks | ~2,000 | Low — mechanical porting |
| **Total** | **38 tasks** | **~7,900** | |
