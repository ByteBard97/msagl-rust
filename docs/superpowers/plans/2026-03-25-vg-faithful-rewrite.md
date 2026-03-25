# Faithful Visibility Graph Rewrite — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the simplified visibility graph generation pipeline with a faithful port of MSAGL's VisibilityGraphGenerator, including full polyline traversal, reflection events, lookahead scanning, and sweep-line segment intersection.

**Architecture:** Abstract base class `VisibilityGraphGenerator` (TS 1,013 lines) + concrete `FullVisibilityGraphGenerator` (C# 320 lines). Two-pass sweep-line: each pass processes 6 event types (Open, Close, LowBend, HighBend, LowReflection, HighReflection), maintains an RBTree-based scanline of obstacle sides with slope tracking, stores lookahead reflection sites in a sorted tree, and creates weighted scan segments. After both passes, `SegmentIntersector` sweeps H/V segments using an RBTree to create the VisibilityGraph with proper vertex tracking per segment.

**Tech Stack:** Rust 2021, `ordered-float`, `slotmap` (existing polyline), `rstar` (existing obstacle tree), `std::collections::BTreeMap` (replaces TS RBTree).

**Scope:** These 4 files are being rewritten:
1. `src/routing/shape.rs` — accept any closed Polyline boundary
2. `src/routing/obstacle.rs` — polyline-based obstacles with IsRectangle detection
3. `src/routing/obstacle_side.rs` — full BasicObstacleSide with slope, polyline traversal
4. `src/routing/visibility_graph_generator.rs` — full sweep-line with all 6 event types
5. `src/routing/segment_intersector.rs` — RBTree-based sweep intersection

**CRITICAL PORTING RULE:** Every method in the TS/C# source has a corresponding Rust function. If the TS does something, the Rust does the same thing. The only deviations are Rust idiom adaptations from the approved table in `msagl-rust/CLAUDE.md`.

---

## Reference Sources (Read Before Each Task)

| Component | TS Source | C# Source |
|-----------|-----------|-----------|
| VG Generator (abstract) | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/VisibilityGraphGenerator.ts` (1,013 lines) | `MSAGL-Reference/.../Routing/Rectilinear/VisibilityGraphGenerator.cs` |
| VG Generator (concrete) | N/A (only SparseVGG in TS) | `MSAGL-Reference/.../Routing/Rectilinear/FullVisibilityGraphGenerator.cs` (320 lines) |
| Obstacle | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/obstacle.ts` (239 lines) | |
| ObstacleSide | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/BasicObstacleSide.ts` (51 lines) | |
| Shape | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/shape.ts` (122 lines) | |
| SegmentIntersector | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/SegmentIntersector.ts` (285 lines) | |
| ScanSegment | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/ScanSegment.ts` | |
| Events | `BasicVertexEvent.ts`, `OpenVertexEvent.ts`, `MiscVertexEvents.ts`, `basicReflectionEvent.ts`, `HighReflectionEvent.ts`, `LowReflectionEvent.ts` | |
| LookaheadScan | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/LookaheadScan.ts` (119 lines) | |
| NeighborSides | `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/NeighborSides.ts` (70 lines) | |

---

## Gap Analysis: Current Rust vs. TS/C# Source

This section documents every behavioral difference between the current Rust code and the TS/C# reference, so the implementer knows exactly what's wrong and what "faithful" means.

### shape.rs (32 lines → ~60 lines)

| TS Feature | Current Rust | Gap |
|------------|--------------|-----|
| `BoundaryCurve: ICurve` — any closed curve | Only `Rectangle` stored as `bounding_box` | **MISSING**: no curve/polyline boundary support |
| `Ports: Set<Port>` | Not present | DEFERRED (port handling is separate PRD item) |
| `IsGroup` / parent-child hierarchy | Not present | DEFERRED (groups are excluded per PRD) |

**What to implement:** Store an `Option<Polyline>` as the boundary curve. `rectangle()` constructors create the polyline from 4 corners. Add `from_polyline(poly: Polyline)` constructor. Keep `bounding_box()` computed from polyline.

### obstacle.rs (167 lines → ~250 lines)

| TS Feature | Current Rust | Gap |
|------------|--------------|-----|
| `InteractiveObstacleCalculator.PaddedPolylineBoundaryOfNode()` padding | Hardcoded 4-corner rectangle from bbox expansion | **WRONG**: TS pads the actual boundary curve, not just the bbox |
| `RoundVerticesAndSimplify()` — rounds vertices, removes close/collinear | Not present | **MISSING** |
| `IsRectangle` — checks if 4-point polyline with all pure compass directions | Not present — always treated as rectangle | **MISSING** |
| `IsPolylineRectangle()` verification | Not present | **MISSING** |
| `VisibilityPolyline` — returns ConvexHull polyline if overlapped | Always returns padded_polyline | DEFERRED (overlap/convex hull) |
| `looseVisibilityPolyline` — slightly expanded polyline for lookahead | Not present | **MISSING** (needed for reflection lookahead) |
| `CreateInitialSides(startPoint, scanDir)` — takes PolylinePoint, creates Low/High sides from polyline traversal | Takes `is_horizontal_scan: bool`, creates sides from bbox corners | **WRONG**: must traverse polyline from startPoint |
| `GetOpenVertex()` finding lowest polyline point | Not present — uses bbox left_bottom | **WRONG**: must walk polyline to find lowest vertex |
| Flat bottom side detection and skip in `CreateInitialSides` | Not present | **MISSING** |
| `Close()` clearing active sides | Not present | **MISSING** |
| Clump/overlap tracking | Not present | DEFERRED |

### obstacle_side.rs (92 lines → ~100 lines)

| TS Feature | Current Rust | Gap |
|------------|--------------|-----|
| `obstacle: Obstacle` back-reference | `obstacle_ordinal: usize` only | **PARTIAL**: ordinal is sufficient for some uses but not for getting obstacle properties |
| `endVertex: PolylinePoint` — key into polyline | `end: Point` only | **WRONG**: must store polyline point key for traversal to next vertex |
| `Slope` / `SlopeInverse` computed from scan direction | Computed from raw dx/dy | **WRONG**: TS computes slope relative to scan direction via `StaticGraphUtility.Slope()` |
| `LowObstacleSide` traverses clockwise for H-scan, counter-clockwise for V-scan | Fixed start/end from bbox corners | **WRONG**: must traverse polyline |
| `HighObstacleSide` traverses counter-clockwise for H-scan, clockwise for V-scan | Fixed start/end from bbox corners | **WRONG**: must traverse polyline |
| Direction property for ScanLineIntersectSide | Not present | **MISSING** |

### visibility_graph_generator.rs (359 lines → ~700 lines)

| TS Feature | Current Rust | Gap |
|------------|--------------|-----|
| 6 event types processed: Open, Close, LowBend, HighBend, LowReflection, HighReflection | Only Open and Close; bend/reflection silently ignored (`_ => {}`) | **WRONG** |
| `ProcessEventO` — adds sides to scanline via `AddSideToScanLine` which loads reflection events, then calls `FindNeighborsAndProcessVertexEvent`, then checks for reflection loading on both neighbor sides, handles flat bottom absorption, enqueues LowBend and HighBend events | Adds sides, finds gaps between consecutive sides, enqueues CloseVertex | **WRONG**: completely different algorithm |
| `ProcessEventLB` — updates ActiveLowSide in scanline if ascending | Not implemented | **MISSING** |
| `ProcessEventHB` — updates ActiveHighSide, checks for extreme high-side lookahead | Not implemented | **MISSING** |
| `ProcessEventCV` — finds neighbors, creates segments, loads reflection events for both neighbor sides, calls `obstacle.Close()` | Removes sides, creates gap segments | **WRONG**: missing neighbor finding, reflection loading |
| `ProcessEventLR/HR` — reflection chain processing with perpendicular/parallel segment insertion | Not implemented | **MISSING** |
| `StoreLookaheadSite` — stores reflection sites in LookaheadScan tree | Not implemented | **MISSING** |
| `LoadReflectionEvents` — loads pending reflections when a new side is added | Not implemented | **MISSING** |
| `AddPerpendicularReflectionSegment` — validates and creates perpendicular reflection segments | Not implemented | **MISSING** |
| `FindNeighborsBRR` + `SkipToNeighbor` — proper neighbor finding with overlap/group awareness | Gap enumeration via `all_sides_ordered().windows(2)` | **WRONG**: completely different approach |
| `ScanLineIntersectSidePBS` — slope-based intersection for non-perpendicular sides | Not present (sides assumed perpendicular) | **MISSING** |
| `GetOpenVertex` — polyline traversal to find lowest vertex | Uses `bb.left_bottom()` | **WRONG** |
| `EnqueueBottomVertexEvents` using `GetAllPrimaryObstacles` and polyline traversal | Uses bbox left_bottom for all obstacles | **WRONG** |
| `TraversePolylineForEvents` — clockwise for H-scan, counter-clockwise for V-scan | Not present | **MISSING** |
| `CreateScanSegments` (from FullVisibilityGraphGenerator.cs) — handles overlap intervals with up to 3 sub-segments | Single gap detection between consecutive sides | **WRONG** |
| Segment subsumption via `ScanSegment.Subsume` + hintScanSegment optimization | `insert_unique` deduplication only | **PARTIAL** |
| `wantReflections` flag (true for Full, false for Sparse) | Not present | **MISSING** |
| `NeighborSides` tracking low/high neighbors with overlap endpoints | Not present | **MISSING** (already has `neighbor_sides.rs` in mod.rs) |

### segment_intersector.rs (186 lines → ~250 lines)

| TS Feature | Current Rust | Gap |
|------------|--------------|-----|
| Event-based sweep: VOpen, VClose, HOpen events sorted by Y→type→X | Nested O(n*m) loop over all H/V segment pairs | **WRONG**: O(n^2) instead of O(n log n + k) |
| RBTree-based scanline of active vertical segments | No scanline — brute force | **WRONG** |
| `ScanIntersect` — uses `findFirst` predicate + `next()` iteration for range query | Nested loop with epsilon comparison | **WRONG** |
| `OnSegmentOpen/Close` — calls `seg.OnSegmentIntersectorBegin/End` for vertex tracking | Not present — vertices tracked externally | **WRONG** |
| `ScanSegment.AppendVisibilityVertex` — maintains Lowest/HighestVisibilityVertex chain | Not present — edges added directly | **WRONG** |
| `RemoveSegmentsWithNoVisibility` — removes segments that received no crossings | Not present | **MISSING** |
| Event ordering: VOpen < HOpen < VClose at same Y | No ordering — all pairs checked | **WRONG** |
| Segment comparison in scanline by X then Y | No scanline | **WRONG** |
| `VisibilityGraphGenerator.NewVisibilityGraph()` creates graph with `VisibilityVertexRectilinear` factory | Graph uses plain VertexData | DEFERRED (VertexEntry[4] is separate PRD item #3) |

---

## Dependency Order

```
Task 1: shape.rs          (no dependencies — leaf node)
Task 2: obstacle_side.rs  (depends on shape.rs polyline, scan_direction.rs)
Task 3: obstacle.rs       (depends on obstacle_side.rs)
Task 4: event_queue.rs    (minor updates to support PolylinePointKey)
Task 5: scan_line.rs      (needs obstacle_side.rs with obstacle index, not just ordinal)
Task 6: scan_segment.rs   (add OnSegmentIntersectorBegin/End, AppendVisibilityVertex)
Task 7: segment_intersector.rs  (depends on scan_segment.rs updates)
Task 8: visibility_graph_generator.rs  (depends on everything above)
```

Tasks 1-3 are the foundation. Task 7 is independent of Task 8 (they both depend on 1-6).

---

## Task 1: Expand shape.rs to Accept Polyline Boundaries

**Files:**
- Modify: `msagl-rust/src/routing/shape.rs`
- Test: `msagl-rust/tests/visibility.rs` (add tests)

**TS Reference:** `SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/shape.ts`

The TS `Shape` stores a `BoundaryCurve: ICurve` which can be any closed curve. For our scope (per PRD), we support `Polyline` boundaries. Rectangles are just 4-point closed polylines.

- [ ] **Step 1: Write failing test for polyline-based shape**

In `msagl-rust/tests/visibility.rs`, add:

```rust
#[test]
fn shape_from_polyline_has_correct_bounding_box() {
    use msagl_rust::geometry::polyline::Polyline;
    use msagl_rust::geometry::point::Point;
    use msagl_rust::routing::shape::Shape;

    // L-shaped polyline (6 vertices)
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(100.0, 0.0));
    poly.add_point(Point::new(100.0, 50.0));
    poly.add_point(Point::new(50.0, 50.0));
    poly.add_point(Point::new(50.0, 100.0));
    poly.add_point(Point::new(0.0, 100.0));
    poly.set_closed(true);

    let shape = Shape::from_polyline(poly);
    let bb = shape.bounding_box();
    assert!((bb.left() - 0.0).abs() < 1e-10);
    assert!((bb.bottom() - 0.0).abs() < 1e-10);
    assert!((bb.right() - 100.0).abs() < 1e-10);
    assert!((bb.top() - 100.0).abs() < 1e-10);
}

#[test]
fn shape_rectangle_creates_4_point_polyline() {
    use msagl_rust::routing::shape::Shape;

    let shape = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let poly = shape.boundary_polyline();
    assert_eq!(poly.count(), 4);
    assert!(poly.is_closed());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test shape_from_polyline shape_rectangle_creates -- --nocapture`
Expected: compilation error — `Shape::from_polyline` and `boundary_polyline()` don't exist

- [ ] **Step 3: Implement Shape with Polyline boundary**

Replace `msagl-rust/src/routing/shape.rs` with:

```rust
use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;

/// User-facing shape input for the router.
///
/// Wraps a closed Polyline boundary. Rectangles are the common case
/// but any closed polyline is accepted.
///
/// Ported from: routing/shape.ts — `class Shape`
/// The TS Shape stores `BoundaryCurve: ICurve`. We store `Polyline` directly
/// since we only support polyline boundaries (not arcs/curves).
#[derive(Clone, Debug)]
pub struct Shape {
    boundary: Polyline,
    bounding_box: Rectangle,
}

impl Shape {
    /// Create a shape from a closed polyline boundary.
    ///
    /// Matches TS: `new Shape(boundaryCurve)`
    pub fn from_polyline(polyline: Polyline) -> Self {
        debug_assert!(polyline.is_closed(), "Shape boundary must be a closed polyline");
        let bounding_box = polyline.bounding_box();
        Self {
            boundary: polyline,
            bounding_box,
        }
    }

    /// Create a rectangular shape at (x, y) with given width and height.
    /// (x, y) is the left-bottom corner.
    pub fn rectangle(x: f64, y: f64, width: f64, height: f64) -> Self {
        let mut poly = Polyline::new();
        // Clockwise from bottom-left (matching TS convention)
        poly.add_point(Point::new(x, y));
        poly.add_point(Point::new(x, y + height));
        poly.add_point(Point::new(x + width, y + height));
        poly.add_point(Point::new(x + width, y));
        poly.set_closed(true);
        Self::from_polyline(poly)
    }

    /// Create from center point and dimensions.
    pub fn rectangle_centered(cx: f64, cy: f64, width: f64, height: f64) -> Self {
        let hw = width / 2.0;
        let hh = height / 2.0;
        Self::rectangle(cx - hw, cy - hh, width, height)
    }

    /// The boundary polyline.
    ///
    /// Matches TS: `Shape.BoundaryCurve`
    pub fn boundary_polyline(&self) -> &Polyline {
        &self.boundary
    }

    pub fn bounding_box(&self) -> &Rectangle {
        &self.bounding_box
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test shape_from_polyline shape_rectangle_creates -- --nocapture`
Expected: PASS

- [ ] **Step 5: Run full test suite to check for regressions**

Run: `cargo test 2>&1 | grep "test result"`
Expected: All existing tests still pass (shape.rs API is backward-compatible since `rectangle()` and `bounding_box()` signatures are unchanged)

- [ ] **Step 6: Commit**

```bash
git add src/routing/shape.rs tests/visibility.rs
git commit -m "feat(shape): accept polyline boundaries, not just rectangles

Shape now stores a closed Polyline boundary instead of just a Rectangle.
Rectangle constructors create 4-point closed polylines internally.
This is required for faithful port of MSAGL obstacle handling.

Matches TS: routing/shape.ts Shape.BoundaryCurve"
```

---

## Task 2: Rewrite obstacle_side.rs with Polyline Traversal and Slope Tracking

**Files:**
- Modify: `msagl-rust/src/routing/obstacle_side.rs`
- Test: `msagl-rust/tests/visibility.rs` (add tests)

**TS Reference:** `BasicObstacleSide.ts` (51 lines)

The TS `BasicObstacleSide` stores a reference to the obstacle, a `startVertex: PolylinePoint`, and traverses to `endVertex` either clockwise (LowObstacleSide for H-scan) or counter-clockwise (HighObstacleSide for H-scan). It computes `Slope` and `SlopeInverse` relative to the scan direction using `StaticGraphUtility.Slope()`.

The Rust adaptation: instead of object references, store `obstacle_index: usize` and `start_key/end_key: PolylinePointKey`. Instead of class hierarchy, use `SideType` enum.

- [ ] **Step 1: Read the TS source**

Read `BasicObstacleSide.ts` lines 1-51. Confirm:
- Constructor takes `(obstacle, startVertex, scanDir, traverseClockwise)`
- `endVertex = traverseClockwise ? startVertex.nextOnPolyline : startVertex.prevOnPolyline`
- Slope computed as `StaticGraphUtility.Slope(start, end, scanDir)` unless perpendicular
- `LowObstacleSide` passes `traverseClockwise = scanDir.IsHorizontal`
- `HighObstacleSide` passes `traverseClockwise = scanDir.IsVertical`

- [ ] **Step 2: Write failing tests**

In `msagl-rust/tests/visibility.rs`:

```rust
#[test]
fn obstacle_side_low_traverses_clockwise_for_hscan() {
    use msagl_rust::geometry::point::Point;
    use msagl_rust::geometry::polyline::Polyline;
    use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
    use msagl_rust::routing::scan_direction::ScanDirection;

    // Square: BL(0,0) -> TL(0,100) -> TR(100,100) -> BR(100,0), closed clockwise
    let mut poly = Polyline::new();
    let bl = poly.add_point(Point::new(0.0, 0.0));
    let tl = poly.add_point(Point::new(0.0, 100.0));
    let _tr = poly.add_point(Point::new(100.0, 100.0));
    let _br = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    // For H-scan, LowObstacleSide traverses clockwise from startVertex
    // Starting at BL(0,0), next clockwise is TL(0,100)
    let side = ObstacleSide::from_polyline_point(
        SideType::Low,
        0, // obstacle_index
        bl,
        &poly,
        ScanDirection::horizontal(),
    );
    assert_eq!(side.start(), Point::new(0.0, 0.0));
    assert_eq!(side.end(), Point::new(0.0, 100.0));
    assert_eq!(side.end_vertex_key(), tl);
}

#[test]
fn obstacle_side_high_traverses_counterclockwise_for_hscan() {
    use msagl_rust::geometry::point::Point;
    use msagl_rust::geometry::polyline::Polyline;
    use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
    use msagl_rust::routing::scan_direction::ScanDirection;

    let mut poly = Polyline::new();
    let bl = poly.add_point(Point::new(0.0, 0.0));
    let _tl = poly.add_point(Point::new(0.0, 100.0));
    let _tr = poly.add_point(Point::new(100.0, 100.0));
    let br = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    // For H-scan, HighObstacleSide traverses counter-clockwise from startVertex
    // Starting at BL(0,0), prev (counter-clockwise) is BR(100,0)
    let side = ObstacleSide::from_polyline_point(
        SideType::High,
        0,
        bl,
        &poly,
        ScanDirection::horizontal(),
    );
    assert_eq!(side.start(), Point::new(0.0, 0.0));
    assert_eq!(side.end(), Point::new(100.0, 0.0));
    assert_eq!(side.end_vertex_key(), br);
}

#[test]
fn obstacle_side_slope_for_angled_side() {
    use msagl_rust::geometry::point::Point;
    use msagl_rust::geometry::polyline::Polyline;
    use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
    use msagl_rust::routing::scan_direction::ScanDirection;

    // Non-rectangular: side from (0,0) to (10,100) — not perpendicular
    let mut poly = Polyline::new();
    let p0 = poly.add_point(Point::new(0.0, 0.0));
    let _p1 = poly.add_point(Point::new(10.0, 100.0));
    let _p2 = poly.add_point(Point::new(110.0, 100.0));
    let _p3 = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low,
        0,
        p0,
        &poly,
        ScanDirection::horizontal(),
    );

    // Slope = (change in scan-parallel coord) / (change in perp coord)
    // H-scan: scan-parallel = X, perp = Y
    // From (0,0) to (10,100): slope = (10-0)/(100-0) = 0.1
    assert!((side.slope() - 0.1).abs() < 1e-10);
    assert!((side.slope_inverse() - 10.0).abs() < 1e-10);
}

#[test]
fn obstacle_side_perpendicular_has_zero_slope() {
    use msagl_rust::geometry::point::Point;
    use msagl_rust::geometry::polyline::Polyline;
    use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
    use msagl_rust::routing::scan_direction::ScanDirection;

    let mut poly = Polyline::new();
    let p0 = poly.add_point(Point::new(0.0, 0.0));
    let _p1 = poly.add_point(Point::new(0.0, 100.0));
    let _p2 = poly.add_point(Point::new(100.0, 100.0));
    let _p3 = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low,
        0,
        p0,
        &poly,
        ScanDirection::horizontal(),
    );

    // Perpendicular side: X doesn't change (0,0) to (0,100)
    // slope = 0, slope_inverse = 0 (matching TS behavior for perpendicular)
    assert!((side.slope()).abs() < 1e-10);
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cargo test obstacle_side_low_traverses obstacle_side_high_traverses obstacle_side_slope obstacle_side_perpendicular -- --nocapture`
Expected: compilation error — `ObstacleSide::from_polyline_point` doesn't exist

- [ ] **Step 4: Implement ObstacleSide with polyline traversal**

Replace `msagl-rust/src/routing/obstacle_side.rs`:

```rust
use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;
use super::scan_direction::ScanDirection;

/// Whether this is a low (bottom/left) or high (top/right) side of an obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SideType {
    Low,
    High,
}

/// A side of an obstacle boundary, with slope tracking for scanline intersection.
///
/// Faithful port of BasicObstacleSide.ts + LowObstacleSide/HighObstacleSide.
///
/// In the TS:
/// - LowObstacleSide traverses clockwise for H-scan (counter-clockwise for V-scan)
/// - HighObstacleSide traverses counter-clockwise for H-scan (clockwise for V-scan)
///
/// Slope is computed relative to the scan direction:
///   slope = (change in scan-parallel coord) / (change in perp coord)
///   slope_inverse = 1 / slope
/// Both are 0 when the side is perpendicular to the scan.
#[derive(Debug, Clone)]
pub struct ObstacleSide {
    side_type: SideType,
    obstacle_index: usize,
    start: Point,
    end: Point,
    start_key: PolylinePointKey,
    end_key: PolylinePointKey,
    slope: f64,
    slope_inverse: f64,
}

impl ObstacleSide {
    /// Create from a polyline point, traversing in the correct direction.
    ///
    /// Matches TS: `new LowObstacleSide(obstacle, startVertex, scanDir)` and
    /// `new HighObstacleSide(obstacle, startVertex, scanDir)`.
    ///
    /// Traversal direction:
    /// - Low + H-scan: clockwise (next)
    /// - Low + V-scan: counter-clockwise (prev)
    /// - High + H-scan: counter-clockwise (prev)
    /// - High + V-scan: clockwise (next)
    pub fn from_polyline_point(
        side_type: SideType,
        obstacle_index: usize,
        start_key: PolylinePointKey,
        polyline: &Polyline,
        scan_direction: ScanDirection,
    ) -> Self {
        let traverse_clockwise = match side_type {
            SideType::Low => scan_direction.is_horizontal(),
            SideType::High => scan_direction.is_vertical(),
        };

        let end_key = if traverse_clockwise {
            polyline.next_key(start_key)
                .expect("polyline must be closed or have a next point")
        } else {
            polyline.prev_key(start_key)
                .expect("polyline must be closed or have a prev point")
        };

        let start = polyline.point_at(start_key);
        let end = polyline.point_at(end_key);

        // Compute slope relative to scan direction.
        // TS: StaticGraphUtility.Slope(start, end, scanDir)
        //   = (scanDir.Coord(end) - scanDir.Coord(start)) / (scanDir.PerpCoord(end) - scanDir.PerpCoord(start))
        let d_scan = scan_direction.coord(end) - scan_direction.coord(start);
        let d_perp = scan_direction.perp_coord(end) - scan_direction.perp_coord(start);

        let (slope, slope_inverse) = if scan_direction.is_perpendicular(start, end) {
            // Perpendicular side — slope is 0 (TS sets both to 0)
            (0.0, 0.0)
        } else {
            let s = d_scan / d_perp;
            (s, 1.0 / s)
        };

        Self {
            side_type,
            obstacle_index,
            start,
            end,
            start_key,
            end_key,
            slope,
            slope_inverse,
        }
    }

    /// Create a sentinel side (no polyline traversal needed).
    /// Used for boundary sentinels that are simple line segments.
    pub fn sentinel(
        side_type: SideType,
        start: Point,
        end: Point,
        obstacle_ordinal: usize,
    ) -> Self {
        Self {
            side_type,
            obstacle_index: obstacle_ordinal,
            start,
            end,
            start_key: PolylinePointKey::default(),
            end_key: PolylinePointKey::default(),
            slope: 0.0,
            slope_inverse: 0.0,
        }
    }

    pub fn side_type(&self) -> SideType { self.side_type }
    pub fn start(&self) -> Point { self.start }
    pub fn end(&self) -> Point { self.end }
    pub fn obstacle_index(&self) -> usize { self.obstacle_index }
    pub fn start_vertex_key(&self) -> PolylinePointKey { self.start_key }
    pub fn end_vertex_key(&self) -> PolylinePointKey { self.end_key }
    pub fn slope(&self) -> f64 { self.slope }
    pub fn slope_inverse(&self) -> f64 { self.slope_inverse }

    /// The direction vector from start to end.
    /// Matches TS: `side.Direction` used in `ScanLineIntersectSidePBS`.
    pub fn direction(&self) -> Point {
        self.end - self.start
    }

    /// Compute the scanline intersection point at a given site.
    ///
    /// Faithful port of TS `VisibilityGraphGenerator.ScanLineIntersectSidePBS()`.
    /// Projects along the slope of the side to find where the scanline at `site`
    /// intersects this side.
    pub fn scanline_intersect(&self, site: Point, scan_direction: ScanDirection) -> Point {
        let dir = self.direction();
        let mut ix = self.start.x();
        let mut iy = self.start.y();

        if scan_direction.is_horizontal() {
            ix += (dir.x() / dir.y()) * (site.y() - self.start.y());
            // MungeIntersect: clamp ix to be within [start.x, end.x] range
            ix = munge_intersect(site.x(), ix, self.start.x(), self.end.x());
            iy = site.y();
        } else {
            ix = site.x();
            iy += (dir.y() / dir.x()) * (site.x() - self.start.x());
            iy = munge_intersect(site.y(), iy, self.start.y(), self.end.y());
        }

        Point::new(ix, iy)
    }

    // Keep backward compatibility alias
    pub fn obstacle_ordinal(&self) -> usize { self.obstacle_index }
}

/// Clamp the intersection coordinate to be within the side's range.
///
/// Faithful port of TS `SpliceUtility.MungeIntersect()`.
/// If `site_coord` is between `start` and `end`, prefer `intersect`;
/// otherwise clamp to the nearest endpoint.
fn munge_intersect(site_coord: f64, intersect: f64, start: f64, end: f64) -> f64 {
    let lo = start.min(end);
    let hi = start.max(end);
    if intersect < lo {
        lo
    } else if intersect > hi {
        hi
    } else {
        intersect
    }
}
```

- [ ] **Step 5: Check that PolylinePointKey has a Default impl**

Read `msagl-rust/src/arenas.rs` to check if `PolylinePointKey` derives `Default`. If not, add `#[derive(Default)]` or use `slotmap::KeyData::null()`.

- [ ] **Step 6: Fix any compilation issues in dependent code**

The old `ObstacleSide::new()` constructor is removed. Update all call sites:
- `src/routing/visibility_graph_generator.rs` — sentinel creation uses `ObstacleSide::sentinel()`
- `src/routing/obstacle.rs` — `create_initial_sides` uses `ObstacleSide::from_polyline_point()`
- `src/routing/scan_line.rs` — may need `obstacle_index()` instead of `obstacle_ordinal()`

- [ ] **Step 7: Run tests**

Run: `cargo test obstacle_side_ -- --nocapture`
Expected: All 4 new tests pass

- [ ] **Step 8: Run full test suite**

Run: `cargo test 2>&1 | grep "test result"`
Expected: All pass (some may need `obstacle_ordinal()` → `obstacle_index()` fixes first)

- [ ] **Step 9: Commit**

```bash
git add src/routing/obstacle_side.rs tests/visibility.rs
git commit -m "feat(obstacle_side): polyline traversal + scan-relative slope

ObstacleSide now traverses polyline points instead of using fixed bbox
corners. Low sides traverse clockwise for H-scan, High sides traverse
counter-clockwise, matching TS BasicObstacleSide.ts exactly.

Slope computed relative to scan direction via StaticGraphUtility.Slope().
Includes ScanLineIntersectSidePBS port and MungeIntersect clamping."
```

---

## Task 3: Rewrite obstacle.rs with Polyline Support and IsRectangle Detection

**Files:**
- Modify: `msagl-rust/src/routing/obstacle.rs`
- Test: `msagl-rust/tests/visibility.rs`

**TS Reference:** `obstacle.ts` (239 lines)

Key changes:
- `from_shape` pads the boundary polyline (not just the bbox)
- `RoundVerticesAndSimplify` rounds vertices and removes close/collinear points
- `IsRectangle` detection by checking 4 vertices with pure compass directions
- `CreateInitialSides` takes a `PolylinePointKey` start point and creates sides via polyline traversal
- `GetOpenVertex` walks the polyline to find the lowest vertex (not just bbox corner)

For padding: the TS uses `InteractiveObstacleCalculator.PaddedPolylineBoundaryOfNode()` which creates an offset polyline. For rectangles, this is equivalent to expanding the bbox. For non-rectangles, it's a Minkowski sum. We implement the rectangle case exactly, and for non-rectangles use a simple vertex offset (acceptable per PRD since non-rectangular obstacles are an EXPAND, not the current scope).

- [ ] **Step 1: Write failing tests**

```rust
#[test]
fn obstacle_is_rectangle_for_rect_shape() {
    use msagl_rust::routing::shape::Shape;
    use msagl_rust::routing::obstacle::Obstacle;

    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&shape, 5.0, 0);
    assert!(obs.is_rectangle());
    assert_eq!(obs.padded_polyline().count(), 4);
}

#[test]
fn obstacle_create_initial_sides_from_open_vertex() {
    use msagl_rust::routing::shape::Shape;
    use msagl_rust::routing::obstacle::Obstacle;
    use msagl_rust::routing::scan_direction::ScanDirection;
    use msagl_rust::geometry::point::Point;

    let shape = Shape::rectangle(10.0, 20.0, 80.0, 60.0);
    let mut obs = Obstacle::from_shape(&shape, 5.0, 0);

    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    obs.create_initial_sides(open_key, scan_dir);

    let low = obs.active_low_side().expect("should have low side");
    let high = obs.active_high_side().expect("should have high side");

    // Low side starts at the open vertex
    // High side also starts at the open vertex (traversing the other direction)
    assert_eq!(low.start(), obs.padded_polyline().point_at(open_key));
    assert_eq!(high.start(), obs.padded_polyline().point_at(open_key));
}

#[test]
fn obstacle_get_open_vertex_finds_lowest() {
    use msagl_rust::routing::shape::Shape;
    use msagl_rust::routing::obstacle::Obstacle;
    use msagl_rust::routing::scan_direction::ScanDirection;
    use msagl_rust::geometry::point::Point;

    let shape = Shape::rectangle(10.0, 20.0, 80.0, 60.0);
    let obs = Obstacle::from_shape(&shape, 5.0, 0);

    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    let open_point = obs.padded_polyline().point_at(open_key);

    // For H-scan, the open vertex should be the bottom-left corner (lowest Y, then lowest X)
    // Padded bbox: (5, 15) to (95, 85)
    assert!((open_point.y() - 15.0).abs() < 1e-10);
    assert!((open_point.x() - 5.0).abs() < 1e-10);
}

#[test]
fn obstacle_close_clears_active_sides() {
    use msagl_rust::routing::shape::Shape;
    use msagl_rust::routing::obstacle::Obstacle;
    use msagl_rust::routing::scan_direction::ScanDirection;

    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let mut obs = Obstacle::from_shape(&shape, 5.0, 0);
    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    obs.create_initial_sides(open_key, scan_dir);
    assert!(obs.active_low_side().is_some());

    obs.close();
    assert!(obs.active_low_side().is_none());
    assert!(obs.active_high_side().is_none());
}
```

- [ ] **Step 2: Run tests to verify they fail**

- [ ] **Step 3: Implement the rewritten obstacle.rs**

Key methods to implement faithfully from TS `obstacle.ts`:

```rust
// In obstacle.rs — pseudocode outline, implementer must write full code:

impl Obstacle {
    /// Faithful port of TS Obstacle constructor.
    /// Pads the shape's boundary polyline, rounds vertices, checks IsRectangle.
    pub fn from_shape(shape: &Shape, padding: f64, index: usize) -> Self {
        // 1. Pad the boundary polyline
        //    For rectangles: expand bbox by padding, create 4-corner polyline
        //    For non-rectangles: offset each vertex outward by padding (simplified Minkowski)
        // 2. Call round_vertices_and_simplify() on the padded polyline
        // 3. Check is_polyline_rectangle() on the result
        // 4. Store padded_polyline, is_rectangle, padded_bbox
    }

    /// Faithful port of TS Obstacle.RoundVerticesAndSimplify()
    fn round_vertices_and_simplify(polyline: &mut Polyline) {
        // 1. Round each vertex to Point::round()
        // 2. Remove close and collinear vertices
    }

    /// Faithful port of TS Obstacle.IsPolylineRectangle()
    fn is_polyline_rectangle(polyline: &Polyline) -> bool {
        // 1. Check count == 4
        // 2. Walk vertices checking that each consecutive pair has a pure compass direction
        // 3. Check that directions rotate consistently (all right turns)
    }

    /// Faithful port of TS VisibilityGraphGenerator.GetOpenVertex()
    /// Walks the polyline to find the vertex with the lowest perpendicular coordinate.
    pub fn get_open_vertex(&self, scan_direction: ScanDirection) -> PolylinePointKey {
        // Walk polyline, comparing by scan_direction.compare()
        // Return the last vertex that is <= the current lowest (for flat bottoms)
    }

    /// Faithful port of TS Obstacle.CreateInitialSides()
    pub fn create_initial_sides(&mut self, start_key: PolylinePointKey, scan_direction: ScanDirection) {
        // Create LowObstacleSide from start_key traversing clockwise (H-scan) or CCW (V-scan)
        // Create HighObstacleSide from start_key traversing CCW (H-scan) or clockwise (V-scan)
        // If high side is flat (TS: scanDir.IsFlatS(highSide)), advance to next vertex
    }

    /// Faithful port of TS Obstacle.Close()
    pub fn close(&mut self) {
        self.active_low_side = None;
        self.active_high_side = None;
    }
}
```

The implementer MUST read `obstacle.ts` lines 78-102 for `CreateInitialSides` — note the flat side detection and skip:
```typescript
if (scanDir.IsFlatS(this.ActiveHighSide)) {
    this.ActiveHighSide = new HighObstacleSide(this, this.ActiveHighSide.EndVertex, scanDir)
}
```

And `VisibilityGraphGenerator.ts` lines 233-252 for `GetOpenVertex` — note the `<=` comparison for flat bottom handling.

- [ ] **Step 4: Run tests**

- [ ] **Step 5: Run full test suite**

- [ ] **Step 6: Commit**

---

## Task 4: Update event_queue.rs to Store PolylinePointKey

**Files:**
- Modify: `msagl-rust/src/routing/event_queue.rs`

The current `SweepEvent` variants store `site: Point` and `obstacle_index: usize`. The TS events store a `PolylinePoint` (which gives both the point and the polyline traversal position). We need the `PolylinePointKey` so that `CreateInitialSides` and bend event processing can traverse the polyline from the event's vertex.

- [ ] **Step 1: Add `vertex_key: PolylinePointKey` to vertex event variants**

Update the `SweepEvent` enum:

```rust
pub enum SweepEvent {
    OpenVertex { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    CloseVertex { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    LowBend { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    HighBend { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    LowReflection { site: Point, initial_obstacle: usize, reflecting_obstacle: usize, prev_event_index: Option<usize> },
    HighReflection { site: Point, initial_obstacle: usize, reflecting_obstacle: usize, prev_event_index: Option<usize> },
}
```

- [ ] **Step 2: Update all match arms and constructors**

Update `site()`, `obstacle_index()`, and all code that constructs or matches `SweepEvent` variants.

- [ ] **Step 3: Run full test suite**

- [ ] **Step 4: Commit**

---

## Task 5: Update scan_line.rs for ObstacleSide with obstacle_index

**Files:**
- Modify: `msagl-rust/src/routing/scan_line.rs`

The scanline key currently uses `obstacle_ordinal`. After Task 2, `ObstacleSide` has `obstacle_index()` instead. Update `SideKey` to use `obstacle_index`. Also add `find()` method for looking up a specific side, and `next_low()/next_high()` for neighbor traversal from a specific side node.

- [ ] **Step 1: Update SideKey to use obstacle_index**

- [ ] **Step 2: Add find/next_low/next_high methods**

These match TS `RectilinearScanLine.Find()`, `NextLow()`, `NextHigh()`:

```rust
/// Find the key for a specific side in the scanline.
pub fn find(&self, side: &ObstacleSide) -> Option<SideKey> { ... }

/// Get the next side in the low direction from a given key.
pub fn next_low(&self, key: &SideKey) -> Option<(&SideKey, &ObstacleSide)> {
    self.sides.range(..key.clone()).next_back()
}

/// Get the next side in the high direction from a given key.
pub fn next_high(&self, key: &SideKey) -> Option<(&SideKey, &ObstacleSide)> {
    self.sides.range((std::ops::Bound::Excluded(key.clone()), std::ops::Bound::Unbounded)).next()
}
```

- [ ] **Step 3: Run full test suite**

- [ ] **Step 4: Commit**

---

## Task 6: Update ScanSegment with Vertex Tracking

**Files:**
- Modify: `msagl-rust/src/routing/scan_segment.rs`

The TS `ScanSegment` tracks `LowestVisibilityVertex` and `HighestVisibilityVertex` as vertices are added during intersection. It has `OnSegmentIntersectorBegin()`, `OnSegmentIntersectorEnd()`, and `AppendVisibilityVertex()` methods.

- [ ] **Step 1: Read TS ScanSegment.ts**

Read the full `ScanSegment.ts` file. Key methods:
- `OnSegmentIntersectorBegin(visGraph)` — creates the start vertex
- `OnSegmentIntersectorEnd(visGraph)` — creates the end vertex
- `AppendVisibilityVertex(visGraph, vertex)` — adds vertex to chain, creates edge from HighestVisibilityVertex

- [ ] **Step 2: Add vertex tracking methods to ScanSegment**

```rust
impl ScanSegment {
    /// Called when the segment intersector begins processing this segment.
    /// Creates a vertex at the start point.
    /// Matches TS: ScanSegment.OnSegmentIntersectorBegin()
    pub fn on_intersector_begin(&mut self, graph: &mut VisibilityGraph) {
        let v = graph.add_vertex(self.start);
        self.lowest_vertex = Some(v);
        self.highest_vertex = Some(v);
    }

    /// Called when the segment intersector finishes this segment.
    /// Creates a vertex at the end point and connects it.
    /// Matches TS: ScanSegment.OnSegmentIntersectorEnd()
    pub fn on_intersector_end(&mut self, graph: &mut VisibilityGraph) {
        let v = graph.add_vertex(self.end);
        self.append_visibility_vertex(graph, v);
    }

    /// Append a visibility vertex to this segment's chain.
    /// Creates an edge from the current highest vertex to the new vertex.
    /// Matches TS: ScanSegment.AppendVisibilityVertex()
    pub fn append_visibility_vertex(&mut self, graph: &mut VisibilityGraph, vertex: VertexId) {
        if let Some(prev) = self.highest_vertex {
            if prev != vertex {
                let p1 = graph.point(prev);
                let p2 = graph.point(vertex);
                let dist = (p2 - p1).length() * self.weight.value() as f64;
                graph.add_edge(prev, vertex, dist);
                graph.add_edge(vertex, prev, dist);
            }
        }
        if self.lowest_vertex.is_none() {
            self.lowest_vertex = Some(vertex);
        }
        self.highest_vertex = Some(vertex);
    }
}
```

- [ ] **Step 3: Run full test suite**

- [ ] **Step 4: Commit**

---

## Task 7: Rewrite segment_intersector.rs with Event-Based Sweep

**Files:**
- Modify: `msagl-rust/src/routing/segment_intersector.rs`
- Test: `msagl-rust/tests/visibility.rs`

**TS Reference:** `SegmentIntersector.ts` (285 lines)

Replace the O(n^2) nested loop with the TS's event-based sweep algorithm:
1. Create VOpen/VClose events for each vertical segment
2. Create HOpen events for each horizontal segment
3. Sort events by Y, then type (VOpen < HOpen < VClose), then X
4. Maintain a BTreeMap of active vertical segments (keyed by X)
5. For each HOpen, scan the BTreeMap for vertical segments in the H segment's X range
6. At each intersection, call `seg.AppendVisibilityVertex()`

- [ ] **Step 1: Write test for event-based intersection**

```rust
#[test]
fn segment_intersector_creates_correct_vertex_count() {
    // 2 horizontal segments + 2 vertical segments = 4 potential crossings
    // Verify exact vertex count matches expectation
}
```

- [ ] **Step 2: Implement SegmentIntersector**

The implementer MUST read `SegmentIntersector.ts` lines 1-285 and port every method. Key design:

```rust
use std::collections::BTreeMap;
use ordered_float::OrderedFloat;

#[derive(PartialEq, Eq, PartialOrd, Ord)]
enum SegEventType { VOpen, VClose, HOpen }

struct SegEvent {
    event_type: SegEventType,
    segment_index: usize,
    is_vertical: bool,
}

impl SegEvent {
    fn site(&self, segments: &[ScanSegment]) -> Point {
        let seg = &segments[self.segment_index];
        match self.event_type {
            SegEventType::VClose => seg.end,
            _ => seg.start,
        }
    }
}

pub fn build_graph_from_segments(
    h_segments: &mut [ScanSegment],
    v_segments: &mut [ScanSegment],
) -> VisibilityGraph {
    // 1. Create events
    // 2. Sort by Compare (TS lines 181-253)
    // 3. Create graph
    // 4. Process events:
    //    VOpen: call on_intersector_begin, insert into scanline BTreeMap
    //    VClose: call on_intersector_end, remove from scanline
    //    HOpen: call on_intersector_begin, scan intersect, call on_intersector_end
    // 5. Return graph
}
```

The sort order (TS lines 196-253) is CRITICAL:
- Primary: Y coordinate
- For two vertical events at same Y: VOpen before VClose
- For V vs H at same Y: VOpen before HOpen, HOpen before VClose
- For two H events at same Y: by X coordinate

- [ ] **Step 3: Run tests**

- [ ] **Step 4: Commit**

---

## Task 8: Rewrite visibility_graph_generator.rs with Full Event Processing

**Files:**
- Modify: `msagl-rust/src/routing/visibility_graph_generator.rs`
- Modify: `msagl-rust/src/routing/neighbor_sides.rs`
- Modify: `msagl-rust/src/routing/lookahead_scan.rs`
- Test: `msagl-rust/tests/visibility.rs`

**TS Reference:** `VisibilityGraphGenerator.ts` (1,013 lines) + `FullVisibilityGraphGenerator.cs` (320 lines)

This is the biggest task. The implementer MUST read both source files in their entirety before writing code.

**Structure:**
- `VisibilityGraphGenerator` struct holds: scan_direction, event_queue, h_scan_segments, v_scan_segments, scanline, lookahead_scan, neighbor_sides, obstacle_tree, want_reflections
- `FullVisibilityGraphGenerator` is the concrete implementation (want_reflections = true)
- Main entry: `generate_visibility_graph()` does two passes (H-scan then V-scan), merges segments, then calls `SegmentIntersector`

**Critical methods to port (in order of the event loop):**

1. `GenerateVisibilityGraph()` — TS lines 108-188: two-pass with sentinels
2. `InitializeEventQueue()` — TS lines 262-268: reset queue, enqueue bottom vertices
3. `EnqueueBottomVertexEvents()` — TS lines 269-274: walk polylines to find open vertices
4. `ProcessEvents()` — TS lines 590-620: dispatch by event type
5. `ProcessEventO()` — TS lines 765-810: open vertex (add sides, find neighbors, create segments, check reflections, enqueue bend/close)
6. `ProcessEventLB()` — TS lines 813-833: low bend (update side if ascending)
7. `ProcessEventHB()` — TS lines 841-878: high bend (update side, extreme vertex lookahead)
8. `ProcessEventCV()` / `CreateCloseEventSegmentsAndFindNeighbors()` — TS lines 895-968: close (find neighbors, create segments, load reflections, remove sides)
9. `ProcessEventLR()` — TS lines 970-982: low reflection
10. `ProcessEventHR()` — TS lines 985-997: high reflection
11. `FindNeighborsBRR()` + `FindNeighbors()` + `SkipToNeighbor()` — TS lines 651-722: neighbor finding with group/overlap awareness
12. `StoreLookaheadSite()` — TS lines 314-358: store reflection lookahead
13. `LoadReflectionEvents()` + `LoadReflectionEventsBB()` — TS lines 360-443: load pending reflections for a new side
14. `AddPerpendicularReflectionSegment()` — TS lines 445-517: validate and create perp reflection
15. `CreateScanSegments()` — C# FullVGG lines 210-283: create 1-3 scan segments with overlap handling

- [ ] **Step 1: Implement the struct and GenerateVisibilityGraph**

The top-level function signature stays the same: `generate_visibility_graph(shapes: &[Shape], padding: f64) -> VisibilityGraph`.

Internally, create a `FullVisibilityGraphGenerator` struct that owns all state.

- [ ] **Step 2: Implement ProcessEventO (Open Vertex)**

This is the most complex single method. Port TS lines 765-810 faithfully:
- Add both sides to scanline via `add_side_to_scanline()` (which calls `LoadReflectionEvents`)
- Find neighbors via `FindNeighborsAndProcessVertexEvent`
- Check for reflection loading on neighbor sides
- Handle flat bottom absorption
- Enqueue LowBend and HighBend/Close events

- [ ] **Step 3: Implement ProcessEventLB and ProcessEventHB**

Port TS lines 813-878. Key: LowBend only replaces the side if the new side is still ascending. HighBend has the extreme-vertex lookahead logic.

- [ ] **Step 4: Implement ProcessEventCV (Close Vertex)**

Port TS lines 935-968 and C# lines. Close is the most complex because it handles reflection chain continuation.

- [ ] **Step 5: Implement reflection event processing**

Port `ProcessEventLR` and `ProcessEventHR` from TS lines 970-997. These call `AddPerpendicularReflectionSegment` and `AddParallelReflectionSegment`.

- [ ] **Step 6: Implement CreateScanSegments from FullVisibilityGraphGenerator.cs**

This is the concrete `ProcessVertexEvent` implementation. Port C# lines 210-317 faithfully. Handles:
- Non-overlapped case: single segment from lowNbor to highNbor
- Overlapped cases: up to 3 sub-segments with different weights

- [ ] **Step 7: Implement neighbor finding (SkipToNeighbor)**

Port TS lines 679-722. Skip past same-obstacle sides, group sides, and overlap-ending sides to find the true neighbor.

- [ ] **Step 8: Implement lookahead/reflection infrastructure**

Port `StoreLookaheadSite`, `LoadReflectionEvents`, `AddPerpendicularReflectionSegment`. These are the reflection chain logic.

- [ ] **Step 9: Write comprehensive tests**

```rust
#[test]
fn vg_two_rectangles_has_correct_vertex_count() {
    // Two non-overlapping rectangles should produce a VG with
    // the same number of vertices as the TS implementation
}

#[test]
fn vg_three_rectangles_stacked_produces_correct_segments() {
    // Verify segment count matches expected for stacked rectangles
}

#[test]
fn vg_single_rectangle_has_four_corners_connected() {
    // Single rectangle should produce exactly 4 corner vertices
    // connected by edges forming the rectangle border
}
```

- [ ] **Step 10: Run full test suite including existing rectilinear tests**

Run: `cargo test 2>&1 | grep "test result"`
Expected: All 468+ tests pass

- [ ] **Step 11: Commit**

---

## Verification Checklist

After all tasks are complete, the implementer MUST verify:

- [ ] Every method in TS `VisibilityGraphGenerator.ts` has a Rust equivalent
- [ ] Every method in C# `FullVisibilityGraphGenerator.cs` has a Rust equivalent
- [ ] Every method in TS `obstacle.ts` has a Rust equivalent (except DEFERRED items: ConvexHull, Groups)
- [ ] Every method in TS `BasicObstacleSide.ts` has a Rust equivalent
- [ ] Every method in TS `SegmentIntersector.ts` has a Rust equivalent
- [ ] All 6 event types are processed (not silently ignored)
- [ ] Reflection events create scan segments (not no-ops)
- [ ] LowBend/HighBend events update active sides
- [ ] The segment intersector uses an RBTree/BTreeMap scanline (not nested loops)
- [ ] `cargo test` passes
- [ ] `cargo clippy -- -D warnings` is clean
