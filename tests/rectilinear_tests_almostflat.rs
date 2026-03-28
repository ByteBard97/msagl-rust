/// AlmostFlat obstacle tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// These tests exercise routing around obstacles whose sides are "almost flat"
/// (within ApproximateComparer.DistanceEpsilon of horizontal), testing the
/// visibility-graph generator's handling of degenerate near-horizontal polygon sides.
///
/// ApproximateComparer.DistanceEpsilon == 1e-6

#[path = "test_harness/mod.rs"]
mod test_harness;

use msagl_rust::Point;
use test_harness::verifier::RECTILINEAR_TOLERANCE;
use test_harness::{ScenarioBuilder, Verifier};

/// C# `ApproximateComparer.DistanceEpsilon`
const DISTANCE_EPSILON: f64 = 1e-6_f64;

/// C# `FlatOffset = ApproximateComparer.DistanceEpsilon * 2`
const FLAT_OFFSET: f64 = DISTANCE_EPSILON * 2.0;

// ── Shared workers ────────────────────────────────────────────────────────────

/// C# `AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(isOpen, wantInteriorOverlap, wantInteriorNeighbor)`
/// (C# line 1727)
///
/// Builds 4 obstacles:
///   0. Fixed-neighbour rectangle (PolylineFromRectanglePoints)
///   1. Quad with almost-flat LowSide (CurveFromPoints)
///   2. Obstacle that may overlap the event vertex (PolylineFromRectanglePoints)
///   3. Obstacle that may be an interior neighbour (PolylineFromRectanglePoints)
///
/// Routes from ids[0] to all others.
fn almost_flat_lowside_worker(
    is_open: bool,
    want_interior_overlap: bool,
    want_interior_neighbor: bool,
) {
    let mut b = ScenarioBuilder::new();

    // Obstacle 0: fixed neighbour rectangle.
    // PolylineFromRectanglePoints(new Point(10, isOpen ? -10 : 0), new Point(20, isOpen ? 30 : 40))
    let id0 = b.add_rectangle_corners(10.0, if is_open { -10.0 } else { 0.0 }, 20.0, if is_open { 30.0 } else { 40.0 });

    // Obstacle 1: quad with almost-flat LowSide.
    // CurveFromPoints: (40, 10 + (isOpen ? ε : 0)), (40, 20), (50, 20 + (!isOpen ? ε : 0)), (50, 10)
    let id1 = b.add_polygon(&[
        Point::new(40.0, 10.0 + if is_open { DISTANCE_EPSILON } else { 0.0 }),
        Point::new(40.0, 20.0),
        Point::new(50.0, 20.0 + if !is_open { DISTANCE_EPSILON } else { 0.0 }),
        Point::new(50.0, 10.0),
    ]);

    // Obstacle 2: may overlap the event vertex.
    // PolylineFromRectanglePoints(new Point(wantInteriorOverlap ? 49.5 : 54, 5), new Point(55, 25))
    let id2 = b.add_rectangle_corners(
        if want_interior_overlap { 49.5 } else { 54.0 },
        5.0,
        55.0,
        25.0,
    );

    // Obstacle 3: may be an interior neighbour.
    // PolylineFromRectanglePoints(new Point(35, 5), new Point(wantInteriorNeighbor ? 45.5 : 36, 25))
    let id3 = b.add_rectangle_corners(
        35.0,
        5.0,
        if want_interior_neighbor { 45.5 } else { 36.0 },
        25.0,
    );

    let ids = [id0, id1, id2, id3];
    for i in 1..ids.len() {
        b.route_between(ids[0], ids[i]);
    }

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C# `AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(isOpen, wantInteriorOverlap, wantInteriorNeighbor)`
/// (C# line 1761)
///
/// Builds 4 obstacles:
///   0. Fixed-neighbour rectangle (PolylineFromRectanglePoints)
///   1. Quad with almost-flat HighSide (CurveFromPoints)
///   2. Obstacle that may overlap the event vertex (PolylineFromRectanglePoints)
///   3. Obstacle that may be an interior neighbour (PolylineFromRectanglePoints)
///
/// Routes from ids[0] to all others.
fn almost_flat_highside_worker(
    is_open: bool,
    want_interior_overlap: bool,
    want_interior_neighbor: bool,
) {
    let mut b = ScenarioBuilder::new();

    // Obstacle 0: fixed neighbour rectangle.
    // PolylineFromRectanglePoints(new Point(60, isOpen ? -10 : 0), new Point(70, isOpen ? 30 : 40))
    let id0 = b.add_rectangle_corners(60.0, if is_open { -10.0 } else { 0.0 }, 70.0, if is_open { 30.0 } else { 40.0 });

    // Obstacle 1: quad with almost-flat HighSide.
    // CurveFromPoints: (40, 10), (40, 20 + (!isOpen ? ε : 0)), (50, 20), (50, 10 + (isOpen ? ε : 0))
    let id1 = b.add_polygon(&[
        Point::new(40.0, 10.0),
        Point::new(40.0, 20.0 + if !is_open { DISTANCE_EPSILON } else { 0.0 }),
        Point::new(50.0, 20.0),
        Point::new(50.0, 10.0 + if is_open { DISTANCE_EPSILON } else { 0.0 }),
    ]);

    // Obstacle 2: may overlap the event vertex.
    // PolylineFromRectanglePoints(new Point(35, 5), new Point(wantInteriorOverlap ? 40.5 : 36, 25))
    let id2 = b.add_rectangle_corners(
        35.0,
        5.0,
        if want_interior_overlap { 40.5 } else { 36.0 },
        25.0,
    );

    // Obstacle 3: may be an interior neighbour.
    // PolylineFromRectanglePoints(new Point(wantInteriorNeighbor ? 44.5 : 54, 5), new Point(55, 25))
    let id3 = b.add_rectangle_corners(
        if want_interior_neighbor { 44.5 } else { 54.0 },
        5.0,
        55.0,
        25.0,
    );

    let ids = [id0, id1, id2, id3];
    for i in 1..ids.len() {
        b.route_between(ids[0], ids[i]);
    }

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C# `FlatWorker(leftBottomOffset, leftTopOffset)` (C# line 1542)
///
/// Builds:
///   0. Quad with (almost-)flat side: (10, 10+lbo), (10, 20+lto), (20, 20), (20, 10)
///   1..2. Two crossing triangles at increments of 0.5
///   3. Upper crossing triangle
///
/// Routes from ids[0] to all others.
fn flat_worker_almostflat(left_bottom_offset: f64, left_top_offset: f64) {
    let mut b = ScenarioBuilder::new();

    // Obstacle 0: quad with (almost-)flat side (CurveFromPoints).
    let id0 = b.add_polygon(&[
        Point::new(10.0, 10.0 + left_bottom_offset),
        Point::new(10.0, 20.0 + left_top_offset),
        Point::new(20.0, 20.0),
        Point::new(20.0, 10.0),
    ]);

    // Obstacles 1..2: crossing triangles at increments of 0.5 (Divisor = 2).
    // C# line 1556: for (int ii = 0; ii < 2; ++ii) { double inc = ii / 2.0; ... }
    let divisor = 2_usize;
    let mut crossing_ids = Vec::new();
    for ii in 0..divisor {
        let inc = ii as f64 / divisor as f64;
        let id = b.add_polygon(&[
            Point::new(18.5 + inc, 15.0),
            Point::new(23.5 + inc, 30.0),
            Point::new(28.5 + inc, 15.0),
        ]);
        crossing_ids.push(id);
    }

    // Obstacle 3: upper crossing triangle (C# line 1567).
    let id_upper = b.add_polygon(&[
        Point::new(18.0, 25.0),
        Point::new(23.0, 40.0),
        Point::new(28.0, 25.0),
    ]);

    // Route from ids[0] to all others (CreateRoutingBetweenObstacles(obstacles, 0, -1)).
    for &id in crossing_ids.iter().chain(std::iter::once(&id_upper)) {
        b.route_between(id0, id);
    }

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Open / LowSide tests ──────────────────────────────────────────────────────

/// Port: AlmostFlat_Open_LowSide_NoOverlap (C# line 1574)
#[test]
fn almost_flat_open_lowside_no_overlap() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(true, false, false)
    almost_flat_lowside_worker(true, false, false);
}

/// Port: AlmostFlat_Open_LowSide_InteriorLowOverlap (C# line 1583)
#[test]
fn almost_flat_open_lowside_interior_low_overlap() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(true, true, false)
    almost_flat_lowside_worker(true, true, false);
}

/// Port: AlmostFlat_Open_LowSide_InteriorLowNeighbor (C# line 1593)
#[test]
fn almost_flat_open_lowside_interior_low_neighbor() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(true, false, true)
    almost_flat_lowside_worker(true, false, true);
}

/// Port: AlmostFlat_Open_LowSide_InteriorLowOverlap_LowNeighbor (C# line 1603)
#[test]
fn almost_flat_open_lowside_interior_low_overlap_low_neighbor() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(true, true, true)
    almost_flat_lowside_worker(true, true, true);
}

// ── Open / HighSide tests ─────────────────────────────────────────────────────

/// Port: AlmostFlat_Open_HighSide_NoOverlap (C# line 1613)
#[test]
fn almost_flat_open_highside_no_overlap() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(true, false, false)
    almost_flat_highside_worker(true, false, false);
}

/// Port: AlmostFlat_Open_HighSide_InteriorHighOverlap (C# line 1622)
#[test]
fn almost_flat_open_highside_interior_high_overlap() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(true, true, false)
    almost_flat_highside_worker(true, true, false);
}

/// Port: AlmostFlat_Open_HighSide_InteriorHighNeighbor (C# line 1632)
#[test]
fn almost_flat_open_highside_interior_high_neighbor() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(true, false, true)
    almost_flat_highside_worker(true, false, true);
}

/// Port: AlmostFlat_Open_HighSide_InteriorHighOverlap_HighNeighbor (C# line 1642)
#[test]
fn almost_flat_open_highside_interior_high_overlap_high_neighbor() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(true, true, true)
    almost_flat_highside_worker(true, true, true);
}

// ── Close / LowSide tests ─────────────────────────────────────────────────────

/// Port: AlmostFlat_Close_LowSide_NoOverlap (C# line 1652)
#[test]
fn almost_flat_close_lowside_no_overlap() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(false, false, false)
    almost_flat_lowside_worker(false, false, false);
}

/// Port: AlmostFlat_Close_LowSide_InteriorLowOverlap (C# line 1661)
#[test]
fn almost_flat_close_lowside_interior_low_overlap() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(false, true, false)
    almost_flat_lowside_worker(false, true, false);
}

/// Port: AlmostFlat_Close_LowSide_InteriorLowNeighbor (C# line 1671)
#[test]
fn almost_flat_close_lowside_interior_low_neighbor() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(false, false, true)
    almost_flat_lowside_worker(false, false, true);
}

/// Port: AlmostFlat_Close_LowSide_InteriorLowOverlap_LowNeighbor (C# line 1681)
#[test]
fn almost_flat_close_lowside_interior_low_overlap_low_neighbor() {
    // AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker(false, true, true)
    almost_flat_lowside_worker(false, true, true);
}

// ── Close / HighSide tests ────────────────────────────────────────────────────

/// Port: AlmostFlat_Close_HighSide_NoOverlap (C# line 1691)
#[test]
fn almost_flat_close_highside_no_overlap() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(false, false, false)
    almost_flat_highside_worker(false, false, false);
}

/// Port: AlmostFlat_Close_HighSide_InteriorHighOverlap (C# line 1700)
#[test]
fn almost_flat_close_highside_interior_high_overlap() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(false, true, false)
    almost_flat_highside_worker(false, true, false);
}

/// Port: AlmostFlat_Close_HighSide_InteriorHighNeighbor (C# line 1710)
#[test]
fn almost_flat_close_highside_interior_high_neighbor() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(false, false, true)
    almost_flat_highside_worker(false, false, true);
}

/// Port: AlmostFlat_Close_HighSide_InteriorHighOverlap_HighNeighbor (C# line 1720)
#[test]
fn almost_flat_close_highside_interior_high_overlap_high_neighbor() {
    // AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker(false, true, true)
    almost_flat_highside_worker(false, true, true);
}

// ── Multiple-crosses tests ────────────────────────────────────────────────────

/// Port: AlmostFlatHighSideWithMultipleCrosses (C# line 1529)
///
/// FlatWorker(leftBottomOffset=0, leftTopOffset=FlatOffset)
/// Quad has almost-flat HighSide (top-left vertex raised by FlatOffset).
#[test]
fn almost_flat_highside_with_multiple_crosses() {
    // C# line 1531: FlatWorker(0, FlatOffset)
    flat_worker_almostflat(0.0, FLAT_OFFSET);
}

/// Port: AlmostFlatLowSideWithMultipleCrosses (C# line 1537)
///
/// FlatWorker(leftBottomOffset=FlatOffset, leftTopOffset=0)
/// Quad has almost-flat LowSide (bottom-left vertex raised by FlatOffset).
#[test]
fn almost_flat_lowside_with_multiple_crosses() {
    // C# line 1539: FlatWorker(FlatOffset, 0)
    flat_worker_almostflat(FLAT_OFFSET, 0.0);
}
