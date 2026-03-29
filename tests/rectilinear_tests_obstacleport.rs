/// ObstaclePort tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// These three tests exercise ObstaclePort behaviour on non-rectangular
/// (triangular) obstacles. All three are currently marked `#[ignore]` because
/// the Rust VG generator represents non-rectangular obstacles by their axis-
/// aligned bounding box, not by their true polygon boundary.  As a result the
/// router cannot faithfully reproduce the C# "port just outside unpadded
/// boundary" / "dead-end open space" scenarios, which rely on the swept
/// visibility algorithm processing angled sides.  When full non-rectangular
/// ICurve support is added these tests should be un-ignored.

#[path = "test_harness/mod.rs"]
mod test_harness;

use msagl_rust::Point;
use test_harness::verifier::RECTILINEAR_TOLERANCE;
use test_harness::{ScenarioBuilder, Verifier};

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Add a polygon obstacle from a list of `(x, y)` tuples.
/// Mirrors C# `CurveFromPoints(points[])` for triangular/polygonal obstacles.
fn add_polygon_pts(b: &mut ScenarioBuilder, points: &[(f64, f64)]) -> usize {
    let pts: Vec<Point> = points.iter().map(|&(x, y)| Point::new(x, y)).collect();
    b.add_polygon(&pts)
}

/// Compute the bounding box of a set of points and add a rectangle obstacle.
/// Used as a conservative placeholder when exact polygon routing is not yet
/// supported (same bounding box as the non-rectangular obstacle).
#[allow(dead_code)]
fn add_polygon_bbox(b: &mut ScenarioBuilder, points: &[(f64, f64)]) -> usize {
    let min_x = points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_y = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    b.add_rectangle_bl(min_x, min_y, max_x - min_x, max_y - min_y)
}

// ── Shared obstacle set for DeadEnd_OpenSpace tests ──────────────────────────

/// Mirrors C# `DeadEnd_OpenSpace_Obstacles()` (RectilinearTests.cs line 2639).
///
/// Three obstacles:
///   - [0]: upward triangle (70,30)→(30,70)→(110,70)
///   - [1]: rectangle (0,60)→(20,80)
///   - [2]: rectangle (60,0)→(80,20)
///
/// "Two dead-ends of ScanSegments strike an angled side of an obstacle,
/// creating open space."
fn dead_end_open_space_obstacles(b: &mut ScenarioBuilder) -> Vec<usize> {
    // obstacle[0]: triangle
    let tri = add_polygon_pts(b, &[(70.0, 30.0), (30.0, 70.0), (110.0, 70.0)]);
    // obstacle[1]: rectangle
    let r1 = b.add_rectangle_corners(0.0, 60.0, 20.0, 80.0);
    // obstacle[2]: rectangle
    let r2 = b.add_rectangle_corners(60.0, 0.0, 80.0, 20.0);
    vec![tri, r1, r2]
}

// ── Tests ─────────────────────────────────────────────────────────────────────

/// Port: `Triangle_ObstaclePort_Outside_Obstacle`
/// C# RectilinearTests.cs line 2488.
///
/// Description: "Test obstacle port just outside the obstacle unpadded boundary."
///
/// Obstacles:
///   - [0]: triangular obstacle — centre just outside its unpadded borders:
///          (101.634, 56.340), (107.685, 62.284), (101.745, 62.278)
///   - [1]: rectangle (80,50)→(90,60)
///
/// C# routes from obstacle[0] to all others via
/// `CreateRoutingBetweenObstacles(obstacles, 0, -1)`.
///
/// # Why ignored
/// The triangular ICurve is approximated as its axis-aligned bounding box in
/// the Rust VG generator. The "port just outside unpadded boundary" property
/// of the C# test depends on the exact triangle shape being processed by the
/// sweep-line visibility algorithm. Until non-rectangular ICurve support is
/// added (`Shape::polygon` routing via true polygon boundaries), this test
/// cannot be run with fidelity.
#[test]
fn triangle_obstacle_port_outside_obstacle() {
    // C# line 2490-2501
    let mut b = ScenarioBuilder::new();

    // obstacle[0]: triangle — centre just outside unpadded boundary
    let tri = add_polygon_pts(
        &mut b,
        &[
            (101.634_005_539_731, 56.340_076_558_977_2),
            (107.685_715_981_301, 62.284_599_905_020_7),
            (101.745_914_212_075, 62.278_826_834_507_3),
        ],
    );

    // obstacle[1]: rectangle (80,50)→(90,60)
    let rect = b.add_rectangle_corners(80.0, 50.0, 90.0, 60.0);

    // C#: CreateRoutingBetweenObstacles(obstacles, 0, -1) — route from [0] to all others
    b.route_between(tri, rect);

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

/// Port: `DeadEnd_OpenSpace_ObstaclePort0`
/// C# RectilinearTests.cs line 2621.
///
/// Description: "Two dead-ends of ScanSegments strike an angled side of an
/// obstacle, creating open space."
///
/// Obstacles (from `DeadEnd_OpenSpace_Obstacles()`):
///   - [0]: triangle (70,30)→(30,70)→(110,70)
///   - [1]: rectangle (0,60)→(20,80)
///   - [2]: rectangle (60,0)→(80,20)
///
/// C# routes from obstacle[0] to all others via
/// `CreateRoutingBetweenObstacles(obstacles, 0, -1)`.
///
/// # Why ignored
/// The triangular obstacle at index 0 is the port host. The "dead-end open
/// space" scenario requires the VG generator to process angled sides of a
/// non-rectangular obstacle when computing port entrance visibility chains.
/// The Rust VG generator currently routes around bounding boxes only; the
/// triangle's angled sides are invisible to it, so the open-space condition
/// cannot be reproduced. Un-ignore when non-rectangular ICurve support lands.
#[test]
fn dead_end_open_space_obstacle_port0() {
    // C# line 2621-2626
    let mut b = ScenarioBuilder::new();
    let ids = dead_end_open_space_obstacles(&mut b);

    // C#: CreateRoutingBetweenObstacles(obstacles, 0, -1) — route from [0] to all others
    for i in 1..ids.len() {
        b.route_between(ids[0], ids[i]);
    }

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, ids.len() - 1);
}

/// Port: `DeadEnd_OpenSpace_ObstaclePort0_EliminateExtraBend`
/// C# RectilinearTests.cs line 2655.
///
/// Description: "Two dead-ends of ScanSegments strike an angled side of an
/// obstacle, creating open space; verify no extraneous bends exist."
///
/// Obstacles (created in-order so lower obstacle port visibility runs first):
///   - [0]: rectangle (0,0)→(90,20)
///   - [1]: triangle (70,30)→(30,70)→(110,70)
///   - [2]: rectangle (0,60)→(20,80)
///
/// C# routes from obstacle[0] to obstacle[1] and asserts exactly 1 bend
/// (direct visibility splice, no extra bend).
///
/// `count_bends()` in C# counts groups of consecutive Ellipse segments in the
/// Curve; in Rust, a "bend" is a direction change between consecutive
/// rectilinear segments. A single bend means exactly 3 waypoints (L-shape).
///
/// # Why ignored
/// The single-bend assertion depends on the direct visibility splice that the
/// C# router performs when an obstacle port can reach its target along a single
/// straight scan segment, bypassing one of the normal L-shape legs. This
/// optimisation is specific to the non-rectangular VG generator processing the
/// triangle's angled side. The bounding-box approximation cannot produce the
/// same splice path, so the bend count assertion is not valid in Rust today.
#[test]
fn dead_end_open_space_obstacle_port0_eliminate_extra_bend() {
    // C# line 2655-2670
    let mut b = ScenarioBuilder::new();

    // Obstacles in specific order to reproduce the C# port-visibility ordering.
    // obstacle[0]: rectangle (0,0)→(90,20)
    let rect0 = b.add_rectangle_corners(0.0, 0.0, 90.0, 20.0);
    // obstacle[1]: triangle (70,30)→(30,70)→(110,70)
    let tri1 = add_polygon_pts(&mut b, &[(70.0, 30.0), (30.0, 70.0), (110.0, 70.0)]);
    // obstacle[2]: rectangle (0,60)→(20,80)
    let _rect2 = b.add_rectangle_corners(0.0, 60.0, 20.0, 80.0);

    // C#: CreateRoutingBetweenObstacles(obstacles, 0, 1) — route from [0] to [1]
    b.route_between(rect0, tri1);

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);

    // C# line 2669: Validate.AreEqual(1, CountBends(router.EdgeGeometries.First()), ...)
    // CountBends counts groups of successive Ellipse curve segments (each bend
    // in the path is one group).  In rectilinear terms: 1 bend → 3 waypoints
    // (the path makes exactly one 90° turn).
    //
    // A path with exactly 1 bend has the waypoint sequence:
    //   source → corner → target  (3 points total)
    let bend_count = count_bends(&result.edges[0].points);
    assert_eq!(
        bend_count, 1,
        "expected exactly 1 bend (direct visibility splice), got {}",
        bend_count
    );
}

// ── Bend counting ─────────────────────────────────────────────────────────────

/// Count the number of direction changes (bends) in a rectilinear path.
///
/// Mirrors C# `CountBends(Curve)` (RectilinearTests.cs line 2696), which
/// counts groups of consecutive `Ellipse` segments. Each Ellipse represents
/// a rounded corner arc. In our straight-segment representation, a bend is
/// simply a change of direction between consecutive waypoint pairs.
///
/// For example:
/// - 2-point path (straight line): 0 bends
/// - 3-point path (L-shape):       1 bend
/// - 4-point path (S-shape):       2 bends
fn count_bends(points: &[Point]) -> usize {
    if points.len() < 3 {
        return 0;
    }
    let mut bends = 0_usize;
    for w in points.windows(3) {
        let _dx1 = w[1].x() - w[0].x();
        let dy1 = w[1].y() - w[0].y();
        let _dx2 = w[2].x() - w[1].x();
        let dy2 = w[2].y() - w[1].y();
        // A bend occurs when the direction changes (horizontal↔vertical).
        // For a rectilinear path, if dx1 ≠ 0 then dy2 ≠ 0 (or vice-versa).
        let seg1_horizontal = dy1.abs() < 1e-6;
        let seg2_horizontal = dy2.abs() < 1e-6;
        if seg1_horizontal != seg2_horizontal {
            bends += 1;
        }
    }
    bends
}

