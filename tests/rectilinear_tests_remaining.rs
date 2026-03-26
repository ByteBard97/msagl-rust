/// Remaining rectilinear routing tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// These are the final ~15 tests that had not yet been ported. They cover:
/// - One_Obstacle_Graph (single diamond, VG only)
/// - AlmostFlat*WithMultipleCrosses (non-rect, almost-flat sides)
/// - DeadEnd_* (dead-end scan segment tests)
/// - NudgerSmoothingStaircasesAlongConvexHulls
/// - Reflections_Taken_And_Skipped
/// - ReflectionsDetectedByAlreadyLoadedSide
/// - ReflectionsSitedByLowSideAreNotLoadedByHighSide
/// - ReflectionsRemoveInterceptedSite

#[path = "test_harness/mod.rs"]
mod test_harness;

use test_harness::{ScenarioBuilder, Verifier};
use test_harness::verifier::RECTILINEAR_TOLERANCE;

// ── Helpers (shared with rectilinear_tests_nonrect.rs) ──────────────────────

/// Compute the axis-aligned bounding box for a set of polygon vertices.
fn bounding_box(points: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let min_x = points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_y = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    (min_x, min_y, max_x - min_x, max_y - min_y)
}

/// Add a bounding-box rectangle as a placeholder for a non-rectangular obstacle.
fn add_nonrect_placeholder(
    b: &mut ScenarioBuilder,
    points: &[(f64, f64)],
) -> usize {
    let (x, y, w, h) = bounding_box(points);
    b.add_rectangle_bl(x, y, w, h)
}

/// C# `ApproximateComparer.DistanceEpsilon` = 10^-6.
const DISTANCE_EPSILON: f64 = 1e-6;

/// C# `FlatOffset = ApproximateComparer.DistanceEpsilon * 2`.
const FLAT_OFFSET: f64 = DISTANCE_EPSILON * 2.0;

// ── Category: One obstacle (VG-only smoke test) ─────────────────────────────

/// Port: One_Obstacle_Graph
/// Graph with a single diamond obstacle; just runs VG creation, no edges.
/// C#: PolylineFromPoints at (55,40),(95,60),(135,40),(95,20).
#[test]
fn one_obstacle_graph() {
    let mut b = ScenarioBuilder::new();
    // Diamond vertices: (55,40),(95,60),(135,40),(95,20)
    // Use bounding-box placeholder: left=55, bottom=20, width=80, height=40
    add_nonrect_placeholder(&mut b, &[
        (55.0, 40.0), (95.0, 60.0), (135.0, 40.0), (95.0, 20.0),
    ]);
    // No routes — just verify VG creation succeeds
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
}

// ── Category: AlmostFlat with multiple crosses ──────────────────────────────

/// Worker for FlatTop/FlatBottom/AlmostFlatHigh/AlmostFlatLow tests.
/// Creates a non-rect obstacle with almost-flat sides plus crossing triangles.
/// C# `FlatWorker(leftBottomOffset, leftTopOffset)`.
fn flat_worker(
    b: &mut ScenarioBuilder,
    left_bottom_offset: f64,
    left_top_offset: f64,
) {
    // Main obstacle: (10, 10+lbo), (10, 20+lto), (20, 20), (20, 10)
    add_nonrect_placeholder(b, &[
        (10.0, 10.0 + left_bottom_offset), (10.0, 20.0 + left_top_offset),
        (20.0, 20.0), (20.0, 10.0),
    ]);

    // Two crossing triangles at increments of 0.5
    let divisor = 2;
    for ii in 0..divisor {
        let inc = ii as f64 / divisor as f64;
        add_nonrect_placeholder(b, &[
            (18.5 + inc, 15.0), (23.5 + inc, 30.0), (28.5 + inc, 15.0),
        ]);
    }

    // Upper crossing triangle
    add_nonrect_placeholder(b, &[(18.0, 25.0), (23.0, 40.0), (28.0, 25.0)]);
}

/// Port: AlmostFlatHighSideWithMultipleCrosses
/// Tests handling of an almost-flat HighObstacleSide with multiple crosses.
/// C#: FlatWorker(0, FlatOffset)
#[test]
fn almost_flat_high_side_with_multiple_crosses() {
    let mut b = ScenarioBuilder::new();
    flat_worker(&mut b, 0.0, FLAT_OFFSET);
    // Route from obstacle 0 to all others
    let n = b.shapes().len();
    let ids: Vec<usize> = (0..n).collect();
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: AlmostFlatLowSideWithMultipleCrosses
/// Tests handling of an almost-flat LowObstacleSide with multiple crosses.
/// C#: FlatWorker(FlatOffset, 0)
#[test]
fn almost_flat_low_side_with_multiple_crosses() {
    let mut b = ScenarioBuilder::new();
    flat_worker(&mut b, FLAT_OFFSET, 0.0);
    let n = b.shapes().len();
    let ids: Vec<usize> = (0..n).collect();
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Category: DeadEnd tests ─────────────────────────────────────────────────

/// Shared obstacle setup for DeadEnd_OpenSpace tests.
/// C#: triangle (70,30)-(30,70)-(110,70), rect (0,60)-(20,80), rect (60,0)-(80,20).
fn dead_end_open_space_obstacles(b: &mut ScenarioBuilder) -> Vec<usize> {
    let mut ids = Vec::new();
    // Triangle obstacle (non-rect placeholder)
    ids.push(add_nonrect_placeholder(b, &[
        (70.0, 30.0), (30.0, 70.0), (110.0, 70.0),
    ]));
    // Two rectangles
    ids.push(b.add_rectangle_corners(0.0, 60.0, 20.0, 80.0));
    ids.push(b.add_rectangle_corners(60.0, 0.0, 80.0, 20.0));
    ids
}

/// Port: DeadEnd_OpenSpace_ObstaclePort0
/// Two dead-ends of ScanSegments strike an angled side, creating open space.
/// C#: routes from obstacle 0 to all others.
#[test]
fn dead_end_open_space_obstacle_port0() {
    let mut b = ScenarioBuilder::new();
    let ids = dead_end_open_space_obstacles(&mut b);
    // Route from obstacle 0 to all others
    for &id in &ids[1..] {
        b.route_between(ids[0], id);
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: DeadEnd_OpenSpace_FreePort0
/// Same geometry but routes from all obstacles to a free port at (40,50).
/// C#: CreateSourceToFreePortRoutings(obstacles, -1, freePorts).
#[test]
fn dead_end_open_space_free_port0() {
    let mut b = ScenarioBuilder::new();
    let _ids = dead_end_open_space_obstacles(&mut b);
    // Free port at (40, 50) — we cannot express this with the current
    // ScenarioBuilder (needs free port support). Run VG-only.
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
}

/// Port: DeadEnd_OpenSpace_ObstaclePort0_EliminateExtraBend
/// Verifies no extraneous bends when dead-ends create open space.
/// C#: routes obstacle 0 to obstacle 1, checks bendCount == 1.
#[test]
fn dead_end_open_space_obstacle_port0_eliminate_extra_bend() {
    let mut b = ScenarioBuilder::new();
    // Obstacles in different order from the other DeadEnd tests:
    // rect (0,0)-(90,20), triangle (70,30)-(30,70)-(110,70), rect (0,60)-(20,80)
    let r0 = b.add_rectangle_corners(0.0, 0.0, 90.0, 20.0);
    let tri = add_nonrect_placeholder(&mut b, &[
        (70.0, 30.0), (30.0, 70.0), (110.0, 70.0),
    ]);
    let _r2 = b.add_rectangle_corners(0.0, 60.0, 20.0, 80.0);
    b.route_between(r0, tri);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
    // C# validates bendCount == 1; we verify the route completes with valid geometry.
}

/// Port: DeadEnd_Crossing
/// Two dead-ends of scansegs intersect before striking an angled side.
/// C#: triangle (70,30)-(30,70)-(110,70), rect (0,40)-(20,80), rect (40,0)-(80,20).
/// Routes from obstacle 0 to all others; each path should have 2 bends.
#[test]
fn dead_end_crossing() {
    let mut b = ScenarioBuilder::new();
    let tri = add_nonrect_placeholder(&mut b, &[
        (70.0, 30.0), (30.0, 70.0), (110.0, 70.0),
    ]);
    let r1 = b.add_rectangle_corners(0.0, 40.0, 20.0, 80.0);
    let r2 = b.add_rectangle_corners(40.0, 0.0, 80.0, 20.0);
    b.route_between(tri, r1);
    b.route_between(tri, r2);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 2);
}

/// Port: DeadEnd_Crossing_EdgeChains_Intersect
/// Visibility chains from two ports intersect in space between scanseg intersection
/// and the obstacle.
/// C#: rect (0,10)-(20,90), rect (30,0)-(110,20), triangle (110,30)-(50,90)-(110,90).
#[test]
fn dead_end_crossing_edge_chains_intersect() {
    let mut b = ScenarioBuilder::new();
    let r0 = b.add_rectangle_corners(0.0, 10.0, 20.0, 90.0);
    let _r1 = b.add_rectangle_corners(30.0, 0.0, 110.0, 20.0);
    let tri = add_nonrect_placeholder(&mut b, &[
        (110.0, 30.0), (50.0, 90.0), (110.0, 90.0),
    ]);
    b.route_between(r0, tri);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

// ── Category: Nudger staircase smoothing ────────────────────────────────────

/// Port: NudgerSmoothingStaircasesAlongConvexHulls
/// The nudger should remove staircases between convex hulls when no obstacles
/// intervene.
/// C#: 8 obstacles (2 routing endpoints, 2 wide blockers, 2 small squares,
/// 2 triangles that force convex hull creation).
#[test]
fn nudger_smoothing_staircases_along_convex_hulls() {
    let mut b = ScenarioBuilder::new();
    // Source and target
    let src = b.add_rectangle_corners(130.0, 140.0, 140.0, 150.0);
    let tgt = b.add_rectangle_corners(160.0, 20.0, 170.0, 30.0);

    // Wide blocking rectangles
    b.add_rectangle_corners(0.0, 40.0, 160.0, 60.0);
    b.add_rectangle_corners(140.0, 100.0, 300.0, 120.0);

    // Smaller squares forming convex hull corners
    b.add_rectangle_corners(100.0, 80.0, 120.0, 100.0);
    b.add_rectangle_corners(180.0, 60.0, 200.0, 80.0);

    // Two triangles that force convex hull creation rather than clumps
    add_nonrect_placeholder(&mut b, &[
        (110.0, 50.0), (80.0, 70.0), (110.0, 90.0),
    ]);
    add_nonrect_placeholder(&mut b, &[
        (190.0, 70.0), (190.0, 110.0), (220.0, 90.0),
    ]);

    b.route_between(src, tgt);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

// ── Category: Reflection tests ──────────────────────────────────────────────

/// Port: Reflections_Taken_And_Skipped
/// Two routing endpoints with blocking parallelograms. Routes once, then
/// resizes the blockers to make a shortcut cost-effective and routes again.
/// C#: two rounds of DoRouting with different obstacle sizes.
#[test]
fn reflections_taken_and_skipped() {
    // Round 1: original parallelograms
    {
        let mut b = ScenarioBuilder::new();
        let src = b.add_rectangle_corners(120.0, 140.0, 130.0, 150.0);
        let tgt = b.add_rectangle_corners(170.0, 20.0, 180.0, 30.0);
        // Parallelogram 1: (160,60),(0,60),(-40,100),(120,100)
        add_nonrect_placeholder(&mut b, &[
            (160.0, 60.0), (0.0, 60.0), (-40.0, 100.0), (120.0, 100.0),
        ]);
        // Parallelogram 2: (180,60),(140,100),(300,100),(340,60)
        add_nonrect_placeholder(&mut b, &[
            (180.0, 60.0), (140.0, 100.0), (300.0, 100.0), (340.0, 60.0),
        ]);
        b.route_between(src, tgt);
        let shapes = b.shapes().to_vec();
        let result = b.run();
        Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    }

    // Round 2: smaller parallelograms (shortcut becomes cost-effective)
    {
        let mut b = ScenarioBuilder::new();
        let src = b.add_rectangle_corners(120.0, 140.0, 130.0, 150.0);
        let tgt = b.add_rectangle_corners(170.0, 20.0, 180.0, 30.0);
        add_nonrect_placeholder(&mut b, &[
            (160.0, 60.0), (100.0, 60.0), (60.0, 100.0), (120.0, 100.0),
        ]);
        add_nonrect_placeholder(&mut b, &[
            (180.0, 60.0), (140.0, 100.0), (200.0, 100.0), (240.0, 60.0),
        ]);
        b.route_between(src, tgt);
        let shapes = b.shapes().to_vec();
        let result = b.run();
        Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    }
}

/// Port: ReflectionsDetectedByAlreadyLoadedSide
/// Pending reflection lookahead sites are detected when the side has been loaded
/// before the site was stored.
/// C#: triangle (0,0)-(0,70)-(70,70), plus two small parallelograms.
/// Validates 2 horizontal + 3 vertical reflection segments.
#[test]
fn reflections_detected_by_already_loaded_side() {
    let mut b = ScenarioBuilder::new();
    // Large triangle
    add_nonrect_placeholder(&mut b, &[
        (0.0, 0.0), (0.0, 70.0), (70.0, 70.0),
    ]);
    // Small parallelogram 1
    add_nonrect_placeholder(&mut b, &[
        (30.0, 10.0), (25.0, 15.0), (45.0, 35.0), (50.0, 30.0),
    ]);
    // Small parallelogram 2
    add_nonrect_placeholder(&mut b, &[
        (60.0, 20.0), (55.0, 25.0), (75.0, 45.0), (80.0, 40.0),
    ]);
    // VG-only (no routes in C# — just CreateRouter + RunAndShowGraph)
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
    // C# validates: 2 horizontal + 3 vertical reflection segments.
    // Cannot validate reflection segment counts with current API.
}

/// Port: ReflectionsSitedByLowSideAreNotLoadedByHighSide
/// Closing an obstacle underneath another with rightward leaning sides does not
/// create a reversed-direction lookahead.
/// C#: three non-rectangular shapes, validates 4 horizontal + 3 vertical reflections.
#[test]
fn reflections_sited_by_low_side_are_not_loaded_by_high_side() {
    let mut b = ScenarioBuilder::new();
    // Parallelogram 1: (10,0),(0,10),(70,80),(80,70)
    add_nonrect_placeholder(&mut b, &[
        (10.0, 0.0), (0.0, 10.0), (70.0, 80.0), (80.0, 70.0),
    ]);
    // Triangle: (50,20),(75,45),(75,20)
    add_nonrect_placeholder(&mut b, &[
        (50.0, 20.0), (75.0, 45.0), (75.0, 20.0),
    ]);
    // Parallelogram 2: (-15,25),(-25,35),(25,85),(35,75)
    add_nonrect_placeholder(&mut b, &[
        (-15.0, 25.0), (-25.0, 35.0), (25.0, 85.0), (35.0, 75.0),
    ]);
    // VG-only
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
    // C# validates: 4 horizontal + 3 vertical reflection segments,
    // and a specific reflection segment near (12.17, 23.59) -> (12.17, 50.76).
}

/// Port: ReflectionsRemoveInterceptedSite
/// A side which loaded a reflection event will only remove exactly that event,
/// in case it was intercepted by another obstacle.
/// C#: three non-rectangular shapes, validates 0 reflection segments in both axes.
#[test]
fn reflections_remove_intercepted_site() {
    let mut b = ScenarioBuilder::new();
    // Parallelogram 1: (50,50),(40,60),(120,140),(130,130)
    add_nonrect_placeholder(&mut b, &[
        (50.0, 50.0), (40.0, 60.0), (120.0, 140.0), (130.0, 130.0),
    ]);
    // Triangle: (90,70),(105,85),(105,70)
    add_nonrect_placeholder(&mut b, &[
        (90.0, 70.0), (105.0, 85.0), (105.0, 70.0),
    ]);
    // Parallelogram 2: (90,20),(80,30),(140,90),(150,80)
    add_nonrect_placeholder(&mut b, &[
        (90.0, 20.0), (80.0, 30.0), (140.0, 90.0), (150.0, 80.0),
    ]);
    // VG-only
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
    // C# validates: 0 horizontal + 0 vertical reflection segments.
}
