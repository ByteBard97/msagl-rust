/// Rectilinear routing tests — AlmostFlat MultipleInversion + Transitive ConvexHull
/// Multiple Accretion Becomes Separate Clumps, ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// Naming convention: C# test name → snake_case Rust test name.

#[path = "test_harness/mod.rs"]
mod test_harness;

use msagl_rust::Point;
use test_harness::verifier::RECTILINEAR_TOLERANCE;
use test_harness::{ScenarioBuilder, Verifier};

// ── AlmostFlat_MultipleInversion shared worker ──────────────────────────────
//
// C# source lines 1907–1961.
//
// Geometry (isBottom = true):
//   obs[0]: top "plate" pentagon with almost-flat bottom sides near y=0.
//   obs[1] (mask & 1): left upper inner triangle.
//   obs[2] (mask & 1): right upper inner triangle.
//   obs[1 or 3] (mask & 2): left lower inner triangle.
//   obs[2 or 4] (mask & 2): right lower inner triangle.
//   ...    (mask & 4): left/right lower outer triangles with almost-flat tops.
//
// For isBottom = false every Y coordinate is negated.
//
// C# routes between obstacles[1] and obstacles[2].

/// Shared worker: build the AlmostFlat MultipleInversion scenario.
///
/// Returns `(id_of_obs1, id_of_obs2)` — the two endpoints to route between.
fn almost_flat_multiple_inversion_worker(b: &mut ScenarioBuilder, mask: u32, is_bottom: bool) {
    // Helper: adds a polygon, negating Y when is_bottom == false.
    fn pts(raw: &[(f64, f64)], flip: bool) -> Vec<Point> {
        raw.iter()
            .map(|&(x, y)| Point::new(x, if flip { -y } else { y }))
            .collect()
    }

    let epsilon = 1e-6_f64; // ApproximateComparer.DistanceEpsilon (C# line 1909)
    let flip = !is_bottom;

    // obs[0]: top plate pentagon (C# lines 1912–1918)
    b.add_polygon(&pts(
        &[
            (0.0, 0.0),
            (20.0, epsilon),
            (20.0, 8.0),
            (-20.0, 8.0),
            (-20.0, epsilon),
        ],
        flip,
    ));

    if 0 != (mask & 1) {
        // obs[1]: left upper inner triangle (C# line 1923)
        b.add_polygon(&pts(
            &[(-6.0, -6.0), (-4.0, 6.0), (-8.0, 6.0)],
            flip,
        ));
        // obs[2]: right upper inner triangle (C# line 1924)
        b.add_polygon(&pts(
            &[(6.0, -6.0), (8.0, 6.0), (4.0, 6.0)],
            flip,
        ));
    }

    if 0 != (mask & 2) {
        // left lower inner triangle (C# line 1930)
        b.add_polygon(&pts(
            &[(-4.0, -3.0), (-1.0, -3.0), (-2.5, 3.0)],
            flip,
        ));
        // right lower inner triangle (C# line 1931)
        b.add_polygon(&pts(
            &[(1.0, -3.0), (4.0, -3.0), (2.5, 3.0)],
            flip,
        ));
    }

    if 0 != (mask & 4) {
        // left lower outer triangle with almost-flat top (C# line 1938)
        b.add_polygon(&pts(
            &[
                (-50.0, -10.0),
                (85.0, -2.0 - epsilon),
                (-50.0, -2.0 + epsilon),
            ],
            flip,
        ));
        // right lower outer triangle with almost-flat top (C# line 1939)
        b.add_polygon(&pts(
            &[
                (50.0, -10.0),
                (50.0, -2.0 + epsilon),
                (-85.0, -2.0 - epsilon),
            ],
            flip,
        ));
    }
}

/// Route obstacles[1] → obstacles[2] given the scenario.
/// Returns a (shapes, result) pair suitable for verification.
fn run_almost_flat(mask: u32, is_bottom: bool) {
    let mut b = ScenarioBuilder::new();
    almost_flat_multiple_inversion_worker(&mut b, mask, is_bottom);

    // C# line 1960: CreateRoutingBetweenObstacles(obstacles, 1, 2)
    // obstacles list has obs[0] always; obs[1] and obs[2] exist iff mask != 0.
    // For mask == 0 there is only one obstacle — the plate — and obs[1]/obs[2]
    // don't exist.  In practice the test values 1–7 always set at least bit 0
    // (since 1&1=1, 2&1=0 but 2 means only bit-1 obstacles, etc.).
    //
    // Index mapping: obs[0] = index 0 (plate), then pairs are added per-bit:
    //   bit 0 → indices 1,2   (upper triangles)
    //   bit 1 → appended after that
    //   bit 2 → appended after that
    // C# uses obstacles[1] and obstacles[2], which are the first two obstacles
    // added after obs[0].  For mask=1 those are the upper triangles.
    // For mask=2 (no bit-0 obstacles) obs[1] and obs[2] are the lower triangles.
    // The C# test always passes masks 1–7, so at least one pair always exists.
    let n = b.shapes().len();
    if n >= 3 {
        b.route_between(1, 2);
    }

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── AlmostFlat_MultipleBottomInversion 1–7 ──────────────────────────────────

/// Port: AlmostFlat_MultipleBottomInversion1 (C# line 1798)
#[test]
fn almost_flat_multiple_bottom_inversion1() {
    // C# line 1800: AlmostFlat_MultipleInversion_Worker(1, isBottom: true)
    run_almost_flat(1, true);
}

/// Port: AlmostFlat_MultipleBottomInversion2 (C# line 1806)
#[test]
fn almost_flat_multiple_bottom_inversion2() {
    // C# line 1808: AlmostFlat_MultipleInversion_Worker(2, isBottom: true)
    run_almost_flat(2, true);
}

/// Port: AlmostFlat_MultipleBottomInversion3 (C# line 1814)
#[test]
fn almost_flat_multiple_bottom_inversion3() {
    // C# line 1816: AlmostFlat_MultipleInversion_Worker(3, isBottom: true)
    run_almost_flat(3, true);
}

/// Port: AlmostFlat_MultipleBottomInversion4 (C# line 1822)
#[test]
fn almost_flat_multiple_bottom_inversion4() {
    // C# line 1824: AlmostFlat_MultipleInversion_Worker(4, isBottom: true)
    run_almost_flat(4, true);
}

/// Port: AlmostFlat_MultipleBottomInversion5 (C# line 1830)
#[test]
fn almost_flat_multiple_bottom_inversion5() {
    // C# line 1832: AlmostFlat_MultipleInversion_Worker(5, isBottom: true)
    run_almost_flat(5, true);
}

/// Port: AlmostFlat_MultipleBottomInversion6 (C# line 1838)
#[test]
fn almost_flat_multiple_bottom_inversion6() {
    // C# line 1840: AlmostFlat_MultipleInversion_Worker(6, isBottom: true)
    run_almost_flat(6, true);
}

/// Port: AlmostFlat_MultipleBottomInversion7 (C# line 1846)
#[test]
fn almost_flat_multiple_bottom_inversion7() {
    // C# line 1848: AlmostFlat_MultipleInversion_Worker(7, isBottom: true)
    run_almost_flat(7, true);
}

// ── AlmostFlat_MultipleTopInversion 1–7 ─────────────────────────────────────

/// Port: AlmostFlat_MultipleTopInversion1 (C# line 1854)
#[test]
fn almost_flat_multiple_top_inversion1() {
    // C# line 1856: AlmostFlat_MultipleInversion_Worker(1, isBottom: false)
    run_almost_flat(1, false);
}

/// Port: AlmostFlat_MultipleTopInversion2 (C# line 1862)
#[test]
fn almost_flat_multiple_top_inversion2() {
    // C# line 1864: AlmostFlat_MultipleInversion_Worker(2, isBottom: false)
    run_almost_flat(2, false);
}

/// Port: AlmostFlat_MultipleTopInversion3 (C# line 1870)
#[test]
fn almost_flat_multiple_top_inversion3() {
    // C# line 1872: AlmostFlat_MultipleInversion_Worker(3, isBottom: false)
    run_almost_flat(3, false);
}

/// Port: AlmostFlat_MultipleTopInversion4 (C# line 1878)
#[test]
fn almost_flat_multiple_top_inversion4() {
    // C# line 1880: AlmostFlat_MultipleInversion_Worker(4, isBottom: false)
    run_almost_flat(4, false);
}

/// Port: AlmostFlat_MultipleTopInversion5 (C# line 1886)
#[test]
fn almost_flat_multiple_top_inversion5() {
    // C# line 1888: AlmostFlat_MultipleInversion_Worker(5, isBottom: false)
    run_almost_flat(5, false);
}

/// Port: AlmostFlat_MultipleTopInversion6 (C# line 1894)
#[test]
fn almost_flat_multiple_top_inversion6() {
    // C# line 1896: AlmostFlat_MultipleInversion_Worker(6, isBottom: false)
    run_almost_flat(6, false);
}

/// Port: AlmostFlat_MultipleTopInversion7 (C# line 1902)
#[test]
fn almost_flat_multiple_top_inversion7() {
    // C# line 1904: AlmostFlat_MultipleInversion_Worker(7, isBottom: false)
    run_almost_flat(7, false);
}

// ── Transitive_ConvexHull_Multiple_Accretion_Becomes_Separate_Clumps ─────────
//
// C# source lines 4534–4597.
//
// This test calls Transitive_Obstacles_Multiple_Accretion three times with
// makeRect=true:
//   sub-case A: makeBothHulls=false, blockAllAxis=false, makeRect=true
//   sub-case B: makeBothHulls=true,  blockAllAxis=false, makeRect=true
//   sub-case C: makeBothHulls=true,  blockAllAxis=true,  makeRect=true
//
// `makeRect=true` sets UseObstacleRectangles, which replaces each polygon
// obstacle's boundary with its axis-aligned bounding rectangle before routing.
// Since our harness has no such flag, we represent each polygon obstacle
// directly as the rectangle that `PolylineFromRectanglePoints` creates, and
// represent the non-rectangular PolylineFromPoints obstacles as bounding-box
// rectangles (they are `PolylineFromPoints`, which creates a polyline but the
// makeRect=true flag turns them into their bbox anyway).
//
// Obstacle list (C# lines 4544–4573):
//   obs[0]: rect (30,30)–(70,50)   → add_rectangle_corners(30,30,70,50)
//   obs[1]: rect (140,30)–(160,50) → add_rectangle_corners(140,30,160,50)
//   obs[2]: triangle pts (0,0),(40,0),(40,20) → bbox add_rectangle_corners(0,0,40,20)
//   obs[3]: rect (0,10)–(20,70)    → add_rectangle_corners(0,10,20,70)
//   obs[4]: triangle pts (0,60),(40,60),(40,80) → bbox add_rectangle_corners(0,60,40,80)
//   obs[5]: triangle pts (60,60),(60,80),(100,60) → bbox add_rectangle_corners(60,60,100,80)
//   obs[6]: rect (80,10)–(100,70)  → add_rectangle_corners(80,10,100,70)
//   obs[7]: triangle pts (60,0),(100,0),(60,20) → bbox add_rectangle_corners(60,0,100,20)
// Optional (blockAllAxis=true):
//   obs[8]:  rect (48,16)–(52,20)
//   obs[9]:  rect (48,60)–(52,64)
//
// Route: obs[0] → obs[1] (crosses transitively-overlapped obstacle).
//
// With makeRect=true, C# creates two clumps so it only does visual comparison
// and skips VerifyAllObstaclesInConvexHull.  We simply verify routing completes.

/// Build a sub-case of Transitive_Obstacles_Multiple_Accretion with makeRect=true.
///
/// `make_both_hulls` — when false, removes the extra 4 triangles/rectangles
///   (obstacles 2–5 in the full list) keeping only obs[0..1] + obs[6..7].
/// `block_all_axis`  — when true, adds two small blocking rects.
fn transitive_multiple_accretion_rect_worker(make_both_hulls: bool, block_all_axis: bool) {
    let mut b = ScenarioBuilder::new();

    // obs[0]: rect (30,30)→(70,50)  (C# line 4545)
    let id0 = b.add_rectangle_corners(30.0, 30.0, 70.0, 50.0);
    // obs[1]: rect (140,30)→(160,50) (C# line 4546)
    let id1 = b.add_rectangle_corners(140.0, 30.0, 160.0, 50.0);

    if make_both_hulls {
        // Triangle (0,20),(40,20),(40,0) — makeRect=true uses bbox (C# line 4547)
        b.add_rectangle_corners(0.0, 0.0, 40.0, 20.0);
        // rect (0,10)→(20,70) (C# line 4553)
        b.add_rectangle_corners(0.0, 10.0, 20.0, 70.0);
        // Triangle (0,60),(40,80),(40,60) — bbox (C# line 4554)
        b.add_rectangle_corners(0.0, 60.0, 40.0, 80.0);
        // Triangle (60,60),(60,80),(100,60) — bbox (C# line 4560)
        b.add_rectangle_corners(60.0, 60.0, 100.0, 80.0);
    }

    // rect (80,10)→(100,70) (C# line 4566)
    b.add_rectangle_corners(80.0, 10.0, 100.0, 70.0);
    // Triangle (60,20),(100,20),(60,0) — bbox (C# line 4567)
    b.add_rectangle_corners(60.0, 0.0, 100.0, 20.0);

    if block_all_axis {
        // rect (48,16)→(52,20) (C# line 4581)
        b.add_rectangle_corners(48.0, 16.0, 52.0, 20.0);
        // rect (48,60)→(52,64) (C# line 4582)
        b.add_rectangle_corners(48.0, 60.0, 52.0, 64.0);
    }

    // C# line 4588: routings from obs[0] to obs[1]
    b.route_between(id0, id1);

    let shapes = b.shapes().to_vec();
    let result = b.run();
    // makeRect=true: C# skips convex-hull verification; we verify basic routing only.
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Transitive_ConvexHull_Multiple_Accretion_Becomes_Separate_Clumps_With_Rectilinear_Shapes
/// (C# line 4534)
///
/// Runs three sub-cases with makeRect=true.
#[test]
fn transitive_convex_hull_multiple_accretion_becomes_separate_clumps_with_rectilinear_shapes() {
    // C# line 4536: (makeBothHulls=false, blockAllAxis=false, makeRect=true)
    transitive_multiple_accretion_rect_worker(false, false);
    // C# line 4537: (makeBothHulls=true,  blockAllAxis=false, makeRect=true)
    transitive_multiple_accretion_rect_worker(true, false);
    // C# line 4538: (makeBothHulls=true,  blockAllAxis=true,  makeRect=true)
    transitive_multiple_accretion_rect_worker(true, true);
}
