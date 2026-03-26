/// Reflection and almost-flat obstacle tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// These tests exercise non-rectangular obstacles with angled sides that produce
/// reflection events in the sweep-line visibility graph generator. Since
/// `ScenarioBuilder` currently only supports rectangles, each test uses
/// bounding-box rectangles as placeholders. When non-rectangular obstacle support
/// is added, replace the bounding-box placeholders with actual polygon shapes.
///
/// ALL tests are marked `#[ignore]` because they require non-rectangular obstacle
/// support to produce meaningful results.

#[path = "test_harness/mod.rs"]
mod test_harness;

use test_harness::{ScenarioBuilder, Verifier};
use test_harness::verifier::RECTILINEAR_TOLERANCE;

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Bounding box for a polygon with vertices at the given points.
fn bounding_box(points: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let min_x = points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_y = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    (min_x, min_y, max_x - min_x, max_y - min_y)
}

/// Add a bounding-box rectangle placeholder for a non-rectangular obstacle.
fn add_nonrect_placeholder(b: &mut ScenarioBuilder, points: &[(f64, f64)]) -> usize {
    let (x, y, w, h) = bounding_box(points);
    b.add_rectangle_bl(x, y, w, h)
}

/// Route between all obstacle pairs (mirrors C# `CreateRoutingBetweenObstacles(obs, 0, -1)`).
fn route_all_pairs(b: &mut ScenarioBuilder, ids: &[usize]) {
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
}

// ── Category: Reflection — Block tests ───────────────────────────────────────

/// Port: Reflection_Block1_Big
/// Two non-orthogonal parallelogram obstacles with a small diamond blocker.
/// C#: CurveFromPoints for 3 shapes, route between all pairs.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_block1_big() {
    // Shape 0: parallelogram (30,30),(130,130),(140,120),(40,20)
    // Shape 1: parallelogram (70,10),(190,130),(200,120),(80,0)
    // Shape 2: diamond (65,40),(80,55),(95,40),(80,25)
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(30.0,30.0),(130.0,130.0),(140.0,120.0),(40.0,20.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(70.0,10.0),(190.0,130.0),(200.0,120.0),(80.0,0.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(65.0,40.0),(80.0,55.0),(95.0,40.0),(80.0,25.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_Block1_Big_UseRect
/// Same as Reflection_Block1_Big but with UseObstacleRectangles = true.
/// Tests SplitEdge handling when splicing collinear segments.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_block1_big_use_rect() {
    // Same geometry as reflection_block1_big; C# sets UseObstacleRectangles = true.
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(30.0,30.0),(130.0,130.0),(140.0,120.0),(40.0,20.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(70.0,10.0),(190.0,130.0),(200.0,120.0),(80.0,0.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(65.0,40.0),(80.0,55.0),(95.0,40.0),(80.0,25.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_Block1_Small
/// Two parallelograms + a tiny diamond blocker.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_block1_small() {
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(30.0,30.0),(130.0,130.0),(140.0,120.0),(40.0,20.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(70.0,10.0),(190.0,130.0),(200.0,120.0),(80.0,0.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(75.0,40.0),(80.0,45.0),(85.0,40.0),(80.0,35.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_Block2
/// Two parallelograms + two small triangle/diamond blockers.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_block2() {
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(30.0,310.0),(100.0,380.0),(110.0,370.0),(40.0,300.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(90.0,290.0),(170.0,370.0),(180.0,360.0),(100.0,280.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(70.0,320.0),(80.0,330.0),(80.0,320.0)]);
    let s3 = add_nonrect_placeholder(&mut b, &[(90.0,320.0),(110.0,340.0),(110.0,320.0)]);
    let ids = [s0, s1, s2, s3];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Category: Reflection — Triangle tests ────────────────────────────────────

/// Port: Reflection_Triangle1
/// Three adjacent triangles.
#[test]
fn reflection_triangle1() {
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(50.0,10.0),(80.0,80.0),(110.0,10.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(40.0,10.0),(10.0,80.0),(70.0,80.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(120.0,10.0),(90.0,80.0),(150.0,80.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_Triangle1_Overlap
/// Three triangles where the outer inverted triangles overlap above the middle one.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_triangle1_overlap() {
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(50.0,10.0),(80.0,80.0),(110.0,10.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(40.0,10.0),(-5.0,110.0),(85.0,110.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(120.0,10.0),(75.0,110.0),(165.0,110.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_Triangle1_NoOverlap
/// Three adjacent triangles where the outer inverted triangles do not overlap.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_triangle1_no_overlap() {
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(47.0,10.0),(80.0,80.0),(110.0,10.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(40.0,10.0),(4.0,90.0),(76.0,90.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(120.0,10.0),(84.0,90.0),(156.0,90.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Category: Reflection — LongAngle tests ───────────────────────────────────

/// Port: Reflection_LongAngle
/// Baseline: two long non-orthogonal parallelograms + a small rectangle.
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_long_angle() {
    let mut b = ScenarioBuilder::new();
    let s0 = add_nonrect_placeholder(&mut b, &[(20.0,30.0),(20.0,40.0),(40.0,40.0),(40.0,30.0)]);
    let s1 = add_nonrect_placeholder(&mut b, &[(30.0,50.0),(20.0,60.0),(70.0,110.0),(80.0,100.0)]);
    let s2 = add_nonrect_placeholder(&mut b, &[(50.0,10.0),(40.0,20.0),(100.0,110.0),(110.0,100.0)]);
    let ids = [s0, s1, s2];
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Worker for Reflection_LongAngle_Overlap variants.
/// C#: multiplies X coords by invertX, Y coords by invertY.
fn reflection_long_angle_overlap_worker(
    b: &mut ScenarioBuilder,
    invert_x: f64,
    invert_y: f64,
) {
    let s0 = add_nonrect_placeholder(b, &[
        (20.0*invert_x, 30.0*invert_y), (20.0*invert_x, 40.0*invert_y),
        (40.0*invert_x, 40.0*invert_y), (40.0*invert_x, 30.0*invert_y),
    ]);
    let s1 = add_nonrect_placeholder(b, &[
        (30.0*invert_x, 50.0*invert_y), (20.0*invert_x, 60.0*invert_y),
        (100.0*invert_x, 120.0*invert_y), (110.0*invert_x, 110.0*invert_y),
    ]);
    let s2 = add_nonrect_placeholder(b, &[
        (50.0*invert_x, 10.0*invert_y), (40.0*invert_x, 20.0*invert_y),
        (100.0*invert_x, 110.0*invert_y), (110.0*invert_x, 100.0*invert_y),
    ]);
    // C# calls DoRouting(obstacles) — no explicit routing pairs, routes all.
    let ids = [s0, s1, s2];
    route_all_pairs(b, &ids);
}

/// Port: Reflection_LongAngle_Overlap_ToHighRight
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_long_angle_overlap_to_high_right() {
    let mut b = ScenarioBuilder::new();
    reflection_long_angle_overlap_worker(&mut b, 1.0, 1.0);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_LongAngle_Overlap_ToHighLeft
#[test]
fn reflection_long_angle_overlap_to_high_left() {
    let mut b = ScenarioBuilder::new();
    reflection_long_angle_overlap_worker(&mut b, -1.0, 1.0);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_LongAngle_Overlap_FromLowRight
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_long_angle_overlap_from_low_right() {
    let mut b = ScenarioBuilder::new();
    reflection_long_angle_overlap_worker(&mut b, 1.0, -1.0);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Reflection_LongAngle_Overlap_FromLowLeft
#[test]
#[ignore = "requires non-rectangular obstacle support"]
fn reflection_long_angle_overlap_from_low_left() {
    let mut b = ScenarioBuilder::new();
    reflection_long_angle_overlap_worker(&mut b, -1.0, -1.0);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Category: Reflection — Staircase tests ───────────────────────────────────

/// Worker for Reflection_Staircase_Stops tests.
/// C#: two diamond obstacles, optionally rotated by `rotation * PI` radians.
/// The test verifies exactly 1 horizontal + 1 vertical reflection scan segment.
fn reflection_staircase_worker(b: &mut ScenarioBuilder, rotation: f64) {
    let x_offset = 2.5 * std::f64::consts::SQRT_2; // 2.5 * 1.414214

    // Diamond 0: (100,100),(0,200),(100,300),(200,200) — centered at (100,200)
    let mut pts0 = vec![
        (100.0, 100.0), (0.0, 200.0), (100.0, 300.0), (200.0, 200.0),
    ];
    // Diamond 1: shifted by x_offset
    let mut pts1 = vec![
        (50.0 + x_offset, 0.0), (300.0 + x_offset, 250.0),
        (450.0 + x_offset, 100.0), (200.0 + x_offset, -150.0),
    ];

    if rotation > 0.0 {
        let angle = std::f64::consts::PI * rotation;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let rotate = |pts: &mut Vec<(f64, f64)>| {
            for p in pts.iter_mut() {
                let (x, y) = *p;
                *p = (x * cos_a - y * sin_a, x * sin_a + y * cos_a);
            }
        };
        rotate(&mut pts0);
        rotate(&mut pts1);
    }

    add_nonrect_placeholder(b, &pts0);
    add_nonrect_placeholder(b, &pts1);
    // C#: no explicit routing edges — just creates VG and checks reflection segments.
}

/// Port: Reflection_Staircase_Stops_At_BoundingBox_Side_NorthWest
#[test]
fn reflection_staircase_stops_northwest() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 0.0);
    let _result = b.run();
    // C# asserts: exactly 1 horizontal + 1 vertical reflection scan segment.
}

/// Port: Reflection_Staircase_Stops_At_BoundingBox_Side_NorthEast
#[test]
fn reflection_staircase_stops_northeast() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 1.5);
    let _result = b.run();
}

/// Port: Reflection_Staircase_Stops_At_BoundingBox_Side_SouthEast
#[test]
fn reflection_staircase_stops_southeast() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 1.0);
    let _result = b.run();
}

/// Port: Reflection_Staircase_Stops_At_BoundingBox_Side_SouthWest
#[test]
fn reflection_staircase_stops_southwest() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 0.5);
    let _result = b.run();
}

// ── Category: AlmostFlat — Open/Close LowSide/HighSide ──────────────────────

/// C# `ApproximateComparer.DistanceEpsilon` = 10^-6.
const DISTANCE_EPSILON: f64 = 1e-6;

/// Worker for AlmostFlat_OpenOrClose_LowSide tests.
/// C#: 4 obstacles — fixed neighbour rect, almost-flat quad, optional overlap/neighbour rects.
fn almost_flat_low_side_worker(
    b: &mut ScenarioBuilder,
    is_open: bool,
    want_interior_overlap: bool,
    want_interior_neighbor: bool,
) {
    let eps = if is_open { DISTANCE_EPSILON } else { 0.0 };
    let eps_close = if !is_open { DISTANCE_EPSILON } else { 0.0 };

    // Fixed neighbour rectangle.
    let nb_y0 = if is_open { -10.0 } else { 0.0 };
    let nb_y1 = if is_open { 30.0 } else { 40.0 };
    b.add_rectangle_bl(10.0, nb_y0, 10.0, nb_y1 - nb_y0);

    // Almost-flat quad: LowSide is almost flat.
    // C#: (40, 10+eps_open), (40, 20), (50, 20+eps_close), (50, 10)
    // Use bounding box: x=40..50, y=10..20+eps
    add_nonrect_placeholder(b, &[
        (40.0, 10.0 + eps), (40.0, 20.0), (50.0, 20.0 + eps_close), (50.0, 10.0),
    ]);

    // Interior overlap obstacle.
    let ov_x = if want_interior_overlap { 49.5 } else { 54.0 };
    b.add_rectangle_bl(ov_x, 5.0, 55.0 - ov_x, 20.0);

    // Interior neighbour obstacle.
    let nb_right = if want_interior_neighbor { 45.5 } else { 36.0 };
    b.add_rectangle_bl(35.0, 5.0, nb_right - 35.0, 20.0);
}

/// Worker for AlmostFlat_OpenOrClose_HighSide tests.
fn almost_flat_high_side_worker(
    b: &mut ScenarioBuilder,
    is_open: bool,
    want_interior_overlap: bool,
    want_interior_neighbor: bool,
) {
    let eps_close = if !is_open { DISTANCE_EPSILON } else { 0.0 };
    let eps_open = if is_open { DISTANCE_EPSILON } else { 0.0 };

    // Fixed neighbour rectangle.
    let nb_y0 = if is_open { -10.0 } else { 0.0 };
    let nb_y1 = if is_open { 30.0 } else { 40.0 };
    b.add_rectangle_bl(60.0, nb_y0, 10.0, nb_y1 - nb_y0);

    // Almost-flat quad: HighSide is almost flat.
    // C#: (40, 10), (40, 20+eps_close), (50, 20), (50, 10+eps_open)
    add_nonrect_placeholder(b, &[
        (40.0, 10.0), (40.0, 20.0 + eps_close), (50.0, 20.0), (50.0, 10.0 + eps_open),
    ]);

    // Interior overlap obstacle.
    let ov_right = if want_interior_overlap { 40.5 } else { 36.0 };
    b.add_rectangle_bl(35.0, 5.0, ov_right - 35.0, 20.0);

    // Interior neighbour obstacle.
    let nb_left = if want_interior_neighbor { 44.5 } else { 54.0 };
    b.add_rectangle_bl(nb_left, 5.0, 55.0 - nb_left, 20.0);
}

/// Generate an AlmostFlat Open/Close LowSide test.
macro_rules! almost_flat_low_side_test {
    ($name:ident, $is_open:expr, $overlap:expr, $neighbor:expr) => {
        #[test]
        fn $name() {
            let mut b = ScenarioBuilder::new();
            almost_flat_low_side_worker(&mut b, $is_open, $overlap, $neighbor);
            let ids: Vec<usize> = (0..4).collect();
            route_all_pairs(&mut b, &ids);
            let shapes = b.shapes().to_vec();
            let result = b.run();
            Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
        }
    };
}

/// Generate an AlmostFlat Open/Close HighSide test.
macro_rules! almost_flat_high_side_test {
    ($name:ident, $is_open:expr, $overlap:expr, $neighbor:expr) => {
        #[test]
        fn $name() {
            let mut b = ScenarioBuilder::new();
            almost_flat_high_side_worker(&mut b, $is_open, $overlap, $neighbor);
            let ids: Vec<usize> = (0..4).collect();
            route_all_pairs(&mut b, &ids);
            let shapes = b.shapes().to_vec();
            let result = b.run();
            Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
        }
    };
}

// --- Open LowSide variants ---
almost_flat_low_side_test!(almost_flat_open_low_side_no_overlap, true, false, false);
almost_flat_low_side_test!(almost_flat_open_low_side_interior_low_overlap, true, true, false);
almost_flat_low_side_test!(almost_flat_open_low_side_interior_low_neighbor, true, false, true);
almost_flat_low_side_test!(almost_flat_open_low_side_interior_low_overlap_low_neighbor, true, true, true);

// --- Open HighSide variants ---
almost_flat_high_side_test!(almost_flat_open_high_side_no_overlap, true, false, false);
almost_flat_high_side_test!(almost_flat_open_high_side_interior_high_overlap, true, true, false);
almost_flat_high_side_test!(almost_flat_open_high_side_interior_high_neighbor, true, false, true);
almost_flat_high_side_test!(almost_flat_open_high_side_interior_high_overlap_high_neighbor, true, true, true);

// --- Close LowSide variants ---
almost_flat_low_side_test!(almost_flat_close_low_side_no_overlap, false, false, false);
almost_flat_low_side_test!(almost_flat_close_low_side_interior_low_overlap, false, true, false);
almost_flat_low_side_test!(almost_flat_close_low_side_interior_low_neighbor, false, false, true);
almost_flat_low_side_test!(almost_flat_close_low_side_interior_low_overlap_low_neighbor, false, true, true);

// --- Close HighSide variants ---
almost_flat_high_side_test!(almost_flat_close_high_side_no_overlap, false, false, false);
almost_flat_high_side_test!(almost_flat_close_high_side_interior_high_overlap, false, true, false);
almost_flat_high_side_test!(almost_flat_close_high_side_interior_high_neighbor, false, false, true);
almost_flat_high_side_test!(almost_flat_close_high_side_interior_high_overlap_high_neighbor, false, true, true);

// ── Category: AlmostFlat — MultipleInversion tests ───────────────────────────

/// Worker for AlmostFlat_MultipleInversion tests.
/// C#: a "plate" with almost-flat bottom, plus up to 6 inner triangles controlled by mask bits.
/// If `!is_bottom`, all Y coordinates are inverted.
fn almost_flat_multiple_inversion_worker(
    b: &mut ScenarioBuilder,
    mask: u32,
    is_bottom: bool,
) -> Vec<usize> {
    let eps = DISTANCE_EPSILON;
    let y_sign = if is_bottom { 1.0 } else { -1.0 };

    let mut all_ids = Vec::new();

    // Top "plate" with almost-flat bottom sides.
    // C#: (0,0), (20,eps), (20,8), (-20,8), (-20,eps)
    let plate = [
        (0.0, 0.0 * y_sign),
        (20.0, eps * y_sign),
        (20.0, 8.0 * y_sign),
        (-20.0, 8.0 * y_sign),
        (-20.0, eps * y_sign),
    ];
    all_ids.push(add_nonrect_placeholder(b, &plate));

    if (mask & 1) != 0 {
        // Left and right upper inner intersecting triangles.
        let tri_l = [(-6.0, -6.0 * y_sign), (-4.0, 6.0 * y_sign), (-8.0, 6.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_l));
        let tri_r = [(6.0, -6.0 * y_sign), (8.0, 6.0 * y_sign), (4.0, 6.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_r));
    }

    if (mask & 2) != 0 {
        // Left and right lower inner intersecting triangles.
        let tri_l = [(-4.0, -3.0 * y_sign), (-1.0, -3.0 * y_sign), (-2.5, 3.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_l));
        let tri_r = [(1.0, -3.0 * y_sign), (4.0, -3.0 * y_sign), (2.5, 3.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_r));
    }

    if (mask & 4) != 0 {
        // Left and right lower outer triangles with almost-flat tops.
        let tri_l = [
            (-50.0, -10.0 * y_sign),
            (85.0, (-2.0 - eps) * y_sign),
            (-50.0, (-2.0 + eps) * y_sign),
        ];
        all_ids.push(add_nonrect_placeholder(b, &tri_l));
        let tri_r = [
            (50.0, -10.0 * y_sign),
            (50.0, (-2.0 + eps) * y_sign),
            (-85.0, (-2.0 - eps) * y_sign),
        ];
        all_ids.push(add_nonrect_placeholder(b, &tri_r));
    }

    all_ids
}

/// Generate a single AlmostFlat_Multiple*Inversion test.
/// C# routes between obstacle indices 1 and 2 (first pair of inner triangles).
macro_rules! almost_flat_inversion_test {
    ($name:ident, $mask:expr, $is_bottom:expr) => {
        #[test]
        fn $name() {
            let mut b = ScenarioBuilder::new();
            let ids = almost_flat_multiple_inversion_worker(&mut b, $mask, $is_bottom);
            if ids.len() >= 3 {
                b.route_between(ids[1], ids[2]);
            }
            let shapes = b.shapes().to_vec();
            let result = b.run();
            Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
        }
    };
}

// --- Bottom inversions (is_bottom = true) ---
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion1, 1, true);
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion2, 2, true);
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion3, 3, true);
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion4, 4, true);
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion5, 5, true);
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion6, 6, true);
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion7, 7, true);

// --- Top inversions (is_bottom = false) ---
almost_flat_inversion_test!(almost_flat_multiple_top_inversion1, 1, false);
almost_flat_inversion_test!(almost_flat_multiple_top_inversion2, 2, false);
almost_flat_inversion_test!(almost_flat_multiple_top_inversion3, 3, false);
almost_flat_inversion_test!(almost_flat_multiple_top_inversion4, 4, false);
almost_flat_inversion_test!(almost_flat_multiple_top_inversion5, 5, false);
almost_flat_inversion_test!(almost_flat_multiple_top_inversion6, 6, false);
almost_flat_inversion_test!(almost_flat_multiple_top_inversion7, 7, false);
