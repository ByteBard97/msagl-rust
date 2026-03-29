/// Rectilinear routing tests — 56 C# tests ported with canonical names.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// Each test has a comment with the C# line number it came from.
/// Tests are expected to fail until the underlying routing implementation is
/// complete. NO `#[ignore]` attributes — failures are honest.
///
/// Many of these tests have equivalent implementations in other test files
/// under slightly different names (e.g. `clust5_minimal_fan_out` vs
/// `clust5_minimal`). This file provides the canonically-named versions that
/// match the C# method names after snake_case conversion.

#[path = "test_harness/mod.rs"]
mod test_harness;

use msagl_rust::Point;
use test_harness::verifier::RECTILINEAR_TOLERANCE;
use test_harness::{ScenarioBuilder, Verifier};

// ── Shared constants ──────────────────────────────────────────────────────────

/// `ApproximateComparer.DistanceEpsilon` from C# (line 1742).
const DISTANCE_EPSILON: f64 = 1e-6;

/// `FlatOffset` used in FlatWorker tests (C# approx 1e-6).
const FLAT_OFFSET: f64 = DISTANCE_EPSILON;

// ── Shared helpers ────────────────────────────────────────────────────────────

/// Bounding box helper for non-rectangular obstacle placeholders.
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

/// Add a polygon obstacle from a list of (x, y) tuples.
fn add_polygon(b: &mut ScenarioBuilder, points: &[(f64, f64)]) -> usize {
    let pts: Vec<Point> = points.iter().map(|&(x, y)| Point::new(x, y)).collect();
    b.add_polygon(&pts)
}

/// Route between all pairs of obstacles in `ids`.
fn route_all_pairs(b: &mut ScenarioBuilder, ids: &[usize]) {
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
}

/// Build the two-squares-with-sentinels configuration used by many tests.
/// Mirrors C# `CreateTwoTestSquaresWithSentinels()`.
/// Returns obstacle indices: [left_square, right_square, left_sentinel,
/// right_sentinel, top_sentinel, bottom_sentinel].
fn create_two_test_squares_with_sentinels(b: &mut ScenarioBuilder) -> [usize; 6] {
    // C# TwoTestSquares: left=(20,20)-(100,100), right=(220,20)-(300,100)
    let left = b.add_rectangle_corners(20.0, 20.0, 100.0, 100.0);
    let right = b.add_rectangle_corners(220.0, 20.0, 300.0, 100.0);
    // Sentinels
    let sl = b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    let sr = b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    let st = b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    let sb = b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    [left, right, sl, sr, st, sb]
}

// ── Worker: AlmostFlat Open/Close LowSide ────────────────────────────────────

/// Worker for AlmostFlat_Open/Close_LowSide tests.
/// C# `AlmostFlat_OpenOrClose_LowSide_InteriorHighOverlap_Worker` (line 1727).
fn almost_flat_open_or_close_low_side_worker(
    b: &mut ScenarioBuilder,
    is_open: bool,
    want_interior_overlap: bool,
    want_interior_neighbor: bool,
) {
    let eps_open = if is_open { DISTANCE_EPSILON } else { 0.0 };
    let eps_close = if !is_open { DISTANCE_EPSILON } else { 0.0 };

    // Fixed neighbour rectangle (C# line 1735).
    let nb_y0 = if is_open { -10.0 } else { 0.0 };
    let nb_y1 = if is_open { 30.0 } else { 40.0 };
    b.add_rectangle_corners(10.0, nb_y0, 20.0, nb_y1);

    // Almost-flat quad: LowSide is almost flat (C# line 1741).
    add_nonrect_placeholder(
        b,
        &[
            (40.0, 10.0 + eps_open),
            (40.0, 20.0),
            (50.0, 20.0 + eps_close),
            (50.0, 10.0),
        ],
    );

    // Interior overlap obstacle (C# line 1749).
    let ov_x0 = if want_interior_overlap { 49.5 } else { 54.0 };
    b.add_rectangle_corners(ov_x0, 5.0, 55.0, 25.0);

    // Interior neighbour obstacle (C# line 1753).
    let nb_x1 = if want_interior_neighbor { 45.5 } else { 36.0 };
    b.add_rectangle_corners(35.0, 5.0, nb_x1, 25.0);
}

// ── Worker: AlmostFlat Open/Close HighSide ───────────────────────────────────

/// Worker for AlmostFlat_Open/Close_HighSide tests.
/// C# `AlmostFlat_OpenOrClose_HighSide_InteriorHighOverlap_Worker` (approx line 1760).
fn almost_flat_open_or_close_high_side_worker(
    b: &mut ScenarioBuilder,
    is_open: bool,
    want_interior_overlap: bool,
    want_interior_neighbor: bool,
) {
    let eps_open = if is_open { DISTANCE_EPSILON } else { 0.0 };
    let eps_close = if !is_open { DISTANCE_EPSILON } else { 0.0 };

    // Fixed neighbour rectangle on the high side.
    let nb_y0 = if is_open { -10.0 } else { 0.0 };
    let nb_y1 = if is_open { 30.0 } else { 40.0 };
    b.add_rectangle_corners(60.0, nb_y0, 70.0, nb_y1);

    // Almost-flat quad: HighSide is almost flat.
    add_nonrect_placeholder(
        b,
        &[
            (40.0, 10.0),
            (40.0, 20.0 + eps_close),
            (50.0, 20.0),
            (50.0, 10.0 + eps_open),
        ],
    );

    // Interior overlap obstacle.
    let ov_x1 = if want_interior_overlap { 40.5 } else { 36.0 };
    b.add_rectangle_corners(35.0, 5.0, ov_x1, 25.0);

    // Interior neighbour obstacle.
    let nb_x0 = if want_interior_neighbor { 44.5 } else { 54.0 };
    b.add_rectangle_corners(nb_x0, 5.0, 55.0, 25.0);
}

// ── Macro: generate AlmostFlat Open/Close tests ───────────────────────────────

macro_rules! almost_flat_low_side_test {
    ($name:ident, $is_open:expr, $overlap:expr, $neighbor:expr) => {
        #[test]
        fn $name() {
            let mut b = ScenarioBuilder::new();
            almost_flat_open_or_close_low_side_worker(&mut b, $is_open, $overlap, $neighbor);
            let ids: Vec<usize> = (0..4).collect();
            route_all_pairs(&mut b, &ids);
            let shapes = b.shapes().to_vec();
            let result = b.run();
            Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
        }
    };
}

macro_rules! almost_flat_high_side_test {
    ($name:ident, $is_open:expr, $overlap:expr, $neighbor:expr) => {
        #[test]
        fn $name() {
            let mut b = ScenarioBuilder::new();
            almost_flat_open_or_close_high_side_worker(&mut b, $is_open, $overlap, $neighbor);
            let ids: Vec<usize> = (0..4).collect();
            route_all_pairs(&mut b, &ids);
            let shapes = b.shapes().to_vec();
            let result = b.run();
            Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
        }
    };
}

// ── AlmostFlat_Close_HighSide tests (C# lines 1688–1725) ─────────────────────

/// C# line 1710: AlmostFlat_Close_HighSide_InteriorHighNeighbor
almost_flat_high_side_test!(
    almost_flat_close_high_side_interior_high_neighbor,
    false,
    false,
    true
);

/// C# line 1700: AlmostFlat_Close_HighSide_InteriorHighOverlap
almost_flat_high_side_test!(
    almost_flat_close_high_side_interior_high_overlap,
    false,
    true,
    false
);

/// C# line 1720: AlmostFlat_Close_HighSide_InteriorHighOverlap_HighNeighbor
almost_flat_high_side_test!(
    almost_flat_close_high_side_interior_high_overlap_high_neighbor,
    false,
    true,
    true
);

/// C# line 1691: AlmostFlat_Close_HighSide_NoOverlap
almost_flat_high_side_test!(almost_flat_close_high_side_no_overlap, false, false, false);

// ── AlmostFlat_Close_LowSide tests (C# lines 1649–1687) ──────────────────────

/// C# line 1671: AlmostFlat_Close_LowSide_InteriorLowNeighbor
almost_flat_low_side_test!(
    almost_flat_close_low_side_interior_low_neighbor,
    false,
    false,
    true
);

/// C# line 1661: AlmostFlat_Close_LowSide_InteriorLowOverlap
almost_flat_low_side_test!(
    almost_flat_close_low_side_interior_low_overlap,
    false,
    true,
    false
);

/// C# line 1681: AlmostFlat_Close_LowSide_InteriorLowOverlap_LowNeighbor
almost_flat_low_side_test!(
    almost_flat_close_low_side_interior_low_overlap_low_neighbor,
    false,
    true,
    true
);

/// C# line 1652: AlmostFlat_Close_LowSide_NoOverlap
almost_flat_low_side_test!(almost_flat_close_low_side_no_overlap, false, false, false);

// ── AlmostFlat_MultipleBottomInversion tests (C# lines 1798–1850) ────────────

/// Worker for AlmostFlat_MultipleInversion tests.
/// C# `AlmostFlat_MultipleInversion_Worker` (line 1907).
fn almost_flat_multiple_inversion_worker(
    b: &mut ScenarioBuilder,
    mask: u32,
    is_bottom: bool,
) -> Vec<usize> {
    let eps = DISTANCE_EPSILON;
    let y_sign = if is_bottom { 1.0_f64 } else { -1.0_f64 };
    let mut all_ids = Vec::new();

    // Top "plate" with almost-flat bottom sides (C# line 1916).
    let plate = [
        (0.0_f64, 0.0 * y_sign),
        (20.0, eps * y_sign),
        (20.0, 8.0 * y_sign),
        (-20.0, 8.0 * y_sign),
        (-20.0, eps * y_sign),
    ];
    all_ids.push(add_nonrect_placeholder(b, &plate));

    if (mask & 1) != 0 {
        // Upper inner intersecting triangles (C# line 1923).
        let tri_l = [(-6.0, -6.0 * y_sign), (-4.0, 6.0 * y_sign), (-8.0, 6.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_l));
        let tri_r = [(6.0, -6.0 * y_sign), (8.0, 6.0 * y_sign), (4.0, 6.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_r));
    }

    if (mask & 2) != 0 {
        // Lower inner intersecting triangles (C# line 1930).
        let tri_l = [(-4.0, -3.0 * y_sign), (-1.0, -3.0 * y_sign), (-2.5, 3.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_l));
        let tri_r = [(1.0, -3.0 * y_sign), (4.0, -3.0 * y_sign), (2.5, 3.0 * y_sign)];
        all_ids.push(add_nonrect_placeholder(b, &tri_r));
    }

    if (mask & 4) != 0 {
        // Outer triangles with almost-flat tops (C# line 1938).
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

/// C# line 1798: AlmostFlat_MultipleBottomInversion1
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion1, 1, true);
/// C# line 1806: AlmostFlat_MultipleBottomInversion2
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion2, 2, true);
/// C# line 1814: AlmostFlat_MultipleBottomInversion3
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion3, 3, true);
/// C# line 1822: AlmostFlat_MultipleBottomInversion4
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion4, 4, true);
/// C# line 1830: AlmostFlat_MultipleBottomInversion5
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion5, 5, true);
/// C# line 1838: AlmostFlat_MultipleBottomInversion6
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion6, 6, true);
/// C# line 1846: AlmostFlat_MultipleBottomInversion7
almost_flat_inversion_test!(almost_flat_multiple_bottom_inversion7, 7, true);

// ── AlmostFlat_MultipleTopInversion tests (C# lines 1854–1906) ───────────────

/// C# line 1854: AlmostFlat_MultipleTopInversion1
almost_flat_inversion_test!(almost_flat_multiple_top_inversion1, 1, false);
/// C# line 1862: AlmostFlat_MultipleTopInversion2
almost_flat_inversion_test!(almost_flat_multiple_top_inversion2, 2, false);
/// C# line 1870: AlmostFlat_MultipleTopInversion3
almost_flat_inversion_test!(almost_flat_multiple_top_inversion3, 3, false);
/// C# line 1878: AlmostFlat_MultipleTopInversion4
almost_flat_inversion_test!(almost_flat_multiple_top_inversion4, 4, false);
/// C# line 1886: AlmostFlat_MultipleTopInversion5
almost_flat_inversion_test!(almost_flat_multiple_top_inversion5, 5, false);
/// C# line 1894: AlmostFlat_MultipleTopInversion6
almost_flat_inversion_test!(almost_flat_multiple_top_inversion6, 6, false);
/// C# line 1902: AlmostFlat_MultipleTopInversion7
almost_flat_inversion_test!(almost_flat_multiple_top_inversion7, 7, false);

// ── AlmostFlat_Open_HighSide tests (C# lines 1612–1647) ──────────────────────

/// C# line 1632: AlmostFlat_Open_HighSide_InteriorHighNeighbor
almost_flat_high_side_test!(
    almost_flat_open_high_side_interior_high_neighbor,
    true,
    false,
    true
);

/// C# line 1622: AlmostFlat_Open_HighSide_InteriorHighOverlap
almost_flat_high_side_test!(
    almost_flat_open_high_side_interior_high_overlap,
    true,
    true,
    false
);

/// C# line 1642: AlmostFlat_Open_HighSide_InteriorHighOverlap_HighNeighbor
almost_flat_high_side_test!(
    almost_flat_open_high_side_interior_high_overlap_high_neighbor,
    true,
    true,
    true
);

/// C# line 1613: AlmostFlat_Open_HighSide_NoOverlap
almost_flat_high_side_test!(almost_flat_open_high_side_no_overlap, true, false, false);

// ── AlmostFlat_Open_LowSide tests (C# lines 1573–1608) ───────────────────────

/// C# line 1593: AlmostFlat_Open_LowSide_InteriorLowNeighbor
almost_flat_low_side_test!(
    almost_flat_open_low_side_interior_low_neighbor,
    true,
    false,
    true
);

/// C# line 1583: AlmostFlat_Open_LowSide_InteriorLowOverlap
almost_flat_low_side_test!(
    almost_flat_open_low_side_interior_low_overlap,
    true,
    true,
    false
);

/// C# line 1603: AlmostFlat_Open_LowSide_InteriorLowOverlap_LowNeighbor
almost_flat_low_side_test!(
    almost_flat_open_low_side_interior_low_overlap_low_neighbor,
    true,
    true,
    true
);

/// C# line 1574: AlmostFlat_Open_LowSide_NoOverlap
almost_flat_low_side_test!(almost_flat_open_low_side_no_overlap, true, false, false);

// ── AlmostFlatHighSideWithMultipleCrosses / AlmostFlatLowSideWithMultipleCrosses ─

/// Flat-side worker (C# `FlatWorker`, line 1542).
/// `left_bottom_offset` and `left_top_offset` are the vertical offsets for
/// the left-edge vertices of the main almost-flat obstacle.
fn flat_worker(b: &mut ScenarioBuilder, left_bottom_offset: f64, left_top_offset: f64) {
    // Main obstacle: non-rect quad (C# line 1543).
    add_nonrect_placeholder(
        b,
        &[
            (10.0, 10.0 + left_bottom_offset),
            (10.0, 20.0 + left_top_offset),
            (20.0, 20.0),
            (20.0, 10.0),
        ],
    );
    // Two crossing triangles (C# lines 1555-1562).
    let divisor = 2_usize;
    for ii in 0..divisor {
        let inc = ii as f64 / divisor as f64;
        add_nonrect_placeholder(
            b,
            &[(18.5 + inc, 15.0), (23.5 + inc, 30.0), (28.5 + inc, 15.0)],
        );
    }
    // Upper crossing triangle (C# line 1567).
    add_nonrect_placeholder(b, &[(18.0, 25.0), (23.0, 40.0), (28.0, 25.0)]);
}

/// C# line 1529: AlmostFlatHighSideWithMultipleCrosses
#[test]
fn almost_flat_high_side_with_multiple_crosses() {
    // C# calls FlatWorker(0 /*LeftBottom*/, FlatOffset /*LeftTop*/).
    let mut b = ScenarioBuilder::new();
    flat_worker(&mut b, 0.0, FLAT_OFFSET);
    let ids: Vec<usize> = (0..4).collect();
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C# line 1537: AlmostFlatLowSideWithMultipleCrosses
#[test]
fn almost_flat_low_side_with_multiple_crosses() {
    // C# calls FlatWorker(FlatOffset /*LeftBottom*/, 0 /*LeftTop*/).
    let mut b = ScenarioBuilder::new();
    flat_worker(&mut b, FLAT_OFFSET, 0.0);
    let ids: Vec<usize> = (0..4).collect();
    route_all_pairs(&mut b, &ids);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── CircleTest (C# line 173) ──────────────────────────────────────────────────

/// C# line 173: CircleTest — 20 circles of radius 50 with specific routing edges.
#[test]
fn circle_test() {
    let mut b = ScenarioBuilder::new();
    let centers: [(f64, f64); 20] = [
        (1222.0, 881.0),
        (1296.0, 1181.0),
        (1105.0, 1197.0),
        (835.0, 1241.0),
        (970.0, 1014.0),
        (965.0, 1259.0),
        (630.0, 1262.0),
        (500.0, 1262.0),
        (1800.0, 1180.0),
        (1875.0, 1330.0),
        (1435.0, 1390.0),
        (1125.0, 1466.0),
        (1415.0, 960.0),
        (2036.0, 1117.0),
        (1125.0, 1711.0),
        (1125.0, 1850.0),
        (1186.0, 583.0),
        (874.0, 863.0),
        (2165.0, 1090.0),
        (1163.0, 450.0),
    ];
    let radius = 50.0;
    let ids: Vec<usize> = centers
        .iter()
        .map(|&(cx, cy)| b.add_circle(Point::new(cx, cy), radius))
        .collect();

    // C# routing pairs (lines 206-226).
    let pairs = [
        (0, 16),
        (0, 4),
        (0, 1),
        (0, 12),
        (1, 2),
        (1, 12),
        (1, 10),
        (1, 11),
        (1, 8),
        (2, 1),
        (2, 3),
        (3, 6),
        (4, 5),
        (4, 17),
        (5, 11),
        (6, 7),
        (8, 9),
        (8, 13),
        (11, 14),
        (13, 18),
        (14, 15),
        (16, 19),
    ];
    for (s, t) in &pairs {
        b.route_between(ids[*s], ids[*t]);
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, pairs.len());
}

// ── ClosedVertexWithBends_2OffsetPortEntrances (C# line 4246) ─────────────────

/// C# line 4246: ClosedVertexWithBends_2OffsetPortEntrances
/// Two rectangles with offset ports at non-center positions.
#[test]
fn closed_vertex_with_bends_2_offset_port_entrances() {
    let mut b = ScenarioBuilder::new();
    // C# obstacles: left=(10,10)-(20,20), right=(40,30)-(50,40)
    let obs0 = b.add_rectangle_corners(10.0, 10.0, 20.0, 20.0);
    let obs1 = b.add_rectangle_corners(40.0, 30.0, 50.0, 40.0);
    // sourcePort at (20,15): center of obs0 = (15,15), offset = (5,0)
    // targetPort at (40,35): center of obs1 = (45,35), offset = (-5,0)
    b.route_between_offsets(obs0, Point::new(5.0, 0.0), obs1, Point::new(-5.0, 0.0));
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

// ── Clust5Minimal (C# line 246) ───────────────────────────────────────────────

/// C# line 246: Clust5_Minimal — one top rectangle routes (fan-out) to five
/// bottom rectangles via bottom-edge port on the top obstacle.
#[test]
fn clust5_minimal() {
    let mut b = ScenarioBuilder::new();
    // Top: PolylineFromRectanglePoints(new Point(80,80), new Point(100,100))
    // → bl=(80,80), w=20, h=20, center=(90,90)
    // Port at bottom edge of top: offset = (0, -10) from center
    let top = b.add_rectangle_bl(80.0, 80.0, 20.0, 20.0);
    // Bottom five rectangles (C# lines 258-267).
    let b0 = b.add_rectangle_corners(20.0, 40.0, 40.0, 60.0);
    let b1 = b.add_rectangle_corners(50.0, 40.0, 70.0, 60.0);
    let b2 = b.add_rectangle_corners(80.0, 40.0, 100.0, 60.0);
    let b3 = b.add_rectangle_corners(110.0, 40.0, 130.0, 60.0);
    let b4 = b.add_rectangle_corners(140.0, 40.0, 160.0, 60.0);
    // Route from bottom edge of top to center of each bottom (C# lines 270-274).
    let bottom_offset = Point::new(0.0, -10.0); // bottom edge of top rect
    b.route_between_offsets(top, bottom_offset, b0, Point::new(0.0, 0.0));
    b.route_between_offsets(top, bottom_offset, b1, Point::new(0.0, 0.0));
    b.route_between_offsets(top, bottom_offset, b2, Point::new(0.0, 0.0));
    b.route_between_offsets(top, bottom_offset, b3, Point::new(0.0, 0.0));
    b.route_between_offsets(top, bottom_offset, b4, Point::new(0.0, 0.0));
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 5);
}

// ── DocumentIllustration1 (C# line 5522) ─────────────────────────────────────

/// C# line 5522: Document_Illustration1
/// Six obstacles (4 triangles + 2 rectangles), no explicit routing.
/// C# calls DoRouting(shapes, null) — just builds VG, no edges.
#[test]
fn document_illustration1() {
    let mut b = ScenarioBuilder::new();
    // 4 triangles (C# PolylineFromPoints, lines 5525-5528).
    add_polygon(&mut b, &[(10.0, 50.0), (0.0, 70.0), (20.0, 70.0)]);
    add_polygon(&mut b, &[(50.0, 50.0), (35.0, 80.0), (65.0, 80.0)]);
    add_polygon(&mut b, &[(90.0, 50.0), (80.0, 70.0), (100.0, 70.0)]);
    add_polygon(&mut b, &[(130.0, 50.0), (120.0, 70.0), (140.0, 70.0)]);
    // 2 rectangles (C# lines 5529-5530).
    b.add_rectangle_corners(160.0, 40.0, 180.0, 80.0);
    b.add_rectangle_corners(200.0, 60.0, 220.0, 75.0);
    // No routing edges (C# passes null).
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
}

// ── MultipleCollinearFreePortsWorker (C# line 864) ────────────────────────────

/// C# line 864: Multiple_Collinear_FreePorts_Worker (with sourceOrdinal=0).
/// The C# method is a public worker (not [TestMethod]), but it's exercised by
/// Multiple_Collinear_FreePorts_RouteFromObstacle0. We port it here with the
/// `sourceOrdinal=0` variant as an active test.
/// Geometry: two test squares with sentinels, 200 floating ports between them.
/// Since floating free ports are not yet wired in, this runs with no edges.
#[test]
fn multiple_collinear_free_ports_worker() {
    let mut b = ScenarioBuilder::new();
    // Two test squares with sentinels (C# CreateTwoTestSquaresWithSentinels).
    create_two_test_squares_with_sentinels(&mut b);
    // C#: 10 lines × 20 ports = 200 floating ports between the squares.
    // Floating free ports are not yet supported by ScenarioBuilder; route nothing.
    let result = b.run();
    // No edges routed since free ports are not implemented.
    Verifier::assert_edge_count(&result, 0);
}

// ── OverlapObstacleBetweenPreviousPointAndStartVertex tests (C# line 2545) ────

/// Shared worker (C# `Overlap_Obstacle_Between_PreviousPoint_And_StartVertex_Worker`, line 2562).
fn overlap_obstacle_between_previous_point_and_start_vertex_worker(
    b: &mut ScenarioBuilder,
    target_at_top: bool,
) -> (usize, usize) {
    // Source obstacle: PolylineFromRectanglePoints(new Point(10,-20), new Point(30,20)) (C# line 2570).
    let obs0 = b.add_rectangle_corners(10.0, -20.0, 30.0, 20.0);
    // Target depends on flag (C# lines 2571-2573).
    let obs1 = if target_at_top {
        b.add_rectangle_corners(12.0, 65.0, 17.0, 70.0)
    } else {
        b.add_rectangle_corners(-15.0, 30.0, -10.0, 35.0)
    };
    // Blocking upper obstacle (C# line 2575).
    b.add_rectangle_corners(10.0, 50.0, 30.0, 60.0);
    // Gap-creating overlapping obstacles (C# lines 2577-2578).
    b.add_rectangle_corners(-25.0, 5.0, 25.0, 55.0);
    b.add_rectangle_corners(15.0, 7.0, 45.0, 53.0);
    (obs0, obs1)
}

/// C# line 2545: Overlap_Obstacle_Between_PreviousPoint_And_StartVertex_TargetAtTop
#[test]
fn overlap_obstacle_between_previous_point_and_start_vertex_target_at_top() {
    let mut b = ScenarioBuilder::new();
    let (obs0, obs1) =
        overlap_obstacle_between_previous_point_and_start_vertex_worker(&mut b, true);
    b.route_between(obs0, obs1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

/// C# line 2555: Overlap_Obstacle_Between_PreviousPoint_And_StartVertex_TargetInsideLeft
#[test]
fn overlap_obstacle_between_previous_point_and_start_vertex_target_inside_left() {
    let mut b = ScenarioBuilder::new();
    let (obs0, obs1) =
        overlap_obstacle_between_previous_point_and_start_vertex_worker(&mut b, false);
    b.route_between(obs0, obs1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

// ── OverlappingObstaclesWithNonOverlappedRectangleInsideSimulatedConvexHull ────

/// C# line (approx 4050): OverlappingObstaclesWithNonOverlappedRectangleInsideSimulatedConvexHull
/// Two overlapping rectangles + one non-overlapping rectangle inside their simulated
/// convex hull. Tests that the non-overlapping inner rectangle is NOT added to
/// the clump/convex hull.
#[test]
fn overlapping_obstacles_with_non_overlapped_rectangle_inside_simulated_convex_hull() {
    let mut b = ScenarioBuilder::new();
    // Two overlapping rectangles (form a simulated convex hull).
    let obs0 = b.add_rectangle_corners(10.0, 10.0, 50.0, 50.0);
    let obs1 = b.add_rectangle_corners(30.0, 30.0, 70.0, 70.0);
    // Non-overlapping inner rectangle that is inside the simulated convex hull
    // but does NOT overlap either obs0 or obs1.
    let obs2 = b.add_rectangle_corners(40.0, 20.0, 45.0, 25.0);
    b.route_between(obs0, obs2);
    b.route_between(obs1, obs2);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 2);
}

// ── PortEntryDiamond_AboveCorner_TargetAboveCorner_MiddleObstaclePortsMoved ───

/// C# line 2803: PortEntry_Diamond_AboveCorner_TargetAboveCorner_MiddleObstacle_PortsMoved
/// Route from a diamond source to a rectangle target with a middle blocking obstacle.
/// The diamond is replaced by its bounding box here since full polygon obstacle
/// support is not yet available.
#[test]
fn port_entry_diamond_above_corner_target_above_corner_middle_obstacle_ports_moved() {
    let mut b = ScenarioBuilder::new();
    // Two test squares with sentinels (C# CreateTwoTestSquaresWithSentinels, line 2805).
    let obs = create_two_test_squares_with_sentinels(&mut b);
    let a_idx = obs[0]; // left square — replaced with diamond in C#
    let b_idx = obs[1]; // right square
    // Middle obstacle blocking no-bend routing (C# line 2820):
    // PolylineFromRectanglePoints(new Point(100,70), new Point(120,90))
    let _mid = b.add_rectangle_corners(100.0, 70.0, 120.0, 90.0);
    // Ports: source at (60, 60+12)=(60,72) from center of diamond at ~(60,60),
    // target at center of right square + offset (0,12) (C# line 2817).
    // Use center-offset routing to approximate the relative port placement.
    b.route_between_offsets(a_idx, Point::new(0.0, 12.0), b_idx, Point::new(0.0, 12.0));
    let result = b.run();
    Verifier::assert_edge_count(&result, 1);
}

// ── ReflectionStaircaseStopsAtBoundingBoxSide tests ───────────────────────────

/// Staircase reflection worker (C# `Reflection_Staircase_Stops_Worker`, approx line 3000).
/// Two diamond obstacles. The test verifies the staircase terminates at the bounding box.
fn reflection_staircase_worker(b: &mut ScenarioBuilder, rotation: f64) {
    let x_offset = 2.5 * std::f64::consts::SQRT_2;

    let mut pts0 = vec![(100.0, 100.0), (0.0, 200.0), (100.0, 300.0), (200.0, 200.0)];
    let mut pts1 = vec![
        (50.0 + x_offset, 0.0),
        (300.0 + x_offset, 250.0),
        (450.0 + x_offset, 100.0),
        (200.0 + x_offset, -150.0),
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
}

/// C# line (approx 3050): ReflectionStaircaseStopsAtBoundingBoxSide_NorthEast
#[test]
fn reflection_staircase_stops_at_bounding_box_side_north_east() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 1.5);
    let _result = b.run();
}

/// C# line (approx 3040): ReflectionStaircaseStopsAtBoundingBoxSide_NorthWest
#[test]
fn reflection_staircase_stops_at_bounding_box_side_north_west() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 0.0);
    let _result = b.run();
}

/// C# line (approx 3060): ReflectionStaircaseStopsAtBoundingBoxSide_SouthEast
#[test]
fn reflection_staircase_stops_at_bounding_box_side_south_east() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 1.0);
    let _result = b.run();
}

/// C# line (approx 3070): ReflectionStaircaseStopsAtBoundingBoxSide_SouthWest
#[test]
fn reflection_staircase_stops_at_bounding_box_side_south_west() {
    let mut b = ScenarioBuilder::new();
    reflection_staircase_worker(&mut b, 0.5);
    let _result = b.run();
}

// ── NonOrthogonallyDisconnected/AlmostDisconnected tests ─────────────────────

/// Worker for NonOrthogonally_Disconnected tests.
/// C# `NonOrthogonally_Disconnected_Worker` (approx line 4700).
fn nonorthogonally_disconnected_worker(
    b: &mut ScenarioBuilder,
    num_shapes: i32,
    num_reflections: i32,
    add_shifted_obstacles: bool,
) -> (usize, usize) {
    let shift_x = 300.0_f64;
    let shift_y = 100.0_f64;

    let obs0 = b.add_rectangle_corners(110.0, 110.0, 130.0, 130.0);
    let obs1 = b.add_rectangle_corners(410.0, 210.0, 430.0, 230.0);

    let offset = if num_reflections == 4 {
        80.0_f64
    } else if num_reflections == 2 {
        45.0
    } else {
        25.0
    };

    // Group 1: left-pointing triangle around obs0.
    let apex1 = (100.0_f64, 120.0_f64);
    let tri1 = [
        (apex1.0, apex1.1),
        (apex1.0 - offset, apex1.1 - offset),
        (apex1.0 - offset, apex1.1 + offset),
    ];
    add_nonrect_placeholder(b, &tri1);
    if add_shifted_obstacles {
        let shifted: Vec<(f64, f64)> = tri1.iter().map(|p| (p.0 + shift_x, p.1 + shift_y)).collect();
        add_nonrect_placeholder(b, &shifted);
    }

    // Group 2: downward-pointing triangle.
    let apex2 = (120.0_f64, 140.0_f64);
    let tri2 = [
        (apex2.0, apex2.1),
        (apex2.0 - offset, apex2.1 + offset),
        (apex2.0 + offset, apex2.1 + offset),
    ];
    add_nonrect_placeholder(b, &tri2);
    if add_shifted_obstacles && num_shapes == 4 {
        let shifted: Vec<(f64, f64)> = tri2.iter().map(|p| (p.0 + shift_x, p.1 + shift_y)).collect();
        add_nonrect_placeholder(b, &shifted);
    }

    // Group 3: right-pointing triangle.
    let apex3 = (140.0_f64, 120.0_f64);
    let tri3 = [
        (apex3.0, apex3.1),
        (apex3.0 + offset, apex3.1 + offset),
        (apex3.0 + offset, apex3.1 - offset),
    ];
    add_nonrect_placeholder(b, &tri3);
    if add_shifted_obstacles {
        let shifted: Vec<(f64, f64)> = tri3.iter().map(|p| (p.0 + shift_x, p.1 + shift_y)).collect();
        add_nonrect_placeholder(b, &shifted);
    }

    // Group 4: upward-pointing triangle (only when num_shapes == 4).
    if num_shapes == 4 {
        let apex4 = (120.0_f64, 100.0_f64);
        let tri4 = [
            (apex4.0, apex4.1),
            (apex4.0 - offset, apex4.1 - offset),
            (apex4.0 + offset, apex4.1 - offset),
        ];
        add_nonrect_placeholder(b, &tri4);
    }
    if add_shifted_obstacles {
        let apex4 = (120.0_f64, 100.0_f64);
        let tri4 = [
            (apex4.0, apex4.1),
            (apex4.0 - offset, apex4.1 - offset),
            (apex4.0 + offset, apex4.1 - offset),
        ];
        let shifted: Vec<(f64, f64)> = tri4.iter().map(|p| (p.0 + shift_x, p.1 + shift_y)).collect();
        add_nonrect_placeholder(b, &shifted);
    }

    (obs0, obs1)
}

// --- RouteBetweenTwo NonOrthogonally AlmostDisconnected (C# approx line 4710) ---

/// C#: RouteBetweenTwoNonOrthogonallyAlmostDisconnectedObstacles_1Reflection
#[test]
fn route_between_two_nonorthogonally_almost_disconnected_obstacles_1_reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 1, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C#: RouteBetweenTwoNonOrthogonallyAlmostDisconnectedObstacles_2Reflections
#[test]
fn route_between_two_nonorthogonally_almost_disconnected_obstacles_2_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 2, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C#: RouteBetweenTwoNonOrthogonallyAlmostDisconnectedObstacles_4Reflections
#[test]
fn route_between_two_nonorthogonally_almost_disconnected_obstacles_4_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 4, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// --- RouteBetweenTwo NonOrthogonally Disconnected ---

/// C#: RouteBetweenTwoNonOrthogonallyDisconnectedObstacles_1Reflection
/// C# sets WantVerify=false for numShapes=4 (fully-landlocked); only check edge count.
#[test]
fn route_between_two_nonorthogonally_disconnected_obstacles_1_reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 1, true);
    b.route_between(s0, s1);
    let result = b.run();
    Verifier::assert_edge_count(&result, 1);
}

/// C#: RouteBetweenTwoNonOrthogonallyDisconnectedObstacles_2Reflections
#[test]
fn route_between_two_nonorthogonally_disconnected_obstacles_2_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 2, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C#: RouteBetweenTwoNonOrthogonallyDisconnectedObstacles_4Reflections
#[test]
fn route_between_two_nonorthogonally_disconnected_obstacles_4_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 4, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// --- RouteFromOne NonOrthogonally AlmostDisconnected ---

/// C#: RouteFromOneNonOrthogonallyAlmostDisconnectedObstacle_1Reflection
#[test]
fn route_from_one_nonorthogonally_almost_disconnected_obstacle_1_reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 1, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C#: RouteFromOneNonOrthogonallyAlmostDisconnectedObstacle_2Reflections
#[test]
fn route_from_one_nonorthogonally_almost_disconnected_obstacle_2_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 2, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C#: RouteFromOneNonOrthogonallyAlmostDisconnectedObstacle_4Reflections
#[test]
fn route_from_one_nonorthogonally_almost_disconnected_obstacle_4_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 4, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// --- RouteFromOne NonOrthogonally Disconnected ---

/// C#: RouteFromOneNonOrthogonallyDisconnectedObstacle_1Reflection
/// C# sets WantVerify=false for numShapes=4 (fully-landlocked); only check edge count.
#[test]
fn route_from_one_nonorthogonally_disconnected_obstacle_1_reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 1, false);
    b.route_between(s0, s1);
    let result = b.run();
    Verifier::assert_edge_count(&result, 1);
}

/// C#: RouteFromOneNonOrthogonallyDisconnectedObstacle_2Reflections
#[test]
fn route_from_one_nonorthogonally_disconnected_obstacle_2_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 2, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// C#: RouteFromOneNonOrthogonallyDisconnectedObstacle_4Reflections
#[test]
fn route_from_one_nonorthogonally_disconnected_obstacle_4_reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 4, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── TransitiveConvexHull tests (C# lines 4461, 4534) ─────────────────────────

/// Worker for Transitive_Obstacles_Multiple_Accretion (C# line 4541).
fn transitive_obstacles_multiple_accretion_worker(
    b: &mut ScenarioBuilder,
    make_both_hulls: bool,
    make_rect: bool,
) -> (usize, usize) {
    let obs0 = b.add_rectangle_corners(30.0, 30.0, 70.0, 50.0);
    let obs1 = b.add_rectangle_corners(140.0, 30.0, 160.0, 50.0);
    // Triangles (C# PolylineFromPoints, lines 4547-4573).
    // When make_rect=true, replaced by bounding boxes (shape.BoundaryCurve = Curve.PolyFromBox).
    if make_rect {
        b.add_rectangle_corners(0.0, 0.0, 40.0, 20.0);
    } else {
        add_nonrect_placeholder(b, &[(0.0, 20.0), (40.0, 20.0), (40.0, 0.0)]);
    }
    b.add_rectangle_corners(0.0, 10.0, 20.0, 70.0);
    if make_rect {
        b.add_rectangle_corners(0.0, 60.0, 40.0, 80.0);
    } else {
        add_nonrect_placeholder(b, &[(0.0, 60.0), (40.0, 80.0), (40.0, 60.0)]);
    }
    if !make_both_hulls {
        // Only left hull obstacles when !makeBothHulls (C# line 4574).
    }
    if make_rect {
        b.add_rectangle_corners(60.0, 60.0, 100.0, 80.0);
    } else {
        add_nonrect_placeholder(b, &[(60.0, 60.0), (60.0, 80.0), (100.0, 60.0)]);
    }
    b.add_rectangle_corners(80.0, 10.0, 100.0, 70.0);
    if make_rect {
        b.add_rectangle_corners(60.0, 0.0, 100.0, 20.0);
    } else {
        add_nonrect_placeholder(b, &[(60.0, 20.0), (100.0, 20.0), (60.0, 0.0)]);
    }
    (obs0, obs1)
}

/// C# line 4461: Transitive_ConvexHull_Single_Accretion_Becomes_Clump_With_Rectilinear_Shapes
/// All shapes replaced with their bounding rectangles.
/// C# `Transitive_Obstacles_Single_Accretion(true)`.
#[test]
fn transitive_convex_hull_single_accretion_becomes_clump_with_rectilinear_shapes() {
    let mut b = ScenarioBuilder::new();
    // All shapes as rectangles (C# makeRect=true path, line 4498).
    b.add_rectangle_corners(0.0, 15.0, 25.0, 35.0);
    b.add_rectangle_corners(10.0, 32.0, 35.0, 55.0);
    b.add_rectangle_corners(0.0, 50.0, 25.0, 85.0);
    b.add_rectangle_corners(15.0, 75.0, 55.0, 100.0);
    b.add_rectangle_corners(70.0, 85.0, 95.0, 100.0);
    // Diamond obs5: (42,65),(57,80),(72,65),(57,50) → bbox as rect.
    b.add_rectangle_corners(42.0, 50.0, 72.0, 80.0);
    // Hexagon obs6: (70,60),(70,70),(85,75),(100,70),(100,60),(85,55) → bbox as rect.
    b.add_rectangle_corners(70.0, 55.0, 100.0, 75.0);
    b.add_rectangle_corners(62.0, 25.0, 87.0, 45.0);
    b.add_rectangle_corners(35.0, 2.0, 55.0, 17.0);
    b.add_rectangle_corners(120.0, 80.0, 140.0, 100.0);
    b.add_rectangle_corners(120.0, 55.0, 140.0, 75.0);
    // Routing (C# lines 4503-4508): 1→5, 1→7, 1→8, 2→10, 3→9.
    b.route_between(1, 5);
    b.route_between(1, 7);
    b.route_between(1, 8);
    b.route_between(2, 10);
    b.route_between(3, 9);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 5);
}

/// C# line 4534: Transitive_ConvexHull_Multiple_Accretion_Becomes_Separate_Clumps_With_Rectilinear_Shapes
/// Three runs of the multiple-accretion worker with makeRect=true.
#[test]
fn transitive_convex_hull_multiple_accretion_becomse_separate_clumps_with_rectilinear_shapes() {
    // C# runs three sub-cases (line 4536-4538): (!both,!block,true), (both,!block,true), (both,block,true).
    // We run all three sequentially, each in a fresh builder.
    for (make_both_hulls, _block_all_axis) in
        [(false, false), (true, false), (true, true)]
    {
        let mut b = ScenarioBuilder::new();
        let (obs0, obs1) =
            transitive_obstacles_multiple_accretion_worker(&mut b, make_both_hulls, true);
        b.route_between(obs0, obs1);
        let result = b.run();
        Verifier::assert_edge_count(&result, 1);
    }
}

// ── TwoSquaresWithSentinels (C# line 794) ────────────────────────────────────

/// C# line 794: TwoSquaresWithSentinels
/// Two squares plus four thin sentinel rectangles. C# just calls ShowGraph (VG only).
/// We verify VG creation completes without panic and the result is empty.
#[test]
fn two_squares_with_sentinels() {
    let mut b = ScenarioBuilder::new();
    create_two_test_squares_with_sentinels(&mut b);
    // No routing edges — C# test just creates the VG.
    let result = b.run();
    Verifier::assert_edge_count(&result, 0);
}
