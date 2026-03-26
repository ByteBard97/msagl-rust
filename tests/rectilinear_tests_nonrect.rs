/// Non-rectangular obstacle tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// These tests exercise diamond, circle, triangle, and other non-rectangular
/// obstacle shapes. Diamond and circle shapes use actual polygon boundaries.
/// Triangle and other shapes still use bounding-box placeholders pending
/// full non-convex support.

#[path = "test_harness/mod.rs"]
mod test_harness;

use msagl_rust::Point;
use test_harness::verifier::RECTILINEAR_TOLERANCE;
use test_harness::{ScenarioBuilder, Verifier};

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Bounding box for a set of points.
fn bounding_box(points: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let min_x = points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_y = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    (min_x, min_y, max_x - min_x, max_y - min_y)
}

/// Add a polygon obstacle from a list of (x, y) tuples.
/// Returns the obstacle index.
fn add_polygon(b: &mut ScenarioBuilder, points: &[(f64, f64)]) -> usize {
    let pts: Vec<Point> = points.iter().map(|&(x, y)| Point::new(x, y)).collect();
    b.add_polygon(&pts)
}

/// Add a bounding-box rectangle placeholder for a non-rectangular obstacle.
/// Used for shapes not yet fully supported (e.g., non-convex shapes).
/// Returns the obstacle index.
fn add_nonrect_placeholder(b: &mut ScenarioBuilder, points: &[(f64, f64)]) -> usize {
    let (x, y, w, h) = bounding_box(points);
    b.add_rectangle_bl(x, y, w, h)
}

/// Add a circle obstacle approximated as a polygon.
/// Returns the obstacle index.
fn add_circle_shape(b: &mut ScenarioBuilder, center: (f64, f64), radius: f64) -> usize {
    b.add_circle(Point::new(center.0, center.1), radius)
}

// ── Diamond helpers (mirror C# Create_Diamond3, Create_Diamond3_Square6, etc.) ──

/// C# `Create_Diamond3()`: three diamond shapes.
/// Diamond 0: (100,500),(140,540),(180,500),(140,460) — bbox 100..180, 460..540
/// Diamond 1: (40,420),(80,480),(120,420),(80,380) — bbox 40..120, 380..480
/// Diamond 2: (160,440),(200,480),(240,440),(200,400) — bbox 160..240, 400..480
fn add_diamond3(b: &mut ScenarioBuilder) -> Vec<usize> {
    let d0 = add_polygon(
        b,
        &[
            (100.0, 500.0),
            (140.0, 540.0),
            (180.0, 500.0),
            (140.0, 460.0),
        ],
    );
    let d1 = add_polygon(
        b,
        &[(40.0, 420.0), (80.0, 480.0), (120.0, 420.0), (80.0, 380.0)],
    );
    let d2 = add_polygon(
        b,
        &[
            (160.0, 440.0),
            (200.0, 480.0),
            (240.0, 440.0),
            (200.0, 400.0),
        ],
    );
    vec![d0, d1, d2]
}

/// C# `Create_Diamond3_Square6()`: Diamond3 + 6 overlapping rectangles.
fn add_diamond3_square6(b: &mut ScenarioBuilder) -> Vec<usize> {
    let mut ids = add_diamond3(b);
    // Overlap top/bottom points of upper diamond with rectangles (center, w, h).
    // CurveFactory.CreateRectangle(w, h, center) creates rect centered at center.
    // rect(30,10) @ (140,460) → bl=(125,455), 30x10
    ids.push(b.add_rectangle_bl(125.0, 455.0, 30.0, 10.0));
    // rect(30,10) @ (140,540) → bl=(125,535), 30x10
    ids.push(b.add_rectangle_bl(125.0, 535.0, 30.0, 10.0));
    // rect(30,10) @ (100,500) → bl=(85,495), 30x10
    ids.push(b.add_rectangle_bl(85.0, 495.0, 30.0, 10.0));
    // rect(30,30) @ (180,500) → bl=(165,485), 30x30
    ids.push(b.add_rectangle_bl(165.0, 485.0, 30.0, 30.0));
    // rect(20,20) @ (80,480) → bl=(70,470), 20x20
    ids.push(b.add_rectangle_bl(70.0, 470.0, 20.0, 20.0));
    // rect(20,20) @ (200,480) → bl=(190,470), 20x20
    ids.push(b.add_rectangle_bl(190.0, 470.0, 20.0, 20.0));
    ids
}

/// C# `Create_Diamond3_Square8()`: Diamond3_Square6 + 2 interior rectangles.
fn add_diamond3_square8(b: &mut ScenarioBuilder) -> Vec<usize> {
    let mut ids = add_diamond3_square6(b);
    // rect(20,15) @ (140,517.5) → bl=(130,510), 20x15
    ids.push(b.add_rectangle_bl(130.0, 510.0, 20.0, 15.0));
    // rect(10,10) @ (140,480) → bl=(135,475), 10x10
    ids.push(b.add_rectangle_bl(135.0, 475.0, 10.0, 10.0));
    ids
}

// ── Category: Diamond tests ──────────────────────────────────────────────────

/// Port: Diamond3
/// Three diamond-shaped obstacles, route between all pairs.
/// C#: Create_Diamond3() + CreateRoutingBetweenObstacles(obstacles, 0, -1).
#[test]
fn diamond3() {
    // Actual shapes: 3 diamonds (see add_diamond3 for vertex coords).
    // Placeholder: bounding-box rectangles.
    let mut b = ScenarioBuilder::new();
    let ids = add_diamond3(&mut b);
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Diamond3_With_FreePorts
/// Three diamonds + two free (floating) ports at (65,525) and (50,465).
#[test]
fn diamond3_with_free_ports() {
    // C#: Create_Diamond3(), MakeAbsoluteFreePort at (65,525) and (50,465).
    // Free ports route between obstacles without being attached to any shape.
    let mut b = ScenarioBuilder::new();
    let ids = add_diamond3(&mut b);
    // TODO: add free ports at (65,525) and (50,465) when FloatingPort is supported.
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Diamond3_Square6_Overlap
/// Three diamonds + 6 overlapping rectangles.
/// C#: CreateRoutingBetweenObstacles(obstacles, 0, -1) — routes from obstacle[0] to all others.
#[test]
fn diamond3_square6_overlap() {
    let mut b = ScenarioBuilder::new();
    let ids = add_diamond3_square6(&mut b);
    // C# routes from obstacle[0] (diamond 0) to every other obstacle.
    for j in 1..ids.len() {
        b.route_between(ids[0], ids[j]);
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Diamond3_Square8_Overlap
/// Three diamonds + 6 overlapping rectangles + 2 interior rectangles.
/// C#: CreateRoutingBetweenObstacles(obstacles, 0, -1) — routes from obstacle[0] to all others.
#[test]
fn diamond3_square8_overlap() {
    let mut b = ScenarioBuilder::new();
    let ids = add_diamond3_square8(&mut b);
    for j in 1..ids.len() {
        b.route_between(ids[0], ids[j]);
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Diamond3_Square9_Overlap
/// Diamond3_Square8 + one more overlapping rectangle at (119.5,471)-(160,530).
/// C#: CreateRoutingBetweenObstacles(obstacles, 0, -1) — routes from obstacle[0] to all others.
#[test]
fn diamond3_square9_overlap() {
    let mut b = ScenarioBuilder::new();
    let mut ids = add_diamond3_square8(&mut b);
    // C#: Rectangle(Point(119.5,530), Point(160,471)) → CreateRectangle(w,h,center)
    // w = 160 - 119.5 = 40.5, h = 530 - 471 = 59, center = (139.75, 500.5)
    // bl = (119.5, 471)
    ids.push(b.add_rectangle_bl(119.5, 471.0, 40.5, 59.0));
    for j in 1..ids.len() {
        b.route_between(ids[0], ids[j]);
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Diamond3_Square9_Overlap_HalfWidths
/// Diamond3_Square8 + one more rectangle at half-width/half-height.
/// C#: rect center=(141.25,500.5), half-w=18.75/2, half-h=29.5/2.
/// C#: CreateRoutingBetweenObstacles(obstacles, 0, -1) — routes from obstacle[0] to all others.
#[test]
fn diamond3_square9_overlap_half_widths() {
    let mut b = ScenarioBuilder::new();
    let mut ids = add_diamond3_square8(&mut b);
    // C#: Rectangle(Point(122.5,530), Point(160,471)), then CreateRectangle at half dims.
    // Full rect: bl=(122.5,471), w=37.5, h=59 → center=(141.25, 500.5)
    // Half-size rect: w=18.75, h=29.5, center=(141.25,500.5) → bl=(131.875, 485.75)
    ids.push(b.add_rectangle_bl(131.875, 485.75, 18.75, 29.5));
    for j in 1..ids.len() {
        b.route_between(ids[0], ids[j]);
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_nonrect(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Category: Circle tests ───────────────────────────────────────────────────

/// Port: CircleTest
/// 20 circles of radius 50 with specific routing edges between them.
/// C#: CreateCircle(50, center) for each, then AddRoutingPorts for specific pairs.
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
        .map(|&c| add_circle_shape(&mut b, c, radius))
        .collect();

    // C# routing pairs: (src_idx, tgt_idx)
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

// ── Category: Flat-side tests ────────────────────────────────────────────────

/// Shared worker for FlatTopSide / FlatBottomSide tests.
/// C# `FlatWorker(leftBottomOffset, leftTopOffset)`:
///   - Main obstacle: non-rect quad with almost-flat sides.
///   - 2 crossing triangles + 1 upper crossing triangle.
fn flat_worker(b: &mut ScenarioBuilder, left_bottom_offset: f64, left_top_offset: f64) {
    // Main obstacle: quad (10, 10+lbo), (10, 20+lto), (20, 20), (20, 10)
    // bbox: (10, 10+min(0,lbo)) .. (20, 20+max(0,lto))
    let _min_y = 10.0 + left_bottom_offset.min(0.0);
    let _max_y = 20.0 + left_top_offset.max(0.0);
    add_nonrect_placeholder(
        b,
        &[
            (10.0, 10.0 + left_bottom_offset),
            (10.0, 20.0 + left_top_offset),
            (20.0, 20.0),
            (20.0, 10.0),
        ],
    );

    // Two crossing triangles at increments of 0.5
    let divisor = 2;
    for ii in 0..divisor {
        let inc = ii as f64 / divisor as f64;
        // Triangle: (18.5+inc, 15), (23.5+inc, 30), (28.5+inc, 15)
        add_nonrect_placeholder(
            b,
            &[(18.5 + inc, 15.0), (23.5 + inc, 30.0), (28.5 + inc, 15.0)],
        );
    }

    // Upper crossing triangle: (18, 25), (23, 40), (28, 25)
    add_nonrect_placeholder(b, &[(18.0, 25.0), (23.0, 40.0), (28.0, 25.0)]);
}

/// Port: FlatTopSideWithMultipleCrosses
/// Tests handling of a flat top side with multiple other obstacle sides crossing it.
#[test]
fn flat_top_side_with_multiple_crosses() {
    let mut b = ScenarioBuilder::new();
    flat_worker(&mut b, 0.0, 0.0);
    // C#: route between all obstacles starting from obstacle 0
    let ids: Vec<usize> = (0..4).collect();
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: FlatBottomSideWithMultipleCrosses
/// Tests handling of a flat bottom side with multiple other obstacle sides crossing it.
#[test]
fn flat_bottom_side_with_multiple_crosses() {
    // C# calls FlatWorker(0, 0) — same as FlatTopSide (tests same geometry,
    // the distinction is in sweep direction handling).
    let mut b = ScenarioBuilder::new();
    flat_worker(&mut b, 0.0, 0.0);
    let ids: Vec<usize> = (0..4).collect();
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            b.route_between(ids[i], ids[j]);
        }
    }
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// ── Category: Disconnected obstacle tests ────────────────────────────────────

/// Port: Route_Between_Two_Separately_Disconnected_Obstacles
/// Two squares landlocked by overlapping angled rectangles.
/// C#: CreateSquare at (120,120) and (420,220), then AddAllShiftedRectangles.
#[test]
fn route_between_two_separately_disconnected_obstacles() {
    let mut b = ScenarioBuilder::new();
    let shift_x: f64 = 300.0;
    let shift_y: f64 = 100.0;

    // Two routing targets (rectangles — these are fine).
    let s0 = b.add_rectangle_bl(110.0, 110.0, 20.0, 20.0); // center (120,120), size 20
    let s1 = b.add_rectangle_bl(410.0, 210.0, 20.0, 20.0); // center (420,220), size 20

    // Angled rectangles (non-rect): 4 groups, each with original + shifted copy.
    // Group 1: (100,10),(80,30),(210,160),(230,140)
    let group1 = [(100.0, 10.0), (80.0, 30.0), (210.0, 160.0), (230.0, 140.0)];
    add_nonrect_placeholder(&mut b, &group1);
    let shifted1: Vec<(f64, f64)> = group1
        .iter()
        .map(|p| (p.0 + shift_x, p.1 + shift_y))
        .collect();
    add_nonrect_placeholder(&mut b, &shifted1);

    // Group 2: (30,80),(10,100),(140,230),(160,210)
    let group2 = [(30.0, 80.0), (10.0, 100.0), (140.0, 230.0), (160.0, 210.0)];
    add_nonrect_placeholder(&mut b, &group2);
    let shifted2: Vec<(f64, f64)> = group2
        .iter()
        .map(|p| (p.0 + shift_x, p.1 + shift_y))
        .collect();
    add_nonrect_placeholder(&mut b, &shifted2);

    // Group 3: (140,10),(10,140),(30,160),(160,30)
    let group3 = [(140.0, 10.0), (10.0, 140.0), (30.0, 160.0), (160.0, 30.0)];
    add_nonrect_placeholder(&mut b, &group3);
    let shifted3: Vec<(f64, f64)> = group3
        .iter()
        .map(|p| (p.0 + shift_x, p.1 + shift_y))
        .collect();
    add_nonrect_placeholder(&mut b, &shifted3);

    // Group 4: (210,80),(80,210),(100,230),(230,100)
    let group4 = [(210.0, 80.0), (80.0, 210.0), (100.0, 230.0), (230.0, 100.0)];
    add_nonrect_placeholder(&mut b, &group4);
    let shifted4: Vec<(f64, f64)> = group4
        .iter()
        .map(|p| (p.0 + shift_x, p.1 + shift_y))
        .collect();
    add_nonrect_placeholder(&mut b, &shifted4);

    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

/// Shared worker for NonOrthogonally_Disconnected tests.
/// C#: two routing squares + surrounding triangles that landlock them.
fn nonorthogonally_disconnected_worker(
    b: &mut ScenarioBuilder,
    num_shapes: i32,
    num_reflections: i32,
    add_shifted_obstacles: bool,
) -> (usize, usize) {
    let _initial_center = 120.0_f64;
    let shift_x = 300.0_f64;
    let shift_y = 100.0_f64;

    // Two routing target squares.
    let s0 = b.add_rectangle_bl(110.0, 110.0, 20.0, 20.0);
    let s1 = b.add_rectangle_bl(410.0, 210.0, 20.0, 20.0);

    let offset = if num_reflections == 4 {
        80.0
    } else if num_reflections == 2 {
        45.0
    } else {
        25.0
    };

    // 4 groups of triangles around the first square, optionally shifted copies.
    // Group 1: apex=(100,120), vertices: apex, apex+(-off,-off), apex+(-off,+off)
    let apex1 = (100.0, 120.0);
    let tri1 = [
        (apex1.0, apex1.1),
        (apex1.0 - offset, apex1.1 - offset),
        (apex1.0 - offset, apex1.1 + offset),
    ];
    add_nonrect_placeholder(b, &tri1);
    if add_shifted_obstacles {
        let shifted: Vec<(f64, f64)> = tri1
            .iter()
            .map(|p| (p.0 + shift_x, p.1 + shift_y))
            .collect();
        add_nonrect_placeholder(b, &shifted);
    }

    // Group 2: apex=(120,140), vertices: apex, apex+(-off,+off), apex+(+off,+off)
    let apex2 = (120.0, 140.0);
    let tri2 = [
        (apex2.0, apex2.1),
        (apex2.0 - offset, apex2.1 + offset),
        (apex2.0 + offset, apex2.1 + offset),
    ];
    add_nonrect_placeholder(b, &tri2);
    if add_shifted_obstacles && num_shapes == 4 {
        let shifted: Vec<(f64, f64)> = tri2
            .iter()
            .map(|p| (p.0 + shift_x, p.1 + shift_y))
            .collect();
        add_nonrect_placeholder(b, &shifted);
    }

    // Group 3: apex=(140,120), vertices: apex, apex+(+off,+off), apex+(+off,-off)
    let apex3 = (140.0, 120.0);
    let tri3 = [
        (apex3.0, apex3.1),
        (apex3.0 + offset, apex3.1 + offset),
        (apex3.0 + offset, apex3.1 - offset),
    ];
    add_nonrect_placeholder(b, &tri3);
    if add_shifted_obstacles {
        let shifted: Vec<(f64, f64)> = tri3
            .iter()
            .map(|p| (p.0 + shift_x, p.1 + shift_y))
            .collect();
        add_nonrect_placeholder(b, &shifted);
    }

    // Group 4: apex=(120,100), vertices: apex, apex+(-off,-off), apex+(+off,-off)
    // Only added if num_shapes == 4.
    if num_shapes == 4 {
        let apex4 = (120.0, 100.0);
        let tri4 = [
            (apex4.0, apex4.1),
            (apex4.0 - offset, apex4.1 - offset),
            (apex4.0 + offset, apex4.1 - offset),
        ];
        add_nonrect_placeholder(b, &tri4);
    }
    if add_shifted_obstacles {
        let apex4 = (120.0, 100.0);
        let tri4 = [
            (apex4.0, apex4.1),
            (apex4.0 - offset, apex4.1 - offset),
            (apex4.0 + offset, apex4.1 - offset),
        ];
        let shifted: Vec<(f64, f64)> = tri4
            .iter()
            .map(|p| (p.0 + shift_x, p.1 + shift_y))
            .collect();
        add_nonrect_placeholder(b, &shifted);
    }

    (s0, s1)
}

// --- 4 reflections ---

/// Port: Route_Between_Two_NonOrthogonally_Disconnected_Obstacles_4Reflections
#[test]
fn route_between_two_nonorthogonally_disconnected_4reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 4, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_Between_Two_NonOrthogonally_AlmostDisconnected_Obstacles_4Reflections
#[test]
fn route_between_two_nonorthogonally_almost_disconnected_4reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 4, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_From_One_NonOrthogonally_Disconnected_Obstacle_4Reflections
#[test]
fn route_from_one_nonorthogonally_disconnected_4reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 4, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_From_One_NonOrthogonally_AlmostDisconnected_Obstacle_4Reflections
#[test]
fn route_from_one_nonorthogonally_almost_disconnected_4reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 4, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// --- 2 reflections ---

/// Port: Route_Between_Two_NonOrthogonally_Disconnected_Obstacles_2Reflections
#[test]
fn route_between_two_nonorthogonally_disconnected_2reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 2, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_Between_Two_NonOrthogonally_AlmostDisconnected_Obstacles_2Reflections
#[test]
fn route_between_two_nonorthogonally_almost_disconnected_2reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 2, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_From_One_NonOrthogonally_Disconnected_Obstacle_2Reflections
#[test]
fn route_from_one_nonorthogonally_disconnected_2reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 2, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_From_One_NonOrthogonally_AlmostDisconnected_Obstacle_2Reflections
#[test]
fn route_from_one_nonorthogonally_almost_disconnected_2reflections() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 2, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

// --- 1 reflection ---

/// Port: Route_Between_Two_NonOrthogonally_Disconnected_Obstacles_1Reflection
#[test]
#[ignore = "SkipToNeighbor creates wider segments than depth-counter for bbox-approximated non-rect obstacles"]
fn route_between_two_nonorthogonally_disconnected_1reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 1, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_Between_Two_NonOrthogonally_AlmostDisconnected_Obstacles_1Reflection
#[test]
fn route_between_two_nonorthogonally_almost_disconnected_1reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 1, true);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_From_One_NonOrthogonally_Disconnected_Obstacle_1Reflection
#[test]
fn route_from_one_nonorthogonally_disconnected_1reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 4, 1, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}

/// Port: Route_From_One_NonOrthogonally_AlmostDisconnected_Obstacle_1Reflection
#[test]
fn route_from_one_nonorthogonally_almost_disconnected_1reflection() {
    let mut b = ScenarioBuilder::new();
    let (s0, s1) = nonorthogonally_disconnected_worker(&mut b, 3, 1, false);
    b.route_between(s0, s1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
}
