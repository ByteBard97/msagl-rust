use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::geometry::polyline::Polyline;
use msagl_rust::Point;

// --- Legacy constructor tests ---

#[test]
fn low_side_has_correct_slope() {
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        0,
    );
    // Horizontal side: slope = dy/dx = 0/10 = 0
    assert!((side.slope() - 0.0).abs() < 1e-10);
    assert_eq!(side.side_type(), SideType::Low);
}

#[test]
fn high_side_stores_type() {
    let side = ObstacleSide::new(
        SideType::High,
        Point::new(0.0, 10.0),
        Point::new(10.0, 10.0),
        1,
    );
    assert_eq!(side.side_type(), SideType::High);
    assert_eq!(side.obstacle_ordinal(), 1);
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

#[test]
fn scanline_intersection_vertical_side() {
    // Vertical side at x=5
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(5.0, 0.0),
        Point::new(5.0, 10.0),
        0,
    );
    // Horizontal scan: given any Y, X should be 5.0
    assert!((side.scanline_intersection(3.0, true) - 5.0).abs() < 1e-10);
    assert!((side.scanline_intersection(7.0, true) - 5.0).abs() < 1e-10);
}

#[test]
fn scanline_intersection_horizontal_side() {
    // Horizontal side at y=10
    let side = ObstacleSide::new(
        SideType::High,
        Point::new(0.0, 10.0),
        Point::new(20.0, 10.0),
        0,
    );
    // Vertical scan: given any X, Y should be 10.0
    assert!((side.scanline_intersection(5.0, false) - 10.0).abs() < 1e-10);
}

#[test]
fn scanline_intersection_diagonal_side() {
    // Diagonal side from (0,0) to (10,10), slope = 1
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 0.0),
        Point::new(10.0, 10.0),
        0,
    );
    assert!((side.slope() - 1.0).abs() < 1e-10);
    // Horizontal scan at y=5: x should be 5
    assert!((side.scanline_intersection(5.0, true) - 5.0).abs() < 1e-10);
    // Vertical scan at x=5: y should be 5
    assert!((side.scanline_intersection(5.0, false) - 5.0).abs() < 1e-10);
}

// --- Polyline-based ObstacleSide tests ---

#[test]
fn from_polyline_low_traverses_clockwise_for_hscan() {
    // Square: BL(0,0) -> TL(0,100) -> TR(100,100) -> BR(100,0), closed
    let mut poly = Polyline::new();
    let bl = poly.add_point(Point::new(0.0, 0.0));
    let tl = poly.add_point(Point::new(0.0, 100.0));
    let _tr = poly.add_point(Point::new(100.0, 100.0));
    let _br = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low, 0, bl, &poly, ScanDirection::horizontal(),
    );
    assert_eq!(side.start(), Point::new(0.0, 0.0));
    assert_eq!(side.end(), Point::new(0.0, 100.0));
    assert_eq!(side.end_vertex_key(), Some(tl));
}

#[test]
fn from_polyline_high_traverses_counterclockwise_for_hscan() {
    let mut poly = Polyline::new();
    let bl = poly.add_point(Point::new(0.0, 0.0));
    let _tl = poly.add_point(Point::new(0.0, 100.0));
    let _tr = poly.add_point(Point::new(100.0, 100.0));
    let br = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::High, 0, bl, &poly, ScanDirection::horizontal(),
    );
    assert_eq!(side.start(), Point::new(0.0, 0.0));
    assert_eq!(side.end(), Point::new(100.0, 0.0));
    assert_eq!(side.end_vertex_key(), Some(br));
}

#[test]
fn from_polyline_slope_for_angled_side() {
    // Non-rectangular: side from (0,0) to (10,100) -- not perpendicular
    let mut poly = Polyline::new();
    let p0 = poly.add_point(Point::new(0.0, 0.0));
    let _p1 = poly.add_point(Point::new(10.0, 100.0));
    let _p2 = poly.add_point(Point::new(110.0, 100.0));
    let _p3 = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low, 0, p0, &poly, ScanDirection::horizontal(),
    );

    // Slope = (change in scan-parallel coord) / (change in perp coord)
    // H-scan: scan-parallel = X, perp = Y
    // From (0,0) to (10,100): slope = (10-0)/(100-0) = 0.1
    assert!((side.slope() - 0.1).abs() < 1e-10);
    assert!((side.slope_inverse() - 10.0).abs() < 1e-10);
}

#[test]
fn from_polyline_perpendicular_has_zero_slope() {
    let mut poly = Polyline::new();
    let p0 = poly.add_point(Point::new(0.0, 0.0));
    let _p1 = poly.add_point(Point::new(0.0, 100.0));
    let _p2 = poly.add_point(Point::new(100.0, 100.0));
    let _p3 = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low, 0, p0, &poly, ScanDirection::horizontal(),
    );

    // Perpendicular side: X doesn't change (0,0) to (0,100)
    assert!((side.slope()).abs() < 1e-10);
}

#[test]
fn sentinel_construction() {
    let side = ObstacleSide::sentinel(
        SideType::High,
        Point::new(-10.0, -10.0),
        Point::new(-10.0, 100.0),
        1,
    );
    assert_eq!(side.side_type(), SideType::High);
    assert_eq!(side.obstacle_ordinal(), 1);
    assert_eq!(side.start(), Point::new(-10.0, -10.0));
    assert_eq!(side.end(), Point::new(-10.0, 100.0));
    assert!(side.start_vertex_key().is_none());
    assert!(side.end_vertex_key().is_none());
}

#[test]
fn direction_vector() {
    let mut poly = Polyline::new();
    let p0 = poly.add_point(Point::new(0.0, 0.0));
    let _p1 = poly.add_point(Point::new(0.0, 100.0));
    let _p2 = poly.add_point(Point::new(100.0, 100.0));
    let _p3 = poly.add_point(Point::new(100.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low, 0, p0, &poly, ScanDirection::horizontal(),
    );
    let dir = side.direction();
    assert!((dir.x() - 0.0).abs() < 1e-10);
    assert!((dir.y() - 100.0).abs() < 1e-10);
}

#[test]
fn scanline_intersect_with_scan_direction() {
    // Vertical side from (10, 0) to (10, 100)
    let mut poly = Polyline::new();
    let p0 = poly.add_point(Point::new(10.0, 0.0));
    let _p1 = poly.add_point(Point::new(10.0, 100.0));
    let _p2 = poly.add_point(Point::new(110.0, 100.0));
    let _p3 = poly.add_point(Point::new(110.0, 0.0));
    poly.set_closed(true);

    let side = ObstacleSide::from_polyline_point(
        SideType::Low, 0, p0, &poly, ScanDirection::horizontal(),
    );

    // Intersect at y=50 (horizontal scan)
    let intersect = side.scanline_intersect(
        Point::new(0.0, 50.0),
        ScanDirection::horizontal(),
    );
    assert!((intersect.x() - 10.0).abs() < 1e-6);
    assert!((intersect.y() - 50.0).abs() < 1e-6);
}

#[test]
fn obstacle_index_alias() {
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        42,
    );
    assert_eq!(side.obstacle_index(), 42);
    assert_eq!(side.obstacle_ordinal(), 42);
}
