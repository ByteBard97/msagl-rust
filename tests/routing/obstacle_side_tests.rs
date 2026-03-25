use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
use msagl_rust::Point;

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
