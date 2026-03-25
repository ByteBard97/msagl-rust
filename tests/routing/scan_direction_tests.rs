use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;
use std::cmp::Ordering;

#[test]
fn horizontal_scan_projects_x_as_primary() {
    let sd = ScanDirection::horizontal();
    let p = Point::new(3.0, 7.0);
    assert_eq!(sd.coord(p), 3.0);
    assert_eq!(sd.perp_coord(p), 7.0);
}

#[test]
fn vertical_scan_projects_y_as_primary() {
    let sd = ScanDirection::vertical();
    let p = Point::new(3.0, 7.0);
    assert_eq!(sd.coord(p), 7.0);
    assert_eq!(sd.perp_coord(p), 3.0);
}

#[test]
fn compare_by_perp_then_primary() {
    let sd = ScanDirection::horizontal();
    let a = Point::new(1.0, 5.0);
    let b = Point::new(3.0, 5.0);
    let c = Point::new(1.0, 10.0);
    assert_eq!(sd.compare(a, b), Ordering::Less);
    assert_eq!(sd.compare(a, c), Ordering::Less);
}

#[test]
fn is_flat() {
    let sd = ScanDirection::horizontal();
    assert!(sd.is_flat(Point::new(0.0, 5.0), Point::new(10.0, 5.0)));
    assert!(!sd.is_flat(Point::new(0.0, 5.0), Point::new(10.0, 7.0)));
}

#[test]
fn is_perpendicular() {
    let sd = ScanDirection::horizontal();
    assert!(sd.is_perpendicular(Point::new(5.0, 0.0), Point::new(5.0, 10.0)));
    assert!(!sd.is_perpendicular(Point::new(5.0, 0.0), Point::new(7.0, 10.0)));
}

#[test]
fn make_point_horizontal() {
    let sd = ScanDirection::horizontal();
    let p = sd.make_point(3.0, 7.0); // coord=x=3, perp=y=7
    assert_eq!(p, Point::new(3.0, 7.0));
}

#[test]
fn make_point_vertical() {
    let sd = ScanDirection::vertical();
    let p = sd.make_point(3.0, 7.0); // coord=y=3, perp=x=7
    assert_eq!(p, Point::new(7.0, 3.0));
}

// --- Tests extracted from inline #[cfg(test)] in scan_direction.rs ---

#[test]
fn horizontal_scan_is_horizontal() {
    let dir = ScanDirection::horizontal();
    assert!(dir.is_horizontal());
    assert!(!dir.is_vertical());
}

#[test]
fn vertical_scan_is_vertical() {
    let dir = ScanDirection::vertical();
    assert!(dir.is_vertical());
    assert!(!dir.is_horizontal());
}

#[test]
fn perpendicular_swaps_direction() {
    let h = ScanDirection::horizontal();
    let v = h.perpendicular();
    assert!(v.is_vertical());
    assert!(v.perpendicular().is_horizontal());
}

#[test]
fn min_max_returns_correct_point() {
    let dir = ScanDirection::horizontal();
    let a = Point::new(5.0, 10.0);
    let b = Point::new(5.0, 20.0);
    // Horizontal scan: perp is Y. a has lower Y, so a is "less"
    assert_eq!(dir.min(a, b), a);
    assert_eq!(dir.max(a, b), b);
}
