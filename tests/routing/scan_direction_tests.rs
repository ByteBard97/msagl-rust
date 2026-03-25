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
