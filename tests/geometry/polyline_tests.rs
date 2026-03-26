use approx::assert_abs_diff_eq;
use msagl_rust::{Point, Polyline};

#[test]
fn empty_polyline() {
    let poly = Polyline::new();
    assert_eq!(poly.count(), 0);
    assert!(poly.start_key().is_none());
    assert!(poly.end_key().is_none());
}

#[test]
fn add_single_point() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(1.0, 2.0));
    assert_eq!(poly.count(), 1);
    assert!(poly.start_key().is_some());
    assert_eq!(poly.start_key(), poly.end_key());
}

#[test]
fn add_multiple_points() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(1.0, 0.0));
    poly.add_point(Point::new(1.0, 1.0));
    assert_eq!(poly.count(), 3);
}

#[test]
fn start_and_end_points() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(5.0, 5.0));
    assert_eq!(poly.start(), Point::new(0.0, 0.0));
    assert_eq!(poly.end(), Point::new(5.0, 5.0));
}

#[test]
fn iterate_points() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(1.0, 0.0));
    poly.add_point(Point::new(1.0, 1.0));
    let points: Vec<Point> = poly.points().collect();
    assert_eq!(points.len(), 3);
    assert_eq!(points[0], Point::new(0.0, 0.0));
    assert_eq!(points[1], Point::new(1.0, 0.0));
    assert_eq!(points[2], Point::new(1.0, 1.0));
}

#[test]
fn iterate_polyline_points_with_keys() {
    let mut poly = Polyline::new();
    let k0 = poly.add_point(Point::new(0.0, 0.0));
    let k1 = poly.add_point(Point::new(1.0, 0.0));
    let k2 = poly.add_point(Point::new(1.0, 1.0));
    assert_eq!(poly.next_key(k0), Some(k1));
    assert_eq!(poly.next_key(k1), Some(k2));
    assert_eq!(poly.next_key(k2), None);
    assert_eq!(poly.prev_key(k2), Some(k1));
    assert_eq!(poly.prev_key(k1), Some(k0));
    assert_eq!(poly.prev_key(k0), None);
}

#[test]
fn point_at_key() {
    let mut poly = Polyline::new();
    let k = poly.add_point(Point::new(3.0, 4.0));
    assert_eq!(poly.point_at(k), Point::new(3.0, 4.0));
}

#[test]
fn bounding_box() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 2.0));
    poly.add_point(Point::new(5.0, 0.0));
    poly.add_point(Point::new(3.0, 7.0));
    let bb = poly.bounding_box();
    assert_eq!(bb.left(), 0.0);
    assert_eq!(bb.bottom(), 0.0);
    assert_eq!(bb.right(), 5.0);
    assert_eq!(bb.top(), 7.0);
}

#[test]
fn from_points_constructor() {
    let poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
    ]);
    assert_eq!(poly.count(), 3);
    assert_eq!(poly.start(), Point::new(0.0, 0.0));
}

#[test]
fn prepend_point() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(1.0, 0.0));
    poly.prepend_point(Point::new(0.0, 0.0));
    assert_eq!(poly.start(), Point::new(0.0, 0.0));
    assert_eq!(poly.end(), Point::new(1.0, 0.0));
    assert_eq!(poly.count(), 2);
}

#[test]
fn set_point_at() {
    let mut poly = Polyline::new();
    let k = poly.add_point(Point::new(0.0, 0.0));
    poly.set_point_at(k, Point::new(5.0, 5.0));
    assert_eq!(poly.point_at(k), Point::new(5.0, 5.0));
}

#[test]
fn polyline_length() {
    let poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(3.0, 0.0),
        Point::new(3.0, 4.0),
    ]);
    assert_abs_diff_eq!(poly.length(), 7.0, epsilon = 1e-10);
}

#[test]
fn translate_all_points() {
    let mut poly = Polyline::from_points(&[Point::new(0.0, 0.0), Point::new(1.0, 1.0)]);
    poly.translate(Point::new(10.0, 20.0));
    assert_eq!(poly.start(), Point::new(10.0, 20.0));
    assert_eq!(poly.end(), Point::new(11.0, 21.0));
}

#[test]
fn closed_polyline_wraps() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(0.5, 1.0),
    ]);
    poly.set_closed(true);
    let start = poly.start_key().unwrap();
    let end = poly.end_key().unwrap();
    assert_eq!(poly.next_key(end), Some(start));
    assert_eq!(poly.prev_key(start), Some(end));
}

#[test]
fn closed_polyline_length_includes_closing_edge() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
        Point::new(0.0, 1.0),
    ]);
    poly.set_closed(true);
    assert_abs_diff_eq!(poly.length(), 4.0, epsilon = 1e-10);
}

#[test]
fn remove_start_point() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(2.0, 0.0),
    ]);
    poly.remove_start_point();
    assert_eq!(poly.count(), 2);
    assert_eq!(poly.start(), Point::new(1.0, 0.0));
}

#[test]
fn remove_end_point() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(2.0, 0.0),
    ]);
    poly.remove_end_point();
    assert_eq!(poly.count(), 2);
    assert_eq!(poly.end(), Point::new(1.0, 0.0));
}

#[test]
fn iterate_polyline_points() {
    let mut poly = Polyline::new();
    let k0 = poly.add_point(Point::new(0.0, 0.0));
    let k1 = poly.add_point(Point::new(1.0, 0.0));
    let pps: Vec<_> = poly.polyline_points().collect();
    assert_eq!(pps.len(), 2);
    assert_eq!(pps[0].key, k0);
    assert_eq!(pps[0].point, Point::new(0.0, 0.0));
    assert_eq!(pps[1].key, k1);
}
