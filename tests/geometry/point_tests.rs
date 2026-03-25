use msagl_rust::{Point, GeomConstants};
use approx::assert_abs_diff_eq;

#[test]
fn construction_rounds_to_six_decimals() {
    let p = Point::new(1.23456789, -0.0000001);
    assert_eq!(p.x(), 1.234568);
    assert_eq!(p.y(), 0.0);
}

#[test]
fn zero_point() {
    let p = Point::ORIGIN;
    assert_eq!(p.x(), 0.0);
    assert_eq!(p.y(), 0.0);
}

#[test]
fn add_points() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(3.0, 4.0);
    let c = a + b;
    assert_eq!(c.x(), 4.0);
    assert_eq!(c.y(), 6.0);
}

#[test]
fn sub_points() {
    let a = Point::new(5.0, 7.0);
    let b = Point::new(2.0, 3.0);
    let c = a - b;
    assert_eq!(c.x(), 3.0);
    assert_eq!(c.y(), 4.0);
}

#[test]
fn mul_scalar() {
    let p = Point::new(3.0, 4.0);
    let scaled = p * 2.0;
    assert_eq!(scaled.x(), 6.0);
    assert_eq!(scaled.y(), 8.0);
}

#[test]
fn div_scalar() {
    let p = Point::new(6.0, 8.0);
    let scaled = p / 2.0;
    assert_eq!(scaled.x(), 3.0);
    assert_eq!(scaled.y(), 4.0);
}

#[test]
fn negate() {
    let p = Point::new(3.0, -4.0);
    let n = -p;
    assert_eq!(n.x(), -3.0);
    assert_eq!(n.y(), 4.0);
}

#[test]
fn length_3_4_5_triangle() {
    let p = Point::new(3.0, 4.0);
    assert_abs_diff_eq!(p.length(), 5.0, epsilon = 1e-10);
}

#[test]
fn length_squared() {
    let p = Point::new(3.0, 4.0);
    assert_abs_diff_eq!(p.length_squared(), 25.0, epsilon = 1e-10);
}

#[test]
fn l1_manhattan() {
    let p = Point::new(3.0, -4.0);
    assert_abs_diff_eq!(p.l1(), 7.0, epsilon = 1e-10);
}

#[test]
fn dot_product() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(3.0, 4.0);
    assert_abs_diff_eq!(a.dot(b), 11.0, epsilon = 1e-10);
}

#[test]
fn cross_product() {
    let a = Point::new(1.0, 0.0);
    let b = Point::new(0.0, 1.0);
    assert_abs_diff_eq!(Point::cross(a, b), 1.0, epsilon = 1e-10);
}

#[test]
fn normalize_unit_vector() {
    let p = Point::new(3.0, 4.0);
    let n = p.normalize();
    assert_abs_diff_eq!(n.length(), 1.0, epsilon = 1e-6);
    assert_abs_diff_eq!(n.x(), 0.6, epsilon = 1e-6);
    assert_abs_diff_eq!(n.y(), 0.8, epsilon = 1e-6);
}

#[test]
fn rotate_90_ccw() {
    let p = Point::new(1.0, 0.0);
    let r = p.rotate90_ccw();
    assert_abs_diff_eq!(r.x(), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(r.y(), 1.0, epsilon = 1e-6);
}

#[test]
fn rotate_90_cw() {
    let p = Point::new(0.0, 1.0);
    let r = p.rotate90_cw();
    assert_abs_diff_eq!(r.x(), 1.0, epsilon = 1e-6);
    assert_abs_diff_eq!(r.y(), 0.0, epsilon = 1e-6);
}

#[test]
fn close_dist_eps_within() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(1.0 + 5e-7, 2.0 - 3e-7);
    assert!(a.close_to(b));
}

#[test]
fn close_dist_eps_beyond() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(1.0 + 1e-3, 2.0);
    assert!(!a.close_to(b));
}

#[test]
fn middle_of_two_points() {
    let a = Point::new(0.0, 0.0);
    let b = Point::new(4.0, 6.0);
    let m = Point::middle(a, b);
    assert_eq!(m.x(), 2.0);
    assert_eq!(m.y(), 3.0);
}

#[test]
fn line_line_intersection() {
    let result = Point::line_line_intersection(
        Point::new(0.0, 1.0), Point::new(4.0, 1.0),
        Point::new(2.0, 0.0), Point::new(2.0, 4.0),
    );
    assert!(result.is_some());
    let p = result.unwrap();
    assert_abs_diff_eq!(p.x(), 2.0, epsilon = 1e-6);
    assert_abs_diff_eq!(p.y(), 1.0, epsilon = 1e-6);
}

#[test]
fn parallel_lines_no_intersection() {
    let result = Point::line_line_intersection(
        Point::new(0.0, 0.0), Point::new(1.0, 0.0),
        Point::new(0.0, 1.0), Point::new(1.0, 1.0),
    );
    assert!(result.is_none());
}

#[test]
fn hashing_identical_points() {
    use std::collections::HashSet;
    let mut set = HashSet::new();
    set.insert(Point::new(1.0, 2.0));
    set.insert(Point::new(1.0, 2.0));
    assert_eq!(set.len(), 1);
}

#[test]
fn ordering_lexicographic() {
    let a = Point::new(1.0, 5.0);
    let b = Point::new(2.0, 1.0);
    let c = Point::new(1.0, 3.0);
    assert!(a < b);
    assert!(c < a);
}
