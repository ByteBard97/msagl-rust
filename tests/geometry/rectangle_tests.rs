use msagl_rust::{Point, Rectangle};
use approx::assert_abs_diff_eq;

#[test]
fn new_from_left_bottom_right_top() {
    let r = Rectangle::new(1.0, 2.0, 5.0, 6.0);
    assert_eq!(r.left(), 1.0);
    assert_eq!(r.bottom(), 2.0);
    assert_eq!(r.right(), 5.0);
    assert_eq!(r.top(), 6.0);
}

#[test]
fn width_and_height() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 5.0);
    assert_abs_diff_eq!(r.width(), 10.0, epsilon = 1e-10);
    assert_abs_diff_eq!(r.height(), 5.0, epsilon = 1e-10);
}

#[test]
fn center() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 6.0);
    let c = r.center();
    assert_abs_diff_eq!(c.x(), 5.0, epsilon = 1e-10);
    assert_abs_diff_eq!(c.y(), 3.0, epsilon = 1e-10);
}

#[test]
fn corner_points() {
    let r = Rectangle::new(1.0, 2.0, 5.0, 6.0);
    assert_eq!(r.left_bottom(), Point::new(1.0, 2.0));
    assert_eq!(r.right_top(), Point::new(5.0, 6.0));
    assert_eq!(r.left_top(), Point::new(1.0, 6.0));
    assert_eq!(r.right_bottom(), Point::new(5.0, 2.0));
}

#[test]
fn from_two_points() {
    let r = Rectangle::from_points(Point::new(5.0, 6.0), Point::new(1.0, 2.0));
    assert_eq!(r.left(), 1.0);
    assert_eq!(r.bottom(), 2.0);
    assert_eq!(r.right(), 5.0);
    assert_eq!(r.top(), 6.0);
}

#[test]
fn contains_point_inside() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(r.contains(Point::new(5.0, 5.0)));
}

#[test]
fn contains_point_on_boundary() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(r.contains(Point::new(0.0, 5.0)));
    assert!(r.contains(Point::new(10.0, 5.0)));
}

#[test]
fn does_not_contain_point_outside() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(!r.contains(Point::new(11.0, 5.0)));
    assert!(!r.contains(Point::new(-1.0, 5.0)));
}

#[test]
fn contains_with_padding() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(r.contains_with_padding(Point::new(10.5, 5.0), 1.0));
    assert!(!r.contains_with_padding(Point::new(12.0, 5.0), 1.0));
}

#[test]
fn intersects_overlapping() {
    let a = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    let b = Rectangle::new(5.0, 5.0, 15.0, 15.0);
    assert!(a.intersects(&b));
    assert!(b.intersects(&a));
}

#[test]
fn intersects_disjoint() {
    let a = Rectangle::new(0.0, 0.0, 5.0, 5.0);
    let b = Rectangle::new(10.0, 10.0, 15.0, 15.0);
    assert!(!a.intersects(&b));
}

#[test]
fn contains_rect() {
    let outer = Rectangle::new(0.0, 0.0, 20.0, 20.0);
    let inner = Rectangle::new(5.0, 5.0, 10.0, 10.0);
    assert!(outer.contains_rect(&inner));
    assert!(!inner.contains_rect(&outer));
}

#[test]
fn intersection_region() {
    let a = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    let b = Rectangle::new(5.0, 3.0, 15.0, 8.0);
    let i = a.intersection(&b);
    assert_eq!(i.left(), 5.0);
    assert_eq!(i.bottom(), 3.0);
    assert_eq!(i.right(), 10.0);
    assert_eq!(i.top(), 8.0);
}

#[test]
fn add_point_expands_bbox() {
    let mut r = Rectangle::from_point(Point::new(0.0, 0.0));
    r.add_point(Point::new(5.0, 3.0));
    assert_eq!(r.left(), 0.0);
    assert_eq!(r.right(), 5.0);
    assert_eq!(r.bottom(), 0.0);
    assert_eq!(r.top(), 3.0);
}

#[test]
fn add_rect_expands_bbox() {
    let mut a = Rectangle::new(0.0, 0.0, 5.0, 5.0);
    let b = Rectangle::new(3.0, 3.0, 10.0, 10.0);
    a.add_rect(&b);
    assert_eq!(a.left(), 0.0);
    assert_eq!(a.right(), 10.0);
    assert_eq!(a.top(), 10.0);
}

#[test]
fn pad() {
    let mut r = Rectangle::new(2.0, 3.0, 8.0, 7.0);
    r.pad(1.0);
    assert_eq!(r.left(), 1.0);
    assert_eq!(r.bottom(), 2.0);
    assert_eq!(r.right(), 9.0);
    assert_eq!(r.top(), 8.0);
}

#[test]
fn empty_rectangle() {
    let r = Rectangle::empty();
    assert!(r.is_empty());
}
