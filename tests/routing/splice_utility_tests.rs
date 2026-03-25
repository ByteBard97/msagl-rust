use msagl_rust::routing::splice_utility::SpliceUtility;
use msagl_rust::Point;

#[test]
fn munge_intersect_clamps_below_lo() {
    // intersect below lo -- should be clamped to lo
    let result = SpliceUtility::munge_intersect(0.0, -5.0, 0.0, 10.0);
    assert_eq!(result, 0.0);
}

#[test]
fn munge_intersect_clamps_above_hi() {
    // intersect above hi -- should be clamped to hi
    let result = SpliceUtility::munge_intersect(0.0, 15.0, 0.0, 10.0);
    assert_eq!(result, 10.0);
}

#[test]
fn munge_intersect_within_range_unchanged() {
    let result = SpliceUtility::munge_intersect(0.0, 5.0, 0.0, 10.0);
    assert_eq!(result, 5.0);
}

#[test]
fn munge_intersect_normalises_reversed_bounds() {
    // lo > hi -- implementation should normalise before clamping
    let result = SpliceUtility::munge_intersect(0.0, 3.0, 10.0, 0.0);
    assert_eq!(result, 3.0);
}

#[test]
fn munge_closest_intersection_clamps_both_axes() {
    let site = Point::new(0.0, 0.0);
    let closest = Point::new(-1.0, 20.0);
    let low = Point::new(0.0, 0.0);
    let high = Point::new(10.0, 10.0);

    let result = SpliceUtility::munge_closest_intersection(site, closest, low, high);
    assert_eq!(result.x(), 0.0);
    assert_eq!(result.y(), 10.0);
}

#[test]
fn munge_closest_intersection_inside_bounds_unchanged() {
    let site = Point::new(0.0, 0.0);
    let closest = Point::new(5.0, 5.0);
    let low = Point::new(0.0, 0.0);
    let high = Point::new(10.0, 10.0);

    let result = SpliceUtility::munge_closest_intersection(site, closest, low, high);
    assert_eq!(result.x(), 5.0);
    assert_eq!(result.y(), 5.0);
}
