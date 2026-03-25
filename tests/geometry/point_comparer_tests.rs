use msagl_rust::GeomConstants;

#[test]
fn epsilon_values_match_reference() {
    assert_eq!(GeomConstants::DISTANCE_EPSILON, 1e-6);
    assert_eq!(GeomConstants::SQUARE_OF_DISTANCE_EPSILON, 1e-12);
    assert_eq!(GeomConstants::INTERSECTION_EPSILON, 0.0001);
    assert_eq!(GeomConstants::TOLERANCE, 1e-8);
}

#[test]
fn close_values_within_epsilon() {
    assert!(GeomConstants::close(1.0, 1.0 + 5e-7));
    assert!(GeomConstants::close(1.0, 1.0 - 5e-7));
}

#[test]
fn not_close_values_beyond_epsilon() {
    assert!(!GeomConstants::close(1.0, 1.0 + 2e-6));
    assert!(!GeomConstants::close(1.0, 1.0 - 2e-6));
}

#[test]
fn compare_less_equal_greater() {
    use std::cmp::Ordering;
    assert_eq!(GeomConstants::compare(1.0, 1.0 + 5e-7), Ordering::Equal);
    assert_eq!(GeomConstants::compare(1.0, 2.0), Ordering::Less);
    assert_eq!(GeomConstants::compare(2.0, 1.0), Ordering::Greater);
}

#[test]
fn sign_within_epsilon_is_zero() {
    assert_eq!(GeomConstants::sign(5e-7), 0);
    assert_eq!(GeomConstants::sign(-5e-7), 0);
    assert_eq!(GeomConstants::sign(2e-6), 1);
    assert_eq!(GeomConstants::sign(-2e-6), -1);
}

#[test]
fn round_to_six_decimals() {
    assert_eq!(GeomConstants::round(1.23456789), 1.234568);
    assert_eq!(GeomConstants::round(-0.0000001), 0.0);
    assert_eq!(GeomConstants::round(3.0), 3.0);
}
