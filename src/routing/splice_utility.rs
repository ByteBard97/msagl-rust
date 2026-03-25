use crate::geometry::point::Point;

/// Utilities for clamping intersection points to bounding-box limits.
///
/// Ported from `SpliceUtility.ts`.
pub struct SpliceUtility;

impl SpliceUtility {
    /// Clamp `intersect` to the range `[lo, hi]`.
    ///
    /// The bounds are normalised so that `lo <= hi` is always ensured before
    /// clamping, matching the TypeScript implementation's intent of handling
    /// cases where `low_bound` and `high_bound` coordinates may be reversed.
    pub fn munge_intersect(site: f64, intersect: f64, lo: f64, hi: f64) -> f64 {
        let _ = site; // `site` is kept for API symmetry with the TS source
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        intersect.clamp(lo, hi)
    }

    /// Clamp a closest-intersection point within the rectangle defined by
    /// `low_bound` and `high_bound`.
    pub fn munge_closest_intersection(
        site: Point,
        closest: Point,
        low_bound: Point,
        high_bound: Point,
    ) -> Point {
        Point::new(
            Self::munge_intersect(site.x(), closest.x(), low_bound.x(), high_bound.x()),
            Self::munge_intersect(site.y(), closest.y(), low_bound.y(), high_bound.y()),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn munge_intersect_clamps_below_lo() {
        // intersect below lo — should be clamped to lo
        let result = SpliceUtility::munge_intersect(0.0, -5.0, 0.0, 10.0);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn munge_intersect_clamps_above_hi() {
        // intersect above hi — should be clamped to hi
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
        // lo > hi — implementation should normalise before clamping
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
}
