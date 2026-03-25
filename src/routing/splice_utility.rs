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