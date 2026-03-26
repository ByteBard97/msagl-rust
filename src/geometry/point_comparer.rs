use std::cmp::Ordering;

pub struct GeomConstants;

const MULT: f64 = 1_000_000.0; // 10^6

impl GeomConstants {
    pub const DISTANCE_EPSILON_PRECISION: usize = 6;
    pub const DISTANCE_EPSILON: f64 = 1e-6;
    /// Half of DISTANCE_EPSILON, used by PointComparer for coordinate comparisons.
    /// Matches C# PointComparer.DifferenceEpsilon = DistanceEpsilon / 2.
    pub const DIFFERENCE_EPSILON: f64 = Self::DISTANCE_EPSILON / 2.0;
    pub const SQUARE_OF_DISTANCE_EPSILON: f64 = 1e-12;
    pub const INTERSECTION_EPSILON: f64 = 0.0001;
    pub const TOLERANCE: f64 = 1e-8;

    #[inline]
    pub fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= Self::DISTANCE_EPSILON
    }

    /// Compare two coordinate values using DIFFERENCE_EPSILON (DistanceEpsilon / 2).
    /// Matches C# PointComparer.Compare which uses DifferenceEpsilon, not DistanceEpsilon.
    #[inline]
    pub fn compare(a: f64, b: f64) -> Ordering {
        if a + Self::DIFFERENCE_EPSILON < b {
            Ordering::Less
        } else if b + Self::DIFFERENCE_EPSILON < a {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }

    #[inline]
    pub fn sign(value: f64) -> i32 {
        if value > Self::DISTANCE_EPSILON {
            1
        } else if value < -Self::DISTANCE_EPSILON {
            -1
        } else {
            0
        }
    }

    #[inline]
    pub fn round(v: f64) -> f64 {
        (v * MULT).round() / MULT
    }
}
