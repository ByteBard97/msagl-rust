pub struct GeomConstants;

const MULT: f64 = 1_000_000.0;

impl GeomConstants {
    pub const DISTANCE_EPSILON: f64 = 1e-6;

    #[inline]
    pub fn round(v: f64) -> f64 {
        (v * MULT).round() / MULT
    }
}
