use ordered_float::OrderedFloat;
use crate::geometry::point_comparer::GeomConstants;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Point {
    x: OrderedFloat<f64>,
    y: OrderedFloat<f64>,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x: OrderedFloat(GeomConstants::round(x)),
            y: OrderedFloat(GeomConstants::round(y)),
        }
    }
    pub fn x(&self) -> f64 { self.x.into_inner() }
    pub fn y(&self) -> f64 { self.y.into_inner() }
}
