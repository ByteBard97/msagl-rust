use crate::geometry::point::Point;

/// One side (edge) of an obstacle's padded boundary.
#[derive(Clone, Debug)]
pub struct ObstacleSide {
    pub obstacle_index: usize,
    pub start: Point,
    pub end: Point,
    /// True for low (left/bottom) sides, false for high (right/top) sides.
    pub is_low_side: bool,
}

impl ObstacleSide {
    pub fn new(obstacle_index: usize, start: Point, end: Point, is_low_side: bool) -> Self {
        Self { obstacle_index, start, end, is_low_side }
    }
}
