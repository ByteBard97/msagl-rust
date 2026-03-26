use crate::geometry::point::Point;

/// A floating port attached to an obstacle.
///
/// Represents a connection point on an obstacle's boundary where an edge
/// can originate or terminate.
#[derive(Clone, Debug)]
pub struct FloatingPort {
    /// Index of the obstacle this port belongs to.
    pub obstacle_index: usize,
    /// Location of the port in 2D space.
    pub location: Point,
}

impl FloatingPort {
    pub fn new(obstacle_index: usize, location: Point) -> Self {
        Self {
            obstacle_index,
            location,
        }
    }
}
