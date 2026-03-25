use super::obstacle_side::ObstacleSide;

/// Tracks the low and high neighbor sides during scanline traversal.
/// Ported from NeighborSides.ts — group-related fields and overlap-end nodes deferred.
#[derive(Debug, Clone, Default)]
pub struct NeighborSides {
    low_neighbor: Option<ObstacleSide>,
    high_neighbor: Option<ObstacleSide>,
}

impl NeighborSides {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_sides(&mut self, low: Option<ObstacleSide>, high: Option<ObstacleSide>) {
        self.low_neighbor = low;
        self.high_neighbor = high;
    }

    pub fn clear(&mut self) {
        self.low_neighbor = None;
        self.high_neighbor = None;
    }

    pub fn low_neighbor(&self) -> Option<&ObstacleSide> {
        self.low_neighbor.as_ref()
    }

    pub fn high_neighbor(&self) -> Option<&ObstacleSide> {
        self.high_neighbor.as_ref()
    }
}
