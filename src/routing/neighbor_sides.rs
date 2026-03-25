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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::point::Point;
    use crate::routing::obstacle_side::SideType;

    #[test]
    fn neighbor_sides_default_is_empty() {
        let ns = NeighborSides::new();
        assert!(ns.low_neighbor().is_none());
        assert!(ns.high_neighbor().is_none());
    }

    #[test]
    fn set_and_get_sides() {
        let mut ns = NeighborSides::new();
        let low = ObstacleSide::new(
            SideType::Low,
            Point::new(0.0, 0.0),
            Point::new(0.0, 10.0),
            0,
        );
        let high = ObstacleSide::new(
            SideType::High,
            Point::new(20.0, 0.0),
            Point::new(20.0, 10.0),
            1,
        );
        ns.set_sides(Some(low), Some(high));
        assert_eq!(ns.low_neighbor().unwrap().obstacle_ordinal(), 0);
        assert_eq!(ns.high_neighbor().unwrap().obstacle_ordinal(), 1);
    }

    #[test]
    fn clear_removes_sides() {
        let mut ns = NeighborSides::new();
        let low = ObstacleSide::new(
            SideType::Low,
            Point::new(0.0, 0.0),
            Point::new(0.0, 10.0),
            0,
        );
        ns.set_sides(Some(low), None);
        ns.clear();
        assert!(ns.low_neighbor().is_none());
    }
}
