use super::obstacle_side::ObstacleSide;

/// Tracks the low and high neighbor sides during scanline traversal.
///
/// Faithful port of NeighborSides.ts.
///
/// For each direction (low and high), stores:
/// - The actual neighbor side (HighObstacleSide for low, LowObstacleSide for high)
/// - The overlap-ending side (the last side of the same type as the search direction
///   that we pass through into open space), if any
///
/// Group-related fields are deferred per the PRD.
#[derive(Debug, Clone, Default)]
pub struct NeighborSides {
    /// The HighObstacleSide of the low neighbor.
    /// Matches TS: `NeighborSides.LowNeighbor.item`
    low_neighbor: Option<ObstacleSide>,

    /// A LowObstacleSide that we pass through in the low direction into open space.
    /// This is the overlap-ending side in the low direction.
    /// Matches TS: `NeighborSides.LowOverlapEnd.item`
    low_overlap_end: Option<ObstacleSide>,

    /// The LowObstacleSide of the high neighbor.
    /// Matches TS: `NeighborSides.HighNeighbor.item`
    high_neighbor: Option<ObstacleSide>,

    /// A HighObstacleSide that we pass through in the high direction into open space.
    /// This is the overlap-ending side in the high direction.
    /// Matches TS: `NeighborSides.HighOverlapEnd.item`
    high_overlap_end: Option<ObstacleSide>,
}

impl NeighborSides {
    pub fn new() -> Self {
        Self::default()
    }

    /// Set sides for a given direction.
    ///
    /// Faithful port of TS `NeighborSides.SetSides(dir, neighborNode, overlapEndNode, interveningGroupSide)`.
    ///
    /// `is_ascending`: true if direction is ascending (East/North in TS IsAscending).
    pub fn set_sides_for_direction(
        &mut self,
        is_ascending: bool,
        neighbor: ObstacleSide,
        overlap_end: Option<ObstacleSide>,
    ) {
        if is_ascending {
            self.high_neighbor = Some(neighbor);
            self.high_overlap_end = overlap_end;
        } else {
            self.low_neighbor = Some(neighbor);
            self.low_overlap_end = overlap_end;
        }
    }

    /// Legacy setter for backward compatibility.
    pub fn set_sides(&mut self, low: Option<ObstacleSide>, high: Option<ObstacleSide>) {
        self.low_neighbor = low;
        self.high_neighbor = high;
    }

    pub fn clear(&mut self) {
        self.low_neighbor = None;
        self.low_overlap_end = None;
        self.high_neighbor = None;
        self.high_overlap_end = None;
    }

    pub fn low_neighbor(&self) -> Option<&ObstacleSide> {
        self.low_neighbor.as_ref()
    }

    pub fn high_neighbor(&self) -> Option<&ObstacleSide> {
        self.high_neighbor.as_ref()
    }

    pub fn low_overlap_end(&self) -> Option<&ObstacleSide> {
        self.low_overlap_end.as_ref()
    }

    pub fn high_overlap_end(&self) -> Option<&ObstacleSide> {
        self.high_overlap_end.as_ref()
    }
}
