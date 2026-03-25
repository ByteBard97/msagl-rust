use crate::visibility::graph::VertexId;
use super::compass_direction::CompassDirection;

/// Index into the VertexEntry arena used during path search.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VertexEntryIndex(pub usize);

/// Records an entry from a specific direction for a vertex during path search.
/// Ported from VertexEntry.ts.
#[derive(Debug, Clone)]
pub struct VertexEntry {
    /// The vertex this entry is for
    pub vertex: VertexId,
    /// Direction of entry (from previous vertex to this vertex)
    pub direction: CompassDirection,
    /// Previous entry along this path (None for source)
    pub previous_entry: Option<VertexEntryIndex>,
    /// Length of path up to this vertex
    pub length: f64,
    /// Number of bends in path up to this vertex
    pub number_of_bends: u32,
    /// Total cost of path up to this vertex
    pub cost: f64,
    /// Whether further entries from this direction are blocked
    pub is_closed: bool,
}

impl VertexEntry {
    pub fn new(
        vertex: VertexId,
        direction: CompassDirection,
        previous_entry: Option<VertexEntryIndex>,
        length: f64,
        number_of_bends: u32,
        cost: f64,
    ) -> Self {
        Self {
            vertex,
            direction,
            previous_entry,
            length,
            number_of_bends,
            cost,
            is_closed: false,
        }
    }

    /// Update this entry with a better path.
    pub fn reset(
        &mut self,
        previous_entry: Option<VertexEntryIndex>,
        length: f64,
        number_of_bends: u32,
        cost: f64,
    ) {
        self.previous_entry = previous_entry;
        self.length = length;
        self.number_of_bends = number_of_bends;
        self.cost = cost;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_entry_new() {
        let entry = VertexEntry::new(VertexId(5), CompassDirection::East, None, 100.0, 0, 100.0);
        assert_eq!(entry.vertex, VertexId(5));
        assert_eq!(entry.direction, CompassDirection::East);
        assert!(entry.previous_entry.is_none());
        assert!((entry.length - 100.0).abs() < 1e-10);
        assert_eq!(entry.number_of_bends, 0);
        assert!(!entry.is_closed);
    }

    #[test]
    fn vertex_entry_with_previous() {
        let entry = VertexEntry::new(
            VertexId(10),
            CompassDirection::North,
            Some(VertexEntryIndex(3)),
            200.0,
            1,
            208.0,
        );
        assert_eq!(entry.previous_entry, Some(VertexEntryIndex(3)));
        assert_eq!(entry.number_of_bends, 1);
    }

    #[test]
    fn vertex_entry_reset() {
        let mut entry = VertexEntry::new(VertexId(5), CompassDirection::East, None, 100.0, 0, 100.0);
        entry.reset(Some(VertexEntryIndex(7)), 80.0, 1, 84.0);
        assert_eq!(entry.previous_entry, Some(VertexEntryIndex(7)));
        assert!((entry.length - 80.0).abs() < 1e-10);
        assert!((entry.cost - 84.0).abs() < 1e-10);
    }
}
