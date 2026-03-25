use crate::visibility::graph::VertexId;
use super::compass_direction::CompassDirection;

/// Index into the VertexEntry arena used during path search.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VertexEntryIndex(pub usize);

/// Records an entry from a specific direction for a vertex during path search.
///
/// Faithfully ported from VertexEntry.ts / VertexEntry.cs.
///
/// Each vertex can have up to 4 entries (one per compass direction). The direction
/// records how we arrived at this vertex (from previous vertex to this vertex).
/// `Direction::None` (represented as `None`) is used for path sources.
#[derive(Debug, Clone)]
pub struct VertexEntry {
    /// The vertex this entry is for.
    /// Corresponds to TS `Vertex: VisibilityVertexRectilinear`.
    pub vertex: VertexId,
    /// Direction of entry to the vertex for this path (i.e. the direction from
    /// PreviousVertex to this.Vertex). `None` corresponds to TS `Direction.None`
    /// for source entries.
    pub direction: Option<CompassDirection>,
    /// Previous entry along this path; None for a path source.
    /// Corresponds to TS `PreviousEntry: VertexEntry`.
    pub previous_entry: Option<VertexEntryIndex>,
    /// Length of path up to this vertex.
    pub length: f64,
    /// Number of bends in path up to this vertex.
    pub number_of_bends: u32,
    /// Cost of the path up to this vertex.
    pub cost: f64,
    /// Indicates whether we are allowing further entries into this vertex
    /// from this direction.
    pub is_closed: bool,
}

impl VertexEntry {
    /// Create a new VertexEntry.
    ///
    /// Faithfully matches the TS constructor:
    /// ```text
    /// constructor(vertex, prevEntry, length, numberOfBends, cost) {
    ///     this.Vertex = vertex;
    ///     this.Direction = prevEntry != null
    ///         ? CompassVector.DirectionFromPointToPoint(prevEntry.Vertex.point, vertex.point)
    ///         : Direction.None;
    ///     this.ResetEntry(prevEntry, length, numberOfBends, cost);
    /// }
    /// ```
    ///
    /// In the Rust version, direction is computed by the caller (the arena's
    /// `create_entry` method) because we need graph access to resolve vertex points.
    pub fn new(
        vertex: VertexId,
        direction: Option<CompassDirection>,
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
    ///
    /// Faithfully matches TS `ResetEntry(prevEntry, length, numberOfBends, cost)`.
    /// A new prevEntry using the same previous vertex but a different entry to
    /// that vertex is valid here; e.g. we could have prevEntry from S, which in
    /// turn had a prevEntry from E, replaced by prevEntry from S which has a
    /// prevEntry from S.
    pub fn reset_entry(
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

/// Arena allocator for VertexEntry instances during path search.
///
/// In the TS/C# source, VertexEntry objects are heap-allocated and stored
/// directly in the vertex's `VertexEntries[4]` array. In Rust, we use an
/// index-based arena to avoid reference cycles and lifetime complexity.
///
/// The arena owns all VertexEntry instances for a single path search session.
/// After the search completes, the arena is dropped (or cleared for reuse).
pub struct VertexEntryArena {
    entries: Vec<VertexEntry>,
}

impl VertexEntryArena {
    pub fn new() -> Self {
        Self { entries: Vec::new() }
    }

    /// Pre-allocate capacity for expected number of entries.
    pub fn with_capacity(capacity: usize) -> Self {
        Self { entries: Vec::with_capacity(capacity) }
    }

    /// Allocate a new VertexEntry in the arena and return its index.
    ///
    /// The `direction` parameter should be computed by the caller:
    /// - `None` for source entries (no previous vertex)
    /// - `Some(CompassDirection::from_points(prev_point, vertex_point))` otherwise
    ///
    /// This matches the TS constructor which computes direction from
    /// `prevEntry.Vertex.point` to `vertex.point`.
    pub fn create_entry(
        &mut self,
        vertex: VertexId,
        direction: Option<CompassDirection>,
        previous_entry: Option<VertexEntryIndex>,
        length: f64,
        number_of_bends: u32,
        cost: f64,
    ) -> VertexEntryIndex {
        let index = VertexEntryIndex(self.entries.len());
        self.entries.push(VertexEntry::new(
            vertex,
            direction,
            previous_entry,
            length,
            number_of_bends,
            cost,
        ));
        index
    }

    /// Get a reference to a VertexEntry by index.
    pub fn get(&self, index: VertexEntryIndex) -> &VertexEntry {
        &self.entries[index.0]
    }

    /// Get a mutable reference to a VertexEntry by index.
    pub fn get_mut(&mut self, index: VertexEntryIndex) -> &mut VertexEntry {
        &mut self.entries[index.0]
    }

    /// Get the vertex that a previous entry came from (the "PreviousVertex").
    ///
    /// Matches TS `get PreviousVertex(): VisibilityVertexRectilinear`.
    pub fn previous_vertex(&self, index: VertexEntryIndex) -> Option<VertexId> {
        let entry = self.get(index);
        entry.previous_entry.map(|prev_idx| self.get(prev_idx).vertex)
    }

    /// Number of entries in the arena.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the arena is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Clear all entries (for reuse between path searches).
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

impl Default for VertexEntryArena {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vertex_entry_new_source() {
        // Source entry has no direction and no previous entry
        let entry = VertexEntry::new(
            VertexId(0),
            None, // Direction.None for source
            None, // no previous entry
            0.0,
            0,
            0.0,
        );
        assert_eq!(entry.vertex, VertexId(0));
        assert_eq!(entry.direction, None);
        assert_eq!(entry.previous_entry, None);
        assert_eq!(entry.length, 0.0);
        assert_eq!(entry.number_of_bends, 0);
        assert_eq!(entry.cost, 0.0);
        assert!(!entry.is_closed);
    }

    #[test]
    fn test_vertex_entry_new_with_direction() {
        let entry = VertexEntry::new(
            VertexId(1),
            Some(CompassDirection::East),
            Some(VertexEntryIndex(0)),
            10.0,
            1,
            12.0,
        );
        assert_eq!(entry.vertex, VertexId(1));
        assert_eq!(entry.direction, Some(CompassDirection::East));
        assert_eq!(entry.previous_entry, Some(VertexEntryIndex(0)));
        assert_eq!(entry.length, 10.0);
        assert_eq!(entry.number_of_bends, 1);
        assert_eq!(entry.cost, 12.0);
        assert!(!entry.is_closed);
    }

    #[test]
    fn test_vertex_entry_reset() {
        let mut entry = VertexEntry::new(
            VertexId(1),
            Some(CompassDirection::East),
            Some(VertexEntryIndex(0)),
            10.0,
            1,
            12.0,
        );
        // Reset with a better path (different previous entry, same direction)
        entry.reset_entry(
            Some(VertexEntryIndex(2)),
            8.0,
            0,
            8.0,
        );
        assert_eq!(entry.previous_entry, Some(VertexEntryIndex(2)));
        assert_eq!(entry.length, 8.0);
        assert_eq!(entry.number_of_bends, 0);
        assert_eq!(entry.cost, 8.0);
        // direction and vertex unchanged
        assert_eq!(entry.direction, Some(CompassDirection::East));
        assert_eq!(entry.vertex, VertexId(1));
    }

    #[test]
    fn test_vertex_entry_is_closed() {
        let mut entry = VertexEntry::new(
            VertexId(0),
            Some(CompassDirection::North),
            None,
            0.0,
            0,
            0.0,
        );
        assert!(!entry.is_closed);
        entry.is_closed = true;
        assert!(entry.is_closed);
    }

    #[test]
    fn test_arena_create_and_get() {
        let mut arena = VertexEntryArena::new();
        assert!(arena.is_empty());
        assert_eq!(arena.len(), 0);

        let idx0 = arena.create_entry(
            VertexId(0),
            None,
            None,
            0.0,
            0,
            0.0,
        );
        assert_eq!(idx0, VertexEntryIndex(0));
        assert_eq!(arena.len(), 1);

        let idx1 = arena.create_entry(
            VertexId(1),
            Some(CompassDirection::East),
            Some(idx0),
            10.0,
            0,
            10.0,
        );
        assert_eq!(idx1, VertexEntryIndex(1));
        assert_eq!(arena.len(), 2);

        let e0 = arena.get(idx0);
        assert_eq!(e0.vertex, VertexId(0));
        assert_eq!(e0.direction, None);

        let e1 = arena.get(idx1);
        assert_eq!(e1.vertex, VertexId(1));
        assert_eq!(e1.direction, Some(CompassDirection::East));
        assert_eq!(e1.previous_entry, Some(idx0));
    }

    #[test]
    fn test_arena_previous_vertex() {
        let mut arena = VertexEntryArena::new();

        // Source entry — no previous vertex
        let idx0 = arena.create_entry(VertexId(5), None, None, 0.0, 0, 0.0);
        assert_eq!(arena.previous_vertex(idx0), None);

        // Entry with previous — previous vertex should be VertexId(5)
        let idx1 = arena.create_entry(
            VertexId(10),
            Some(CompassDirection::South),
            Some(idx0),
            20.0,
            0,
            20.0,
        );
        assert_eq!(arena.previous_vertex(idx1), Some(VertexId(5)));

        // Chain: idx2 -> idx1 -> idx0
        let idx2 = arena.create_entry(
            VertexId(15),
            Some(CompassDirection::East),
            Some(idx1),
            30.0,
            1,
            34.0,
        );
        assert_eq!(arena.previous_vertex(idx2), Some(VertexId(10)));
    }

    #[test]
    fn test_arena_get_mut() {
        let mut arena = VertexEntryArena::new();
        let idx = arena.create_entry(
            VertexId(0),
            Some(CompassDirection::North),
            None,
            0.0,
            0,
            0.0,
        );

        // Mutate via get_mut
        arena.get_mut(idx).is_closed = true;
        assert!(arena.get(idx).is_closed);

        // Reset entry via get_mut
        arena.get_mut(idx).reset_entry(None, 5.0, 1, 7.0);
        assert_eq!(arena.get(idx).length, 5.0);
        assert_eq!(arena.get(idx).number_of_bends, 1);
        assert_eq!(arena.get(idx).cost, 7.0);
    }

    #[test]
    fn test_arena_clear() {
        let mut arena = VertexEntryArena::new();
        arena.create_entry(VertexId(0), None, None, 0.0, 0, 0.0);
        arena.create_entry(VertexId(1), Some(CompassDirection::East), None, 5.0, 0, 5.0);
        assert_eq!(arena.len(), 2);

        arena.clear();
        assert_eq!(arena.len(), 0);
        assert!(arena.is_empty());
    }

    #[test]
    fn test_arena_with_capacity() {
        let arena = VertexEntryArena::with_capacity(100);
        assert_eq!(arena.len(), 0);
        assert!(arena.is_empty());
    }

    #[test]
    fn test_direction_index_matches_vertex_entries_array() {
        // Verify the direction index mapping matches the [4] array layout:
        // North=0, East=1, South=2, West=3
        // This is critical for VertexEntries[CompassVector.ToIndex(entry.Direction)]
        assert_eq!(CompassDirection::North.index(), 0);
        assert_eq!(CompassDirection::East.index(), 1);
        assert_eq!(CompassDirection::South.index(), 2);
        assert_eq!(CompassDirection::West.index(), 3);
    }

    #[test]
    fn test_four_direction_entries_per_vertex() {
        // Simulate the TS pattern: a vertex can have up to 4 entries,
        // one per direction. The graph stores Option<VertexEntryIndex>
        // for each direction slot.
        let mut arena = VertexEntryArena::new();
        let vertex = VertexId(42);

        let mut vertex_entries: [Option<VertexEntryIndex>; 4] = [None; 4];

        // Entry from North
        let north_idx = arena.create_entry(
            vertex,
            Some(CompassDirection::North),
            None,
            10.0,
            0,
            10.0,
        );
        vertex_entries[CompassDirection::North.index()] = Some(north_idx);

        // Entry from East
        let east_idx = arena.create_entry(
            vertex,
            Some(CompassDirection::East),
            None,
            15.0,
            1,
            19.0,
        );
        vertex_entries[CompassDirection::East.index()] = Some(east_idx);

        // Verify we can look up entries by direction
        assert!(vertex_entries[CompassDirection::North.index()].is_some());
        assert!(vertex_entries[CompassDirection::East.index()].is_some());
        assert!(vertex_entries[CompassDirection::South.index()].is_none());
        assert!(vertex_entries[CompassDirection::West.index()].is_none());

        let n_entry = arena.get(vertex_entries[CompassDirection::North.index()].unwrap());
        assert_eq!(n_entry.cost, 10.0);
        let e_entry = arena.get(vertex_entries[CompassDirection::East.index()].unwrap());
        assert_eq!(e_entry.cost, 19.0);
    }

    #[test]
    fn test_path_reconstruction_chain() {
        // Simulate a path: source(0) -> v1(East) -> v2(East) -> v3(South)
        let mut arena = VertexEntryArena::new();

        let src_idx = arena.create_entry(VertexId(0), None, None, 0.0, 0, 0.0);
        let v1_idx = arena.create_entry(
            VertexId(1),
            Some(CompassDirection::East),
            Some(src_idx),
            10.0,
            0,
            10.0,
        );
        let v2_idx = arena.create_entry(
            VertexId(2),
            Some(CompassDirection::East),
            Some(v1_idx),
            20.0,
            0,
            20.0,
        );
        let v3_idx = arena.create_entry(
            VertexId(3),
            Some(CompassDirection::South),
            Some(v2_idx),
            30.0,
            1,
            34.0,
        );

        // Walk back from v3 to source
        let mut path = Vec::new();
        let mut current = Some(v3_idx);
        while let Some(idx) = current {
            let entry = arena.get(idx);
            path.push(entry.vertex);
            current = entry.previous_entry;
        }
        path.reverse();

        assert_eq!(path, vec![VertexId(0), VertexId(1), VertexId(2), VertexId(3)]);
    }

    #[test]
    fn test_reset_entry_preserves_direction_and_vertex() {
        // Matches TS behavior: ResetEntry only changes prevEntry, length,
        // bends, cost. Direction and Vertex are set once in the constructor.
        let mut arena = VertexEntryArena::new();
        let idx = arena.create_entry(
            VertexId(7),
            Some(CompassDirection::West),
            Some(VertexEntryIndex(99)),
            50.0,
            3,
            100.0,
        );

        arena.get_mut(idx).reset_entry(
            Some(VertexEntryIndex(88)),
            40.0,
            2,
            80.0,
        );

        let entry = arena.get(idx);
        assert_eq!(entry.vertex, VertexId(7));
        assert_eq!(entry.direction, Some(CompassDirection::West));
        assert_eq!(entry.previous_entry, Some(VertexEntryIndex(88)));
        assert_eq!(entry.length, 40.0);
        assert_eq!(entry.number_of_bends, 2);
        assert_eq!(entry.cost, 80.0);
    }
}
