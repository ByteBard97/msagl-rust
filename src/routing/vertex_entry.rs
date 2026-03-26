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
/// `None` corresponds to TS `Direction.None` for source entries.
#[derive(Debug, Clone)]
pub struct VertexEntry {
    /// The vertex this entry is for.
    pub vertex: VertexId,
    /// Direction of entry to the vertex for this path.
    /// `None` corresponds to TS `Direction.None` for source entries.
    pub direction: Option<CompassDirection>,
    /// Previous entry along this path; None for a path source.
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
    /// Matches TS `ResetEntry(prevEntry, length, numberOfBends, cost)`.
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
    /// Matches TS `get PreviousVertex()`.
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
