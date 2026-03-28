use super::compass_direction::Direction;
use crate::visibility::graph::VertexId;

/// Index into the VertexEntry arena used during path search.
///
/// The arena is owned by `SsstRectilinearPath` and lives for the duration of a
/// single path search. Indices are stable within one search.
///
/// Corresponds to C# object references on `VertexEntry` — approved Rust
/// adaptation: "Object references with GC → Index-based arenas."
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VertexEntryIndex(pub usize);

/// Records an entry from a specific direction for a vertex during path search.
///
/// Faithful port of C# `VertexEntry` (VertexEntry.cs) and TS `VertexEntry`
/// (VertexEntry.ts).
///
/// Each `VisibilityVertexRectilinear` stores up to four of these, one per
/// compass direction. They form a linked list back to the path source via
/// `previous_entry`.
///
/// **Rust adaptation:** C# stores object references for `Vertex`,
/// `PreviousEntry`, and `PreviousVertex`. Rust uses `VertexId` (arena index)
/// and `Option<VertexEntryIndex>` (index into the path-search arena).
#[derive(Debug, Clone)]
pub struct VertexEntry {
    /// The vertex this entry arrives at.
    /// Matches C# `VertexEntry.Vertex`.
    pub vertex: VertexId,

    /// Direction of entry into `vertex` (i.e. the direction from the previous
    /// vertex to this vertex).
    /// Matches C# `VertexEntry.Direction`.
    /// `Direction::NONE` for the path source.
    pub direction: Direction,

    /// The previous `VertexEntry` along this path.
    /// `None` for a path source.
    /// Matches C# `VertexEntry.PreviousEntry`.
    pub previous_entry: Option<VertexEntryIndex>,

    /// Cumulative path length up to this vertex.
    /// Matches C# `VertexEntry.Length`.
    pub length: f64,

    /// Number of direction changes (bends) in the path up to this vertex.
    /// Matches C# `VertexEntry.NumberOfBends`.
    pub number_of_bends: i32,

    /// Total path cost up to this vertex (includes heuristic).
    /// Matches C# `VertexEntry.Cost`.
    pub cost: f64,

    /// Whether this entry is closed (no further relaxation allowed from this
    /// direction).
    /// Matches C# `VertexEntry.IsClosed`.
    pub is_closed: bool,
}

impl VertexEntry {
    /// Create a new entry.
    ///
    /// Matches C# `new VertexEntry(vertex, prevEntry, length, numberOfBends, cost)`.
    /// Direction is computed from `prev_direction` (the direction of arrival
    /// from the previous vertex) — callers supply it directly since in the
    /// arena model we compute direction at the call site.
    pub fn new(
        vertex: VertexId,
        direction: Direction,
        previous_entry: Option<VertexEntryIndex>,
        length: f64,
        number_of_bends: i32,
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

    /// Update this entry with a better path from the same previous vertex.
    ///
    /// Matches C# `VertexEntry.ResetEntry(prevEntry, length, numberOfBends, cost)`.
    /// Only legal to call when the previous vertex is the same but the cost
    /// improved; in C# there are debug assertions that enforce this.
    pub fn reset_entry(
        &mut self,
        previous_entry: Option<VertexEntryIndex>,
        length: f64,
        number_of_bends: i32,
        cost: f64,
    ) {
        self.previous_entry = previous_entry;
        self.length = length;
        self.number_of_bends = number_of_bends;
        self.cost = cost;
    }
}
