use std::collections::{BTreeSet, HashMap};
use crate::geometry::point::Point;
use crate::routing::compass_direction::CompassDirection;
use crate::routing::vertex_entry::VertexEntryIndex;
use super::edge::VisEdge;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

/// Per-vertex data in the visibility graph.
///
/// Corresponds to `VisibilityVertex` + `VisibilityVertexRectilinear` in the
/// TS/C# source. In those implementations, `VisibilityVertexRectilinear`
/// extends `VisibilityVertex` adding `VertexEntries[4]`. In Rust, we flatten
/// both into a single struct.
pub struct VertexData {
    pub point: Point,
    pub out_edges: BTreeSet<VisEdge>,
    pub in_edges: Vec<VertexId>,
    pub distance: f64,
    pub prev_vertex: Option<VertexId>,
    pub is_terminal: bool,
    /// Whether this vertex is a shortest-path terminal.
    /// Matches TS `VisibilityVertex._isShortestPathTerminal`.
    pub is_shortest_path_terminal: bool,
    /// Direction-indexed entries for the rectilinear path search.
    /// `vertex_entries[dir.index()]` holds the VertexEntryIndex for direction `dir`.
    /// When all 4 slots are `None`, this is equivalent to the TS null check
    /// `if (vertex.VertexEntries == null)`.
    ///
    /// Matches TS `VisibilityVertexRectilinear.VertexEntries: VertexEntry[]`.
    pub vertex_entries: [Option<VertexEntryIndex>; 4],
}

pub struct VisibilityGraph {
    vertices: Vec<VertexData>,
    point_to_vertex: HashMap<Point, VertexId>,
}

impl VisibilityGraph {
    pub fn new() -> Self {
        Self { vertices: Vec::new(), point_to_vertex: HashMap::new() }
    }

    pub fn add_vertex(&mut self, point: Point) -> VertexId {
        if let Some(&id) = self.point_to_vertex.get(&point) {
            return id;
        }
        let id = VertexId(self.vertices.len());
        self.vertices.push(VertexData {
            point,
            out_edges: BTreeSet::new(),
            in_edges: Vec::new(),
            distance: f64::INFINITY,
            prev_vertex: None,
            is_terminal: false,
            is_shortest_path_terminal: false,
            vertex_entries: [None; 4],
        });
        self.point_to_vertex.insert(point, id);
        id
    }

    pub fn find_vertex(&self, point: Point) -> Option<VertexId> {
        self.point_to_vertex.get(&point).copied()
    }

    pub fn find_or_add_vertex(&mut self, point: Point) -> VertexId {
        self.add_vertex(point)
    }

    pub fn add_edge(&mut self, source: VertexId, target: VertexId, weight: f64) {
        self.vertices[source.0].out_edges.insert(VisEdge::new(target, weight));
        self.vertices[target.0].in_edges.push(source);
    }

    /// Add a toll-free (transient) edge used during port splicing.
    pub fn add_toll_free_edge(&mut self, source: VertexId, target: VertexId, weight: f64) {
        let mut edge = VisEdge::new(target, weight);
        edge.is_toll_free = true;
        self.vertices[source.0].out_edges.insert(edge);
        self.vertices[target.0].in_edges.push(source);
    }

    pub fn remove_edge(&mut self, source: VertexId, target: VertexId) {
        self.vertices[source.0].out_edges.remove(&VisEdge::new(target, 0.0));
        self.vertices[target.0].in_edges.retain(|&v| v != source);
    }

    pub fn point(&self, v: VertexId) -> Point {
        self.vertices[v.0].point
    }

    pub fn vertex(&self, v: VertexId) -> &VertexData {
        &self.vertices[v.0]
    }

    pub fn vertex_mut(&mut self, v: VertexId) -> &mut VertexData {
        &mut self.vertices[v.0]
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn out_degree(&self, v: VertexId) -> usize {
        self.vertices[v.0].out_edges.len()
    }

    pub fn in_degree(&self, v: VertexId) -> usize {
        self.vertices[v.0].in_edges.len()
    }

    pub fn out_edges(&self, v: VertexId) -> impl Iterator<Item = &VisEdge> {
        self.vertices[v.0].out_edges.iter()
    }

    /// Find an edge from `source` to `target`, returning its weight if it exists.
    pub fn find_edge(&self, source: VertexId, target: VertexId) -> Option<&VisEdge> {
        self.vertices[source.0].out_edges.get(&VisEdge::new(target, 0.0))
    }

    /// Find an edge between two points (checks both directions since edges are ascending).
    pub fn find_edge_pp(&self, a: Point, b: Point) -> Option<(VertexId, VertexId, f64)> {
        let va = self.find_vertex(a)?;
        let vb = self.find_vertex(b)?;
        if let Some(e) = self.find_edge(va, vb) {
            return Some((va, vb, e.weight));
        }
        if let Some(e) = self.find_edge(vb, va) {
            return Some((vb, va, e.weight));
        }
        None
    }

    /// Check if a vertex has any out-edges or in-edges.
    pub fn degree(&self, v: VertexId) -> usize {
        self.vertices[v.0].out_edges.len() + self.vertices[v.0].in_edges.len()
    }

    /// Check if a vertex has any VertexEntries set.
    ///
    /// Matches the TS pattern `if (vertex.VertexEntries != null)`.
    /// In TS, the array is lazily allocated; in Rust, we check if any slot is Some.
    pub fn has_vertex_entries(&self, v: VertexId) -> bool {
        self.vertices[v.0].vertex_entries.iter().any(|e| e.is_some())
    }

    /// Set a vertex entry by its direction, matching TS `SetVertexEntry(entry)`.
    ///
    /// The TS method does:
    /// ```text
    /// SetVertexEntry(entry: VertexEntry) {
    ///     if (this.VertexEntries == null) { this.VertexEntries = new Array(4); }
    ///     this.VertexEntries[CompassVector.ToIndex(entry.Direction)] = entry;
    /// }
    /// ```
    ///
    /// Here, `dir` is the entry's direction (must be Some -- source entries with
    /// Direction.None are not stored in the vertex_entries array).
    pub fn set_vertex_entry(&mut self, v: VertexId, dir: CompassDirection, entry: Option<VertexEntryIndex>) {
        self.vertices[v.0].vertex_entries[dir.index()] = entry;
    }

    /// Get the entry index for a vertex from a specific direction.
    pub fn vertex_entry(&self, v: VertexId, dir: CompassDirection) -> Option<VertexEntryIndex> {
        self.vertices[v.0].vertex_entries[dir.index()]
    }

    /// Get all 4 vertex entries as a slice.
    pub fn vertex_entries(&self, v: VertexId) -> &[Option<VertexEntryIndex>; 4] {
        &self.vertices[v.0].vertex_entries
    }

    /// Remove all vertex entries for a single vertex.
    /// Matches TS `RemoveVertexEntries()` which sets `VertexEntries = null`.
    pub fn remove_vertex_entries(&mut self, v: VertexId) {
        self.vertices[v.0].vertex_entries = [None; 4];
    }

    /// Clear all vertex entries across all vertices (for reuse between path searches).
    pub fn clear_vertex_entries(&mut self) {
        for vd in &mut self.vertices {
            vd.vertex_entries = [None; 4];
        }
    }

    /// Iterate over all vertex IDs.
    pub fn vertex_ids(&self) -> impl Iterator<Item = VertexId> + '_ {
        (0..self.vertices.len()).map(VertexId)
    }

    /// Get in-edge sources for a vertex.
    pub fn in_edges(&self, v: VertexId) -> &[VertexId] {
        &self.vertices[v.0].in_edges
    }
}

impl Default for VisibilityGraph {
    fn default() -> Self { Self::new() }
}
