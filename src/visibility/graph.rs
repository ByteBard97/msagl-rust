use super::edge::VisEdge;
use crate::geometry::point::Point;
use crate::routing::compass_direction::CompassDirection;
use crate::routing::vertex_entry::VertexEntryIndex;
use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

pub struct VertexData {
    pub point: Point,
    pub out_edges: BTreeSet<VisEdge>,
    pub in_edges: Vec<VertexId>,
    pub distance: f64,
    pub prev_vertex: Option<VertexId>,
    pub is_terminal: bool,
    pub vertex_entries: [Option<VertexEntryIndex>; 4],
}

pub struct VisibilityGraph {
    vertices: Vec<VertexData>,
    point_to_vertex: HashMap<Point, VertexId>,
}

impl VisibilityGraph {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            point_to_vertex: HashMap::new(),
        }
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
        let target_point = self.vertices[target.0].point;
        self.vertices[source.0]
            .out_edges
            .insert(VisEdge::new(target, target_point, weight));
        self.vertices[target.0].in_edges.push(source);
    }

    /// Add a toll-free (transient) edge used during port splicing.
    pub fn add_toll_free_edge(&mut self, source: VertexId, target: VertexId, weight: f64) {
        let target_point = self.vertices[target.0].point;
        let mut edge = VisEdge::new(target, target_point, weight);
        edge.is_toll_free = true;
        self.vertices[source.0].out_edges.insert(edge);
        self.vertices[target.0].in_edges.push(source);
    }

    pub fn remove_edge(&mut self, source: VertexId, target: VertexId) {
        let target_point = self.vertices[target.0].point;
        self.vertices[source.0]
            .out_edges
            .remove(&VisEdge::new(target, target_point, 0.0));
        self.vertices[target.0].in_edges.retain(|&v| v != source);
    }

    /// Remove a vertex and all its incident edges from the graph.
    ///
    /// For each out-edge, removes the corresponding in-edge reference from the target vertex.
    /// For each in-edge source, removes the out-edge targeting this vertex.
    /// Then clears the vertex's own edge lists and removes it from the point-to-vertex map.
    ///
    /// Faithfully ports `RemoveVertex` from VisibilityGraph.ts lines 250-261.
    ///
    /// **Rust adaptation:** In TS, `deleteP` removes the vertex from the map and the GC
    /// reclaims it. In Rust with an arena (Vec-based), the slot persists and its index
    /// may be reused by `add_vertex`. We keep the slot in the map (as an empty vertex)
    /// so that future `add_vertex` calls at the same point reuse the same `VertexId`,
    /// preserving deterministic edge ordering in `BTreeSet<VisEdge>` and `BinaryHeap`
    /// tie-breaking. This is an approved Rust adaptation: "Object references with GC →
    /// Index-based arenas."
    pub fn remove_vertex(&mut self, vertex: VertexId) {
        // For each out-edge of the vertex, remove the in-edge from the target.
        let out_targets: Vec<VertexId> = self.vertices[vertex.0]
            .out_edges
            .iter()
            .map(|e| e.target)
            .collect();
        for target in out_targets {
            self.vertices[target.0].in_edges.retain(|&v| v != vertex);
        }

        // For each in-edge source, remove the out-edge that targets this vertex.
        let vertex_point = self.vertices[vertex.0].point;
        let in_sources: Vec<VertexId> = self.vertices[vertex.0].in_edges.clone();
        for source in in_sources {
            self.vertices[source.0]
                .out_edges
                .remove(&VisEdge::new(vertex, vertex_point, 0.0));
        }

        // Clear the removed vertex's own edge lists.
        // In TS/C# the object gets GC'd; in Rust the Vec slot persists.
        self.vertices[vertex.0].out_edges.clear();
        self.vertices[vertex.0].in_edges.clear();

        // NOTE: We intentionally do NOT remove the vertex from point_to_vertex.
        // In TS, deleteP makes the vertex unreachable and GC reclaims the object.
        // In Rust's arena model, the slot persists, so keeping it in the map allows
        // subsequent add_vertex calls at the same point to reuse the same VertexId.
        // The vertex is effectively dead (no edges) but its map entry remains.
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

    /// Check if a vertex is still live (present in the point-to-vertex map
    /// with a matching VertexId).
    ///
    /// Currently `remove_vertex` keeps the map entry, so this always returns
    /// true for any vertex in the Vec. Retained for future use if the removal
    /// strategy changes.
    pub fn is_live(&self, v: VertexId) -> bool {
        if v.0 >= self.vertices.len() {
            return false;
        }
        self.point_to_vertex
            .get(&self.vertices[v.0].point)
            .map_or(false, |&id| id == v)
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
        let target_point = self.vertices[target.0].point;
        self.vertices[source.0]
            .out_edges
            .get(&VisEdge::new(target, target_point, 0.0))
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

    /// Get the entry index for a vertex from a specific direction.
    pub fn vertex_entry(&self, v: VertexId, dir: CompassDirection) -> Option<VertexEntryIndex> {
        self.vertices[v.0].vertex_entries[dir.index()]
    }

    /// Set the entry index for a vertex from a specific direction.
    pub fn set_vertex_entry(
        &mut self,
        v: VertexId,
        dir: CompassDirection,
        entry: Option<VertexEntryIndex>,
    ) {
        self.vertices[v.0].vertex_entries[dir.index()] = entry;
    }

    /// Clear all vertex entries (for reuse between path searches).
    pub fn clear_vertex_entries(&mut self) {
        for vd in &mut self.vertices {
            vd.vertex_entries = [None; 4];
        }
    }
}

impl Default for VisibilityGraph {
    fn default() -> Self {
        Self::new()
    }
}
