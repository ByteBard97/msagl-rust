use std::collections::{BTreeSet, HashMap};
use crate::geometry::point::Point;
use super::edge::VisEdge;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

pub struct VertexData {
    pub point: Point,
    pub out_edges: BTreeSet<VisEdge>,
    pub in_edges: Vec<VertexId>,
    pub distance: f64,
    pub prev_vertex: Option<VertexId>,
    pub is_terminal: bool,
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
}

impl Default for VisibilityGraph {
    fn default() -> Self { Self::new() }
}
