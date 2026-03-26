use super::graph::VertexId;

#[derive(Clone, Debug)]
pub struct VisEdge {
    pub target: VertexId,
    pub weight: f64,
    pub length_multiplier: f64,
    /// Transient zero-cost edge used during port splicing.
    ///
    /// Toll-free edges are removed after path search and are never
    /// persisted in the static visibility graph.
    pub is_toll_free: bool,
}

impl VisEdge {
    /// Create a normal weighted edge.
    pub fn new(target: VertexId, weight: f64) -> Self {
        Self {
            target,
            weight,
            length_multiplier: 1.0,
            is_toll_free: false,
        }
    }

    /// Create a transient, zero-cost edge for port-to-graph splicing.
    pub fn toll_free(target: VertexId) -> Self {
        Self {
            target,
            weight: 0.0,
            length_multiplier: 1.0,
            is_toll_free: true,
        }
    }
}

impl Eq for VisEdge {}
impl PartialEq for VisEdge {
    fn eq(&self, other: &Self) -> bool {
        self.target == other.target
    }
}
impl Ord for VisEdge {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.target.cmp(&other.target)
    }
}
impl PartialOrd for VisEdge {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
