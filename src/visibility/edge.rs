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
        Self { target, weight, length_multiplier: 1.0, is_toll_free: false }
    }

    /// Create a transient, zero-cost edge for port-to-graph splicing.
    pub fn toll_free(target: VertexId) -> Self {
        Self { target, weight: 0.0, length_multiplier: 1.0, is_toll_free: true }
    }
}

impl Eq for VisEdge {}
impl PartialEq for VisEdge {
    fn eq(&self, other: &Self) -> bool { self.target == other.target }
}
impl Ord for VisEdge {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering { self.target.cmp(&other.target) }
}
impl PartialOrd for VisEdge {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(other)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn toll_free_edge_has_zero_weight() {
        let edge = VisEdge::toll_free(VertexId(5));
        assert!(edge.is_toll_free);
        assert_eq!(edge.weight, 0.0);
        assert_eq!(edge.target, VertexId(5));
    }

    #[test]
    fn normal_edge_is_not_toll_free() {
        let edge = VisEdge::new(VertexId(3), 2.5);
        assert!(!edge.is_toll_free);
        assert_eq!(edge.weight, 2.5);
    }

    #[test]
    fn equality_ignores_toll_free_flag() {
        // Two edges to the same target are equal regardless of toll-free status.
        let normal = VisEdge::new(VertexId(7), 1.0);
        let free = VisEdge::toll_free(VertexId(7));
        assert_eq!(normal, free);
    }

    #[test]
    fn ordering_ignores_toll_free_flag() {
        let a = VisEdge::new(VertexId(1), 99.0);
        let b = VisEdge::toll_free(VertexId(2));
        assert!(a < b);
    }
}
