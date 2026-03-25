use super::graph::VertexId;

#[derive(Clone, Debug)]
pub struct VisEdge {
    pub target: VertexId,
    pub weight: f64,
    pub length_multiplier: f64,
}

impl VisEdge {
    pub fn new(target: VertexId, weight: f64) -> Self {
        Self { target, weight, length_multiplier: 1.0 }
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
