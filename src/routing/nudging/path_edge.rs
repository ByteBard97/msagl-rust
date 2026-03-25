//! PathEdge: one segment of a path, referencing an AxisEdge.

use super::axis_edge::AxisEdgeId;

/// Unique identifier for a PathEdge.
pub type PathEdgeId = usize;

/// One segment of a routed path.
///
/// References an AxisEdge (which always points North/East).
/// If the path traverses the axis edge in the opposite direction,
/// `reversed` is true.
#[derive(Debug, Clone)]
pub struct PathEdge {
    pub axis_edge_id: AxisEdgeId,
    pub reversed: bool,
    /// Index of the path this edge belongs to.
    pub path_index: usize,
    /// Width of this edge (for arrowheads etc). Default 0.
    #[allow(dead_code)]
    pub width: f64,
    /// Index within the ordered list on its AxisEdge (set by combinatorial nudger).
    pub index: i32,
    /// The LongestNudgedSegment this edge belongs to (if parallel to nudging direction).
    pub longest_seg_id: Option<usize>,
    /// Next PathEdge in the path (linked list via indices).
    pub next: Option<PathEdgeId>,
    /// Previous PathEdge in the path.
    pub prev: Option<PathEdgeId>,
}

impl PathEdge {
    pub fn new(axis_edge_id: AxisEdgeId, reversed: bool, path_index: usize) -> Self {
        Self {
            axis_edge_id,
            reversed,
            path_index,
            width: 0.0,
            index: -1,
            longest_seg_id: None,
            next: None,
            prev: None,
        }
    }
}
