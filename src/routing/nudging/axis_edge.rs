//! AxisEdge: an atomic rectilinear segment with movement bounds.
//!
//! Each AxisEdge represents a segment of the axis-edge DAG. It always
//! points North or East (source < target in that direction). PathEdges
//! reference an AxisEdge and may be reversed.

use crate::geometry::point::Point;
use crate::routing::scan_direction::Direction;

/// Unique identifier for an AxisEdge within the nudger's storage.
pub type AxisEdgeId = usize;

/// An atomic rectilinear segment in the axis-edge DAG.
///
/// Oriented so that `source -> target` goes North or East.
/// Movement bounds track how far the segment can shift perpendicular
/// to its direction before hitting an obstacle.
#[derive(Debug, Clone)]
pub struct AxisEdge {
    pub source: Point,
    pub target: Point,
    pub direction: Direction,
    /// How far left (in the perpendicular projection) this edge can move.
    pub left_bound: f64,
    /// How far right (in the perpendicular projection) this edge can move.
    pub right_bound: f64,
    /// AxisEdges that are immediately to the right of this one
    /// (used for inter-segment separation constraints).
    pub right_neighbors: Vec<AxisEdgeId>,
}

impl AxisEdge {
    pub fn new(source: Point, target: Point) -> Self {
        let direction = direction_from_points(source, target);
        Self {
            source,
            target,
            direction,
            left_bound: f64::NEG_INFINITY,
            right_bound: f64::INFINITY,
            right_neighbors: Vec::new(),
        }
    }

    /// Tighten the left bound (increase it).
    pub fn bound_from_left(&mut self, leftbound: f64) {
        let leftbound = leftbound.min(self.right_bound);
        self.left_bound = self.left_bound.max(leftbound);
    }

    /// Tighten the right bound (decrease it).
    pub fn bound_from_right(&mut self, rightbound: f64) {
        let rightbound = rightbound.max(self.left_bound);
        self.right_bound = self.right_bound.min(rightbound);
    }
}

/// Determine direction from p0 to p1 for rectilinear segments.
/// Returns North or East; panics if diagonal.
fn direction_from_points(p0: Point, p1: Point) -> Direction {
    let dx = p1.x() - p0.x();
    let dy = p1.y() - p0.y();
    if dx.abs() > dy.abs() {
        debug_assert!(dx > 0.0, "AxisEdge source must be west of target for East");
        Direction::East
    } else {
        debug_assert!(
            dy > 0.0,
            "AxisEdge source must be south of target for North"
        );
        Direction::North
    }
}
