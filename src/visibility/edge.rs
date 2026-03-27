use crate::geometry::point::Point;
use std::sync::Arc;

use super::graph::VertexId;

#[derive(Clone)]
pub struct VisEdge {
    pub target: VertexId,
    /// The target vertex's point, cached for ordering.
    ///
    /// The TS `VisibilityEdge` orders edges by `TargetPoint.compareTo()` (lexicographic
    /// point comparison). To faithfully replicate this, we store the target point and
    /// use it for `Ord`/`Eq` instead of the VertexId index. This ensures deterministic
    /// edge ordering regardless of arena slot assignment.
    pub target_point: Point,
    pub weight: f64,
    pub length_multiplier: f64,
    /// Transient zero-cost edge used during port splicing.
    ///
    /// Toll-free edges are removed after path search and are never
    /// persisted in the static visibility graph.
    pub is_toll_free: bool,
    /// Optional passability callback for group boundary crossing edges.
    ///
    /// Matches C# `VisibilityEdge.IsPassable: Func<bool>`.
    /// `None` means always passable (the common case).
    /// `Some(f)` means the edge is passable only when `f()` returns true.
    ///
    /// Arc is used so that Clone increments a reference count rather than
    /// copying the closure, and Send+Sync allow use across threads.
    pub is_passable: Option<Arc<dyn Fn() -> bool + Send + Sync>>,
}

impl VisEdge {
    /// Create a normal weighted edge.
    pub fn new(target: VertexId, target_point: Point, weight: f64) -> Self {
        Self {
            target,
            target_point,
            weight,
            length_multiplier: 1.0,
            is_toll_free: false,
            is_passable: None,
        }
    }

    /// Create a transient, zero-cost edge for port-to-graph splicing.
    pub fn toll_free(target: VertexId, target_point: Point) -> Self {
        Self {
            target,
            target_point,
            weight: 0.0,
            length_multiplier: 1.0,
            is_toll_free: true,
            is_passable: None,
        }
    }

    /// Check whether this edge is passable.
    ///
    /// Matches C# `SsstRectilinearPath.IsPassable(edge)`:
    /// `return edge.IsPassable == null || edge.IsPassable();`
    pub fn check_is_passable(&self) -> bool {
        self.is_passable.as_ref().map_or(true, |f| f())
    }
}

impl Eq for VisEdge {}
impl PartialEq for VisEdge {
    fn eq(&self, other: &Self) -> bool {
        self.target_point == other.target_point
    }
}
impl Ord for VisEdge {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.target_point.cmp(&other.target_point)
    }
}
impl PartialOrd for VisEdge {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl std::fmt::Debug for VisEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VisEdge")
            .field("target", &self.target)
            .field("target_point", &self.target_point)
            .field("weight", &self.weight)
            .field("length_multiplier", &self.length_multiplier)
            .field("is_toll_free", &self.is_toll_free)
            .field("is_passable", &self.is_passable.as_ref().map(|_| "Some(Fn)"))
            .finish()
    }
}
