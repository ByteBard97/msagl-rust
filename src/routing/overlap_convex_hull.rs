use crate::geometry::polyline::Polyline;

/// Convex hull enclosing a group of overlapping obstacles.
///
/// Faithful port of TS `OverlapConvexHull.ts`.
///
/// When two or more non-rectangular obstacles overlap, their polyline vertices
/// are collected and wrapped in a convex hull. This hull then becomes the
/// `VisibilityPolyline` used for scanline processing and visibility graph
/// generation, replacing the individual obstacle boundaries.
#[derive(Clone, Debug)]
pub struct OverlapConvexHull {
    /// The convex hull polyline enclosing all obstacles in this group.
    pub polyline: Polyline,

    /// Indices of the obstacles contained in this convex hull.
    /// The first entry is the "primary" obstacle — the one whose
    /// polyline actually participates in the visibility graph hierarchy.
    pub obstacle_indices: Vec<usize>,
}

impl OverlapConvexHull {
    /// Create a new convex hull from a polyline and the obstacle indices it covers.
    ///
    /// Matches TS: `new OverlapConvexHull(polyline, obstacles)`.
    /// The first obstacle index becomes the `primary_obstacle_index`.
    pub fn new(polyline: Polyline, obstacle_indices: Vec<usize>) -> Self {
        debug_assert!(
            !obstacle_indices.is_empty(),
            "OverlapConvexHull must contain at least one obstacle"
        );
        Self {
            polyline,
            obstacle_indices,
        }
    }

    /// The index of the primary obstacle (first in the list).
    ///
    /// Matches TS: `ConvexHull.PrimaryObstacle`.
    pub fn primary_obstacle_index(&self) -> usize {
        self.obstacle_indices[0]
    }
}
