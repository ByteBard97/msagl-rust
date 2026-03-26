//! FreeSpaceFinder: computes movement bounds for axis edges from obstacles.
//!
//! Uses the sweep-line algorithm (FreeSpaceFinderSweep) ported from C#
//! FreeSpaceFinder.cs for O(n log n) obstacle constraint computation.

use crate::geometry::rectangle::Rectangle;
use crate::routing::scan_direction::Direction;

use super::axis_edge::AxisEdge;
use super::free_space_finder_sweep::FreeSpaceFinderSweep;

/// Find free space around axis edges by constraining them with obstacles.
///
/// For each axis edge parallel to `direction`, compute how far it can move
/// perpendicular to that direction before hitting an obstacle.
///
/// Also discovers right-neighbor relationships between axis edges:
/// two axis edges are neighbors if they are parallel, adjacent perpendicular
/// to the direction, and their projections along the direction overlap.
pub fn find_free_space(
    axis_edges: &mut [AxisEdge],
    obstacles: &[Rectangle],
    direction: Direction,
) {
    if axis_edges.is_empty() {
        return;
    }

    // No obstacle-to-edge mapping available at this call site;
    // pass None for all edges (no edge is exempt from any obstacle).
    let axis_edge_to_obstacle: Vec<Option<usize>> = vec![None; axis_edges.len()];

    let mut sweep = FreeSpaceFinderSweep::new(direction, axis_edges.len());
    sweep.find_free_space(axis_edges, obstacles, &axis_edge_to_obstacle);
    sweep.apply_results(axis_edges);
}
