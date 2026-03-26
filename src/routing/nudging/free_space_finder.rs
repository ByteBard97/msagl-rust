//! FreeSpaceFinder: computes movement bounds for axis edges from obstacles.
//!
//! Uses the sweep-line algorithm (FreeSpaceFinderSweep) ported from C#
//! FreeSpaceFinder.cs for O(n log n) obstacle constraint computation.
//!
//! The sweep handles both bound computation and neighbor discovery.
//! For neighbor discovery, the sweep correctly finds all adjacent-container
//! neighbor pairs (matching C#), but the simplified O(n log n) sorted-list
//! approach is used instead because the downstream projection solver has
//! O(n^2) behavior with large constraint sets. Once the solver is optimized,
//! the sweep's neighbor discovery can be re-enabled.

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

    // Use sweep for obstacle bound computation (O(n log n)).
    let axis_edge_to_obstacle: Vec<Option<usize>> = vec![None; axis_edges.len()];
    let mut sweep = FreeSpaceFinderSweep::new(direction, axis_edges.len());
    sweep.find_free_space(axis_edges, obstacles, &axis_edge_to_obstacle);
    // Apply only bounds (not neighbors) from the sweep.
    sweep.apply_bounds_only(axis_edges);

    // Use simplified neighbor finding (sorted-list approach).
    // The sweep's neighbor discovery is correct per C# but produces O(n*m)
    // pairs for large adjacent containers, overwhelming the solver.
    super::free_space_finder_simple::find_right_neighbors_only(axis_edges, direction);
}
