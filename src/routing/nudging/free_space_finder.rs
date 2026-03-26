//! FreeSpaceFinder: computes movement bounds for axis edges from obstacles.
//!
//! Simplified implementation: directly computes bounds from axis-aligned
//! obstacle rectangles rather than using a full sweep-line.

use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::routing::scan_direction::Direction;

use super::axis_edge::{AxisEdge, AxisEdgeId};

/// Find free space around axis edges by constraining them with obstacles.
///
/// For each axis edge parallel to `direction`, compute how far it can move
/// perpendicular to that direction before hitting an obstacle.
///
/// Also discovers right-neighbor relationships between axis edges:
/// two axis edges are neighbors if they are parallel, adjacent perpendicular
/// to the direction, and their projections along the direction overlap.
pub fn find_free_space(axis_edges: &mut [AxisEdge], obstacles: &[Rectangle], direction: Direction) {
    // Process edges parallel to the given direction.
    let parallel_ids: Vec<AxisEdgeId> = axis_edges
        .iter()
        .enumerate()
        .filter(|(_, ae)| ae.direction == direction)
        .map(|(id, _)| id)
        .collect();

    // Bound each axis edge by obstacles.
    for &ae_id in &parallel_ids {
        bound_edge_by_obstacles(ae_id, axis_edges, obstacles, direction);
    }

    // Find right-neighbor relationships.
    find_right_neighbors(&parallel_ids, axis_edges, direction);
}

/// Constrain an axis edge's left/right bounds using obstacle rectangles.
fn bound_edge_by_obstacles(
    ae_id: AxisEdgeId,
    axis_edges: &mut [AxisEdge],
    obstacles: &[Rectangle],
    direction: Direction,
) {
    let ae = &axis_edges[ae_id];
    let (src, tgt) = (ae.source, ae.target);

    // The axis edge's perpendicular position.
    let perp_pos = perp_coord(src, direction);

    // The range along the direction.
    let (along_min, along_max) = along_range(src, tgt, direction);

    for obs in obstacles {
        let (obs_perp_min, obs_perp_max) = obstacle_perp_range(obs, direction);
        let (obs_along_min, obs_along_max) = obstacle_along_range(obs, direction);

        // Check if the obstacle overlaps along the axis edge's direction.
        if obs_along_max < along_min + 1e-6 || obs_along_min > along_max - 1e-6 {
            continue;
        }

        // If obstacle is to the left (lower perp), it constrains left bound.
        if obs_perp_max <= perp_pos + 1e-6 {
            let bound = obs_perp_max;
            axis_edges[ae_id].bound_from_left(bound);
        }
        // If obstacle is to the right (higher perp), it constrains right bound.
        if obs_perp_min >= perp_pos - 1e-6 {
            let bound = obs_perp_min;
            axis_edges[ae_id].bound_from_right(bound);
        }
    }
}

/// Find right-neighbor relationships between parallel axis edges.
fn find_right_neighbors(
    parallel_ids: &[AxisEdgeId],
    axis_edges: &mut [AxisEdge],
    direction: Direction,
) {
    // Sort by perpendicular coordinate.
    let mut sorted: Vec<AxisEdgeId> = parallel_ids.to_vec();
    sorted.sort_by(|&a, &b| {
        let pa = perp_coord(axis_edges[a].source, direction);
        let pb = perp_coord(axis_edges[b].source, direction);
        pa.partial_cmp(&pb).unwrap()
    });

    // For each consecutive pair, check if their projections overlap.
    for i in 0..sorted.len() {
        for j in (i + 1)..sorted.len() {
            let left_id = sorted[i];
            let right_id = sorted[j];
            let left_perp = perp_coord(axis_edges[left_id].source, direction);
            let right_perp = perp_coord(axis_edges[right_id].source, direction);

            // Only immediate neighbors (no edge between them).
            if (right_perp - left_perp).abs() < 1e-6 {
                continue; // Same position, skip.
            }

            if projections_overlap(&axis_edges[left_id], &axis_edges[right_id], direction) {
                axis_edges[left_id].right_neighbors.push(right_id);
            }

            // Only look at the nearest neighbor(s) — break after finding
            // any that are strictly to the right.
            break;
        }
    }
}

/// Check if two axis edges' projections along the direction overlap.
fn projections_overlap(left: &AxisEdge, right: &AxisEdge, direction: Direction) -> bool {
    let (l_min, l_max) = along_range(left.source, left.target, direction);
    let (r_min, r_max) = along_range(right.source, right.target, direction);
    let eps = 1e-6;
    !(l_max < r_min - eps || r_max < l_min - eps)
}

/// Get the perpendicular coordinate of a point relative to the direction.
fn perp_coord(p: Point, direction: Direction) -> f64 {
    match direction {
        Direction::North => p.x(),
        Direction::East => -p.y(),
    }
}

/// Get the range along the direction for two points.
fn along_range(src: Point, tgt: Point, direction: Direction) -> (f64, f64) {
    match direction {
        Direction::North => (src.y().min(tgt.y()), src.y().max(tgt.y())),
        Direction::East => (src.x().min(tgt.x()), src.x().max(tgt.x())),
    }
}

/// Get the perpendicular range of an obstacle rectangle.
fn obstacle_perp_range(obs: &Rectangle, direction: Direction) -> (f64, f64) {
    match direction {
        Direction::North => (obs.left(), obs.right()),
        Direction::East => (-obs.top(), -obs.bottom()),
    }
}

/// Get the along-direction range of an obstacle rectangle.
fn obstacle_along_range(obs: &Rectangle, direction: Direction) -> (f64, f64) {
    match direction {
        Direction::North => (obs.bottom(), obs.top()),
        Direction::East => (obs.left(), obs.right()),
    }
}
