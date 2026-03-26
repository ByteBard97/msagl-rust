//! Simplified FreeSpaceFinder implementation for comparison.
//! This is the old O(n*m) implementation kept as a fallback.

use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::routing::scan_direction::Direction;

use super::axis_edge::{AxisEdge, AxisEdgeId};

/// Find free space using the simplified O(n*m) approach.
pub fn find_free_space_simple(
    axis_edges: &mut [AxisEdge],
    obstacles: &[Rectangle],
    direction: Direction,
) {
    let parallel_ids: Vec<AxisEdgeId> = axis_edges
        .iter()
        .enumerate()
        .filter(|(_, ae)| ae.direction == direction)
        .map(|(id, _)| id)
        .collect();

    for &ae_id in &parallel_ids {
        bound_edge_by_obstacles(ae_id, axis_edges, obstacles, direction);
    }

    find_right_neighbors(&parallel_ids, axis_edges, direction);
}

fn bound_edge_by_obstacles(
    ae_id: AxisEdgeId,
    axis_edges: &mut [AxisEdge],
    obstacles: &[Rectangle],
    direction: Direction,
) {
    let ae = &axis_edges[ae_id];
    let (src, tgt) = (ae.source, ae.target);
    let perp_pos = perp_coord(src, direction);
    let (along_min, along_max) = along_range(src, tgt, direction);

    for obs in obstacles {
        let (obs_perp_min, obs_perp_max) = obstacle_perp_range(obs, direction);
        let (obs_along_min, obs_along_max) = obstacle_along_range(obs, direction);

        if obs_along_max < along_min + 1e-6 || obs_along_min > along_max - 1e-6 {
            continue;
        }

        if obs_perp_max <= perp_pos + 1e-6 {
            axis_edges[ae_id].bound_from_left(obs_perp_max);
        }
        if obs_perp_min >= perp_pos - 1e-6 {
            axis_edges[ae_id].bound_from_right(obs_perp_min);
        }
    }
}

/// Find right-neighbor relationships only (no bounds computation).
/// Public entry point for use when bounds are computed by the sweep.
pub fn find_right_neighbors_only(
    axis_edges: &mut [AxisEdge],
    direction: Direction,
) {
    let parallel_ids: Vec<AxisEdgeId> = axis_edges
        .iter()
        .enumerate()
        .filter(|(_, ae)| ae.direction == direction)
        .map(|(id, _)| id)
        .collect();
    find_right_neighbors(&parallel_ids, axis_edges, direction);
}

fn find_right_neighbors(
    parallel_ids: &[AxisEdgeId],
    axis_edges: &mut [AxisEdge],
    direction: Direction,
) {
    let mut sorted: Vec<AxisEdgeId> = parallel_ids.to_vec();
    sorted.sort_by(|&a, &b| {
        let pa = perp_coord(axis_edges[a].source, direction);
        let pb = perp_coord(axis_edges[b].source, direction);
        pa.partial_cmp(&pb).unwrap()
    });

    for i in 0..sorted.len() {
        for j in (i + 1)..sorted.len() {
            let left_id = sorted[i];
            let right_id = sorted[j];
            let left_perp = perp_coord(axis_edges[left_id].source, direction);
            let right_perp = perp_coord(axis_edges[right_id].source, direction);

            if (right_perp - left_perp).abs() < 1e-6 {
                continue;
            }

            if projections_overlap(&axis_edges[left_id], &axis_edges[right_id], direction) {
                axis_edges[left_id].right_neighbors.push(right_id);
            }

            break;
        }
    }
}

fn projections_overlap(left: &AxisEdge, right: &AxisEdge, direction: Direction) -> bool {
    let (l_min, l_max) = along_range(left.source, left.target, direction);
    let (r_min, r_max) = along_range(right.source, right.target, direction);
    let eps = 1e-6;
    !(l_max < r_min - eps || r_max < l_min - eps)
}

fn perp_coord(p: Point, direction: Direction) -> f64 {
    match direction {
        Direction::North => p.x(),
        Direction::East => -p.y(),
    }
}

fn along_range(src: Point, tgt: Point, direction: Direction) -> (f64, f64) {
    match direction {
        Direction::North => (src.y().min(tgt.y()), src.y().max(tgt.y())),
        Direction::East => (src.x().min(tgt.x()), src.x().max(tgt.x())),
    }
}

fn obstacle_perp_range(obs: &Rectangle, direction: Direction) -> (f64, f64) {
    match direction {
        Direction::North => (obs.left(), obs.right()),
        Direction::East => (-obs.top(), -obs.bottom()),
    }
}

fn obstacle_along_range(obs: &Rectangle, direction: Direction) -> (f64, f64) {
    match direction {
        Direction::North => (obs.bottom(), obs.top()),
        Direction::East => (obs.left(), obs.right()),
    }
}
