//! FreeSpaceFinder: computes movement bounds for axis edges from obstacles.
//!
//! Faithful port of the C#/TS FreeSpaceFinder and Nudger.BoundAxisEdgesByRectsKnownInAdvance.
//!
//! Two-phase approach matching the C# architecture:
//! 1. Pre-bound axis edges by overlapping obstacle rectangles (C# BoundAxisEdgeByRect)
//! 2. Run sweep-line for neighbor discovery and incremental side constraints
//!    (C# FreeSpaceFinder : LineSweeperBase)

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::routing::scan_direction::Direction;

use super::axis_edge::AxisEdge;
use super::free_space_finder_sweep::FreeSpaceFinder;

/// Find free space around axis edges by constraining them with obstacles.
///
/// For each axis edge parallel to `direction`, computes how far it can move
/// perpendicular to that direction before hitting an obstacle. Also discovers
/// right-neighbor relationships between axis edges.
///
/// Uses the same two-phase approach as the C#/TS:
/// 1. C# Nudger.BoundAxisEdgesByRectsKnownInAdvance — pre-bound edges by rectangles
/// 2. C# FreeSpaceFinder sweep-line — incremental constraint and neighbor discovery
pub fn find_free_space(axis_edges: &mut [AxisEdge], obstacles: &[Rectangle], direction: Direction) {
    // Phase 1: C# Nudger.BoundAxisEdgesByRectsKnownInAdvance()
    // Pre-bound axis edges by overlapping obstacle rectangles.
    bound_axis_edges_by_rects(axis_edges, obstacles, direction);

    // Phase 2: C# FreeSpaceFinder sweep-line.
    // Discover right-neighbor relationships and apply incremental obstacle-side
    // constraints as the sweep line progresses.
    let mut finder = FreeSpaceFinder::new(axis_edges, obstacles, direction);
    finder.find_free_space();
    finder.apply(axis_edges);
}

/// C# Nudger.BoundAxisEdgeByRect — bound axis edges by overlapping obstacle rectangles.
///
/// For each axis edge parallel to the direction, check each obstacle rectangle
/// for overlap along the direction axis and bound the edge's perpendicular
/// movement accordingly. The C# does this for source/target obstacle ports
/// via BoundAxisEdgesAdjacentToSourceAndTargetOnEdge; we do it for all
/// overlapping rectangles since the Rust API doesn't track port-to-obstacle
/// mappings.
fn bound_axis_edges_by_rects(
    axis_edges: &mut [AxisEdge],
    obstacles: &[Rectangle],
    direction: Direction,
) {
    for ae in axis_edges.iter_mut() {
        if ae.direction != direction {
            continue;
        }
        let (along_min, along_max) = along_range(ae.source, ae.target, direction);
        let perp_pos = perp_coord(ae.source, direction);

        for obs in obstacles {
            let (obs_along_min, obs_along_max) = obstacle_along_range(obs, direction);
            let (obs_perp_min, obs_perp_max) = obstacle_perp_range(obs, direction);

            // Skip obstacles that don't overlap along the axis edge's direction.
            if obs_along_max < along_min + GeomConstants::DISTANCE_EPSILON
                || obs_along_min > along_max - GeomConstants::DISTANCE_EPSILON
            {
                continue;
            }

            // Bound from left: obstacle is to the left (lower perp).
            if obs_perp_max <= perp_pos + GeomConstants::DISTANCE_EPSILON {
                ae.bound_from_left(obs_perp_max);
            }
            // Bound from right: obstacle is to the right (higher perp).
            if obs_perp_min >= perp_pos - GeomConstants::DISTANCE_EPSILON {
                ae.bound_from_right(obs_perp_min);
            }
        }
    }
}

/// Get perpendicular coordinate of a point for the given direction.
/// C# xProjection: North → p.x, East → -p.y.
fn perp_coord(p: Point, direction: Direction) -> f64 {
    match direction {
        Direction::North => p.x(),
        Direction::East => -p.y(),
    }
}

/// Get the along-direction range of two points.
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
