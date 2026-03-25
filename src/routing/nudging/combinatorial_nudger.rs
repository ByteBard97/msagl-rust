//! CombinatorialNudger: determines path ordering on each axis edge.
//!
//! Walks all paths, creates AxisEdges for each segment, collects PathEdges
//! onto AxisEdges, and sorts them to establish a consistent ordering.

use std::collections::HashMap;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::routing::scan_direction::Direction;

use super::axis_edge::{AxisEdge, AxisEdgeId};
use super::path_edge::{PathEdge, PathEdgeId};

/// Result of the combinatorial nudger: axis edges and path edges with ordering.
pub struct CombinatorialResult {
    pub axis_edges: Vec<AxisEdge>,
    pub path_edges: Vec<PathEdge>,
    /// For each path, the ID of its first PathEdge.
    pub path_first_edges: Vec<Option<PathEdgeId>>,
    /// For each AxisEdge, the ordered list of PathEdge IDs on it.
    pub axis_edge_orders: Vec<Vec<PathEdgeId>>,
}

/// Build the axis-edge DAG from refined paths and determine path ordering.
pub fn get_order(paths: &[Vec<Point>]) -> CombinatorialResult {
    let mut axis_edges: Vec<AxisEdge> = Vec::new();
    let mut path_edges: Vec<PathEdge> = Vec::new();
    let mut path_first_edges: Vec<Option<PathEdgeId>> = Vec::new();

    // Map from (source, target) to AxisEdgeId for deduplication.
    let mut edge_map: HashMap<(Point, Point), AxisEdgeId> = HashMap::new();

    for (path_idx, path) in paths.iter().enumerate() {
        let mut first_pe: Option<PathEdgeId> = None;
        let mut prev_pe: Option<PathEdgeId> = None;

        for i in 0..path.len().saturating_sub(1) {
            let (p0, p1) = (path[i], path[i + 1]);
            if p0.close_to(p1) {
                continue;
            }

            // Determine canonical direction (North or East).
            let (src, tgt, reversed) = canonicalize(p0, p1);
            let ae_id = *edge_map.entry((src, tgt)).or_insert_with(|| {
                let id = axis_edges.len();
                axis_edges.push(AxisEdge::new(src, tgt));
                id
            });

            let pe_id = path_edges.len();
            let mut pe = PathEdge::new(ae_id, reversed, path_idx);
            pe.prev = prev_pe;
            path_edges.push(pe);

            if let Some(prev_id) = prev_pe {
                path_edges[prev_id].next = Some(pe_id);
            }
            if first_pe.is_none() {
                first_pe = Some(pe_id);
            }
            prev_pe = Some(pe_id);
        }
        path_first_edges.push(first_pe);
    }

    // Collect PathEdges per AxisEdge.
    let mut axis_edge_orders: Vec<Vec<PathEdgeId>> = vec![Vec::new(); axis_edges.len()];
    for (pe_id, pe) in path_edges.iter().enumerate() {
        axis_edge_orders[pe.axis_edge_id].push(pe_id);
    }

    // Sort PathEdges on each AxisEdge.
    // Simple approach: sort by path index for initial ordering,
    // then refine by looking at neighboring edges.
    for (ae_id, order) in axis_edge_orders.iter_mut().enumerate() {
        sort_path_edges_on_axis(
            ae_id,
            order,
            &path_edges,
            &axis_edges,
            paths,
        );
        // Assign index to each PathEdge.
        for (idx, &pe_id) in order.iter().enumerate() {
            path_edges[pe_id].index = idx as i32;
        }
    }

    CombinatorialResult {
        axis_edges,
        path_edges,
        path_first_edges,
        axis_edge_orders,
    }
}

/// Canonicalize a segment so source < target (North or East).
fn canonicalize(p0: Point, p1: Point) -> (Point, Point, bool) {
    let dx = p1.x() - p0.x();
    let dy = p1.y() - p0.y();
    if dx.abs() > dy.abs() {
        // Horizontal: East means p0.x < p1.x
        if dx > 0.0 {
            (p0, p1, false)
        } else {
            (p1, p0, true)
        }
    } else {
        // Vertical: North means p0.y < p1.y
        if dy > 0.0 {
            (p0, p1, false)
        } else {
            (p1, p0, true)
        }
    }
}

/// Sort PathEdges sharing an AxisEdge to establish a consistent left-to-right order.
///
/// Uses a simplified comparison: look at neighboring perpendicular edges
/// to determine which path is "to the left" vs "to the right".
fn sort_path_edges_on_axis(
    _ae_id: AxisEdgeId,
    order: &mut [PathEdgeId],
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
    _paths: &[Vec<Point>],
) {
    if order.len() <= 1 {
        return;
    }

    let ae = &axis_edges[_ae_id];

    order.sort_by(|&a_id, &b_id| {
        let a = &path_edges[a_id];
        let b = &path_edges[b_id];

        // Try to compare by looking at the next perpendicular edge.
        let a_perp = get_perp_offset(a, path_edges, axis_edges, ae);
        let b_perp = get_perp_offset(b, path_edges, axis_edges, ae);

        GeomConstants::compare(a_perp, b_perp)
            .then_with(|| a.path_index.cmp(&b.path_index))
    });
}

/// Get the perpendicular offset for a path at the next turn after this axis edge.
///
/// For a path going through a vertical AxisEdge, this looks at where the
/// path goes next (left or right) to determine ordering.
fn get_perp_offset(
    pe: &PathEdge,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
    current_ae: &AxisEdge,
) -> f64 {
    // Look at the next edge in the path (forward direction).
    let next_id = if pe.reversed { pe.prev } else { pe.next };
    if let Some(nid) = next_id {
        let next_pe = &path_edges[nid];
        let next_ae = &axis_edges[next_pe.axis_edge_id];
        // If the next edge is perpendicular, its position tells us the offset.
        if next_ae.direction != current_ae.direction {
            let midpoint = Point::middle(next_ae.source, next_ae.target);
            return match current_ae.direction {
                Direction::North => midpoint.x(),
                Direction::East => -midpoint.y(),
            };
        }
    }

    // Look backward.
    let prev_id = if pe.reversed { pe.next } else { pe.prev };
    if let Some(pid) = prev_id {
        let prev_pe = &path_edges[pid];
        let prev_ae = &axis_edges[prev_pe.axis_edge_id];
        if prev_ae.direction != current_ae.direction {
            let midpoint = Point::middle(prev_ae.source, prev_ae.target);
            return match current_ae.direction {
                Direction::North => midpoint.x(),
                Direction::East => -midpoint.y(),
            };
        }
    }

    // Fallback: use path index as tiebreaker.
    pe.path_index as f64
}
