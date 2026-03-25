//! Nudger: orchestrates the full nudging pipeline.
//!
//! 1. PathRefiner.refine_paths(paths)
//! 2. CombinatorialNudger.get_order(paths) -> path ordering per axis edge
//! 3. FreeSpaceFinder.find_free_space(axis_edges, obstacles)
//! 4. Group PathEdges into LongestNudgedSegments
//! 5. Create solver variables and constraints
//! 6. Solve
//! 7. Apply solved positions back to path points

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::projection_solver::uniform_solver::UniformOneDimensionalSolver;
use crate::routing::scan_direction::Direction;

use super::axis_edge::AxisEdge;
use super::combinatorial_nudger;
use super::free_space_finder;
use super::longest_nudged_segment::LongestNudgedSegment;
use super::path_edge::PathEdge;
use super::path_refiner;
use super::staircase_remover;

/// Nudge paths to separate parallel edges.
///
/// Modifies `paths` in-place so that overlapping parallel segments are
/// spread apart by at least `edge_separation`.
pub fn nudge_paths(
    paths: &mut [Vec<Point>],
    obstacles: &[Rectangle],
    edge_separation: f64,
) {
    if paths.is_empty() {
        return;
    }

    // Run nudging in both directions (like the TS: North, East, North).
    calculate(paths, obstacles, edge_separation, Direction::North);
    calculate(paths, obstacles, edge_separation, Direction::East);
    calculate(paths, obstacles, edge_separation, Direction::North);

    // Remove staircases.
    staircase_remover::remove_staircases(paths, obstacles);

    // Final cleanup: remove switchbacks and middle points.
    for path in paths.iter_mut() {
        remove_switchbacks(path);
    }
}

/// Run one pass of nudging in the given direction.
fn calculate(
    paths: &mut [Vec<Point>],
    obstacles: &[Rectangle],
    edge_separation: f64,
    direction: Direction,
) {
    // Step 1: Refine paths.
    path_refiner::refine_paths(paths);

    // Step 2: Build axis-edge DAG and get path ordering.
    let mut result = combinatorial_nudger::get_order(paths);

    // Step 3: Find free space (obstacle bounds).
    free_space_finder::find_free_space(&mut result.axis_edges, obstacles, direction);

    // Step 4: Group PathEdges into LongestNudgedSegments.
    let longest_segs = create_longest_segments(
        &mut result.path_edges,
        &result.path_first_edges,
        &result.axis_edges,
        direction,
    );

    if longest_segs.is_empty() {
        return;
    }

    // Step 5: Create solver and solve.
    let positions = solve_positions(
        &longest_segs,
        &result.path_edges,
        &result.axis_edges,
        &result.axis_edge_orders,
        edge_separation,
        direction,
    );

    // Step 6: Apply solved positions back to paths.
    apply_positions(
        paths,
        &result.path_edges,
        &result.path_first_edges,
        &result.axis_edges,
        &longest_segs,
        &positions,
        direction,
    );
}

/// Group consecutive PathEdges parallel to the nudging direction into
/// LongestNudgedSegments.
fn create_longest_segments(
    path_edges: &mut [PathEdge],
    path_first_edges: &[Option<usize>],
    axis_edges: &[AxisEdge],
    direction: Direction,
) -> Vec<LongestNudgedSegment> {
    let mut segments: Vec<LongestNudgedSegment> = Vec::new();
    let opposite = opposite_dir(direction);

    for &first_pe_opt in path_first_edges {
        let mut current_seg: Option<usize> = None;
        let mut pe_id_opt = first_pe_opt;

        while let Some(pe_id) = pe_id_opt {
            let ae = &axis_edges[path_edges[pe_id].axis_edge_id];
            let edge_dir = ae.direction;

            if edge_dir == direction || edge_dir == opposite {
                // Parallel to nudging direction — extend or create segment.
                let seg_id = if let Some(sid) = current_seg {
                    sid
                } else {
                    let sid = segments.len();
                    segments.push(LongestNudgedSegment::new(sid));
                    current_seg = Some(sid);
                    sid
                };

                let (src, tgt) = edge_source_target(
                    &path_edges[pe_id],
                    ae,
                );
                segments[seg_id].add_edge(pe_id, src, tgt);
                path_edges[pe_id].longest_seg_id = Some(seg_id);
            } else {
                // Perpendicular — break the current segment.
                path_edges[pe_id].longest_seg_id = None;
                current_seg = None;
            }

            pe_id_opt = path_edges[pe_id].next;
        }
    }

    // Set ideal positions.
    for seg in &mut segments {
        seg.ideal_position = seg.position();
    }

    segments
}

/// Solve for optimal positions using the projection solver.
fn solve_positions(
    segments: &[LongestNudgedSegment],
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
    axis_edge_orders: &[Vec<usize>],
    edge_separation: f64,
    direction: Direction,
) -> Vec<f64> {
    let mut solver = UniformOneDimensionalSolver::new();

    // Create variables for each LongestNudgedSegment.
    let mut var_ids: Vec<usize> = Vec::with_capacity(segments.len());
    for seg in segments {
        let pos = seg.position();
        let ideal = seg.ideal_position;
        // Clamp ideal within bounds.
        let left_bound = get_left_bound(seg, path_edges, axis_edges);
        let right_bound = get_right_bound(seg, path_edges, axis_edges);
        let clamped_ideal = ideal.max(left_bound).min(right_bound);

        let var_id = if seg.is_fixed || left_bound >= right_bound {
            // Fixed variable: high weight to stay in place.
            solver.add_variable(pos, 1e6, 1.0)
        } else {
            let vid = solver.add_variable(clamped_ideal, 1.0, 1.0);
            // Apply bounds.
            if left_bound.is_finite() {
                solver.set_lower_bound(vid, left_bound);
            }
            if right_bound.is_finite() {
                solver.set_upper_bound(vid, right_bound);
            }
            vid
        };
        var_ids.push(var_id);
    }

    // Create separation constraints from path ordering.
    for (ae_id, order) in axis_edge_orders.iter().enumerate() {
        let ae = &axis_edges[ae_id];
        if ae.direction != direction {
            continue;
        }

        // For consecutive path edges in the order that belong to
        // different LongestNudgedSegments, add separation constraints.
        let mut prev_seg: Option<usize> = None;
        for &pe_id in order {
            let seg_id = path_edges[pe_id].longest_seg_id;
            if let Some(sid) = seg_id {
                if let Some(prev_sid) = prev_seg {
                    if prev_sid != sid {
                        solver.add_constraint(
                            var_ids[prev_sid],
                            var_ids[sid],
                            edge_separation,
                        );
                    }
                }
                prev_seg = Some(sid);
            }
        }
    }

    // Create constraints between segments on neighboring axis edges.
    for seg in segments {
        let right_neighbor_segs = collect_right_neighbor_segs(seg, path_edges, axis_edges, segments);
        for &rsid in &right_neighbor_segs {
            if rsid != seg.id {
                solver.add_constraint(
                    var_ids[seg.id],
                    var_ids[rsid],
                    edge_separation,
                );
            }
        }
    }

    solver.solve()
}

/// Collect LongestNudgedSegment IDs that are right neighbors of this segment.
fn collect_right_neighbor_segs(
    seg: &LongestNudgedSegment,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
    all_segs: &[LongestNudgedSegment],
) -> Vec<usize> {
    let mut result = Vec::new();
    for &pe_id in &seg.edges {
        let ae_id = path_edges[pe_id].axis_edge_id;
        for &rn_id in &axis_edges[ae_id].right_neighbors {
            // Find LongestNudgedSegments on the right neighbor axis edge.
            for other_seg in all_segs {
                for &other_pe_id in &other_seg.edges {
                    if path_edges[other_pe_id].axis_edge_id == rn_id
                        && !result.contains(&other_seg.id)
                    {
                        result.push(other_seg.id);
                    }
                }
            }
        }
    }
    result
}

/// Apply solved positions back to path points.
fn apply_positions(
    paths: &mut [Vec<Point>],
    path_edges: &[PathEdge],
    path_first_edges: &[Option<usize>],
    axis_edges: &[AxisEdge],
    segments: &[LongestNudgedSegment],
    positions: &[f64],
    direction: Direction,
) {
    for (path_idx, path) in paths.iter_mut().enumerate() {
        let first_pe = path_first_edges[path_idx];
        if first_pe.is_none() {
            continue;
        }

        let mut new_points: Vec<Point> = Vec::new();
        let mut pe_id_opt = first_pe;

        // Add the source of the first edge.
        if let Some(pe_id) = pe_id_opt {
            let ae = &axis_edges[path_edges[pe_id].axis_edge_id];
            let (src, _) = edge_source_target(&path_edges[pe_id], ae);
            let shifted = shift_point(
                src,
                path_edges[pe_id].longest_seg_id,
                segments,
                positions,
                direction,
            );
            new_points.push(shifted);
        }

        // Walk along path edges, adding each target.
        while let Some(pe_id) = pe_id_opt {
            let pe = &path_edges[pe_id];
            let ae = &axis_edges[pe.axis_edge_id];
            let (_, tgt) = edge_source_target(pe, ae);

            // Use the segment of the current edge, or the next edge if this
            // edge has no segment.
            let seg_id = pe.longest_seg_id.or_else(|| {
                pe.next.and_then(|nid| path_edges[nid].longest_seg_id)
            });

            let shifted = shift_point(tgt, seg_id, segments, positions, direction);
            new_points.push(shifted);
            pe_id_opt = pe.next;
        }

        if new_points.len() >= 2 {
            *path = new_points;
        }
    }
}

/// Shift a point according to the solved position of its segment.
fn shift_point(
    point: Point,
    seg_id: Option<usize>,
    _segments: &[LongestNudgedSegment],
    positions: &[f64],
    direction: Direction,
) -> Point {
    let Some(sid) = seg_id else { return point };
    if sid >= positions.len() {
        return point;
    }

    let solved_pos = positions[sid];
    match direction {
        Direction::North => Point::new(solved_pos, point.y()),
        Direction::East => Point::new(point.x(), -solved_pos),
    }
}

/// Get the left bound for a segment from its axis edges.
fn get_left_bound(
    seg: &LongestNudgedSegment,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
) -> f64 {
    if seg.is_fixed {
        return seg.position();
    }
    let mut lb = f64::NEG_INFINITY;
    for &pe_id in &seg.edges {
        let ae = &axis_edges[path_edges[pe_id].axis_edge_id];
        lb = lb.max(ae.left_bound);
    }
    lb
}

/// Get the right bound for a segment from its axis edges.
fn get_right_bound(
    seg: &LongestNudgedSegment,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
) -> f64 {
    if seg.is_fixed {
        return seg.position();
    }
    let mut rb = f64::INFINITY;
    for &pe_id in &seg.edges {
        let ae = &axis_edges[path_edges[pe_id].axis_edge_id];
        rb = rb.min(ae.right_bound);
    }
    rb
}

/// Get source and target of a PathEdge (accounting for reversal).
fn edge_source_target(pe: &PathEdge, ae: &AxisEdge) -> (Point, Point) {
    if pe.reversed {
        (ae.target, ae.source)
    } else {
        (ae.source, ae.target)
    }
}

fn opposite_dir(d: Direction) -> Direction {
    match d {
        Direction::North => Direction::North, // South doesn't exist in our enum
        Direction::East => Direction::East,   // West doesn't exist
    }
}

/// Remove switchbacks (direction reversals) and collinear middle points.
fn remove_switchbacks(path: &mut Vec<Point>) {
    if path.len() < 3 {
        return;
    }

    let mut result: Vec<Point> = Vec::with_capacity(path.len());
    result.push(path[0]);

    let mut i = 1;
    while i < path.len() {
        let a = *result.last().unwrap();
        let b = path[i];

        if a.close_to(b) {
            i += 1;
            continue;
        }

        if i + 1 < path.len() {
            let c = path[i + 1];
            // Check if b is collinear between a and c.
            if is_collinear(a, b, c) {
                // Skip b, it's a middle point.
                i += 1;
                continue;
            }
            // Check for switchback: a-b reverses at b-c.
            if is_switchback(a, b, c) {
                // Skip b.
                i += 1;
                continue;
            }
        }

        // Rectilinearise: snap to nearest axis.
        result.push(rectilinearise(a, b));
        i += 1;
    }

    // Add the last point if not already there.
    if let Some(&last) = path.last() {
        if let Some(&r_last) = result.last() {
            if !r_last.close_to(last) {
                result.push(rectilinearise(*result.last().unwrap(), last));
            }
        }
    }

    if result.len() >= 2 {
        *path = result;
    }
}

/// Check if three points are collinear (all on same horizontal or vertical line).
fn is_collinear(a: Point, b: Point, c: Point) -> bool {
    (GeomConstants::close(a.x(), b.x()) && GeomConstants::close(b.x(), c.x()))
        || (GeomConstants::close(a.y(), b.y()) && GeomConstants::close(b.y(), c.y()))
}

/// Check if a-b-c is a switchback (reversal of direction).
fn is_switchback(a: Point, b: Point, c: Point) -> bool {
    let dx1 = b.x() - a.x();
    let dy1 = b.y() - a.y();
    let dx2 = c.x() - b.x();
    let dy2 = c.y() - b.y();

    // Horizontal switchback: both horizontal but opposite direction.
    if GeomConstants::close(dy1, 0.0) && GeomConstants::close(dy2, 0.0) {
        return dx1 * dx2 < 0.0;
    }
    // Vertical switchback.
    if GeomConstants::close(dx1, 0.0) && GeomConstants::close(dx2, 0.0) {
        return dy1 * dy2 < 0.0;
    }
    false
}

/// Snap a point to be rectilinear relative to the previous point.
fn rectilinearise(a: Point, b: Point) -> Point {
    if GeomConstants::close(a.x(), b.x()) || GeomConstants::close(a.y(), b.y()) {
        return b;
    }
    let dx = (a.x() - b.x()).abs();
    let dy = (a.y() - b.y()).abs();
    if dx < dy {
        Point::new(a.x(), b.y())
    } else {
        Point::new(b.x(), a.y())
    }
}
