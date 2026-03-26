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
use super::path_merger::PathMerger;
use super::path_refiner;
use super::staircase_remover;

/// Nudge paths to separate parallel edges.
///
/// Modifies `paths` in-place so that overlapping parallel segments are
/// spread apart by at least `edge_separation`.
pub fn nudge_paths(paths: &mut [Vec<Point>], obstacles: &[Rectangle], edge_separation: f64) {
    if paths.is_empty() {
        return;
    }

    // Save pre-nudge paths in case nudging introduces obstacle crossings.
    let original_paths: Vec<Vec<Point>> = paths.to_vec();

    // Run nudging in both directions (like the TS: North, East, North).
    // TS: nudger.Calculate(Direction.North, true)  — merge paths on first call
    // TS: nudger.Calculate(Direction.East, false)
    // TS: nudger.Calculate(Direction.North, false)
    calculate(paths, obstacles, edge_separation, Direction::North, true);
    calculate(paths, obstacles, edge_separation, Direction::East, false);
    calculate(paths, obstacles, edge_separation, Direction::North, false);

    // Remove staircases.
    staircase_remover::remove_staircases(paths, obstacles);

    // Safety: if nudging caused any path to cross an obstacle it didn't
    // cross before, restore the original path. This guards against the
    // simplified FreeSpaceFinder allowing nudges through obstacles.
    // NOTE: not present in TS/C# (their FreeSpaceFinder is fully implemented).
    restore_if_crossing(paths, &original_paths, obstacles);
}

/// Run one pass of nudging in the given direction.
///
/// Faithfully matches TS `Nudger.Calculate(direction, mergePaths)`:
/// 1. Refine paths (+ optional merge on first call)
/// 2. Build axis-edge DAG and get path ordering
/// 3. Find free space (obstacle bounds)
/// 4. Group into LongestNudgedSegments
/// 5. Solve positions
/// 6. Apply positions + remove switchbacks per path
fn calculate(
    paths: &mut [Vec<Point>],
    obstacles: &[Rectangle],
    edge_separation: f64,
    direction: Direction,
    merge_paths: bool,
) {
    // Step 1: Refine paths.
    path_refiner::refine_paths(paths);

    // Step 1b: Merge paths — only on first Calculate call (like TS).
    if merge_paths {
        PathMerger::merge_paths(paths);
    }

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

    // Step 6: Apply solved positions back to paths, then remove switchbacks.
    // TS: ShiftPathEdges → GetShiftedPoints → RemoveSwitchbacksAndMiddlePoints
    apply_positions(
        paths,
        &result.path_edges,
        &result.path_first_edges,
        &result.axis_edges,
        &longest_segs,
        &positions,
        direction,
    );

    // Switchback/middle-point removal applied per path after shifting.
    // Faithful to TS: GetShiftedPoints wraps GetShiftedPointsSimple with
    // RemoveSwitchbacksAndMiddlePoints.
    for path in paths.iter_mut() {
        let cleaned = remove_switchbacks_and_middle_points(path);
        if cleaned.len() >= 2 {
            *path = cleaned;
        }
    }
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

                let (src, tgt) = edge_source_target(&path_edges[pe_id], ae);
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
                        solver.add_constraint(var_ids[prev_sid], var_ids[sid], edge_separation);
                    }
                }
                prev_seg = Some(sid);
            }
        }
    }

    // Create constraints between segments on neighboring axis edges.
    for seg in segments {
        let right_neighbor_segs =
            collect_right_neighbor_segs(seg, path_edges, axis_edges, segments);
        for &rsid in &right_neighbor_segs {
            if rsid != seg.id {
                solver.add_constraint(var_ids[seg.id], var_ids[rsid], edge_separation);
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
            let seg_id = pe
                .longest_seg_id
                .or_else(|| pe.next.and_then(|nid| path_edges[nid].longest_seg_id));

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

fn opposite_dir(_d: Direction) -> Direction {
    // In our 2-direction enum (North/East), the "opposite" for nudging purposes
    // is the same direction (we only nudge in perpendicular axis).
    _d
}

/// Compass direction bitflags, matching the TS/C# `Direction` enum.
/// Used by `remove_switchbacks_and_middle_points` for direction tracking.
///
/// TS reference: `direction.ts` — `None=0, North=1, East=2, South=4, West=8`.
mod compass_dir {
    pub const NONE: u8 = 0;
    pub const NORTH: u8 = 1;
    pub const EAST: u8 = 2;
    pub const SOUTH: u8 = 4;
    pub const WEST: u8 = 8;

    /// Compute compass direction from point `a` to point `b`.
    /// Faithfully ports `CompassVector.VectorDirectionPP` from TS.
    pub fn vector_direction(a: super::Point, b: super::Point) -> u8 {
        let eps = super::GeomConstants::DISTANCE_EPSILON;
        let mut r: u8 = NONE;
        let horizontal_diff = b.x() - a.x();
        let vertical_diff = b.y() - a.y();
        if horizontal_diff > eps {
            r = EAST;
        } else if -horizontal_diff > eps {
            r = WEST;
        }
        if vertical_diff > eps {
            r |= NORTH;
        } else if -vertical_diff > eps {
            r |= SOUTH;
        }
        r
    }

    /// Return the opposite compass direction.
    /// Faithfully ports `CompassVector.OppositeDir` from TS.
    pub fn opposite_dir(dir: u8) -> u8 {
        match dir {
            NORTH => SOUTH,
            SOUTH => NORTH,
            EAST => WEST,
            WEST => EAST,
            _ => dir, // combined or None — no simple opposite
        }
    }
}

/// Remove switchbacks (direction reversals) and collinear middle points.
///
/// Faithful port of TS `Nudger.RemoveSwitchbacksAndMiddlePoints`.
/// Algorithm: walk the point sequence tracking compass direction. When the
/// direction from `b` to the next point is the same as `prevDir`, its
/// opposite, or `None`, we keep accumulating (skipping middle/switchback
/// points). Otherwise we emit the accumulated point.
fn remove_switchbacks_and_middle_points(points: &[Point]) -> Vec<Point> {
    if points.len() < 2 {
        return points.to_vec();
    }

    let mut ret: Vec<Point> = Vec::with_capacity(points.len());
    let mut a = points[0];
    ret.push(a);
    let mut b = points[1];
    let mut prev_dir = compass_dir::vector_direction(a, b);

    let mut i = 2;
    while i < points.len() {
        let dir = compass_dir::vector_direction(b, points[i]);
        // Continue walking along the same straight line, maybe going backwards.
        if !(dir == prev_dir
            || compass_dir::opposite_dir(dir) == prev_dir
            || dir == compass_dir::NONE)
        {
            // Direction changed — emit a point if a and b are not too close.
            if !a.close_to(b) {
                a = rectilinearise(a, b);
                ret.push(a);
            }
            prev_dir = dir;
        }
        b = points[i];
        i += 1;
    }

    // Emit the final point.
    if !a.close_to(b) {
        ret.push(rectilinearise(a, b));
    }
    ret
}

/// Restore original paths if nudging introduced obstacle crossings.
///
/// For each path, check if any interior segment midpoint (excluding
/// source/target obstacle) now lies strictly inside an obstacle that
/// the original path did not cross. If so, restore the original path.
fn restore_if_crossing(paths: &mut [Vec<Point>], original: &[Vec<Point>], obstacles: &[Rectangle]) {
    for (i, path) in paths.iter_mut().enumerate() {
        if path_crosses_intermediate_obstacle(path, obstacles)
            && !path_crosses_intermediate_obstacle(&original[i], obstacles)
        {
            *path = original[i].clone();
        }
    }
}

/// Tolerance for strictly-inside tests in the obstacle crossing check.
/// Larger than DISTANCE_EPSILON to avoid false positives from paths
/// that legitimately brush obstacle boundaries.
const CROSSING_CHECK_TOLERANCE: f64 = 0.1;

/// Check if any segment midpoint lies strictly inside an obstacle that
/// does not contain the path endpoints.
fn path_crosses_intermediate_obstacle(path: &[Point], obstacles: &[Rectangle]) -> bool {
    if path.len() < 2 {
        return false;
    }
    let source = path.first().unwrap();
    let target = path.last().unwrap();
    let eps = CROSSING_CHECK_TOLERANCE;

    for w in path.windows(2) {
        let mid = Point::new((w[0].x() + w[1].x()) / 2.0, (w[0].y() + w[1].y()) / 2.0);
        for obs in obstacles {
            // Skip obstacles containing source or target.
            let contains_source = source.x() > obs.left() + eps
                && source.x() < obs.right() - eps
                && source.y() > obs.bottom() + eps
                && source.y() < obs.top() - eps;
            let contains_target = target.x() > obs.left() + eps
                && target.x() < obs.right() - eps
                && target.y() > obs.bottom() + eps
                && target.y() < obs.top() - eps;
            if contains_source || contains_target {
                continue;
            }
            let strictly_inside = mid.x() > obs.left() + eps
                && mid.x() < obs.right() - eps
                && mid.y() > obs.bottom() + eps
                && mid.y() < obs.top() - eps;
            if strictly_inside {
                return true;
            }
        }
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
