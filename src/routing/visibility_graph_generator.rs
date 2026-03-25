use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VisibilityGraph;
use super::obstacle_tree::ObstacleTree;
use super::scan_direction::ScanDirection;
use super::scan_segment::ScanSegment;
use super::segment_intersector::build_graph_from_segments;
use super::shape::Shape;

/// Generate a visibility graph from rectangular shapes.
///
/// Creates padded obstacles, casts horizontal and vertical rays from each
/// obstacle corner, and intersects the resulting segments to form the graph.
pub fn generate_visibility_graph(shapes: &[Shape], padding: f64) -> VisibilityGraph {
    if shapes.is_empty() {
        return VisibilityGraph::new();
    }

    let obstacle_tree = ObstacleTree::new(shapes, padding);
    let mut graph = VisibilityGraph::new();

    let h_segments = create_segments(&obstacle_tree, ScanDirection::horizontal());
    let v_segments = create_segments(&obstacle_tree, ScanDirection::vertical());

    build_graph_from_segments(&mut graph, &h_segments, &v_segments);

    graph
}

/// Create scan segments by sweeping in the given direction.
///
/// For horizontal direction: creates horizontal segments (constant Y).
/// For each unique Y coordinate among obstacle corners, finds the gaps
/// between obstacles and creates segments spanning those gaps.
///
/// For vertical direction: creates vertical segments (constant X).
fn create_segments(tree: &ObstacleTree, direction: ScanDirection) -> Vec<ScanSegment> {
    let mut segments = Vec::new();

    // Collect all unique perpendicular coordinates (Y for horizontal, X for vertical)
    let mut perp_coords = collect_perp_coordinates(tree, direction);
    perp_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());
    perp_coords.dedup_by(|a, b| GeomConstants::close(*a, *b));

    // For each perpendicular coordinate, cast a ray in the sweep direction
    // and find visible spans (gaps between obstacles)
    for &perp in &perp_coords {
        let spans = find_visible_spans(tree, direction, perp);
        for (start_coord, end_coord) in spans {
            let start = direction.make_point(start_coord, perp);
            let end = direction.make_point(end_coord, perp);
            if !GeomConstants::close(start_coord, end_coord) {
                segments.push(ScanSegment::new(start, end));
            }
        }
    }

    segments
}

/// Collect all unique perpendicular coordinates from obstacle corners.
fn collect_perp_coordinates(tree: &ObstacleTree, direction: ScanDirection) -> Vec<f64> {
    let mut coords = Vec::with_capacity(tree.len() * 2);
    for obs in &tree.obstacles {
        let bb = obs.padded_bounding_box();
        // Each obstacle contributes 2 unique perp coordinates
        let low_perp = direction.perp_coord(bb.left_bottom());
        let high_perp = direction.perp_coord(bb.right_top());
        coords.push(low_perp);
        coords.push(high_perp);
    }
    coords
}

/// Find visible spans along a ray at the given perpendicular coordinate.
///
/// Returns a list of (start_coord, end_coord) pairs representing gaps
/// between obstacles. Each span is a segment of free space.
fn find_visible_spans(
    tree: &ObstacleTree,
    direction: ScanDirection,
    perp: f64,
) -> Vec<(f64, f64)> {
    // Find all obstacles that overlap this perpendicular coordinate
    let mut blocking_intervals = Vec::new();

    for obs in &tree.obstacles {
        let bb = obs.padded_bounding_box();
        let obs_perp_lo = direction.perp_coord(bb.left_bottom());
        let obs_perp_hi = direction.perp_coord(bb.right_top());
        let eps = GeomConstants::DISTANCE_EPSILON;

        // Does this obstacle's perpendicular range strictly contain our coordinate?
        // "Strictly" means the ray at perp passes through the interior.
        // At the boundary (perp == obs_perp_lo or obs_perp_hi), the ray grazes
        // the obstacle corner — this should NOT block, it should pass through.
        if perp > obs_perp_lo + eps && perp < obs_perp_hi - eps {
            let obs_coord_lo = direction.coord(bb.left_bottom());
            let obs_coord_hi = direction.coord(bb.right_top());
            blocking_intervals.push((obs_coord_lo, obs_coord_hi));
        }
    }

    // Sort intervals by start coordinate
    blocking_intervals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    // Merge overlapping intervals
    let merged = merge_intervals(&blocking_intervals);

    // Build spans between/around obstacles along this scan line.
    // We need a bounding range. Use the global extent of all obstacle corners
    // at this perpendicular coordinate.
    let (global_lo, global_hi) = global_coord_range(tree, direction, perp);

    build_gap_spans(&merged, global_lo, global_hi)
}

/// Get the range of sweep coordinates at a given perp coordinate.
/// This is the extent from the leftmost obstacle corner to the rightmost.
fn global_coord_range(tree: &ObstacleTree, direction: ScanDirection, perp: f64) -> (f64, f64) {
    let eps = GeomConstants::DISTANCE_EPSILON;
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;

    for obs in &tree.obstacles {
        let bb = obs.padded_bounding_box();
        let obs_perp_lo = direction.perp_coord(bb.left_bottom());
        let obs_perp_hi = direction.perp_coord(bb.right_top());

        // Include obstacles that touch this perp coordinate (at boundary or interior)
        if perp >= obs_perp_lo - eps && perp <= obs_perp_hi + eps {
            let coord_lo = direction.coord(bb.left_bottom());
            let coord_hi = direction.coord(bb.right_top());
            lo = lo.min(coord_lo);
            hi = hi.max(coord_hi);
        }
    }

    (lo, hi)
}

/// Merge overlapping intervals into non-overlapping sorted intervals.
fn merge_intervals(intervals: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if intervals.is_empty() {
        return Vec::new();
    }
    let mut result = vec![intervals[0]];
    for &(lo, hi) in &intervals[1..] {
        let last = result.last_mut().unwrap();
        if lo <= last.1 + GeomConstants::DISTANCE_EPSILON {
            last.1 = last.1.max(hi);
        } else {
            result.push((lo, hi));
        }
    }
    result
}

/// Build gap spans between blocking intervals within [global_lo, global_hi].
///
/// For each pair of consecutive blocking intervals, the gap between them
/// is a visible span. Also creates spans from the global boundary to the
/// first/last obstacle.
fn build_gap_spans(
    blocking: &[(f64, f64)],
    global_lo: f64,
    global_hi: f64,
) -> Vec<(f64, f64)> {
    let eps = GeomConstants::DISTANCE_EPSILON;

    if blocking.is_empty() {
        // No blockers: single span across the entire range
        if global_hi > global_lo + eps {
            return vec![(global_lo, global_hi)];
        }
        return Vec::new();
    }

    let mut spans = Vec::new();

    // Span from global_lo to first blocker
    if blocking[0].0 > global_lo + eps {
        spans.push((global_lo, blocking[0].0));
    }

    // Gaps between consecutive blockers
    for pair in blocking.windows(2) {
        let gap_start = pair[0].1;
        let gap_end = pair[1].0;
        if gap_end > gap_start + eps {
            spans.push((gap_start, gap_end));
        }
    }

    // Span from last blocker to global_hi
    let last_hi = blocking.last().unwrap().1;
    if global_hi > last_hi + eps {
        spans.push((last_hi, global_hi));
    }

    spans
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merge_overlapping_intervals() {
        let intervals = vec![(1.0, 3.0), (2.0, 5.0), (7.0, 9.0)];
        let merged = merge_intervals(&intervals);
        assert_eq!(merged.len(), 2);
        assert!(GeomConstants::close(merged[0].0, 1.0));
        assert!(GeomConstants::close(merged[0].1, 5.0));
        assert!(GeomConstants::close(merged[1].0, 7.0));
        assert!(GeomConstants::close(merged[1].1, 9.0));
    }

    #[test]
    fn gap_spans_between_blockers() {
        let blocking = vec![(2.0, 4.0), (6.0, 8.0)];
        let spans = build_gap_spans(&blocking, 0.0, 10.0);
        assert_eq!(spans.len(), 3);
        // [0, 2], [4, 6], [8, 10]
        assert!(GeomConstants::close(spans[0].0, 0.0));
        assert!(GeomConstants::close(spans[0].1, 2.0));
        assert!(GeomConstants::close(spans[1].0, 4.0));
        assert!(GeomConstants::close(spans[1].1, 6.0));
        assert!(GeomConstants::close(spans[2].0, 8.0));
        assert!(GeomConstants::close(spans[2].1, 10.0));
    }
}
