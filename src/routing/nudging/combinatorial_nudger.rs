//! CombinatorialNudger: determines path ordering on each axis edge.
//!
//! Faithful port of C#/TS `CombinatorialNudger`. Walks all paths, creates
//! AxisEdges for each segment, collects PathEdges onto AxisEdges, then sorts
//! them using topological ordering + recursive walk-ahead comparison.
//!
//! Key algorithms ported:
//! - `WalkGraphEdgesInTopologicalOrderIfPossible` — BFS topological sort
//! - `CompareTwoPathEdges` — entry point trying both directions
//! - `CompareInDirectionStartingFromAxisEdge` — recursive walk-ahead
//! - `FindContinuedDirection` — direction propagation at shared edges
//! - `ProjectionForCompare` — perpendicular projection at fork points

use std::collections::{HashMap, VecDeque};

use crate::geometry::point::Point;
use crate::routing::compass_direction::CompassDirection;
use crate::routing::scan_direction::Direction;

use super::axis_edge::{AxisEdge, AxisEdgeId};
use super::path_edge::{PathEdge, PathEdgeId};

/// Sentinel value indicating that two path edges have not yet been ordered
/// on a shared axis edge.
const NOT_ORDERED: i32 = i32::MAX;

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
///
/// Faithful port of C#/TS `CombinatorialNudger.GetOrder()`:
/// 1. FillTheVisibilityGraphByWalkingThePaths
/// 2. InitPathOrder
/// 3. OrderPaths (topological traversal + walk-ahead comparison)
pub fn get_order(paths: &[Vec<Point>]) -> CombinatorialResult {
    let mut axis_edges: Vec<AxisEdge> = Vec::new();
    let mut path_edges: Vec<PathEdge> = Vec::new();
    let mut path_first_edges: Vec<Option<PathEdgeId>> = Vec::new();

    // Map from (source, target) to AxisEdgeId for deduplication.
    // Matches C#/TS: PathVisibilityGraph.AddEdge deduplicates by endpoints.
    let mut edge_map: HashMap<(Point, Point), AxisEdgeId> = HashMap::new();

    // Step 1: FillTheVisibilityGraphByWalkingThePaths
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

    // Step 2: InitPathOrder — collect PathEdges per AxisEdge.
    let mut axis_edge_orders: Vec<Vec<PathEdgeId>> = vec![Vec::new(); axis_edges.len()];
    for (pe_id, pe) in path_edges.iter().enumerate() {
        axis_edge_orders[pe.axis_edge_id].push(pe_id);
    }

    // Step 3: OrderPaths — process axis edges in topological order.
    let topo_order = walk_graph_edges_in_topological_order(&axis_edges);
    for ae_id in topo_order {
        order_path_edges_sharing_edge(
            ae_id,
            &mut axis_edge_orders[ae_id],
            &mut path_edges,
            &axis_edges,
        );
    }

    CombinatorialResult {
        axis_edges,
        path_edges,
        path_first_edges,
        axis_edge_orders,
    }
}

/// Sort PathEdges on a single AxisEdge and assign their indices.
///
/// Faithful port of C# `OrderPathEdgesSharingEdge`.
fn order_path_edges_sharing_edge(
    _ae_id: AxisEdgeId,
    order: &mut [PathEdgeId],
    path_edges: &mut [PathEdge],
    axis_edges: &[AxisEdge],
) {
    if order.len() <= 1 {
        // Still assign index for single-element orders.
        if let Some(&pe_id) = order.first() {
            path_edges[pe_id].index = 0;
        }
        return;
    }

    // Sort using CompareTwoPathEdges — the walk-ahead comparison.
    //
    // The walk-ahead comparison (ported from C#/TS) can produce non-transitive
    // orderings in complex topologies (e.g., a < b, b < c, but a > c) because
    // different pairs walk to different fork points. C#'s List.Sort and JS's
    // Array.sort silently tolerate this, but Rust's sort_by detects the
    // violation and panics.
    //
    // Fix: use insertion sort, which performs only pairwise comparisons and
    // does not check transitivity. This faithfully matches the C# behavior.
    // The lists are small (paths sharing a single axis edge), so O(n^2) is fine.
    insertion_sort_by(order, |&a_id, &b_id| {
        compare_two_path_edges(a_id, b_id, path_edges, axis_edges)
    });

    // Fill the index — C#: `pathEdge.Index = i++`
    for (idx, &pe_id) in order.iter().enumerate() {
        path_edges[pe_id].index = idx as i32;
    }
}

/// Insertion sort: tolerates non-transitive comparison functions.
///
/// Unlike Rust's built-in sort (which panics on total-ordering violations),
/// insertion sort performs only direct pairwise comparisons and produces a
/// valid permutation regardless of transitivity. This matches the behavior
/// of C#'s `List<T>.Sort` and JS's `Array.sort` for this use case.
fn insertion_sort_by<T, F>(slice: &mut [T], mut cmp: F)
where
    F: FnMut(&T, &T) -> i32,
{
    for i in 1..slice.len() {
        let mut j = i;
        while j > 0 && cmp(&slice[j - 1], &slice[j]) > 0 {
            slice.swap(j - 1, j);
            j -= 1;
        }
    }
}

/// Compare two PathEdges sharing the same AxisEdge.
///
/// Faithful port of C# `CompareTwoPathEdges`. Tries walking forward
/// along the axis edge's direction first; if that returns 0, tries the
/// opposite direction (negated).
fn compare_two_path_edges(
    x: PathEdgeId,
    y: PathEdgeId,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
) -> i32 {
    if x == y {
        return 0;
    }
    debug_assert_eq!(path_edges[x].axis_edge_id, path_edges[y].axis_edge_id);

    let ae_id = path_edges[x].axis_edge_id;
    let ae_dir = to_compass(axis_edges[ae_id].direction);

    let r = compare_in_direction_starting_from_axis_edge(
        x, y, ae_id, ae_dir, path_edges, axis_edges,
    );
    if r != 0 {
        return r;
    }
    -compare_in_direction_starting_from_axis_edge(
        x, y, ae_id, ae_dir.opposite(), path_edges, axis_edges,
    )
}

/// Recursive walk-ahead comparison.
///
/// Faithful port of C# `CompareInDirectionStartingFromAxisEdge`.
/// Walks both paths in the given direction until they diverge (fork) or
/// share an already-ordered edge.
///
/// Uses `CompassDirection` (4 variants) for the traversal direction,
/// matching C#/TS's full Direction enum. AxisEdge.direction (2 variants)
/// is compared via `to_compass()`.
fn compare_in_direction_starting_from_axis_edge(
    mut x: PathEdgeId,
    mut y: PathEdgeId,
    mut ae_id: AxisEdgeId,
    mut direction: CompassDirection,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
) -> i32 {
    loop {
        let x_next = get_next_path_edge_in_direction(x, ae_id, direction, path_edges, axis_edges);
        let Some(x_next_id) = x_next else {
            return 0;
        };
        let y_next = get_next_path_edge_in_direction(y, ae_id, direction, path_edges, axis_edges);
        let Some(y_next_id) = y_next else {
            return 0;
        };

        if path_edges[x_next_id].axis_edge_id == path_edges[y_next_id].axis_edge_id {
            // Both paths continue on the same axis edge — check existing order.
            let next_ae_id = path_edges[x_next_id].axis_edge_id;
            direction = find_continued_direction(ae_id, direction, next_ae_id, axis_edges);
            ae_id = next_ae_id;

            let r = get_existing_order(x_next_id, y_next_id, path_edges);
            if r == NOT_ORDERED {
                x = x_next_id;
                y = y_next_id;
                continue;
            }
            return if direction == to_compass(axis_edges[ae_id].direction) {
                r
            } else {
                -r
            };
        }

        // There is a fork — use perpendicular projection to determine order.
        let ae_compass = to_compass(axis_edges[ae_id].direction);
        let fork_vertex = if direction == ae_compass {
            axis_edges[ae_id].target
        } else {
            axis_edges[ae_id].source
        };

        let x_ae = &axis_edges[path_edges[x_next_id].axis_edge_id];
        let y_ae = &axis_edges[path_edges[y_next_id].axis_edge_id];
        let x_fork = other_vertex(x_ae, fork_vertex);
        let y_fork = other_vertex(y_ae, fork_vertex);

        let is_reversed = direction != ae_compass;
        let x_proj = projection_for_compare(&axis_edges[ae_id], is_reversed, x_fork);
        let y_proj = projection_for_compare(&axis_edges[ae_id], is_reversed, y_fork);

        return compare_f64(x_proj, y_proj);
    }
}

/// Get the next PathEdge in the given traversal direction.
///
/// Faithful port of C# `GetNextPathEdgeInDirection`.
/// Compares the traversal direction to the axis edge's canonical direction
/// to determine forward vs backward traversal.
fn get_next_path_edge_in_direction(
    pe_id: PathEdgeId,
    ae_id: AxisEdgeId,
    direction: CompassDirection,
    path_edges: &[PathEdge],
    axis_edges: &[AxisEdge],
) -> Option<PathEdgeId> {
    debug_assert_eq!(path_edges[pe_id].axis_edge_id, ae_id);
    let pe = &path_edges[pe_id];
    if to_compass(axis_edges[ae_id].direction) == direction {
        // Forward: follow next (or prev if reversed).
        if pe.reversed { pe.prev } else { pe.next }
    } else {
        // Backward: follow prev (or next if reversed).
        if pe.reversed { pe.next } else { pe.prev }
    }
}

/// Determine the continued traversal direction when moving from one axis edge
/// to the next.
///
/// Faithful port of C# `FindContinuedDirection`.
fn find_continued_direction(
    edge_id: AxisEdgeId,
    direction: CompassDirection,
    next_ae_id: AxisEdgeId,
    axis_edges: &[AxisEdge],
) -> CompassDirection {
    let edge = &axis_edges[edge_id];
    let next_ae = &axis_edges[next_ae_id];
    let next_compass = to_compass(next_ae.direction);

    if to_compass(edge.direction) == direction {
        // Moving in the edge's canonical direction: shared vertex is edge.target.
        if next_ae.source.close_to(edge.target) {
            next_compass
        } else {
            next_compass.opposite()
        }
    } else {
        // Moving against the edge's canonical direction: shared vertex is edge.source.
        if next_ae.source.close_to(edge.source) {
            next_compass
        } else {
            next_compass.opposite()
        }
    }
}

/// Get the vertex of an axis edge that is NOT `v`.
///
/// Faithful port of C# `OtherVertex`.
fn other_vertex(ae: &AxisEdge, v: Point) -> Point {
    if ae.source.close_to(v) {
        ae.target
    } else {
        ae.source
    }
}

/// Compute perpendicular projection for fork-point comparison.
///
/// Faithful port of C# `ProjectionForCompare`:
/// - North axis edge, not reversed: project to X
/// - North axis edge, reversed: project to -X
/// - East axis edge, not reversed: project to -Y
/// - East axis edge, reversed: project to Y
fn projection_for_compare(ae: &AxisEdge, is_reversed: bool, point: Point) -> f64 {
    match ae.direction {
        Direction::North => {
            if is_reversed { -point.x() } else { point.x() }
        }
        Direction::East => {
            if is_reversed { point.y() } else { -point.y() }
        }
    }
}

/// Get existing ordering between two PathEdges on the same AxisEdge.
///
/// Faithful port of C# `GetExistingOrder`. Returns `NOT_ORDERED` if
/// the edge hasn't been ordered yet (index == -1).
fn get_existing_order(x: PathEdgeId, y: PathEdgeId, path_edges: &[PathEdge]) -> i32 {
    let xi = path_edges[x].index;
    if xi == -1 {
        return NOT_ORDERED;
    }
    let yi = path_edges[y].index;
    debug_assert!(yi != -1, "If x is ordered, y should be too");
    compare_i32(xi, yi)
}

/// Walk graph edges in topological order.
///
/// Faithful port of C# `WalkGraphEdgesInTopologicalOrderIfPossible`.
/// The axis-edge DAG always has edges pointing North or East, so it's a DAG.
/// Uses BFS from source vertices (in-degree 0).
fn walk_graph_edges_in_topological_order(axis_edges: &[AxisEdge]) -> Vec<AxisEdgeId> {
    if axis_edges.is_empty() {
        return Vec::new();
    }

    // Build the vertex-to-edges adjacency structure.
    let mut vertex_out_edges: HashMap<PointKey, Vec<AxisEdgeId>> = HashMap::new();
    let mut vertex_in_degree: HashMap<PointKey, usize> = HashMap::new();

    for (ae_id, ae) in axis_edges.iter().enumerate() {
        let src_key = PointKey::from(ae.source);
        let tgt_key = PointKey::from(ae.target);

        vertex_out_edges.entry(src_key).or_default().push(ae_id);
        vertex_out_edges.entry(tgt_key).or_default();

        *vertex_in_degree.entry(tgt_key).or_insert(0) += 1;
        vertex_in_degree.entry(src_key).or_insert(0);
    }

    // Initialize queue with source vertices (in-degree 0).
    let mut queue: VecDeque<PointKey> = VecDeque::new();
    for (&vertex, &in_deg) in &vertex_in_degree {
        if in_deg == 0 {
            queue.push_back(vertex);
        }
    }

    let mut result: Vec<AxisEdgeId> = Vec::with_capacity(axis_edges.len());

    while let Some(vertex) = queue.pop_front() {
        if let Some(out_edges) = vertex_out_edges.get(&vertex) {
            for &ae_id in out_edges {
                let tgt_key = PointKey::from(axis_edges[ae_id].target);
                let in_deg = vertex_in_degree.get_mut(&tgt_key).unwrap();
                *in_deg -= 1;
                if *in_deg == 0 {
                    queue.push_back(tgt_key);
                }
                result.push(ae_id);
            }
        }
    }

    result
}

/// Canonicalize a segment so source < target (North or East).
fn canonicalize(p0: Point, p1: Point) -> (Point, Point, bool) {
    let dx = p1.x() - p0.x();
    let dy = p1.y() - p0.y();
    if dx.abs() > dy.abs() {
        if dx > 0.0 {
            (p0, p1, false)
        } else {
            (p1, p0, true)
        }
    } else {
        if dy > 0.0 {
            (p0, p1, false)
        } else {
            (p1, p0, true)
        }
    }
}

/// Convert 2-variant `Direction` (North/East) to 4-variant `CompassDirection`.
fn to_compass(d: Direction) -> CompassDirection {
    match d {
        Direction::North => CompassDirection::North,
        Direction::East => CompassDirection::East,
    }
}

/// Compare two f64 values, returning -1, 0, or 1.
///
/// Uses exact comparison (f64::total_cmp), matching C#'s Double.CompareTo
/// which the original code uses at fork-point projections. The previous
/// epsilon-based GeomConstants::compare introduced intransitivity:
/// a ≈ b and b ≈ c within epsilon, but a < c outside epsilon.
fn compare_f64(a: f64, b: f64) -> i32 {
    match a.total_cmp(&b) {
        std::cmp::Ordering::Less => -1,
        std::cmp::Ordering::Equal => 0,
        std::cmp::Ordering::Greater => 1,
    }
}

/// Compare two i32 values, returning -1, 0, or 1.
fn compare_i32(a: i32, b: i32) -> i32 {
    match a.cmp(&b) {
        std::cmp::Ordering::Less => -1,
        std::cmp::Ordering::Equal => 0,
        std::cmp::Ordering::Greater => 1,
    }
}

/// A hashable point key for the topological sort's vertex maps.
/// Uses ordered_float semantics via integer bit representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct PointKey {
    x_bits: i64,
    y_bits: i64,
}

impl PointKey {
    fn from(p: Point) -> Self {
        Self {
            x_bits: p.x().to_bits() as i64,
            y_bits: p.y().to_bits() as i64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn test_single_path_single_edge() {
        let paths = vec![vec![pt(0.0, 0.0), pt(10.0, 0.0)]];
        let result = get_order(&paths);
        assert_eq!(result.axis_edges.len(), 1);
        assert_eq!(result.path_edges.len(), 1);
        assert_eq!(result.path_edges[0].index, 0);
    }

    #[test]
    fn test_two_paths_sharing_edge_fork_determines_order() {
        // Two paths share a vertical segment then fork horizontally.
        // Path 0: (0,0) -> (0,10) -> (5,10)  (forks right)
        // Path 1: (0,0) -> (0,10) -> (-5,10) (forks left)
        // On the shared vertical edge, path 1 should be ordered before
        // path 0 because it forks to the left (lower X).
        let paths = vec![
            vec![pt(0.0, 0.0), pt(0.0, 10.0), pt(5.0, 10.0)],
            vec![pt(0.0, 0.0), pt(0.0, 10.0), pt(-5.0, 10.0)],
        ];
        let result = get_order(&paths);

        let shared_ae = result
            .axis_edges
            .iter()
            .enumerate()
            .find(|(_, ae)| ae.direction == Direction::North)
            .map(|(id, _)| id)
            .unwrap();

        let order = &result.axis_edge_orders[shared_ae];
        assert_eq!(order.len(), 2);

        // Path 1 (forks left, -X) should come before path 0 (forks right, +X).
        let first_path = result.path_edges[order[0]].path_index;
        let second_path = result.path_edges[order[1]].path_index;
        assert_eq!(first_path, 1, "Path forking left should be first");
        assert_eq!(second_path, 0, "Path forking right should be second");
    }

    #[test]
    fn test_topological_order_respects_existing_ordering() {
        // Three paths share two consecutive vertical edges.
        // The ordering on the first edge should be reused on the second via
        // the walk-ahead existing-order shortcut.
        let paths = vec![
            vec![pt(0.0, 0.0), pt(0.0, 10.0), pt(0.0, 20.0), pt(8.0, 20.0)],
            vec![
                pt(0.0, 0.0),
                pt(0.0, 10.0),
                pt(0.0, 20.0),
                pt(-8.0, 20.0),
            ],
            vec![pt(0.0, 0.0), pt(0.0, 10.0), pt(0.0, 20.0), pt(3.0, 20.0)],
        ];
        let result = get_order(&paths);

        for (ae_id, ae) in result.axis_edges.iter().enumerate() {
            if ae.direction == Direction::North {
                let order = &result.axis_edge_orders[ae_id];
                if order.len() == 3 {
                    for &pe_id in order {
                        assert!(result.path_edges[pe_id].index >= 0);
                    }
                }
            }
        }
    }

    #[test]
    fn test_empty_paths() {
        let paths: Vec<Vec<Point>> = vec![];
        let result = get_order(&paths);
        assert!(result.axis_edges.is_empty());
        assert!(result.path_edges.is_empty());
    }

    #[test]
    fn test_reversed_edge_direction() {
        let paths = vec![vec![pt(0.0, 10.0), pt(0.0, 0.0)]];
        let result = get_order(&paths);
        assert_eq!(result.axis_edges.len(), 1);
        assert_eq!(result.axis_edges[0].direction, Direction::North);
        assert!(result.path_edges[0].reversed);
    }

    #[test]
    fn test_walk_ahead_uses_fork_projection() {
        // Two paths share a horizontal edge, then fork vertically.
        // Path 0: (0,0) -> (10,0) -> (10,5)  (forks up, +Y)
        // Path 1: (0,0) -> (10,0) -> (10,-5) (forks down, -Y)
        // For East, not reversed: projection is -Y.
        // Path 0 fork at (10,5): -5.0; Path 1 fork at (10,-5): 5.0
        // -5.0 < 5.0 => path 0 first.
        let paths = vec![
            vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, 5.0)],
            vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, -5.0)],
        ];
        let result = get_order(&paths);

        let shared_ae = result
            .axis_edges
            .iter()
            .enumerate()
            .find(|(_, ae)| ae.direction == Direction::East)
            .map(|(id, _)| id)
            .unwrap();

        let order = &result.axis_edge_orders[shared_ae];
        assert_eq!(order.len(), 2);

        let first_path = result.path_edges[order[0]].path_index;
        let second_path = result.path_edges[order[1]].path_index;
        assert_eq!(first_path, 0);
        assert_eq!(second_path, 1);
    }

    #[test]
    fn test_backward_walk_resolves_ordering() {
        // Two paths share a vertical edge and fork only *before* the
        // shared segment (nothing after). The forward walk returns 0,
        // so CompareTwoPathEdges must use the backward (opposite dir)
        // walk to find the fork and determine ordering.
        //
        // Path 0: (5,0) -> (0,0) -> (0,10)   (comes from the right)
        // Path 1: (-5,0) -> (0,0) -> (0,10)  (comes from the left)
        let paths = vec![
            vec![pt(5.0, 0.0), pt(0.0, 0.0), pt(0.0, 10.0)],
            vec![pt(-5.0, 0.0), pt(0.0, 0.0), pt(0.0, 10.0)],
        ];
        let result = get_order(&paths);

        let shared_ae = result
            .axis_edges
            .iter()
            .enumerate()
            .find(|(_, ae)| {
                ae.direction == Direction::North
                    && ae.source.close_to(pt(0.0, 0.0))
                    && ae.target.close_to(pt(0.0, 10.0))
            })
            .map(|(id, _)| id)
            .unwrap();

        let order = &result.axis_edge_orders[shared_ae];
        assert_eq!(order.len(), 2);

        // Backward walk (South) finds the fork at (0,0): path 0 from (5,0),
        // path 1 from (-5,0). Projection -X gives path 1 < path 0, negated
        // by CompareTwoPathEdges => path 1 ordered first.
        let first_path = result.path_edges[order[0]].path_index;
        let second_path = result.path_edges[order[1]].path_index;
        assert_eq!(first_path, 1, "Path from left should be first");
        assert_eq!(second_path, 0, "Path from right should be second");
    }
}
