//! PathMerger: removes self-loops and collapses cross-path loops.
//!
//! Faithfully ported from PathMerger.cs / PathMerger.ts.
//!
//! Two paths that cross each other more than once create a "loop" between them.
//! This module detects those loops and replaces the looping path's segment with
//! the stem path's segment, eliminating unnecessary crossings.
//!
//! Also removes self-cycles (a single path visiting the same point twice).

use std::collections::HashMap;

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;

use super::linked_point::LinkedPointIndex;

/// Removes self-loops and cross-path loop patterns from paths.
///
/// Faithfully ports PathMerger.cs / PathMerger.ts.
pub struct PathMerger {
    /// Map from point -> (path_index -> LinkedPointIndex).
    /// Tracks which paths pass through each vertex and where.
    vertices_to_path_offsets: HashMap<PointKey, HashMap<usize, LinkedPointIndex>>,
    /// Arena of linked-list nodes shared across all paths.
    nodes: Vec<LinkedNode>,
    /// Head index for each path's linked list.
    path_heads: Vec<LinkedPointIndex>,
}

/// A node in the shared arena-based linked list.
#[derive(Debug, Clone)]
struct LinkedNode {
    point: Point,
    next: Option<LinkedPointIndex>,
}

/// Hashable point key for HashMap lookup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct PointKey(OrderedFloat<f64>, OrderedFloat<f64>);

impl From<Point> for PointKey {
    fn from(p: Point) -> Self {
        PointKey(OrderedFloat(p.x()), OrderedFloat(p.y()))
    }
}

impl PathMerger {
    /// Remove self-cycles and cross-path loops from all paths.
    ///
    /// Faithfully ports `PathMerger.MergePaths()` from C#/TS:
    /// 1. Build linked lists for all paths and remove self-cycles
    /// 2. Process each path to detect and collapse cross-path loops
    /// 3. Collect results back to Vec<Point>
    pub fn merge_paths(paths: &mut [Vec<Point>]) {
        if paths.is_empty() {
            return;
        }

        let mut merger = Self::new(paths);
        merger.init_vertices_and_remove_self_cycles();
        let path_count = merger.path_heads.len();
        for path_idx in 0..path_count {
            merger.process_path(path_idx);
        }
        merger.collect_into(paths);
    }

    /// Build the merger from a set of paths, converting each to a linked list.
    fn new(paths: &[Vec<Point>]) -> Self {
        let mut nodes = Vec::new();
        let mut path_heads = Vec::with_capacity(paths.len());

        for path in paths {
            if path.is_empty() {
                // Degenerate: store a dummy head (will produce empty path on collect).
                // This shouldn't happen in practice, but handle gracefully.
                let idx = LinkedPointIndex(nodes.len());
                nodes.push(LinkedNode {
                    point: Point::new(0.0, 0.0),
                    next: None,
                });
                path_heads.push(idx);
                continue;
            }
            let head_idx = LinkedPointIndex(nodes.len());
            for (i, &pt) in path.iter().enumerate() {
                let next = if i + 1 < path.len() {
                    Some(LinkedPointIndex(nodes.len() + 1))
                } else {
                    None
                };
                nodes.push(LinkedNode { point: pt, next });
            }
            path_heads.push(head_idx);
        }

        Self {
            vertices_to_path_offsets: HashMap::new(),
            nodes,
            path_heads,
        }
    }

    /// Faithfully ports `InitVerticesToPathOffsetsAndRemoveSelfCycles`.
    ///
    /// Walks each path's linked list. For each vertex, checks if this path
    /// has already visited that point (self-cycle). If so, removes the loop
    /// by splicing the linked list. Otherwise, records the path's position
    /// at this vertex.
    fn init_vertices_and_remove_self_cycles(&mut self) {
        let path_count = self.path_heads.len();
        for path_idx in 0..path_count {
            let mut lp_idx = Some(self.path_heads[path_idx]);
            while let Some(current) = lp_idx {
                let point = self.nodes[current.0].point;
                let key = PointKey::from(point);

                let path_offsets = self
                    .vertices_to_path_offsets
                    .entry(key)
                    .or_default();

                if let Some(&loop_point) = path_offsets.get(&path_idx) {
                    // Self-cycle: path visits this point twice.
                    // Clean the disappeared piece (between loop_point and current).
                    self.clean_disappeared_piece(loop_point, current, path_idx);
                    // Splice: loop_point.next = current.next
                    let next_after = self.nodes[current.0].next;
                    self.nodes[loop_point.0].next = next_after;
                    // Continue from next_after
                    lp_idx = next_after;
                } else {
                    path_offsets.insert(path_idx, current);
                    lp_idx = self.nodes[current.0].next;
                }
            }
        }
    }

    /// Faithfully ports `ProcessPath`.
    ///
    /// Walks a path's linked list tracking which other paths share vertices.
    /// When a path "departs" (was present at previous vertex but not current),
    /// it is recorded. When a departed path "returns" at a later vertex,
    /// `collapse_looping_path` is called to merge the two paths' shared segment.
    fn process_path(&mut self, path_idx: usize) {
        let mut departed_paths: HashMap<usize, LinkedPointIndex> = HashMap::new();
        let mut prev_location_path_offsets: Option<HashMap<usize, LinkedPointIndex>> = None;

        let mut lp_idx = Some(self.path_heads[path_idx]);
        while let Some(current) = lp_idx {
            let point = self.nodes[current.0].point;
            let key = PointKey::from(point);

            let path_offsets = match self.vertices_to_path_offsets.get(&key) {
                Some(po) => po.clone(),
                None => HashMap::new(),
            };

            if let Some(ref prev_offsets) = prev_location_path_offsets {
                // Handle returning paths
                if !departed_paths.is_empty() {
                    // Collect returns to process (to avoid borrow issues)
                    let mut returns: Vec<(usize, LinkedPointIndex, LinkedPointIndex)> = Vec::new();
                    for (&path0, &arrival_to_looping) in &path_offsets {
                        if let Some(&departure_lp) = departed_paths.get(&path0) {
                            // This path returned!
                            returns.push((path0, departure_lp, arrival_to_looping));
                        }
                    }
                    for (path0, departure_lp, arrival_to_looping) in returns {
                        self.collapse_looping_path(
                            path0,
                            departure_lp,
                            arrival_to_looping,
                            path_idx,
                            current,
                        );
                        departed_paths.remove(&path0);
                    }
                }

                // Find departed paths: paths in prev_offsets but not in current path_offsets
                for (&k, &v) in prev_offsets {
                    if !path_offsets.contains_key(&k) {
                        departed_paths.insert(k, v);
                    }
                }
            }

            prev_location_path_offsets = Some(path_offsets);
            lp_idx = self.nodes[current.0].next;
        }
    }

    /// Faithfully ports `CollapseLoopingPath`.
    ///
    /// When two paths share vertices A and B (forming a loop between them),
    /// replaces the looping path's A->B segment with the stem path's A->B segment.
    fn collapse_looping_path(
        &mut self,
        looping_path: usize,
        departure_from_looping: LinkedPointIndex,
        arrival_to_looping: LinkedPointIndex,
        stem_path: usize,
        arrival_to_stem: LinkedPointIndex,
    ) {
        let departure_point = self.nodes[departure_from_looping.0].point;
        let departure_on_stem = self.find_linked_point_in_path(stem_path, departure_point);

        let points_to_insert = self.get_points_in_between(departure_on_stem, arrival_to_stem);

        if self.before(departure_from_looping, arrival_to_looping) {
            self.clean_disappeared_piece(
                departure_from_looping,
                arrival_to_looping,
                looping_path,
            );
            self.replace_piece(
                departure_from_looping,
                arrival_to_looping,
                &points_to_insert,
                looping_path,
            );
        } else {
            self.clean_disappeared_piece(
                arrival_to_looping,
                departure_from_looping,
                looping_path,
            );
            let mut reversed = points_to_insert;
            reversed.reverse();
            self.replace_piece(
                arrival_to_looping,
                departure_from_looping,
                &reversed,
                looping_path,
            );
        }
    }

    /// Faithfully ports `GetPointsInBetween`.
    ///
    /// Returns the points strictly between linked nodes `a` and `b`
    /// (exclusive of both endpoints).
    fn get_points_in_between(
        &self,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
    ) -> Vec<Point> {
        let mut result = Vec::new();
        let mut current = self.nodes[a.0].next;
        while let Some(idx) = current {
            if idx == b {
                break;
            }
            result.push(self.nodes[idx.0].point);
            current = self.nodes[idx.0].next;
        }
        result
    }

    /// Faithfully ports `ReplacePiece`.
    ///
    /// Replaces the linked list segment between `a` and `b` with new nodes
    /// for the given points, and updates the vertices_to_path_offsets map.
    fn replace_piece(
        &mut self,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
        points: &[Point],
        looping_path: usize,
    ) {
        let mut prev_idx = a;
        for &point in points {
            // Allocate a new node in the arena
            let new_idx = LinkedPointIndex(self.nodes.len());
            self.nodes.push(LinkedNode {
                point,
                next: None,
            });
            // Link prev -> new
            self.nodes[prev_idx.0].next = Some(new_idx);
            prev_idx = new_idx;

            // Update the vertices_to_path_offsets map
            let key = PointKey::from(point);
            let path_offset = self
                .vertices_to_path_offsets
                .entry(key)
                .or_default();
            debug_assert!(
                !path_offset.contains_key(&looping_path),
                "looping path already present at replaced point"
            );
            path_offset.insert(looping_path, new_idx);
        }
        // Link last new node -> b
        self.nodes[prev_idx.0].next = Some(b);
    }

    /// Faithfully ports `CleanDisappearedPiece`.
    ///
    /// Removes the looping path from the vertices_to_path_offsets map
    /// for all points strictly between `a` and `b`.
    fn clean_disappeared_piece(
        &mut self,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
        looping_path: usize,
    ) {
        let points = self.get_points_in_between(a, b);
        for point in points {
            let key = PointKey::from(point);
            if let Some(path_offset) = self.vertices_to_path_offsets.get_mut(&key) {
                debug_assert!(
                    path_offset.contains_key(&looping_path),
                    "looping path not found at disappeared point"
                );
                path_offset.remove(&looping_path);
            }
        }
    }

    /// Faithfully ports `Before`.
    ///
    /// Checks that node `a` comes before node `b` in the linked list.
    fn before(&self, a: LinkedPointIndex, b: LinkedPointIndex) -> bool {
        let mut current = self.nodes[a.0].next;
        while let Some(idx) = current {
            if idx == b {
                return true;
            }
            current = self.nodes[idx.0].next;
        }
        false
    }

    /// Faithfully ports `FindLinkedPointInPath`.
    ///
    /// Searches a path's linked list for a node with the given point.
    /// Panics if not found (matches C#/TS behavior of throwing on failure).
    fn find_linked_point_in_path(
        &self,
        path_idx: usize,
        point: Point,
    ) -> LinkedPointIndex {
        let key = PointKey::from(point);
        let target_key = key;
        let mut current = Some(self.path_heads[path_idx]);
        while let Some(idx) = current {
            if PointKey::from(self.nodes[idx.0].point) == target_key {
                return idx;
            }
            current = self.nodes[idx.0].next;
        }
        panic!(
            "FindLinkedPointInPath: point ({}, {}) not found in path {}",
            point.x(),
            point.y(),
            path_idx
        );
    }

    /// Collect linked lists back into Vec<Point> paths.
    fn collect_into(&self, paths: &mut [Vec<Point>]) {
        for (i, path) in paths.iter_mut().enumerate() {
            let mut result = Vec::new();
            let mut current = Some(self.path_heads[i]);
            while let Some(idx) = current {
                result.push(self.nodes[idx.0].point);
                current = self.nodes[idx.0].next;
            }
            // Only replace if we got a valid path (at least 2 points).
            if result.len() >= 2 {
                *path = result;
            }
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
    fn test_no_paths() {
        let mut paths: Vec<Vec<Point>> = vec![];
        PathMerger::merge_paths(&mut paths);
        assert!(paths.is_empty());
    }

    #[test]
    fn test_single_path_no_cycle() {
        let mut paths = vec![vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, 10.0)]];
        let original = paths.clone();
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths, original);
    }

    #[test]
    fn test_self_cycle_removal() {
        // Path: A -> B -> C -> B -> D
        // Self-cycle at B: should remove C and second B, yielding A -> B -> D
        let mut paths = vec![vec![
            pt(0.0, 0.0),
            pt(10.0, 0.0),
            pt(10.0, 10.0),
            pt(10.0, 0.0),
            pt(20.0, 0.0),
        ]];
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths[0], vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(20.0, 0.0)]);
    }

    #[test]
    fn test_self_cycle_preserves_minimum_length() {
        // Path with only 2 points that are the same — don't collapse
        let mut paths = vec![vec![pt(5.0, 5.0), pt(5.0, 5.0)]];
        PathMerger::merge_paths(&mut paths);
        // Should preserve at least 2 points
        assert!(paths[0].len() >= 2);
    }

    #[test]
    fn test_cross_path_merging() {
        // Two paths that share vertices at (10,0) and (30,0), forming a loop.
        // Path 0 (stem):    (0,0) -> (10,0) -> (20,0) -> (30,0) -> (40,0)
        // Path 1 (looping): (10,10) -> (10,0) -> (20,10) -> (30,0) -> (30,10)
        //
        // Path 1 shares (10,0) and (30,0) with Path 0. Between those shared
        // points, Path 1 goes through (20,10) while Path 0 goes through (20,0).
        // The cross-path merger should replace Path 1's (10,0)->(20,10)->(30,0)
        // with Path 0's (10,0)->(20,0)->(30,0).
        let mut paths = vec![
            vec![
                pt(0.0, 0.0),
                pt(10.0, 0.0),
                pt(20.0, 0.0),
                pt(30.0, 0.0),
                pt(40.0, 0.0),
            ],
            vec![
                pt(10.0, 10.0),
                pt(10.0, 0.0),
                pt(20.0, 10.0),
                pt(30.0, 0.0),
                pt(30.0, 10.0),
            ],
        ];
        PathMerger::merge_paths(&mut paths);

        // Path 0 should be unchanged
        assert_eq!(
            paths[0],
            vec![
                pt(0.0, 0.0),
                pt(10.0, 0.0),
                pt(20.0, 0.0),
                pt(30.0, 0.0),
                pt(40.0, 0.0),
            ]
        );

        // Path 1 should have its middle segment replaced:
        // (10,10) -> (10,0) -> (20,0) -> (30,0) -> (30,10)
        assert_eq!(
            paths[1],
            vec![
                pt(10.0, 10.0),
                pt(10.0, 0.0),
                pt(20.0, 0.0),
                pt(30.0, 0.0),
                pt(30.0, 10.0),
            ]
        );
    }

    #[test]
    fn test_cross_path_merging_reversed_order() {
        // Same as above but the looping path visits the shared points
        // in reversed order relative to the stem.
        // Path 0 (stem):    (0,0) -> (10,0) -> (20,0) -> (30,0) -> (40,0)
        // Path 1 (looping): (30,10) -> (30,0) -> (20,10) -> (10,0) -> (10,10)
        //
        // Path 1 visits (30,0) first then (10,0), opposite to Path 0's order.
        // The merger should still replace the looping segment.
        let mut paths = vec![
            vec![
                pt(0.0, 0.0),
                pt(10.0, 0.0),
                pt(20.0, 0.0),
                pt(30.0, 0.0),
                pt(40.0, 0.0),
            ],
            vec![
                pt(30.0, 10.0),
                pt(30.0, 0.0),
                pt(20.0, 10.0),
                pt(10.0, 0.0),
                pt(10.0, 10.0),
            ],
        ];
        PathMerger::merge_paths(&mut paths);

        // Path 0 unchanged
        assert_eq!(
            paths[0],
            vec![
                pt(0.0, 0.0),
                pt(10.0, 0.0),
                pt(20.0, 0.0),
                pt(30.0, 0.0),
                pt(40.0, 0.0),
            ]
        );

        // Path 1: the segment between (30,0) and (10,0) should be replaced.
        // Since arrival_to_looping=(10,0) is AFTER departure_from_looping=(30,0)
        // in path 1, Before returns true, so we replace (30,0)->(20,10)->(10,0)
        // with stem's segment from (30,0) to (10,0), reversed:
        // stem from (10,0) to (30,0) has intermediate (20,0), reversed = [(20,0)]
        // So path 1 becomes: (30,10) -> (30,0) -> (20,0) -> (10,0) -> (10,10)
        assert_eq!(
            paths[1],
            vec![
                pt(30.0, 10.0),
                pt(30.0, 0.0),
                pt(20.0, 0.0),
                pt(10.0, 0.0),
                pt(10.0, 10.0),
            ]
        );
    }

    #[test]
    fn test_no_cross_path_merge_needed() {
        // Two paths that share only one vertex — no loop to collapse.
        let mut paths = vec![
            vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(20.0, 0.0)],
            vec![pt(10.0, 10.0), pt(10.0, 0.0), pt(10.0, -10.0)],
        ];
        let original = paths.clone();
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths, original);
    }

    #[test]
    fn test_multiple_self_cycles() {
        // Path with two self-cycles
        // A -> B -> C -> B -> D -> E -> D -> F
        let mut paths = vec![vec![
            pt(0.0, 0.0),   // A
            pt(10.0, 0.0),  // B
            pt(10.0, 10.0), // C
            pt(10.0, 0.0),  // B (cycle 1)
            pt(20.0, 0.0),  // D
            pt(20.0, 10.0), // E
            pt(20.0, 0.0),  // D (cycle 2)
            pt(30.0, 0.0),  // F
        ]];
        PathMerger::merge_paths(&mut paths);
        assert_eq!(
            paths[0],
            vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(20.0, 0.0), pt(30.0, 0.0)]
        );
    }

    #[test]
    fn test_three_paths_no_interference() {
        // Three independent paths — no shared vertices, no cycles.
        let mut paths = vec![
            vec![pt(0.0, 0.0), pt(10.0, 0.0)],
            vec![pt(0.0, 10.0), pt(10.0, 10.0)],
            vec![pt(0.0, 20.0), pt(10.0, 20.0)],
        ];
        let original = paths.clone();
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths, original);
    }
}
