//! PathMerger: removes self-loops and collapses multi-crossing paths.
//!
//! Faithful port of PathMerger.ts (160 lines). Avoids a situation where two
//! paths cross each other more than once, and removes self-loops.
//!
//! The algorithm:
//! 1. Build a map from each vertex point to (path_index -> linked_point_index).
//! 2. During construction, detect self-cycles (same point visited twice on
//!    one path) and short-circuit the loop.
//! 3. For each path, walk its vertices. Track which other paths share those
//!    vertices. When another path departs and returns, collapse the looping
//!    path's detour to match the stem path's route between those points.

use std::collections::HashMap;

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use super::linked_point::{LinkedPointIndex, LinkedPointList};

/// Hashable point key for HashMap lookup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct PointKey(OrderedFloat<f64>, OrderedFloat<f64>);

impl From<Point> for PointKey {
    fn from(p: Point) -> Self {
        PointKey(OrderedFloat(p.x()), OrderedFloat(p.y()))
    }
}

/// Per-path head index for the linked list within the shared arena.
#[derive(Debug, Clone, Copy)]
struct PathHead {
    head: LinkedPointIndex,
}

/// Removes self-loops and multi-crossing patterns from paths.
/// Faithful port of PathMerger.ts.
pub struct PathMerger;

impl PathMerger {
    /// Remove self-cycles and collapse multi-crossing paths.
    ///
    /// Converts paths to linked-list form, runs the merge algorithm from
    /// PathMerger.ts, and writes the results back to `paths`.
    pub fn merge_paths(paths: &mut [Vec<Point>]) {
        if paths.len() < 1 {
            return;
        }

        // Build a shared arena and per-path head indices.
        let mut arena = LinkedPointList::new();
        let mut heads: Vec<Option<PathHead>> = Vec::with_capacity(paths.len());

        for path in paths.iter() {
            if path.is_empty() {
                heads.push(None);
                continue;
            }
            let first = arena.add(path[0]);
            let mut prev = first;
            for &pt in &path[1..] {
                let cur = arena.add(pt);
                arena.set_next(prev, Some(cur));
                prev = cur;
            }
            heads.push(Some(PathHead { head: first }));
        }

        // vertices_to_path_offsets: Point -> (path_idx -> LinkedPointIndex)
        let mut vertices_to_path_offsets: HashMap<PointKey, HashMap<usize, LinkedPointIndex>> =
            HashMap::new();

        // Phase 1: InitVerticesToPathOffsetsAndRemoveSelfCycles
        Self::init_vertices_and_remove_self_cycles(
            &mut arena,
            &heads,
            &mut vertices_to_path_offsets,
        );

        // Phase 2: ProcessPath for each path
        for path_idx in 0..heads.len() {
            if heads[path_idx].is_none() {
                continue;
            }
            Self::process_path(
                path_idx,
                &mut arena,
                &heads,
                &mut vertices_to_path_offsets,
            );
        }

        // Write results back to paths.
        for (path_idx, path) in paths.iter_mut().enumerate() {
            if let Some(ph) = &heads[path_idx] {
                *path = arena.collect_points(ph.head);
            }
        }
    }

    /// Faithful port of InitVerticesToPathOffsetsAndRemoveSelfCycles.
    ///
    /// Walks each path's linked list. For each point:
    /// - If the path already visited this point, we have a self-cycle.
    ///   Clean the disappeared piece and short-circuit (loopPoint.next = current.next).
    /// - Otherwise, record the mapping.
    fn init_vertices_and_remove_self_cycles(
        arena: &mut LinkedPointList,
        heads: &[Option<PathHead>],
        vertices_to_path_offsets: &mut HashMap<PointKey, HashMap<usize, LinkedPointIndex>>,
    ) {
        for (path_idx, ph) in heads.iter().enumerate() {
            let Some(ph) = ph else { continue };
            let mut current = Some(ph.head);

            while let Some(idx) = current {
                let point = arena.point(idx);
                let key = PointKey::from(point);

                let path_offsets = vertices_to_path_offsets.entry(key).or_default();

                if let Some(&loop_point) = path_offsets.get(&path_idx) {
                    // Self-cycle detected: clean disappeared piece and short-circuit.
                    // TS: this.CleanDisappearedPiece(loopPoint, linkedPoint, path)
                    Self::clean_disappeared_piece_static(
                        arena,
                        loop_point,
                        idx,
                        path_idx,
                        vertices_to_path_offsets,
                    );
                    // TS: loopPoint.Next = linkedPoint.Next
                    let next_after_current = arena.next(idx);
                    arena.set_next(loop_point, next_after_current);
                } else {
                    path_offsets.insert(path_idx, idx);
                }

                current = arena.next(idx);
            }
        }
    }

    /// Faithful port of ProcessPath.
    ///
    /// For a given stem path, walk its vertices tracking which other paths
    /// share those vertices. When another path departs (leaves a shared vertex)
    /// and returns (reappears at a later shared vertex), collapse the looping
    /// path's detour to match the stem's route.
    fn process_path(
        stem_path_idx: usize,
        arena: &mut LinkedPointList,
        heads: &[Option<PathHead>],
        vertices_to_path_offsets: &mut HashMap<PointKey, HashMap<usize, LinkedPointIndex>>,
    ) {
        let Some(ph) = &heads[stem_path_idx] else {
            return;
        };

        // departed_paths: path_idx -> LinkedPointIndex where it departed
        let mut departed_paths: HashMap<usize, LinkedPointIndex> = HashMap::new();
        let mut prev_location_path_offsets: Option<HashMap<usize, LinkedPointIndex>> = None;

        let mut current = Some(ph.head);
        while let Some(linked_point) = current {
            let point = arena.point(linked_point);
            let key = PointKey::from(point);

            // Get current path offsets at this vertex (clone to avoid borrow issues).
            let path_offsets = vertices_to_path_offsets
                .get(&key)
                .cloned()
                .unwrap_or_default();

            if let Some(ref prev_offsets) = prev_location_path_offsets {
                // Handle returning paths.
                if !departed_paths.is_empty() {
                    // Collect returns to process (can't mutate while iterating).
                    let mut returns: Vec<(usize, LinkedPointIndex, LinkedPointIndex)> = Vec::new();
                    for (&path0, &v) in &path_offsets {
                        if let Some(&departer_lp) = departed_paths.get(&path0) {
                            // path0 has returned!
                            returns.push((path0, departer_lp, v));
                        }
                    }

                    for (path0, departer_lp, arrival_to_looping) in returns {
                        Self::collapse_looping_path(
                            arena,
                            path0,
                            departer_lp,
                            arrival_to_looping,
                            stem_path_idx,
                            linked_point,
                            heads,
                            vertices_to_path_offsets,
                        );
                        departed_paths.remove(&path0);
                    }
                }

                // Find departed paths: paths in prev_offsets but not in current.
                for (&k, &v) in prev_offsets {
                    if !path_offsets.contains_key(&k) {
                        departed_paths.insert(k, v);
                    }
                }
            }

            prev_location_path_offsets = Some(path_offsets);
            current = arena.next(linked_point);
        }
    }

    /// Faithful port of CollapseLoopingPath.
    ///
    /// The looping path has departed from the stem at `departure_from_looping`
    /// and returned at `arrival_to_looping`. Replace the looping path's detour
    /// with the stem path's route between those same points.
    #[allow(clippy::too_many_arguments)]
    fn collapse_looping_path(
        arena: &mut LinkedPointList,
        looping_path_idx: usize,
        departure_from_looping: LinkedPointIndex,
        arrival_to_looping: LinkedPointIndex,
        stem_path_idx: usize,
        arrival_to_stem: LinkedPointIndex,
        heads: &[Option<PathHead>],
        vertices_to_path_offsets: &mut HashMap<PointKey, HashMap<usize, LinkedPointIndex>>,
    ) {
        // Find the departure point on the stem path.
        let departure_point = arena.point(departure_from_looping);
        let Some(departure_on_stem) =
            Self::find_linked_point_in_path(arena, heads, stem_path_idx, departure_point)
        else {
            return;
        };

        // Collect points between departure and arrival on stem.
        let points_to_insert = Self::get_points_in_between(arena, departure_on_stem, arrival_to_stem);

        // Determine order: is departure before arrival in the looping path?
        if Self::before(arena, departure_from_looping, arrival_to_looping) {
            Self::clean_disappeared_piece_static(
                arena,
                departure_from_looping,
                arrival_to_looping,
                looping_path_idx,
                vertices_to_path_offsets,
            );
            Self::replace_piece(
                arena,
                departure_from_looping,
                arrival_to_looping,
                &points_to_insert,
                looping_path_idx,
                vertices_to_path_offsets,
            );
        } else {
            Self::clean_disappeared_piece_static(
                arena,
                arrival_to_looping,
                departure_from_looping,
                looping_path_idx,
                vertices_to_path_offsets,
            );
            let mut reversed = points_to_insert;
            reversed.reverse();
            Self::replace_piece(
                arena,
                arrival_to_looping,
                departure_from_looping,
                &reversed,
                looping_path_idx,
                vertices_to_path_offsets,
            );
        }
    }

    /// Faithful port of GetPointsInBetween.
    /// Returns all points strictly between `a` and `b` (exclusive of both).
    fn get_points_in_between(
        arena: &LinkedPointList,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
    ) -> Vec<Point> {
        let mut result = Vec::new();
        let mut current = arena.next(a);
        while let Some(idx) = current {
            if idx == b {
                break;
            }
            result.push(arena.point(idx));
            current = arena.next(idx);
        }
        result
    }

    /// Faithful port of ReplacePiece.
    /// Replaces the linked-list segment between `a` and `b` with `points`.
    /// After this, a -> [new nodes for points] -> b.
    fn replace_piece(
        arena: &mut LinkedPointList,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
        points: &[Point],
        looping_path_idx: usize,
        vertices_to_path_offsets: &mut HashMap<PointKey, HashMap<usize, LinkedPointIndex>>,
    ) {
        let mut prev = a;
        for &point in points {
            let lp = arena.add(point);
            arena.set_next(prev, Some(lp));
            prev = lp;

            // Update vertices_to_path_offsets.
            let key = PointKey::from(point);
            let path_offset = vertices_to_path_offsets.entry(key).or_default();
            path_offset.insert(looping_path_idx, prev);
        }
        arena.set_next(prev, Some(b));
    }

    /// Faithful port of CleanDisappearedPiece.
    /// Removes the looping path from vertices_to_path_offsets for all points
    /// strictly between `a` and `b`.
    fn clean_disappeared_piece_static(
        arena: &LinkedPointList,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
        looping_path_idx: usize,
        vertices_to_path_offsets: &mut HashMap<PointKey, HashMap<usize, LinkedPointIndex>>,
    ) {
        for point in Self::get_points_in_between(arena, a, b) {
            let key = PointKey::from(point);
            if let Some(path_offset) = vertices_to_path_offsets.get_mut(&key) {
                path_offset.remove(&looping_path_idx);
            }
        }
    }

    /// Faithful port of Before.
    /// Returns true if `a` comes before `b` in the linked list.
    fn before(
        arena: &LinkedPointList,
        a: LinkedPointIndex,
        b: LinkedPointIndex,
    ) -> bool {
        let mut current = arena.next(a);
        while let Some(idx) = current {
            if idx == b {
                return true;
            }
            current = arena.next(idx);
        }
        false
    }

    /// Faithful port of FindLinkedPointInPath.
    /// Finds the LinkedPointIndex in the given path whose point equals `target`.
    fn find_linked_point_in_path(
        arena: &LinkedPointList,
        heads: &[Option<PathHead>],
        path_idx: usize,
        target: Point,
    ) -> Option<LinkedPointIndex> {
        let Some(ph) = &heads[path_idx] else {
            return None;
        };
        let mut current = Some(ph.head);
        while let Some(idx) = current {
            if arena.point(idx) == target {
                return Some(idx);
            }
            current = arena.next(idx);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn self_cycle_removal() {
        // Path: A -> B -> C -> A -> D
        // Self-cycle at A: should become A -> D
        let mut paths = vec![vec![
            pt(0.0, 0.0),
            pt(10.0, 0.0),
            pt(10.0, 10.0),
            pt(0.0, 0.0),
            pt(0.0, 20.0),
        ]];
        PathMerger::merge_paths(&mut paths);
        // After removing self-cycle: A -> D (loop A->B->C->A collapsed)
        assert_eq!(paths[0].len(), 2);
        assert!(paths[0][0].close_to(pt(0.0, 0.0)));
        assert!(paths[0].last().unwrap().close_to(pt(0.0, 20.0)));
    }

    #[test]
    fn no_self_cycle() {
        let mut paths = vec![vec![
            pt(0.0, 0.0),
            pt(10.0, 0.0),
            pt(20.0, 0.0),
        ]];
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths[0].len(), 3);
    }

    #[test]
    fn empty_paths() {
        let mut paths: Vec<Vec<Point>> = vec![];
        PathMerger::merge_paths(&mut paths);
        assert!(paths.is_empty());
    }

    #[test]
    fn single_point_path() {
        let mut paths = vec![vec![pt(5.0, 5.0)]];
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths[0].len(), 1);
    }

    #[test]
    fn two_paths_crossing_once_no_change() {
        // Two paths that share a segment but only cross once.
        let mut paths = vec![
            vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, 10.0)],
            vec![pt(10.0, 0.0), pt(10.0, 10.0), pt(20.0, 10.0)],
        ];
        let original = paths.clone();
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths[0].len(), original[0].len());
        assert_eq!(paths[1].len(), original[1].len());
    }

    #[test]
    fn multi_crossing_collapse() {
        // Two paths that cross each other twice. The looping path shares
        // start/end vertices with the stem but takes a detour through vertices
        // NOT on the stem. The stem has intermediate vertices that the looping
        // path does NOT visit — so the looping path "departs" and "returns".
        //
        // Stem:    A(0,0) -> B(10,0) -> M(15,0) -> C(20,0) -> D(30,0)
        // Looping: A(0,0) -> B(10,0) -> E(10,10) -> F(20,10) -> C(20,0) -> D(30,0)
        //
        // At stem vertex M(15,0), looping path is NOT present -> departs at B.
        // At stem vertex C(20,0), looping path IS present -> returns at C.
        // Collapse should replace looping's B->E->F->C with stem's B->M->C.
        let mut paths = vec![
            vec![pt(0.0, 0.0), pt(10.0, 0.0), pt(15.0, 0.0), pt(20.0, 0.0), pt(30.0, 0.0)],
            vec![
                pt(0.0, 0.0),
                pt(10.0, 0.0),
                pt(10.0, 10.0),
                pt(20.0, 10.0),
                pt(20.0, 0.0),
                pt(30.0, 0.0),
            ],
        ];
        PathMerger::merge_paths(&mut paths);
        // After merging, looping path should be collapsed.
        // B->E->F->C replaced by stem's B->M->C.
        // Looping becomes: A -> B -> M -> C -> D
        assert_eq!(paths[1].len(), 5);
        assert!(paths[1][0].close_to(pt(0.0, 0.0)));
        assert!(paths[1][1].close_to(pt(10.0, 0.0)));
        assert!(paths[1][2].close_to(pt(15.0, 0.0)));
        assert!(paths[1][3].close_to(pt(20.0, 0.0)));
        assert!(paths[1][4].close_to(pt(30.0, 0.0)));
    }

    #[test]
    fn self_cycle_in_middle() {
        // Path: A -> B -> C -> B -> D
        // Self-cycle at B: should become A -> B -> D
        let mut paths = vec![vec![
            pt(0.0, 0.0),
            pt(10.0, 0.0),
            pt(20.0, 0.0),
            pt(10.0, 0.0),
            pt(30.0, 0.0),
        ]];
        PathMerger::merge_paths(&mut paths);
        assert_eq!(paths[0].len(), 3);
        assert!(paths[0][0].close_to(pt(0.0, 0.0)));
        assert!(paths[0][1].close_to(pt(10.0, 0.0)));
        assert!(paths[0][2].close_to(pt(30.0, 0.0)));
    }
}
