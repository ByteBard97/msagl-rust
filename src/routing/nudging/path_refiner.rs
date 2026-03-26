//! PathRefiner: pre-processes paths before nudging.
//!
//! - Removes duplicate/near-duplicate consecutive points.
//! - For each perpendicular coordinate where segments exist, ensures
//!   all collinear paths have vertices at that coordinate.
//! - Finds intersections between path segments and inserts crossing points.

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use std::collections::{BTreeMap, BTreeSet};

use super::linked_point::{LinkedPointIndex, LinkedPointList};
use super::linked_point_splitter::LinkedPointSplitter;

use ordered_float::OrderedFloat;

/// Refine a set of paths in-place.
///
/// After refinement, collinear segments on the same line share the same
/// set of perpendicular subdivision points, which allows the combinatorial
/// nudger to establish consistent ordering.
pub fn refine_paths(paths: &mut [Vec<Point>]) {
    for path in paths.iter_mut() {
        deduplicate(path);
    }
    refine_in_direction(paths, true); // horizontal segments (same y)
    refine_in_direction(paths, false); // vertical segments (same x)
    cross_vertical_and_horizontal(paths);
}

/// Remove consecutive duplicate points from a path.
/// Never collapses below 2 points — a degenerate path (source == target
/// for coincident obstacles) is still valid.
pub fn deduplicate(path: &mut Vec<Point>) {
    if path.len() < 2 {
        return;
    }
    path.dedup_by(|a, b| a.close_to(*b));
    // Restore the minimum 2-point invariant for degenerate self-loop paths.
    if path.len() < 2 {
        path.push(*path.last().unwrap());
    }
}

/// For all segments parallel to a given direction, collect all distinct
/// perpendicular coordinates and insert subdivision points so every
/// collinear segment has vertices at those coordinates.
fn refine_in_direction(paths: &mut [Vec<Point>], horizontal: bool) {
    // Group segments by their perpendicular coordinate.
    // For horizontal segments (same y), group by y; subdivide along x.
    // For vertical segments (same x), group by x; subdivide along y.
    let perp = |p: &Point| -> OrderedFloat<f64> {
        if horizontal {
            OrderedFloat(p.y())
        } else {
            OrderedFloat(p.x())
        }
    };
    let along = |p: &Point| -> f64 {
        if horizontal {
            p.x()
        } else {
            p.y()
        }
    };

    // Collect: for each perp-coordinate bucket, the set of along-coordinates.
    let mut buckets: BTreeMap<OrderedFloat<f64>, BTreeSet<OrderedFloat<f64>>> = BTreeMap::new();

    for path in paths.iter() {
        for w in path.windows(2) {
            let (p0, p1) = (w[0], w[1]);
            if GeomConstants::close(perp(&p0).into_inner(), perp(&p1).into_inner()) {
                let key = perp(&p0);
                let bucket = buckets.entry(key).or_default();
                bucket.insert(OrderedFloat(along(&p0)));
                bucket.insert(OrderedFloat(along(&p1)));
            }
        }
    }

    // For each path, for each segment in the matching bucket, insert
    // intermediate points.
    for path in paths.iter_mut() {
        let mut new_points: Vec<Point> = Vec::with_capacity(path.len() * 2);
        for i in 0..path.len() {
            new_points.push(path[i]);
            if i + 1 < path.len() {
                let (p0, p1) = (path[i], path[i + 1]);
                if !GeomConstants::close(perp(&p0).into_inner(), perp(&p1).into_inner()) {
                    continue;
                }
                let key = perp(&p0);
                if let Some(bucket) = buckets.get(&key) {
                    let a0 = along(&p0);
                    let a1 = along(&p1);
                    let (lo, hi, reversed) = if a0 <= a1 {
                        (a0, a1, false)
                    } else {
                        (a1, a0, true)
                    };
                    let mut intermediates: Vec<f64> = bucket
                        .range(OrderedFloat(lo)..=OrderedFloat(hi))
                        .map(|v| v.into_inner())
                        .filter(|&v| !GeomConstants::close(v, a0) && !GeomConstants::close(v, a1))
                        .collect();
                    if reversed {
                        intermediates.reverse();
                    }
                    for coord in intermediates {
                        let pt = if horizontal {
                            Point::new(coord, p0.y())
                        } else {
                            Point::new(p0.x(), coord)
                        };
                        new_points.push(pt);
                    }
                }
            }
        }
        *path = new_points;
    }
}

/// Insert crossing points where a horizontal segment intersects a vertical one.
///
/// Uses the sweep-line `LinkedPointSplitter` (ported from C#/TS) instead of
/// brute-force O(H*V) comparison. The splitter processes events sorted by Y,
/// maintaining a balanced tree of active vertical segments for O((H+V) log V)
/// performance.
fn cross_vertical_and_horizontal(paths: &mut [Vec<Point>]) {
    // Build LinkedPointLists for horizontal and vertical segments across all paths.
    // We track (path_index, segment_start_in_path) so we can write results back.
    let mut h_list = LinkedPointList::new();
    let mut v_list = LinkedPointList::new();
    let mut h_starts: Vec<LinkedPointIndex> = Vec::new();
    let mut v_starts: Vec<LinkedPointIndex> = Vec::new();

    // Track which linked-point nodes belong to which path segment, so we can
    // reconstruct paths afterwards. Each entry: (path_idx, seg_idx_in_path, start_node, is_horizontal).
    struct SegInfo {
        path_idx: usize,
        seg_idx: usize,
        start: LinkedPointIndex,
        is_horizontal: bool,
    }
    let mut seg_infos: Vec<SegInfo> = Vec::new();

    for (pi, path) in paths.iter().enumerate() {
        for si in 0..path.len().saturating_sub(1) {
            let (p0, p1) = (path[si], path[si + 1]);
            if GeomConstants::close(p0.y(), p1.y()) && !GeomConstants::close(p0.x(), p1.x()) {
                // Horizontal segment.
                let start = h_list.add(p0);
                let end = h_list.add(p1);
                // Link start -> end.
                h_list.insert_after_raw(start, end);
                h_starts.push(start);
                seg_infos.push(SegInfo {
                    path_idx: pi,
                    seg_idx: si,
                    start,
                    is_horizontal: true,
                });
            } else if GeomConstants::close(p0.x(), p1.x()) && !GeomConstants::close(p0.y(), p1.y())
            {
                // Vertical segment.
                let start = v_list.add(p0);
                let end = v_list.add(p1);
                v_list.insert_after_raw(start, end);
                v_starts.push(start);
                seg_infos.push(SegInfo {
                    path_idx: pi,
                    seg_idx: si,
                    start,
                    is_horizontal: false,
                });
            }
        }
    }

    if h_starts.is_empty() || v_starts.is_empty() {
        return; // No crossings possible.
    }

    // Run the sweep-line splitter — inserts crossing points into both lists.
    LinkedPointSplitter::split(&mut h_list, &h_starts, &mut v_list, &v_starts);

    // Collect insertion points per path segment. For each segment, walk from
    // its start node to the original endpoint, collecting any newly-inserted
    // intermediate points.
    // insertions[path_idx] = Vec<(seg_idx, point)>
    let mut insertions: Vec<Vec<(usize, Point)>> = vec![Vec::new(); paths.len()];

    for info in &seg_infos {
        let list = if info.is_horizontal {
            &h_list
        } else {
            &v_list
        };
        // Walk linked list from start. The original segment was just start -> end.
        // Any nodes between them are crossing points inserted by the splitter.
        let start_pt = list.point(info.start);
        let mut cur = list.next(info.start);
        while let Some(idx) = cur {
            let pt = list.point(idx);
            let next = list.next(idx);
            if next.is_none() {
                // This is the original endpoint — don't insert it.
                break;
            }
            // This is an intermediate crossing point.
            if !pt.close_to(start_pt) {
                insertions[info.path_idx].push((info.seg_idx, pt));
            }
            cur = next;
        }
    }

    // Apply insertions (rebuild paths with crossing points).
    for (pi, ins) in insertions.iter_mut().enumerate() {
        if ins.is_empty() {
            continue;
        }
        // Sort by segment index, then by distance from segment start for stability.
        ins.sort_by(|a, b| a.0.cmp(&b.0));
        let path = &paths[pi];
        let mut new_path: Vec<Point> = Vec::with_capacity(path.len() + ins.len());
        let mut ins_idx = 0;
        for (i, &pt) in path.iter().enumerate() {
            new_path.push(pt);
            while ins_idx < ins.len() && ins[ins_idx].0 == i {
                let cross_pt = ins[ins_idx].1;
                // Only insert if not too close to last added point.
                if let Some(&last) = new_path.last() {
                    if !last.close_to(cross_pt) {
                        new_path.push(cross_pt);
                    }
                }
                ins_idx += 1;
            }
        }
        paths[pi] = new_path;
    }
}
