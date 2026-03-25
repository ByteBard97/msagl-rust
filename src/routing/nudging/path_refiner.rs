//! PathRefiner: pre-processes paths before nudging.
//!
//! - Removes duplicate/near-duplicate consecutive points.
//! - For each perpendicular coordinate where segments exist, ensures
//!   all collinear paths have vertices at that coordinate.
//! - Finds intersections between path segments and inserts crossing points.

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use std::collections::{BTreeMap, BTreeSet};

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
    refine_in_direction(paths, true);  // horizontal segments (same y)
    refine_in_direction(paths, false); // vertical segments (same x)
    cross_vertical_and_horizontal(paths);
}

/// Remove consecutive duplicate points from a path.
fn deduplicate(path: &mut Vec<Point>) {
    if path.len() < 2 {
        return;
    }
    path.dedup_by(|a, b| a.close_to(*b));
}

/// For all segments parallel to a given direction, collect all distinct
/// perpendicular coordinates and insert subdivision points so every
/// collinear segment has vertices at those coordinates.
fn refine_in_direction(paths: &mut [Vec<Point>], horizontal: bool) {
    // Group segments by their perpendicular coordinate.
    // For horizontal segments (same y), group by y; subdivide along x.
    // For vertical segments (same x), group by x; subdivide along y.
    let perp = |p: &Point| -> OrderedFloat<f64> {
        if horizontal { OrderedFloat(p.y()) } else { OrderedFloat(p.x()) }
    };
    let along = |p: &Point| -> f64 {
        if horizontal { p.x() } else { p.y() }
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
                if !GeomConstants::close(
                    perp(&p0).into_inner(),
                    perp(&p1).into_inner(),
                ) {
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
                        .filter(|&v| {
                            !GeomConstants::close(v, a0) && !GeomConstants::close(v, a1)
                        })
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
fn cross_vertical_and_horizontal(paths: &mut [Vec<Point>]) {
    // Collect all horizontal and vertical segments across all paths.
    struct Seg {
        path_idx: usize,
        seg_idx: usize,
        p0: Point,
        p1: Point,
    }

    let mut horizontals: Vec<Seg> = Vec::new();
    let mut verticals: Vec<Seg> = Vec::new();

    for (pi, path) in paths.iter().enumerate() {
        for si in 0..path.len().saturating_sub(1) {
            let (p0, p1) = (path[si], path[si + 1]);
            if GeomConstants::close(p0.y(), p1.y()) {
                horizontals.push(Seg { path_idx: pi, seg_idx: si, p0, p1 });
            } else if GeomConstants::close(p0.x(), p1.x()) {
                verticals.push(Seg { path_idx: pi, seg_idx: si, p0, p1 });
            }
        }
    }

    // For each (horizontal, vertical) pair from different paths,
    // check if they cross and record insertion points.
    // insertions[path_idx] = list of (seg_idx, point)
    let mut insertions: Vec<Vec<(usize, Point)>> = vec![Vec::new(); paths.len()];

    for h in &horizontals {
        for v in &verticals {
            if h.path_idx == v.path_idx {
                continue;
            }
            let hy = h.p0.y();
            let vx = v.p0.x();
            let hx_min = h.p0.x().min(h.p1.x());
            let hx_max = h.p0.x().max(h.p1.x());
            let vy_min = v.p0.y().min(v.p1.y());
            let vy_max = v.p0.y().max(v.p1.y());

            let eps = GeomConstants::DISTANCE_EPSILON;
            if vx > hx_min + eps && vx < hx_max - eps
                && hy > vy_min + eps && hy < vy_max - eps
            {
                let cross = Point::new(vx, hy);
                insertions[h.path_idx].push((h.seg_idx, cross));
                insertions[v.path_idx].push((v.seg_idx, cross));
            }
        }
    }

    // Apply insertions (rebuild paths with crossing points).
    for (pi, ins) in insertions.iter_mut().enumerate() {
        if ins.is_empty() {
            continue;
        }
        ins.sort_by(|a, b| a.0.cmp(&b.0));
        let path = &paths[pi];
        let mut new_path: Vec<Point> = Vec::with_capacity(path.len() + ins.len());
        let mut ins_idx = 0;
        for (i, &pt) in path.iter().enumerate() {
            new_path.push(pt);
            while ins_idx < ins.len() && ins[ins_idx].0 == i {
                let pt = ins[ins_idx].1;
                // Only insert if not too close to last added point.
                if let Some(&last) = new_path.last() {
                    if !last.close_to(pt) {
                        new_path.push(pt);
                    }
                }
                ins_idx += 1;
            }
        }
        paths[pi] = new_path;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deduplicate_removes_consecutive_dups() {
        let mut path = vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
        ];
        deduplicate(&mut path);
        assert_eq!(path.len(), 3);
    }

    #[test]
    fn refine_inserts_subdivision_points() {
        // Two horizontal segments on y=0: one from x=0..10, another from x=5..15.
        // After refinement, both should have a vertex at x=5 and x=10.
        let mut paths = vec![
            vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            vec![Point::new(5.0, 0.0), Point::new(15.0, 0.0)],
        ];
        refine_paths(&mut paths);
        // Path 0 should now have points at x=0, 5, 10
        assert!(paths[0].len() >= 3, "path0 len = {}", paths[0].len());
        // Path 1 should now have points at x=5, 10, 15
        assert!(paths[1].len() >= 3, "path1 len = {}", paths[1].len());
    }
}
