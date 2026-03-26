//! Overlap detection and resolution for ObstacleTree.
//!
//! Faithful port of the overlap-related methods from TS `ObstacleTree.ts`:
//! - `OverlapsExist()` — detect whether any obstacles overlap
//! - `AccreteClumps()` — group overlapping rectangles into clumps
//! - `AccreteConvexHulls()` — merge overlapping non-rectangles into convex hulls
//! - `ObstaclesIntersect()` — test two obstacles for intersection or containment
//!
//! Group/cluster routing overlap handling is deferred per the PRD.

use std::collections::{HashMap, HashSet};
use crate::geometry::convex_hull::convex_hull_from_points;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use super::obstacle::Obstacle;
use super::overlap_convex_hull::OverlapConvexHull;

/// Result of testing two obstacles for intersection.
#[derive(Debug, Clone, Copy)]
struct IntersectResult {
    /// Whether the obstacle boundaries actually cross.
    curves_intersect: bool,
    /// Whether obstacle A is fully inside obstacle B.
    a_inside_b: bool,
    /// Whether obstacle B is fully inside obstacle A.
    b_inside_a: bool,
}

/// Resolve all overlaps among the given obstacles.
///
/// Faithful port of TS `ObstacleTree.CreateRoot()` overlap resolution logic:
/// 1. Check all pairs for overlaps
/// 2. Group overlapping rectangles into clumps
/// 3. Group overlapping non-rectangles into convex hulls (transitively)
///
/// Modifies obstacles in place by setting clump/convex hull fields.
pub(crate) fn resolve_overlaps(obstacles: &mut Vec<Obstacle>) {
    if obstacles.is_empty() {
        return;
    }

    // Phase 1: Check if any overlaps exist at all (early exit).
    if !overlaps_exist(obstacles) {
        return;
    }

    // Phase 2: Accrete clumps (overlapping rectangles).
    accrete_clumps(obstacles);

    // Phase 3: Accrete convex hulls (overlapping non-rectangles, transitive).
    accrete_convex_hulls(obstacles);
}

/// Check whether any two obstacles overlap.
///
/// Faithful port of TS `ObstacleTree.OverlapsExist()`.
fn overlaps_exist(obstacles: &[Obstacle]) -> bool {
    let n = obstacles.len();
    for i in 0..n {
        for j in (i + 1)..n {
            // Quick bounding box rejection
            let bb_i = obstacles[i].visibility_bounding_box();
            let bb_j = obstacles[j].visibility_bounding_box();
            if !bb_i.intersects(&bb_j) {
                continue;
            }

            let result = obstacles_intersect(&obstacles[i], &obstacles[j]);
            if result.curves_intersect || result.a_inside_b || result.b_inside_a {
                return true;
            }
        }
    }
    false
}

/// Accrete clumps from overlapping rectangular obstacles.
///
/// Faithful port of TS `ObstacleTree.AccreteClumps()`.
fn accrete_clumps(obstacles: &mut Vec<Obstacle>) {
    let overlap_pairs = find_overlapping_rect_pairs(obstacles);
    if overlap_pairs.is_empty() {
        return;
    }

    let components = connected_components(&overlap_pairs, obstacles.len());

    for component in &components {
        if component.len() <= 1 {
            continue;
        }
        let clump: Vec<usize> = component.clone();
        for &idx in component {
            obstacles[idx].set_clump(clump.clone());
        }
    }
}

/// Find pairs of overlapping rectangular obstacles.
///
/// Faithful port of TS `AccumulateObstaclesForClumps()` +
/// `EvaluateOverlappedPairForClump()`.
fn find_overlapping_rect_pairs(obstacles: &[Obstacle]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    let n = obstacles.len();

    for i in 0..n {
        if !obstacles[i].is_rectangle() || obstacles[i].is_sentinel() {
            continue;
        }
        for j in (i + 1)..n {
            if !obstacles[j].is_rectangle() || obstacles[j].is_sentinel() {
                continue;
            }

            let bb_i = obstacles[i].visibility_bounding_box();
            let bb_j = obstacles[j].visibility_bounding_box();
            if !bb_i.intersects(&bb_j) {
                continue;
            }

            let result = obstacles_intersect(&obstacles[i], &obstacles[j]);
            if result.curves_intersect || result.a_inside_b || result.b_inside_a {
                pairs.push((i, j));
            }
        }
    }

    pairs
}

/// Accrete convex hulls from overlapping non-rectangular obstacles.
/// Transitive: if hull A overlaps obstacle C, they merge.
///
/// Faithful port of TS `ObstacleTree.AccreteConvexHulls()`.
fn accrete_convex_hulls(obstacles: &mut Vec<Obstacle>) {
    loop {
        let pairs = find_overlapping_nonrect_pairs(obstacles);
        if pairs.is_empty() {
            return;
        }
        if !create_convex_hulls(obstacles, &pairs) {
            return;
        }
    }
}

/// Find pairs of overlapping primary non-group obstacles where at least one
/// is non-rectangular or in a convex hull.
///
/// Faithful port of TS `AccumulateObstaclesForConvexHulls()` +
/// `EvaluateOverlappedPairForConvexHull()`.
fn find_overlapping_nonrect_pairs(obstacles: &[Obstacle]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    let n = obstacles.len();

    for i in 0..n {
        if !obstacles[i].is_primary_obstacle() {
            continue;
        }
        for j in (i + 1)..n {
            if !obstacles[j].is_primary_obstacle() {
                continue;
            }

            let bb_i = obstacles[i].visibility_bounding_box();
            let bb_j = obstacles[j].visibility_bounding_box();
            if !bb_i.intersects(&bb_j) {
                continue;
            }

            let result = obstacles_intersect(&obstacles[i], &obstacles[j]);
            if !result.curves_intersect && !result.a_inside_b && !result.b_inside_a {
                continue;
            }

            // If neither is in a convex hull, and both are rectangles, skip
            if !obstacles[i].is_in_convex_hull()
                && !obstacles[j].is_in_convex_hull()
                && obstacles[i].is_rectangle()
                && obstacles[j].is_rectangle()
            {
                continue;
            }

            pairs.push((i, j));
        }
    }

    pairs
}

/// Create convex hulls from connected components of overlap pairs.
/// Returns true if any hull was created.
///
/// Faithful port of TS `ObstacleTree.CreateConvexHulls()`.
fn create_convex_hulls(
    obstacles: &mut Vec<Obstacle>,
    pairs: &[(usize, usize)],
) -> bool {
    if pairs.is_empty() {
        return false;
    }

    // Expand pairs to include existing clump/hull members
    let mut expanded_pairs: Vec<(usize, usize)> = pairs.to_vec();
    for &(a, b) in pairs {
        add_clump_to_pairs(&mut expanded_pairs, obstacles, a);
        add_clump_to_pairs(&mut expanded_pairs, obstacles, b);
        add_hull_to_pairs(&mut expanded_pairs, obstacles, a);
        add_hull_to_pairs(&mut expanded_pairs, obstacles, b);
    }

    let components = connected_components(&expanded_pairs, obstacles.len());
    let mut found = false;

    for component in &components {
        if component.len() <= 1 {
            continue;
        }

        found = true;

        // Collect all points from visibility polylines
        let mut all_points: Vec<Point> = Vec::new();
        for &idx in component {
            let poly = obstacles[idx].visibility_polyline();
            all_points.extend(poly.points());
        }

        let hull_polyline = convex_hull_from_points(&all_points);
        let indices: Vec<usize> = component.clone();
        let hull = OverlapConvexHull::new(hull_polyline, indices.clone());

        for &idx in &indices {
            obstacles[idx].set_convex_hull(hull.clone());
        }
    }

    found
}

/// Add all members of an obstacle's clump as pairs with the obstacle.
fn add_clump_to_pairs(
    pairs: &mut Vec<(usize, usize)>,
    obstacles: &[Obstacle],
    idx: usize,
) {
    if !obstacles[idx].is_overlapped() {
        return;
    }
    for &sibling_idx in obstacles[idx].clump() {
        if sibling_idx != idx {
            pairs.push((idx, sibling_idx));
        }
    }
}

/// Add all members of an obstacle's convex hull as pairs with the obstacle.
fn add_hull_to_pairs(
    pairs: &mut Vec<(usize, usize)>,
    obstacles: &[Obstacle],
    idx: usize,
) {
    if let Some(hull) = obstacles[idx].convex_hull() {
        for &sibling_idx in &hull.obstacle_indices {
            if sibling_idx != idx {
                pairs.push((idx, sibling_idx));
            }
        }
    }
}

/// Test whether two obstacles intersect.
///
/// Faithful port of TS `ObstacleTree.ObstaclesIntersect()`.
fn obstacles_intersect(a: &Obstacle, b: &Obstacle) -> IntersectResult {
    let bb_a = a.visibility_bounding_box();
    let bb_b = b.visibility_bounding_box();

    if !bb_a.intersects(&bb_b) {
        return IntersectResult {
            curves_intersect: false,
            a_inside_b: false,
            b_inside_a: false,
        };
    }

    let a_inside_b = bb_b.contains_rect(&bb_a);
    let b_inside_a = bb_a.contains_rect(&bb_b);

    if a.is_rectangle() && b.is_rectangle() {
        let curves_intersect = !a_inside_b && !b_inside_a
            && rects_border_intersect(&bb_a, &bb_b);
        return IntersectResult {
            curves_intersect,
            a_inside_b,
            b_inside_a,
        };
    }

    // For non-rectangular obstacles, use bounding box overlap as proxy
    let curves_intersect = !a_inside_b && !b_inside_a && bb_a.intersects(&bb_b);

    IntersectResult {
        curves_intersect,
        a_inside_b,
        b_inside_a,
    }
}

/// Check if two rectangles' borders actually cross.
fn rects_border_intersect(a: &Rectangle, b: &Rectangle) -> bool {
    let x_overlap = a.left() < b.right() && b.left() < a.right();
    let y_overlap = a.bottom() < b.top() && b.bottom() < a.top();
    x_overlap && y_overlap
}

/// Find connected components using union-find.
fn connected_components(
    pairs: &[(usize, usize)],
    max_index: usize,
) -> Vec<Vec<usize>> {
    let mut parent: Vec<usize> = (0..max_index).collect();
    let mut rank: Vec<usize> = vec![0; max_index];

    fn find(parent: &mut [usize], x: usize) -> usize {
        if parent[x] != x {
            parent[x] = find(parent, parent[x]);
        }
        parent[x]
    }

    fn union(parent: &mut [usize], rank: &mut [usize], x: usize, y: usize) {
        let rx = find(parent, x);
        let ry = find(parent, y);
        if rx == ry {
            return;
        }
        if rank[rx] < rank[ry] {
            parent[rx] = ry;
        } else if rank[rx] > rank[ry] {
            parent[ry] = rx;
        } else {
            parent[ry] = rx;
            rank[rx] += 1;
        }
    }

    let mut involved: HashSet<usize> = HashSet::new();
    for &(a, b) in pairs {
        involved.insert(a);
        involved.insert(b);
        union(&mut parent, &mut rank, a, b);
    }

    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for &idx in &involved {
        let root = find(&mut parent, idx);
        groups.entry(root).or_default().push(idx);
    }

    groups.into_values().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::routing::shape::Shape;

    fn make_obstacle(x: f64, y: f64, w: f64, h: f64, idx: usize) -> Obstacle {
        let shape = Shape::rectangle(x, y, w, h);
        let mut obs = Obstacle::from_shape(&shape, 0.0, idx);
        obs.ordinal = Obstacle::FIRST_NON_SENTINEL_ORDINAL + idx;
        obs
    }

    #[test]
    fn non_overlapping_rects_produce_no_clumps() {
        let mut obstacles = vec![
            make_obstacle(0.0, 0.0, 10.0, 10.0, 0),
            make_obstacle(20.0, 0.0, 10.0, 10.0, 1),
        ];
        resolve_overlaps(&mut obstacles);
        assert!(!obstacles[0].is_overlapped());
        assert!(!obstacles[1].is_overlapped());
    }

    #[test]
    fn overlapping_rects_form_clump() {
        let mut obstacles = vec![
            make_obstacle(0.0, 0.0, 20.0, 10.0, 0),
            make_obstacle(10.0, 0.0, 20.0, 10.0, 1),
        ];
        resolve_overlaps(&mut obstacles);
        assert!(obstacles[0].is_overlapped());
        assert!(obstacles[1].is_overlapped());
        assert_eq!(obstacles[0].clump().len(), 2);
    }

    #[test]
    fn contained_rect_forms_clump() {
        let mut obstacles = vec![
            make_obstacle(0.0, 0.0, 100.0, 100.0, 0),
            make_obstacle(10.0, 10.0, 10.0, 10.0, 1),
        ];
        resolve_overlaps(&mut obstacles);
        assert!(obstacles[0].is_overlapped());
        assert!(obstacles[1].is_overlapped());
    }

    #[test]
    fn transitive_clump() {
        let mut obstacles = vec![
            make_obstacle(0.0, 0.0, 20.0, 10.0, 0),
            make_obstacle(10.0, 0.0, 20.0, 10.0, 1),
            make_obstacle(20.0, 0.0, 20.0, 10.0, 2),
        ];
        resolve_overlaps(&mut obstacles);
        assert!(obstacles[0].is_overlapped());
        assert!(obstacles[1].is_overlapped());
        assert!(obstacles[2].is_overlapped());
        assert_eq!(obstacles[0].clump().len(), 3);
    }

    #[test]
    fn connected_components_simple() {
        let comps = connected_components(&[(0, 1), (2, 3)], 4);
        assert_eq!(comps.len(), 2);
    }

    #[test]
    fn connected_components_transitive() {
        let comps = connected_components(&[(0, 1), (1, 2)], 3);
        assert_eq!(comps.len(), 1);
        assert_eq!(comps[0].len(), 3);
    }
}
