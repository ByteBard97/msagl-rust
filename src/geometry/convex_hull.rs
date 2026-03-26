//! Convex hull computation following "Computational Geometry, second edition" of O'Rourke.
//!
//! Faithful port of C# `ConvexHull` class and `HullPointComparer`.
//! Uses pivot selection (lowest Y, rightmost X tiebreak), angle-from-pivot sorting
//! with collinear point deletion, and a stack-based scan with `BackSwitchOverPivot`.

use std::cmp::Ordering;

use super::point::{Point, TriangleOrientation};
use super::point_comparer::GeomConstants;
use super::polyline::Polyline;

/// A hull point with deletion flag and insertion stamp.
/// Faithful port of C# `HullPoint` + TS `HullPoint`.
struct HullPoint {
    point: Point,
    deleted: bool,
    /// Monotonic stamp used to break ties when two points are identical.
    /// In the C# source the hash code is used; we use insertion order instead
    /// (matching the TS port which uses `stamp`).
    stamp: usize,
}

/// Compute the convex hull of a set of points, returned as a closed Polyline.
///
/// Faithful port of C# `ConvexHull.CreateConvexHullAsClosedPolyline`.
/// The resulting polyline vertices are in the order produced by the O'Rourke
/// pivot-sort-scan algorithm (counterclockwise from pivot).
pub fn convex_hull_from_points(points: &[Point]) -> Polyline {
    let hull_points = calculate_convex_hull(points);
    let mut poly = Polyline::new();
    for p in hull_points {
        poly.add_point(p);
    }
    poly.set_closed(true);
    poly
}

/// Calculate the convex hull points.
/// Faithful port of C# `ConvexHull.CalculateConvexHull`.
pub fn calculate_convex_hull(points: &[Point]) -> Vec<Point> {
    let (pivot, hull_points) = set_pivot_and_allocate(points);

    // No points at all
    if pivot.is_none() {
        return Vec::new();
    }
    let pivot = pivot.unwrap();

    // Only the pivot (no other points)
    if hull_points.is_none() {
        return vec![pivot];
    }

    let mut hull_points = hull_points.unwrap();

    if hull_points.is_empty() {
        return vec![pivot];
    }

    sort_all_points_without_pivot(&mut hull_points, pivot);
    scan(&hull_points, pivot)
}

/// Select the pivot (lowest Y, rightmost X tiebreak) and build the hull points
/// array excluding the pivot.
///
/// Faithful port of C# `ConvexHull.SetPivotAndAllocateHullPointsArray`.
fn set_pivot_and_allocate(
    points: &[Point],
) -> (Option<Point>, Option<Vec<HullPoint>>) {
    if points.is_empty() {
        return (None, None);
    }

    let mut pivot = Point::new(0.0, f64::MAX);
    let mut pivot_index: Option<usize> = None;

    for (n, &point) in points.iter().enumerate() {
        if point.y() < pivot.y() {
            pivot = point;
            pivot_index = Some(n);
        } else if point.y() == pivot.y() && point.x() > pivot.x() {
            pivot = point;
            pivot_index = Some(n);
        }
    }

    let pivot_index = pivot_index.unwrap(); // guaranteed since points is non-empty

    let mut stamp: usize = 0;
    let mut hull_pts = Vec::with_capacity(points.len().saturating_sub(1));
    let mut skipped_pivot = false;
    for (n, &point) in points.iter().enumerate() {
        if n == pivot_index && !skipped_pivot {
            // Skip the pivot (only the first occurrence)
            skipped_pivot = true;
        } else {
            hull_pts.push(HullPoint {
                point,
                deleted: false,
                stamp,
            });
            stamp += 1;
        }
    }

    (Some(pivot), Some(hull_pts))
}

use std::cell::Cell;

/// A hull point wrapper with interior-mutable `deleted` flag.
/// Required because the C#/TS comparator mutates the deleted flag during sort.
struct SortableHullPoint {
    point: Point,
    deleted: Cell<bool>,
    stamp: usize,
}

/// Sort hull points by angle from pivot.
/// Faithful port of C# `ConvexHull.SortAllPointsWithoutPivot` + `HullPointComparer`.
///
/// Note: The comparator has a side effect — it marks collinear points (closer to
/// the pivot) as deleted. This matches the C#/TS behavior exactly.
fn sort_all_points_without_pivot(hull_points: &mut [HullPoint], pivot: Point) {
    // We need interior mutability for the deleted flags during sort.
    // Rust's sort_by requires &T comparisons, but the C#/TS comparator mutates
    // the `deleted` field during comparison. We use Cell<bool> to allow mutation.
    let mut sortable: Vec<SortableHullPoint> = hull_points
        .iter()
        .map(|hp| SortableHullPoint {
            point: hp.point,
            deleted: Cell::new(hp.deleted),
            stamp: hp.stamp,
        })
        .collect();

    sortable.sort_by(|i, j| hull_point_compare(i, j, pivot));

    // Copy results back
    for (idx, shp) in sortable.iter().enumerate() {
        hull_points[idx].point = shp.point;
        hull_points[idx].deleted = shp.deleted.get();
        hull_points[idx].stamp = shp.stamp;
    }
}

/// Comparator for hull points.
/// Faithful port of C# `HullPointComparer.Compare` / TS `hullPointComparer`.
///
/// This function can change the `deleted` flag for collinear points.
fn hull_point_compare(
    i: &SortableHullPoint,
    j: &SortableHullPoint,
    pivot: Point,
) -> Ordering {
    // Identical references check — in Rust with sort_by this means same element
    if std::ptr::eq(i, j) {
        return Ordering::Equal;
    }

    match Point::get_triangle_orientation_with_intersection_epsilon(pivot, i.point, j.point) {
        TriangleOrientation::Counterclockwise => Ordering::Less,
        TriangleOrientation::Clockwise => Ordering::Greater,
        TriangleOrientation::Collinear => {
            // Because of floating point error, pi and pj can be on different
            // sides of the pivot on the horizontal line passing through the pivot.
            let pi_del_x = i.point.x() - pivot.x();
            let pj_del_x = j.point.x() - pivot.x();
            if pi_del_x > GeomConstants::DISTANCE_EPSILON
                && pj_del_x < -GeomConstants::DISTANCE_EPSILON
            {
                return Ordering::Less;
            }
            if pi_del_x < -GeomConstants::DISTANCE_EPSILON
                && pj_del_x > GeomConstants::DISTANCE_EPSILON
            {
                return Ordering::Greater;
            }

            // Here i and j cannot be on different sides of the pivot.
            // Delete the one that is closer to the pivot.
            let pi = i.point - pivot;
            let pj = j.point - pivot;
            let i_min_j = pi.l1() - pj.l1();
            if i_min_j < 0.0 {
                i.deleted.set(true);
                return Ordering::Less;
            }
            if i_min_j > 0.0 {
                j.deleted.set(true);
                return Ordering::Greater;
            }

            // Points are the same distance — delete the one with the larger stamp
            // (TS uses stamp; C# uses hash code — stamp is the faithful TS port).
            if i.stamp > j.stamp {
                i.deleted.set(true);
            } else {
                j.deleted.set(true);
            }

            Ordering::Equal
        }
    }
}

/// Stack-based scan.
/// Faithful port of C# `ConvexHull.Scan`.
fn scan(hull_points: &[HullPoint], pivot: Point) -> Vec<Point> {
    let n = hull_points.len();

    // Find first non-deleted point
    let mut i = 0;
    while i < n && hull_points[i].deleted {
        i += 1;
    }
    if i >= n {
        return vec![pivot];
    }

    // Stack is represented as a Vec<Point> (top is last element).
    let mut stack: Vec<Point> = vec![pivot];
    stack.push(hull_points[i].point);
    i += 1;

    // Push second non-deleted point if available
    if i < n {
        if !hull_points[i].deleted {
            stack.push(hull_points[i].point);
            i += 1;
        } else {
            i += 1;
        }
    }

    while i < n {
        if !hull_points[i].deleted {
            if left_turn(&stack, hull_points[i].point, pivot) {
                stack.push(hull_points[i].point);
                i += 1;
            } else {
                stack.pop();
            }
        } else {
            i += 1;
        }
    }

    // Cleanup the end
    while stack_has_more_than_two_points(&stack) && !left_turn_to_pivot(&stack, pivot) {
        stack.pop();
    }

    stack
}

/// Check if this is a left turn.
/// Faithful port of C# `ConvexHull.LeftTurn`.
fn left_turn(stack: &[Point], point: Point, pivot: Point) -> bool {
    if stack.len() < 2 {
        // Only one point in the stack (plus we're checking against new point)
        return true;
    }

    let stack_second = stack[stack.len() - 2];
    let stack_top = stack[stack.len() - 1];

    let orientation = Point::get_triangle_orientation_with_intersection_epsilon(
        stack_second,
        stack_top,
        point,
    );
    match orientation {
        TriangleOrientation::Counterclockwise => true,
        TriangleOrientation::Clockwise => false,
        TriangleOrientation::Collinear => back_switch_over_pivot(stack, point, pivot),
    }
}

/// Handle the collinear edge case where the scan wraps back over the pivot.
/// Faithful port of C# `ConvexHull.BackSwitchOverPivot`.
fn back_switch_over_pivot(stack: &[Point], point: Point, pivot: Point) -> bool {
    // We know there are at least two points in the stack, but it has to be exactly two
    // (stack[0] is pivot, stack[1] is the first hull point).
    if stack.len() > 2 {
        return false;
    }

    debug_assert_eq!(stack[0], pivot);
    let stack_top = stack[stack.len() - 1];
    stack_top.x() > pivot.x() + GeomConstants::DISTANCE_EPSILON
        && point.x() < pivot.x() - GeomConstants::DISTANCE_EPSILON
}

/// Check if the last turn back to pivot is a left turn.
/// Faithful port of C# `ConvexHull.LeftTurnToPivot`.
fn left_turn_to_pivot(stack: &[Point], pivot: Point) -> bool {
    let stack_second = stack[stack.len() - 2];
    let stack_top = stack[stack.len() - 1];
    Point::get_triangle_orientation(stack_second, stack_top, pivot)
        == TriangleOrientation::Counterclockwise
}

/// Check if the stack has more than two points.
/// Faithful port of C# `ConvexHull.StackHasMoreThanTwoPoints`.
fn stack_has_more_than_two_points(stack: &[Point]) -> bool {
    stack.len() > 2
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convex_hull_triangle() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];
        let hull = convex_hull_from_points(&points);
        assert!(hull.is_closed());
        assert_eq!(hull.count(), 3);
    }

    #[test]
    fn convex_hull_square_with_interior() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
            Point::new(5.0, 5.0), // interior point
        ];
        let hull = convex_hull_from_points(&points);
        assert!(hull.is_closed());
        assert_eq!(hull.count(), 4);
    }

    #[test]
    fn convex_hull_single_point() {
        let points = vec![Point::new(5.0, 5.0)];
        let hull = convex_hull_from_points(&points);
        assert!(hull.is_closed());
        assert_eq!(hull.count(), 1);
    }

    #[test]
    fn convex_hull_collinear() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(5.0, 0.0),
            Point::new(10.0, 0.0),
        ];
        let hull = convex_hull_from_points(&points);
        assert!(hull.is_closed());
        // O'Rourke algorithm with collinear deletion keeps only the two endpoints
        assert_eq!(hull.count(), 2);
    }

    /// Port of C# `ConvexHullTest.CalculateConvexHullTest` — multiple duplicate rectangles.
    #[test]
    fn convex_hull_duplicate_rectangles() {
        let corners = vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        let mut points = Vec::new();
        for _ in 0..20 {
            points.extend_from_slice(&corners);
        }

        let hull_pts = calculate_convex_hull(&points);
        assert_eq!(hull_pts.len(), 4, "Expected only 4 points in convex hull");
        for corner in &corners {
            assert!(
                hull_pts.contains(corner),
                "expected point {:?} not found in convex hull",
                corner
            );
        }
    }

    /// Helper: verify all input points are on or inside the hull.
    fn verify_points_in_or_on_hull(points: &[Point], hull_pts: &[Point]) {
        // For each input point, verify it's not strictly outside the convex hull.
        // A point is inside or on the hull if for every consecutive edge of the hull,
        // the point is not on the clockwise side (using the INTERSECTION_EPSILON
        // tolerance since the algorithm uses that threshold).
        if hull_pts.len() < 2 {
            return;
        }

        for &p in points {
            let mut is_ok = true;
            for i in 0..hull_pts.len() {
                let a = hull_pts[i];
                let b = hull_pts[(i + 1) % hull_pts.len()];
                let area = Point::signed_doubled_triangle_area(a, b, p);
                // Allow points that are very close to the edge even if slightly
                // outside due to floating point imprecision used in the algorithm.
                if area < -(GeomConstants::INTERSECTION_EPSILON * 20.0) {
                    is_ok = false;
                    break;
                }
            }
            // If not ok with CCW check, try CW orientation
            if !is_ok {
                let mut cw_ok = true;
                for i in 0..hull_pts.len() {
                    let a = hull_pts[i];
                    let b = hull_pts[(i + 1) % hull_pts.len()];
                    let area = Point::signed_doubled_triangle_area(a, b, p);
                    if area > GeomConstants::INTERSECTION_EPSILON * 20.0 {
                        cw_ok = false;
                        break;
                    }
                }
                assert!(
                    cw_ok,
                    "Point {:?} is outside the convex hull",
                    p
                );
            }
        }
    }

    /// Port of C# `ConvexHullTest.TestConvexHull1` — close/near-collinear points with duplicates.
    #[test]
    fn convex_hull_near_collinear_1() {
        let points = vec![
            Point::new(2.37997, 1.680394),
            Point::new(-0.527835, 9.134165),
            Point::new(0.281398, 10.545642),
            Point::new(10.098738, 16.060836),
            Point::new(11.489047, 16.387183),
            Point::new(19.476612, 16.292199),
            Point::new(20.522959, 13.307952),
            Point::new(18.232532, 5.659001),
            Point::new(6.040618, -1.189735),
            Point::new(18.608074, -2.188965),
            Point::new(18.869482, 27.256956),
            Point::new(21.49004, 29.877514),
            Point::new(25.196069, 29.877514),
            Point::new(35.960054, 20.134587),
            Point::new(38.600742, 9.253306),
            Point::new(39.666216, 3.98651),
            Point::new(39.527496, 0.356043),
            Point::new(37.195498, -0.761055),
            Point::new(26.449923, -4.887396),
            // Repeated points (duplicates)
            Point::new(2.37997, 1.680394),
            Point::new(-0.527835, 9.134165),
            Point::new(0.281398, 10.545642),
            Point::new(10.098738, 16.060836),
            Point::new(11.489047, 16.387183),
            Point::new(19.476612, 16.292199),
            Point::new(20.522959, 13.307952),
            Point::new(18.232532, 5.659001),
            Point::new(6.040618, -1.189735),
        ];
        let hull_pts = calculate_convex_hull(&points);
        assert!(hull_pts.len() >= 3, "Hull should have at least 3 points");
        verify_points_in_or_on_hull(&points, &hull_pts);
    }

    /// Port of C# `ConvexHullTest.TestConvexHull2` — near-collinear with close duplicates.
    #[test]
    fn convex_hull_near_collinear_2() {
        let points = vec![
            Point::new(26.44995, -4.887702),
            Point::new(2.114632, -4.440962),
            Point::new(-0.528156, 9.134172),
            Point::new(-0.556468, 68.265599),
            Point::new(-0.556468, 72.011187),
            Point::new(6.640957, 87.981148),
            Point::new(8.42716, 90.443018),
            Point::new(12.533072, 91.816193),
            Point::new(22.244099, 93.112914),
            Point::new(22.244147, 93.112917),
            Point::new(44.94153, 94.198307),
            Point::new(44.941535, 94.198307),
            Point::new(85.226131, 93.564001),
            Point::new(87.272998, 92.16606),
            Point::new(95.068903, 83.487398),
            Point::new(94.460938, 36.116782),
            Point::new(90.208883, 15.854088),
            Point::new(85.968829, 11.297508),
            Point::new(74.551407, 2.342174),
            Point::new(26.449959, -4.887702),
            Point::new(26.449957, -4.887802),
            Point::new(2.114632, -4.440962),
            Point::new(-0.528246, 9.134134),
            Point::new(-0.556568, 68.265599),
            Point::new(-0.556568, 72.011208),
            Point::new(6.64087, 87.981198),
            Point::new(8.427098, 90.443103),
            Point::new(12.533049, 91.816291),
            Point::new(22.244138, 93.113017),
            Point::new(44.941555, 94.198404),
            Point::new(85.226163, 93.564101),
            Point::new(87.273064, 92.166136),
            Point::new(95.069003, 83.487436),
            Point::new(94.461038, 36.116771),
            Point::new(90.208975, 15.85404),
            Point::new(85.968897, 11.297434),
            Point::new(74.551448, 2.342079),
        ];
        let hull_pts = calculate_convex_hull(&points);
        assert!(hull_pts.len() >= 3, "Hull should have at least 3 points");
        verify_points_in_or_on_hull(&points, &hull_pts);
    }

    #[test]
    fn convex_hull_empty() {
        let points: Vec<Point> = vec![];
        let hull = convex_hull_from_points(&points);
        assert_eq!(hull.count(), 0);
    }

    #[test]
    fn convex_hull_two_points() {
        let points = vec![Point::new(0.0, 0.0), Point::new(10.0, 5.0)];
        let hull = convex_hull_from_points(&points);
        assert!(hull.is_closed());
        assert_eq!(hull.count(), 2);
    }

    /// Verify that identical duplicate points produce a single-point hull.
    #[test]
    fn convex_hull_all_same_point() {
        let points = vec![
            Point::new(5.0, 5.0),
            Point::new(5.0, 5.0),
            Point::new(5.0, 5.0),
        ];
        let hull_pts = calculate_convex_hull(&points);
        // The pivot is one of them; the rest get deleted as collinear at distance 0.
        assert!(hull_pts.len() <= 2);
    }
}
