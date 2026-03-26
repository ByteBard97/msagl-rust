//! Convex hull computation using Andrew's monotone chain algorithm.
//!
//! Faithful port of TS `ConvexHull.createConvexHullAsClosedPolyline()`.

use super::point::Point;
use super::polyline::Polyline;

/// Compute the convex hull of a set of points, returned as a closed Polyline.
///
/// Uses Andrew's monotone chain algorithm (O(n log n)).
/// The resulting polyline is clockwise to match MSAGL conventions.
pub fn convex_hull_from_points(points: &[Point]) -> Polyline {
    if points.len() < 2 {
        let mut poly = Polyline::new();
        for &p in points {
            poly.add_point(p);
        }
        if !points.is_empty() {
            poly.set_closed(true);
        }
        return poly;
    }

    // Sort points lexicographically (x then y)
    let mut sorted: Vec<Point> = points.to_vec();
    sorted.sort_by(|a, b| {
        a.x().partial_cmp(&b.x())
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                a.y().partial_cmp(&b.y())
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });
    sorted.dedup_by(|a, b| (a.x() - b.x()).abs() < 1e-10 && (a.y() - b.y()).abs() < 1e-10);

    if sorted.len() < 2 {
        let mut poly = Polyline::new();
        for &p in &sorted {
            poly.add_point(p);
        }
        poly.set_closed(true);
        return poly;
    }

    let n = sorted.len();
    let mut hull: Vec<Point> = Vec::with_capacity(2 * n);

    // Build lower hull
    for &p in &sorted {
        while hull.len() >= 2 {
            let a = hull[hull.len() - 2];
            let b = hull[hull.len() - 1];
            if cross_2d(a, b, p) <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    // Build upper hull
    let lower_len = hull.len() + 1;
    for &p in sorted.iter().rev() {
        while hull.len() >= lower_len {
            let a = hull[hull.len() - 2];
            let b = hull[hull.len() - 1];
            if cross_2d(a, b, p) <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    // Remove the last point (duplicate of first)
    hull.pop();

    // Andrew's algorithm produces counter-clockwise; MSAGL expects clockwise.
    hull.reverse();

    let mut poly = Polyline::new();
    for p in hull {
        poly.add_point(p);
    }
    poly.set_closed(true);
    poly
}

/// 2D cross product of vectors OA and OB.
/// Positive if counter-clockwise, negative if clockwise, zero if collinear.
fn cross_2d(o: Point, a: Point, b: Point) -> f64 {
    (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x())
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
        // Collinear points produce a degenerate hull of 2 points
        assert_eq!(hull.count(), 2);
    }
}
