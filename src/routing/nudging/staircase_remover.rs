//! StaircaseRemover: detects and collapses zig-zag patterns in nudged paths.
//!
//! A staircase is a pattern of 5 consecutive points where segments alternate
//! direction: a-b-c-d-f where dir(a,b)==dir(c,d) and dir(b,c)==dir(d,f).
//! The staircase can be collapsed by replacing the middle 3 points with 2.

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;

/// Remove staircase patterns from all paths.
///
/// Iterates until no more staircases can be removed.
pub fn remove_staircases(paths: &mut [Vec<Point>], obstacles: &[Rectangle]) {
    let mut changed = true;
    while changed {
        changed = false;
        for path in paths.iter_mut() {
            if try_remove_staircase(path, obstacles) {
                changed = true;
            }
        }
    }
}

/// Try to find and remove one staircase in the path. Returns true if one was removed.
fn try_remove_staircase(path: &mut Vec<Point>, obstacles: &[Rectangle]) -> bool {
    if path.len() < 5 {
        return false;
    }

    for i in 0..path.len() - 4 {
        if is_staircase(path, i, obstacles) {
            collapse_staircase(path, i);
            return true;
        }
    }

    false
}

/// Check if points[offset..offset+5] form a staircase pattern.
fn is_staircase(pts: &[Point], offset: usize, obstacles: &[Rectangle]) -> bool {
    let a = pts[offset];
    let b = pts[offset + 1];
    let c = pts[offset + 2];
    let d = pts[offset + 3];
    let f = pts[offset + 4];

    // Check alternating directions: dir(a,b) == dir(c,d) and dir(b,c) == dir(d,f).
    let dir_ab = seg_direction(a, b);
    let dir_cd = seg_direction(c, d);
    let dir_bc = seg_direction(b, c);
    let dir_df = seg_direction(d, f);

    if dir_ab != dir_cd || dir_bc != dir_df {
        return false;
    }

    // Compute the flipped point.
    let flipped = get_flipped_point(pts, offset);

    // Check that the new segments (b-flipped and flipped-d) don't intersect obstacles.
    !segments_intersect_obstacles(b, flipped, d, obstacles)
}

/// Get the flipped point for staircase collapse.
fn get_flipped_point(pts: &[Point], offset: usize) -> Point {
    let horiz = GeomConstants::close(pts[offset].y(), pts[offset + 1].y());
    if horiz {
        Point::new(pts[offset + 4].x(), pts[offset].y())
    } else {
        Point::new(pts[offset].x(), pts[offset + 4].y())
    }
}

/// Check if segments (a-b) and (b-c) intersect any obstacle.
fn segments_intersect_obstacles(a: Point, b: Point, c: Point, obstacles: &[Rectangle]) -> bool {
    let seg1_rect = Rectangle::from_points(a, b);
    let seg2_rect = Rectangle::from_points(b, c);

    for obs in obstacles {
        if obs.intersects(&seg1_rect) || obs.intersects(&seg2_rect) {
            // More precise check: the segment must actually cross the obstacle interior.
            if segment_crosses_obstacle(a, b, obs) || segment_crosses_obstacle(b, c, obs) {
                return true;
            }
        }
    }
    false
}

/// Check if a rectilinear segment strictly crosses an obstacle's interior.
fn segment_crosses_obstacle(p0: Point, p1: Point, obs: &Rectangle) -> bool {
    let eps = GeomConstants::DISTANCE_EPSILON;
    if GeomConstants::close(p0.y(), p1.y()) {
        // Horizontal segment.
        let y = p0.y();
        if y <= obs.bottom() + eps || y >= obs.top() - eps {
            return false;
        }
        let x_min = p0.x().min(p1.x());
        let x_max = p0.x().max(p1.x());
        x_min < obs.right() - eps && x_max > obs.left() + eps
    } else if GeomConstants::close(p0.x(), p1.x()) {
        // Vertical segment.
        let x = p0.x();
        if x <= obs.left() + eps || x >= obs.right() - eps {
            return false;
        }
        let y_min = p0.y().min(p1.y());
        let y_max = p0.y().max(p1.y());
        y_min < obs.top() - eps && y_max > obs.bottom() + eps
    } else {
        false
    }
}

/// Collapse a staircase starting at offset: remove pts[offset+2] and pts[offset+3],
/// replace with the flipped point.
fn collapse_staircase(pts: &mut Vec<Point>, offset: usize) {
    let flipped = get_flipped_point(pts, offset);
    // Replace: keep a, b, insert flipped, keep f (skip c and d).
    // Result: a, b, flipped, f, ...rest
    pts[offset + 2] = flipped;
    pts.remove(offset + 3);
}

/// Get the direction character of a segment (simplified).
fn seg_direction(a: Point, b: Point) -> u8 {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    if dx.abs() > dy.abs() {
        if dx > 0.0 { b'E' } else { b'W' }
    } else if dy > 0.0 {
        b'N'
    } else {
        b'S'
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn removes_simple_staircase() {
        // Staircase pattern: right, up, right, up, right
        // But we need alternating: E, N, E, N pattern over 5 points.
        let mut paths = vec![vec![
            Point::new(0.0, 0.0),
            Point::new(5.0, 0.0),
            Point::new(5.0, 5.0),
            Point::new(10.0, 5.0),
            Point::new(10.0, 10.0),
        ]];
        let original_len = paths[0].len();
        remove_staircases(&mut paths, &[]);
        assert!(
            paths[0].len() < original_len,
            "staircase should have been collapsed: {:?}",
            paths[0]
        );
    }

    #[test]
    fn does_not_remove_non_staircase() {
        let mut paths = vec![vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
        ]];
        let original = paths[0].clone();
        remove_staircases(&mut paths, &[]);
        assert_eq!(paths[0], original);
    }
}
