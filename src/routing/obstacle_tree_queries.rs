use super::compass_direction::{CompassDirection, Direction};
use super::group_boundary_crossing::{compare_points, PointAndCrossingsList};
use super::obstacle::Obstacle;
use super::obstacle_tree::{points_equal, segment_segment_intersection, ObstacleTree};
use super::scan_direction::ScanDirection;
use super::static_graph_utility::StaticGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;

impl ObstacleTree {
    /// Populate `current_group_boundary_crossing_map` with intersections of the
    /// segment [start, end] against the given group obstacle's boundary.
    ///
    /// Matches C# `AddGroupIntersectionsToRestrictedRay` (lines 730-767).
    ///
    /// For each intersection of [start, end] with the group's padded polyline:
    /// - determine `dir_toward_intersect = GetPureDirection(start, end)`
    /// - determine `dirs_of_side` = direction of the polyline side at the intersection
    /// - if `RotateRight(dir_toward_intersect)` is contained in `dirs_of_side`,
    ///   then `dir_to_inside = Opposite(dir_toward_intersect)`
    ///   else `dir_to_inside = dir_toward_intersect`
    pub(crate) fn add_group_intersections_to_restricted_ray(
        &mut self,
        obstacle_idx: usize,
        start: Point,
        end: Point,
    ) {
        let dir_toward = match CompassDirection::from_points(start, end) {
            Some(d) => d,
            None => return,
        };

        // For rectangular groups (the common case), iterate the 4 padded sides and
        // find axis-aligned intersections with the segment [start, end].
        let padded_pts: Vec<Point> = {
            let obs = &self.obstacles[obstacle_idx];
            let bb = obs.padded_bounding_box();
            // Clockwise: bottom-left, top-left, top-right, bottom-right.
            vec![
                bb.left_bottom(),
                bb.left_top(),
                bb.right_top(),
                bb.right_bottom(),
            ]
        };

        let n = padded_pts.len();
        for i in 0..n {
            let pa = padded_pts[i];
            let pb = padded_pts[(i + 1) % n];

            // Find intersection of [start,end] with side [pa,pb].
            if let Some(intersect) = axis_aligned_segment_intersection(start, end, pa, pb) {
                // Round the intersection (matches C# ApproximateComparer.Round).
                let intersect = Point::round(intersect);

                // Determine the direction of the polyline side (its derivative).
                // The side goes from pa to pb; its direction is a compass direction.
                let dirs_of_side = CompassDirection::from_points(pa, pb);

                // C# logic:
                // dirToInsideOfGroup = dirTowardIntersect (default)
                // if (0 != (dirsOfSide & CompassVector.RotateRight(dirTowardIntersect)))
                //     dirToInsideOfGroup = CompassVector.OppositeDir(dirTowardIntersect)
                let rotate_right = dir_toward.right();
                let dir_to_inside =
                    if dirs_of_side == Some(rotate_right) {
                        dir_toward.opposite()
                    } else {
                        dir_toward
                    };

                self.current_group_boundary_crossing_map.add_intersection(
                    intersect,
                    obstacle_idx,
                    dir_to_inside,
                );
            }
        }
    }

    /// Compute the maximum-visibility segment from `start_point` in `dir`,
    /// clipped to the graph box, restricted by non-group obstacles.
    ///
    /// Also returns the `PointAndCrossingsList` of group boundary crossings
    /// along the restricted segment (or `None` if there are none).
    ///
    /// Matches C# `ObstacleTree.CreateMaxVisibilitySegment` (lines 509-519).
    pub fn create_max_visibility_segment(
        &mut self,
        start_point: Point,
        dir: CompassDirection,
        graph_box: &Rectangle,
    ) -> (Point, Point, Option<PointAndCrossingsList>) {
        // graphBoxBorderIntersect = StaticGraphUtility.RectangleBorderIntersect(this.GraphBox, startPoint, dir)
        let border_coord = StaticGraphUtility::get_rectangle_bound(graph_box, dir);
        let end_point = if StaticGraphUtility::is_vertical(dir) {
            Point::round(Point::new(start_point.x(), border_coord))
        } else {
            Point::round(Point::new(border_coord, start_point.y()))
        };

        // if (PointComparer.GetDirections(startPoint, graphBoxBorderIntersect) == Direction.None)
        if compare_points(start_point, end_point) == std::cmp::Ordering::Equal {
            return (start_point, start_point, None);
        }

        // segment = this.RestrictSegmentWithObstacles(startPoint, graphBoxBorderIntersect)
        let (seg_start, seg_end) =
            self.restrict_segment_with_obstacles(start_point, end_point);

        // pacList = this.CurrentGroupBoundaryCrossingMap.GetOrderedListBetween(segment.Start, segment.End)
        let pac_list = self
            .current_group_boundary_crossing_map
            .get_ordered_list_between(seg_start, seg_end);

        (seg_start, seg_end, pac_list)
    }

    /// Returns true if the line segment from `start_point` to `end_point` crosses
    /// any obstacle (the restricted segment end differs from the original end).
    ///
    /// Matches C# `ObstacleTree.SegmentCrossesAnObstacle()` (line 619).
    pub fn segment_crosses_an_obstacle(&mut self, start_point: Point, end_point: Point) -> bool {
        let (_start, restricted_end) =
            self.restrict_segment_private(start_point, end_point, true);
        !points_equal(restricted_end, end_point)
    }

    /// Returns true if the line segment from `start_point` to `end_point` crosses
    /// any non-group obstacle. Group obstacles are transparent to this query.
    ///
    /// Matches C# `ObstacleTree.SegmentCrossesANonGroupObstacle()` (line 626).
    /// Differs from `segment_crosses_an_obstacle` only in passing
    /// `stop_at_groups = false`, which allows group obstacle boundaries to be
    /// traversed without triggering a crossing result.
    pub fn segment_crosses_a_non_group_obstacle(
        &mut self,
        start_point: Point,
        end_point: Point,
    ) -> bool {
        let (_start, restricted_end) =
            self.restrict_segment_private(start_point, end_point, false);
        !points_equal(restricted_end, end_point)
    }

    /// Returns true if the given point is strictly inside any obstacle
    /// (not on the boundary).
    ///
    /// Matches C# `ObstacleTree.PointIsInsideAnObstacle()` (line 548/552).
    /// Uses the scan direction to shoot a test ray through the obstacle and
    /// count intersections.
    pub fn point_is_inside_an_obstacle(
        &self,
        point: Point,
        scan_direction: ScanDirection,
    ) -> bool {
        self.point_is_inside_an_obstacle_impl(point, scan_direction, None, None)
    }

    /// Returns true if the given point is strictly inside any obstacle,
    /// with a direction-based overload.
    ///
    /// Matches C# overload at line 548.
    pub fn point_is_inside_an_obstacle_dir(&self, point: Point, direction: Direction) -> bool {
        let scan_dir = scan_direction_from_direction(direction);
        self.point_is_inside_an_obstacle(point, scan_dir)
    }

    /// Returns true if the intersection point lies inside an obstacle other than
    /// the two given obstacles (the side obstacle and the event obstacle).
    ///
    /// Matches C# `ObstacleTree.IntersectionIsInsideAnotherObstacle()` (line 540).
    pub fn intersection_is_inside_another_obstacle(
        &self,
        side_obstacle_idx: usize,
        event_obstacle_idx: usize,
        intersect: Point,
        scan_direction: ScanDirection,
    ) -> bool {
        self.point_is_inside_an_obstacle_impl(
            intersect,
            scan_direction,
            Some(event_obstacle_idx),
            Some(side_obstacle_idx),
        )
    }

    /// Internal hit-test implementation.
    ///
    /// Matches C# `InsideObstacleHitTest` callback (line 565) called via
    /// `Root.FirstHitNode`. Ignores up to two obstacles.
    fn point_is_inside_an_obstacle_impl(
        &self,
        point: Point,
        scan_direction: ScanDirection,
        ignore1: Option<usize>,
        ignore2: Option<usize>,
    ) -> bool {
        // Query all obstacles whose bounding box contains the point.
        let candidates = self.query_point(point);

        for idx in candidates {
            let obs = &self.obstacles[idx];

            // C# line 566-567: skip ignored obstacles.
            if Some(obs.index) == ignore1 || Some(obs.index) == ignore2 {
                continue;
            }

            // C# line 571-576: skip group obstacles — they are transparent containers
            // and do not block point-inside tests.
            if obs.is_group() {
                continue;
            }

            // C# line 578: the point must be strictly inside the rectangle interior.
            if !StaticGraphUtility::point_is_in_rectangle_interior(
                point,
                &obs.visibility_bounding_box(),
            ) {
                continue;
            }

            // C# line 585-614: for non-rectangular obstacles, shoot a ray through
            // the bounding box along the scan direction and count intersections.
            // For rectangular obstacles (which is all we currently have), being
            // strictly inside the bounding box interior IS being inside the obstacle.
            if obs.is_rectangle() {
                return true;
            }

            // Non-rectangular obstacle: use ray-casting through the polyline.
            // Shoot a test segment through the obstacle in the scan direction.
            if point_inside_polyline_via_ray(point, obs, scan_direction) {
                return true;
            }
        }

        false
    }
}

/// Convert a Direction to a ScanDirection.
/// Matches C# `ScanDirection.GetInstance(direction)`.
fn scan_direction_from_direction(dir: Direction) -> ScanDirection {
    if dir.contains(Direction::EAST) || dir.contains(Direction::WEST) {
        ScanDirection::horizontal()
    } else {
        ScanDirection::vertical()
    }
}

/// Find the intersection of an axis-aligned segment [seg_a, seg_b] with a
/// single polyline side [side_a, side_b], both axis-aligned.
///
/// Used by `add_group_intersections_to_restricted_ray`.
/// Returns the intersection point if it lies strictly on both segments.
fn axis_aligned_segment_intersection(
    seg_a: Point,
    seg_b: Point,
    side_a: Point,
    side_b: Point,
) -> Option<Point> {
    let eps = 1e-9;

    // Horizontal segment (constant Y) vs vertical side (constant X)
    if (seg_a.y() - seg_b.y()).abs() < eps && (side_a.x() - side_b.x()).abs() < eps {
        let seg_y = seg_a.y();
        let side_x = side_a.x();
        let seg_x_min = seg_a.x().min(seg_b.x());
        let seg_x_max = seg_a.x().max(seg_b.x());
        let side_y_min = side_a.y().min(side_b.y());
        let side_y_max = side_a.y().max(side_b.y());
        if side_x > seg_x_min + eps
            && side_x < seg_x_max - eps
            && seg_y > side_y_min + eps
            && seg_y < side_y_max - eps
        {
            return Some(Point::new(side_x, seg_y));
        }
    }

    // Vertical segment (constant X) vs horizontal side (constant Y)
    if (seg_a.x() - seg_b.x()).abs() < eps && (side_a.y() - side_b.y()).abs() < eps {
        let seg_x = seg_a.x();
        let side_y = side_a.y();
        let seg_y_min = seg_a.y().min(seg_b.y());
        let seg_y_max = seg_a.y().max(seg_b.y());
        let side_x_min = side_a.x().min(side_b.x());
        let side_x_max = side_a.x().max(side_b.x());
        if side_y > seg_y_min + eps
            && side_y < seg_y_max - eps
            && seg_x > side_x_min + eps
            && seg_x < side_x_max - eps
        {
            return Some(Point::new(seg_x, side_y));
        }
    }

    None
}

/// Test if a point is inside a non-rectangular obstacle polyline using ray casting.
///
/// Matches C# `InsideObstacleHitTest` (line 565) ray-through-bounding-box approach.
/// Shoots a test segment along the scan direction through the obstacle's bounding box,
/// then counts intersections with the visibility polyline.
fn point_inside_polyline_via_ray(
    point: Point,
    obstacle: &Obstacle,
    scan_direction: ScanDirection,
) -> bool {
    let bb = obstacle.visibility_bounding_box();
    let poly = obstacle.visibility_polyline();

    // Build a test segment that extends beyond the bounding box in the scan direction.
    let (low, high) = if scan_direction.is_horizontal() {
        (
            Point::new(bb.left() - 1.0, point.y()),
            Point::new(bb.right() + 1.0, point.y()),
        )
    } else {
        (
            Point::new(point.x(), bb.bottom() - 1.0),
            Point::new(point.x(), bb.top() + 1.0),
        )
    };

    // Count intersections of the test segment with the polyline edges.
    let points: Vec<Point> = poly.points().collect();
    let n = points.len();
    if n < 2 {
        return false;
    }

    let mut intersection_count = 0;
    let edge_count = if poly.is_closed() { n } else { n - 1 };

    for i in 0..edge_count {
        let p1 = points[i];
        let p2 = points[(i + 1) % n];
        if segment_segment_intersection(low, high, p1, p2).is_some() {
            intersection_count += 1;
        }
    }

    // C# line 598: the interesting case is exactly 2 intersections, meaning
    // the ray entered and exited the obstacle. For non-degenerate cases,
    // 2 intersections means the point is inside.
    intersection_count == 2
}
