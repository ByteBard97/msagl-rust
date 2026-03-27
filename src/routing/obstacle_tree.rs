use super::compass_direction::{CompassDirection, Direction};
use super::group_boundary_crossing::{
    compare_points, GroupBoundaryCrossingMap, PointAndCrossingsList,
};
use super::obstacle::Obstacle;
use super::obstacle_tree_overlap::resolve_overlaps;
use super::scan_direction::ScanDirection;
use super::shape::Shape;
use super::static_graph_utility::StaticGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use rstar::{PointDistance, RTree, RTreeObject, AABB};

/// Wrapper for R-tree spatial indexing of obstacles.
pub(crate) struct ObstacleEnvelope {
    obstacle_index: usize,
    min: [f64; 2],
    max: [f64; 2],
}

impl RTreeObject for ObstacleEnvelope {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.min, self.max)
    }
}

impl PointDistance for ObstacleEnvelope {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        self.envelope().distance_2(point)
    }
}

/// Spatial index of obstacles using an R-tree.
pub struct ObstacleTree {
    pub obstacles: Vec<Obstacle>,
    rtree: RTree<ObstacleEnvelope>,
    /// Accumulates group boundary crossings during segment restriction.
    ///
    /// Matches C# `ObstacleTree.CurrentGroupBoundaryCrossingMap`.
    /// Cleared at the start of each `restrict_segment_private` call, then
    /// populated by `add_group_intersections_to_restricted_ray` for each group
    /// obstacle encountered during the scan.
    pub current_group_boundary_crossing_map: GroupBoundaryCrossingMap,
}

impl ObstacleTree {
    /// Create from shapes with given padding.
    pub fn new(shapes: &[Shape], padding: f64) -> Self {
        let mut obstacles: Vec<Obstacle> = shapes
            .iter()
            .enumerate()
            .map(|(i, s)| Obstacle::from_shape(s, padding, i))
            .collect();

        // Detect and resolve overlaps (clumps and convex hulls).
        // Matches TS ObstacleTree.CreateRoot() overlap resolution.
        resolve_overlaps(&mut obstacles);

        let envelopes: Vec<ObstacleEnvelope> = obstacles
            .iter()
            .map(|obs| {
                let bb = obs.padded_bounding_box();
                ObstacleEnvelope {
                    obstacle_index: obs.index,
                    min: [bb.left(), bb.bottom()],
                    max: [bb.right(), bb.top()],
                }
            })
            .collect();

        Self {
            obstacles,
            rtree: RTree::bulk_load(envelopes),
            current_group_boundary_crossing_map: GroupBoundaryCrossingMap::new(),
        }
    }

    /// Number of obstacles.
    pub fn len(&self) -> usize {
        self.obstacles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.obstacles.is_empty()
    }

    /// Find all obstacles whose padded bounding box contains the point.
    pub fn query_point(&self, p: Point) -> Vec<usize> {
        self.rtree
            .locate_all_at_point(&[p.x(), p.y()])
            .map(|env| env.obstacle_index)
            .collect()
    }

    /// Find all obstacles whose padded bounding box intersects the rectangle.
    pub fn query_rect(&self, rect: &Rectangle) -> Vec<usize> {
        let aabb = AABB::from_corners([rect.left(), rect.bottom()], [rect.right(), rect.top()]);
        self.rtree
            .locate_in_envelope_intersecting(&aabb)
            .map(|env| env.obstacle_index)
            .collect()
    }

    /// Get obstacle by index.
    pub fn obstacle(&self, index: usize) -> &Obstacle {
        &self.obstacles[index]
    }

    /// Returns a bounding box encompassing all padded obstacles plus a sentinel offset margin.
    pub fn graph_box(&self) -> Rectangle {
        let mut result = Rectangle::empty();
        for obs in &self.obstacles {
            result.add_rect(obs.padded_bounding_box());
        }
        result.pad(SENTINEL_OFFSET);
        result
    }

    /// Find the obstacle whose padded bounding box contains the given point.
    /// Returns the obstacle index, or None if no obstacle contains it.
    pub fn inside_hit_test(&self, point: Point) -> Option<usize> {
        self.query_point(point).into_iter().next()
    }

    /// Returns a slice of all obstacles.
    pub fn all_obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    /// Returns a mutable slice of all obstacles.
    pub fn obstacles_mut(&mut self) -> &mut [Obstacle] {
        &mut self.obstacles
    }

    /// Get a mutable reference to an obstacle by index.
    pub fn obstacle_mut(&mut self, index: usize) -> &mut Obstacle {
        &mut self.obstacles[index]
    }

    /// Build an R-tree from a subset of obstacles (filtered by index).
    ///
    /// Matches C# `ObstacleTree.CalculateHierarchy(IEnumerable<Obstacle>)` (line 167).
    /// Uses `rstar` bulk_load instead of the C# custom RectangleNode hierarchy.
    pub(crate) fn calculate_hierarchy_from_subset(&self, indices: &[usize]) -> RTree<ObstacleEnvelope> {
        let envelopes: Vec<ObstacleEnvelope> = indices
            .iter()
            .map(|&idx| {
                let obs = &self.obstacles[idx];
                let bb = obs.visibility_bounding_box();
                ObstacleEnvelope {
                    obstacle_index: obs.index,
                    min: [bb.left(), bb.bottom()],
                    max: [bb.right(), bb.top()],
                }
            })
            .collect();
        RTree::bulk_load(envelopes)
    }

    /// Returns indices of all primary obstacles (those that participate in the
    /// visibility graph hierarchy).
    ///
    /// Non-primary obstacles are those subsumed into a convex hull where they
    /// are not the primary index.
    ///
    /// Matches C# `ObstacleTree.GetAllPrimaryObstacles()` (line 535) which returns
    /// `Root.GetAllLeaves()` — i.e. only obstacles that were inserted into the
    /// hierarchy after overlap resolution.
    pub fn get_all_primary_obstacles(&self) -> Vec<usize> {
        self.obstacles
            .iter()
            .filter(|obs| obs.is_primary_obstacle())
            .map(|obs| obs.index)
            .collect()
    }

    /// Clips a line segment so it stops at the first obstacle boundary it hits.
    ///
    /// Matches C# `ObstacleTree.RestrictSegmentWithObstacles()` (line 634).
    /// The segment direction must be a pure compass direction (axis-aligned).
    ///
    /// Also populates `current_group_boundary_crossing_map` with group crossings.
    ///
    /// Returns `(clipped_start, clipped_end)`.
    pub fn restrict_segment_with_obstacles(
        &mut self,
        start_point: Point,
        end_point: Point,
    ) -> (Point, Point) {
        // Matches C# RestrictSegmentPrivate (line 640) with stopAtGroups=false.
        self.restrict_segment_private(start_point, end_point, false)
    }

    /// Internal implementation of segment restriction.
    ///
    /// Matches C# `RestrictSegmentPrivate` (line 640).
    /// `stop_at_groups`: if true, treat group obstacles as blocking (used by
    /// `segment_crosses_an_obstacle`).
    ///
    /// Clears `current_group_boundary_crossing_map` at the start, then populates
    /// it with group crossings during the scan (when `stop_at_groups` is false).
    fn restrict_segment_private(
        &mut self,
        start_point: Point,
        end_point: Point,
        stop_at_groups: bool,
    ) -> (Point, Point) {
        // C# RestrictSegmentPrivate line 640-641: clear group crossings before scan.
        self.current_group_boundary_crossing_map.clear();

        // Build the segment bounding box for spatial query.
        let seg_rect = Rectangle::from_points(start_point, end_point);

        // Query all obstacles whose bounding boxes intersect the segment's bbox.
        let candidates = self.query_rect(&seg_rect);

        let mut best_end = end_point;
        let mut best_dist_sq = (start_point - end_point).length_squared();

        let seg_dir = Direction::from_point_to_point(start_point, end_point);
        let opp_dir = opposite_direction(seg_dir);

        // Pass 1: non-group obstacles — restrict best_end.
        for &idx in &candidates {
            let obs = &self.obstacles[idx];

            // Skip non-primary obstacles (inside convex hulls).
            if !obs.is_primary_obstacle() {
                continue;
            }

            // Groups do not shorten the ray (handled in pass 2 below).
            if obs.is_group() {
                continue;
            }

            // A segment that moves along the boundary of an obstacle is not blocked.
            // Check that the segment's bounding box interior intersects the obstacle's.
            let obs_bb = obs.visibility_bounding_box();
            if !rectangle_interiors_intersect(&seg_rect, &obs_bb) {
                continue;
            }

            // For rectangular obstacles, find the intersection of the axis-aligned
            // segment with the obstacle boundary.
            // Matches C# LookForCloserNonGroupIntersectionToRestrictRay (line 695).
            if let Some(intersect) =
                segment_obstacle_intersection(start_point, end_point, obs, seg_dir, opp_dir)
            {
                let dist_sq = (intersect - start_point).length_squared();
                if dist_sq < best_dist_sq {
                    // C# line 719: skip if the raw distance is below epsilon squared.
                    if dist_sq < DISTANCE_EPSILON_SQUARED {
                        continue;
                    }
                    best_dist_sq = dist_sq;
                    best_end = intersect;
                }
            }
        }

        // Pass 2: group obstacles — collect boundary crossings (when not stop_at_groups).
        //
        // Matches C# RecurseRestrictRayWithObstacles: groups call
        // AddGroupIntersectionsToRestrictedRay instead of
        // LookForCloserNonGroupIntersectionToRestrictRay.
        if !stop_at_groups {
            let group_indices: Vec<usize> = candidates
                .iter()
                .copied()
                .filter(|&idx| self.obstacles[idx].is_primary_obstacle() && self.obstacles[idx].is_group())
                .collect();
            for idx in group_indices {
                self.add_group_intersections_to_restricted_ray(idx, start_point, best_end);
            }
        }

        (start_point, best_end)
    }

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
    fn add_group_intersections_to_restricted_ray(
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

    /// Finds pairs of obstacles whose bounding boxes overlap.
    ///
    /// Matches C# `RectangleNodeUtils.CrossRectangleNodes` pattern used at
    /// lines 110, 178, 201, 246.
    ///
    /// Returns pairs of obstacle indices `(a, b)` where `a < b` and the
    /// visibility bounding boxes intersect.
    pub fn cross_rectangle_nodes(&self) -> Vec<(usize, usize)> {
        // Build a temporary R-tree of primary obstacles for efficient pair detection.
        let primary_indices = self.get_all_primary_obstacles();
        self.cross_rectangle_nodes_for(&primary_indices)
    }

    /// Finds pairs of overlapping bounding boxes among the given obstacle indices.
    ///
    /// Matches C# `RectangleNodeUtils.CrossRectangleNodes(tree, tree, callback)`.
    pub fn cross_rectangle_nodes_for(&self, indices: &[usize]) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();

        // For each obstacle, query the R-tree for overlapping obstacles.
        // Use a temporary R-tree built from the subset for O(n log n) performance.
        let subtree = self.calculate_hierarchy_from_subset(indices);

        for &idx in indices {
            let obs = &self.obstacles[idx];
            let bb = obs.visibility_bounding_box();
            let aabb =
                AABB::from_corners([bb.left(), bb.bottom()], [bb.right(), bb.top()]);

            for env in subtree.locate_in_envelope_intersecting(&aabb) {
                let other_idx = env.obstacle_index;
                // Only emit each pair once, with smaller index first.
                if idx < other_idx {
                    pairs.push((idx, other_idx));
                }
            }
        }

        pairs
    }
}

/// Sentinel offset margin applied to the graph bounding box.
/// Matches TS ScanSegment.SentinelOffset = 1.0.
const SENTINEL_OFFSET: f64 = 1.0;

/// Matches C# `ApproximateComparer.SquareOfDistanceEpsilon`.
const DISTANCE_EPSILON_SQUARED: f64 = 1e-12;

/// Tolerance for point equality checks.
const POINT_EQUALITY_TOLERANCE: f64 = 1e-6;

/// Check if two points are approximately equal.
/// Matches C# `PointComparer.Equal`.
fn points_equal(a: Point, b: Point) -> bool {
    (a - b).length() < POINT_EQUALITY_TOLERANCE
}

/// Check if two rectangle interiors intersect (excluding boundary-only contact).
/// Matches C# `StaticGraphUtility.RectangleInteriorsIntersect`.
fn rectangle_interiors_intersect(a: &Rectangle, b: &Rectangle) -> bool {
    a.left() < b.right() && a.right() > b.left() && a.bottom() < b.top() && a.top() > b.bottom()
}

/// Get the opposite of a (possibly compound) direction.
fn opposite_direction(dir: Direction) -> Direction {
    let mut result = Direction::NONE;
    if dir.contains(Direction::NORTH) {
        result |= Direction::SOUTH;
    }
    if dir.contains(Direction::SOUTH) {
        result |= Direction::NORTH;
    }
    if dir.contains(Direction::EAST) {
        result |= Direction::WEST;
    }
    if dir.contains(Direction::WEST) {
        result |= Direction::EAST;
    }
    result
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

/// Find where an axis-aligned segment intersects a rectangular obstacle boundary.
///
/// Matches the logic in C# `LookForCloserNonGroupIntersectionToRestrictRay` (line 695)
/// specialized for rectangular obstacles. For axis-aligned segments hitting rectangles,
/// the intersection is the nearest boundary of the obstacle along the segment direction.
fn segment_obstacle_intersection(
    start: Point,
    _end: Point,
    obstacle: &Obstacle,
    seg_dir: Direction,
    opp_dir: Direction,
) -> Option<Point> {
    let bb = obstacle.visibility_bounding_box();

    // For a pure horizontal or vertical segment, find the nearest obstacle boundary
    // in the segment direction.
    if seg_dir == Direction::EAST {
        // Moving right: hit the left side of the obstacle.
        if start.x() < bb.left() && start.y() > bb.bottom() && start.y() < bb.top() {
            return Some(Point::new(bb.left(), start.y()));
        }
    } else if seg_dir == Direction::WEST {
        // Moving left: hit the right side.
        if start.x() > bb.right() && start.y() > bb.bottom() && start.y() < bb.top() {
            return Some(Point::new(bb.right(), start.y()));
        }
    } else if seg_dir == Direction::NORTH {
        // Moving up: hit the bottom side.
        if start.y() < bb.bottom() && start.x() > bb.left() && start.x() < bb.right() {
            return Some(Point::new(start.x(), bb.bottom()));
        }
    } else if seg_dir == Direction::SOUTH {
        // Moving down: hit the top side.
        if start.y() > bb.top() && start.x() > bb.left() && start.x() < bb.right() {
            return Some(Point::new(start.x(), bb.top()));
        }
    }

    // For compound directions or cases where the start is already inside/alongside
    // the obstacle, fall back to general segment-rectangle intersection.
    let _ = opp_dir;
    segment_rect_intersection_general(start, _end, &bb, seg_dir)
}

/// General segment-rectangle intersection for compound or non-trivial cases.
///
/// Returns the closest intersection point of the segment with the rectangle boundary
/// that lies in the direction of travel from `start`.
fn segment_rect_intersection_general(
    start: Point,
    end: Point,
    rect: &Rectangle,
    _seg_dir: Direction,
) -> Option<Point> {
    // Parametric line clipping (Cohen-Sutherland / Liang-Barsky style).
    let dx = end.x() - start.x();
    let dy = end.y() - start.y();

    let mut best: Option<Point> = None;
    let mut best_dist_sq = f64::MAX;

    // Check intersection with each of the 4 rectangle sides.
    let sides: [(f64, f64, f64, f64, f64, f64); 4] = [
        // left side: x = rect.left, y in [bottom, top]
        (rect.left(), rect.bottom(), rect.left(), rect.top(), dx, dy),
        // right side
        (rect.right(), rect.bottom(), rect.right(), rect.top(), dx, dy),
        // bottom side: y = rect.bottom, x in [left, right]
        (rect.left(), rect.bottom(), rect.right(), rect.bottom(), dx, dy),
        // top side
        (rect.left(), rect.top(), rect.right(), rect.top(), dx, dy),
    ];

    for (sx, sy, ex, ey, _, _) in &sides {
        if let Some(pt) = segment_segment_intersection(
            start,
            end,
            Point::new(*sx, *sy),
            Point::new(*ex, *ey),
        ) {
            let dist_sq = (pt - start).length_squared();
            if dist_sq < best_dist_sq && dist_sq > DISTANCE_EPSILON_SQUARED {
                best_dist_sq = dist_sq;
                best = Some(pt);
            }
        }
    }

    best
}

/// Find the intersection of two line segments, if any.
fn segment_segment_intersection(a1: Point, a2: Point, b1: Point, b2: Point) -> Option<Point> {
    let d1 = a2 - a1;
    let d2 = b2 - b1;
    let cross = Point::cross(d1, d2);

    if cross.abs() < 1e-12 {
        return None; // Parallel or coincident.
    }

    let d = b1 - a1;
    let t = Point::cross(d, d2) / cross;
    let u = Point::cross(d, d1) / cross;

    if t >= -1e-10 && t <= 1.0 + 1e-10 && u >= -1e-10 && u <= 1.0 + 1e-10 {
        Some(Point::new(a1.x() + t * d1.x(), a1.y() + t * d1.y()))
    } else {
        None
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
