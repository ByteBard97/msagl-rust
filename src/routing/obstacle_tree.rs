use super::compass_direction::Direction;
use super::group_boundary_crossing::GroupBoundaryCrossingMap;
use super::obstacle::Obstacle;
use super::obstacle_tree_overlap::resolve_overlaps;
use super::shape::Shape;
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
    /// Create an empty obstacle tree (no obstacles).
    pub fn empty() -> Self {
        Self {
            obstacles: Vec::new(),
            rtree: RTree::new(),
            current_group_boundary_crossing_map: super::group_boundary_crossing::GroupBoundaryCrossingMap::new(),
        }
    }

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
    pub(crate) fn restrict_segment_private(
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

            // Matches C# RecurseRestrictRayWithObstacles line 677:
            // `if (!obstacle.IsGroup || stopAtGroups)` — groups block the ray only
            // when stop_at_groups = true; otherwise they are handled in Pass 2.
            if obs.is_group() {
                if !stop_at_groups {
                    continue;
                }
                // stop_at_groups = true: fall through and treat group as a blocking obstacle
            }
            // Transparent obstacles are ports whose interior is temporarily passable.
            if obs.is_transparent() {
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
pub(crate) fn points_equal(a: Point, b: Point) -> bool {
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
pub(crate) fn segment_segment_intersection(a1: Point, a2: Point, b1: Point, b2: Point) -> Option<Point> {
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


