//! Spatial index of obstacles using an R-tree, with overlap detection
//! and convex hull merging.
//!
//! Faithful port of TS `ObstacleTree.ts` (823 lines).
//! Split across `obstacle_tree.rs` (core + queries) and
//! `obstacle_tree_overlap.rs` (overlap detection, clump/hull creation).
//!
//! Approved deviation: uses `rstar` for spatial indexing instead of the
//! TS custom `RectangleNode` hierarchy.

use rstar::{RTree, RTreeObject, AABB, PointDistance};
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use super::obstacle::Obstacle;
use super::scan_direction::ScanDirection;
use super::shape::Shape;
use super::static_graph_utility::StaticGraphUtility;

/// Sentinel offset margin applied to the graph bounding box.
/// Matches TS `ScanSegment.SentinelOffset = 1.0`.
const SENTINEL_OFFSET: f64 = 1.0;

/// Wrapper for R-tree spatial indexing of obstacles.
struct ObstacleEnvelope {
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
///
/// Faithful port of TS `ObstacleTree`.
///
/// Key responsibilities:
/// - Spatial queries (point containment, rectangle intersection)
/// - Overlap detection and resolution (clumps + convex hulls)
/// - Inside-obstacle hit testing for the visibility graph generator
/// - Segment restriction (restricting visibility rays to obstacle boundaries)
pub struct ObstacleTree {
    /// All obstacles (including those inside convex hulls).
    pub obstacles: Vec<Obstacle>,
    /// R-tree for spatial queries. Built from visibility bounding boxes
    /// of primary obstacles (after overlap resolution).
    rtree: RTree<ObstacleEnvelope>,
    /// Cached graph bounding box.
    graph_box: Rectangle,
    /// Ignore these obstacle indices during InsideHitTest.
    inside_hit_test_ignore: [Option<usize>; 2],
    /// Scan direction for hit testing.
    inside_hit_test_scan_dir: Option<ScanDirection>,
}

impl ObstacleTree {
    /// Create from shapes with given padding.
    ///
    /// Matches TS `ObstacleTree.Init()`:
    /// 1. Creates obstacles with ordinals
    /// 2. Detects overlaps
    /// 3. Creates clumps for overlapping rectangles
    /// 4. Creates convex hulls for overlapping non-rectangles
    /// 5. Rebuilds spatial index with primary obstacles only
    pub fn new(shapes: &[Shape], padding: f64) -> Self {
        let mut obstacles: Vec<Obstacle> = shapes.iter().enumerate()
            .map(|(i, s)| Obstacle::from_shape(s, padding, i))
            .collect();

        // Assign ordinals (TS: CreateObstacleListAndOrdinals)
        for (i, obs) in obstacles.iter_mut().enumerate() {
            obs.ordinal = Obstacle::FIRST_NON_SENTINEL_ORDINAL + i;
        }

        // Detect and resolve overlaps
        super::obstacle_tree_overlap::resolve_overlaps(&mut obstacles);

        // Build R-tree from primary obstacles using visibility bounding boxes
        let rtree = Self::build_rtree(&obstacles);
        let graph_box = Self::compute_graph_box(&obstacles);

        Self {
            obstacles,
            rtree,
            graph_box,
            inside_hit_test_ignore: [None, None],
            inside_hit_test_scan_dir: None,
        }
    }

    fn build_rtree(obstacles: &[Obstacle]) -> RTree<ObstacleEnvelope> {
        let envelopes: Vec<ObstacleEnvelope> = obstacles.iter()
            .filter(|obs| obs.is_primary_obstacle())
            .map(|obs| {
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

    fn compute_graph_box(obstacles: &[Obstacle]) -> Rectangle {
        let mut result = Rectangle::empty();
        for obs in obstacles {
            result.add_rect(&obs.visibility_bounding_box());
        }
        result.pad(SENTINEL_OFFSET);
        result
    }

    // ---------------------------------------------------------------
    // Basic accessors
    // ---------------------------------------------------------------

    pub fn len(&self) -> usize { self.obstacles.len() }
    pub fn is_empty(&self) -> bool { self.obstacles.is_empty() }

    pub fn obstacle(&self, index: usize) -> &Obstacle {
        &self.obstacles[index]
    }

    pub fn obstacle_mut(&mut self, index: usize) -> &mut Obstacle {
        &mut self.obstacles[index]
    }

    pub fn all_obstacles(&self) -> &[Obstacle] { &self.obstacles }
    pub fn obstacles_mut(&mut self) -> &mut [Obstacle] { &mut self.obstacles }

    /// Iterator over primary obstacles only (for VG generation).
    ///
    /// Matches TS `ObstacleTree.GetAllPrimaryObstacles()`.
    pub fn primary_obstacles(&self) -> impl Iterator<Item = &Obstacle> {
        self.obstacles.iter().filter(|obs| obs.is_primary_obstacle())
    }

    /// The graph bounding box (all obstacles + sentinel margin).
    /// Matches TS `ObstacleTree.GraphBox`.
    pub fn graph_box(&self) -> Rectangle { self.graph_box }

    // ---------------------------------------------------------------
    // Spatial queries
    // ---------------------------------------------------------------

    /// Find all primary obstacles whose visibility bbox contains the point.
    pub fn query_point(&self, p: Point) -> Vec<usize> {
        self.rtree.locate_all_at_point(&[p.x(), p.y()])
            .map(|env| env.obstacle_index)
            .collect()
    }

    /// Find all primary obstacles whose visibility bbox intersects the rect.
    pub fn query_rect(&self, rect: &Rectangle) -> Vec<usize> {
        let aabb = AABB::from_corners(
            [rect.left(), rect.bottom()],
            [rect.right(), rect.top()],
        );
        self.rtree.locate_in_envelope_intersecting(&aabb)
            .map(|env| env.obstacle_index)
            .collect()
    }

    // ---------------------------------------------------------------
    // Inside hit testing
    // ---------------------------------------------------------------

    /// Simple inside-hit-test: find the first obstacle containing the point.
    /// Returns the obstacle index, or None.
    pub fn inside_hit_test(&self, point: Point) -> Option<usize> {
        for idx in self.query_point(point) {
            let obs = &self.obstacles[idx];
            if StaticGraphUtility::point_is_in_rectangle_interior(
                point, &obs.visibility_bounding_box(),
            ) {
                return Some(idx);
            }
        }
        None
    }

    /// Test whether a point is inside any obstacle.
    ///
    /// Faithful port of TS `ObstacleTree.PointIsInsideAnObstacle()`.
    pub fn point_is_inside_an_obstacle(
        &mut self,
        point: Point,
        scan_direction: ScanDirection,
    ) -> bool {
        self.inside_hit_test_ignore = [None, None];
        self.inside_hit_test_scan_dir = Some(scan_direction);
        self.find_obstacle_containing_point(point).is_some()
    }

    /// Test if an intersection point lies inside an obstacle other than
    /// the two given obstacles.
    ///
    /// Faithful port of TS `ObstacleTree.IntersectionIsInsideAnotherObstacle()`.
    pub fn intersection_is_inside_another_obstacle(
        &mut self,
        side_obstacle_idx: usize,
        event_obstacle_idx: usize,
        intersect: Point,
        scan_direction: ScanDirection,
    ) -> bool {
        self.inside_hit_test_ignore = [
            Some(event_obstacle_idx),
            Some(side_obstacle_idx),
        ];
        self.inside_hit_test_scan_dir = Some(scan_direction);
        self.find_obstacle_containing_point(intersect).is_some()
    }

    /// Core inside-obstacle hit test.
    ///
    /// Faithful port of TS `ObstacleTree.InsideObstacleHitTest()`.
    fn find_obstacle_containing_point(&self, location: Point) -> Option<usize> {
        let scan_dir = self.inside_hit_test_scan_dir?;

        for idx in self.query_point(location) {
            if self.inside_hit_test_ignore[0] == Some(idx)
                || self.inside_hit_test_ignore[1] == Some(idx)
            {
                continue;
            }

            let obs = &self.obstacles[idx];
            let vis_bb = obs.visibility_bounding_box();
            if !StaticGraphUtility::point_is_in_rectangle_interior(location, &vis_bb) {
                continue;
            }

            // For rectangular obstacles, the bounding box test is sufficient
            if obs.is_rectangle() {
                return Some(idx);
            }

            // Non-rectangular: use ray casting
            if self.point_inside_nonrect_obstacle(location, obs, scan_dir) {
                return Some(idx);
            }
        }

        None
    }

    /// Test if a point is inside a non-rectangular obstacle via ray casting.
    ///
    /// Faithful port of the non-rectangular path in TS `InsideObstacleHitTest`.
    fn point_inside_nonrect_obstacle(
        &self,
        location: Point,
        obstacle: &Obstacle,
        scan_dir: ScanDirection,
    ) -> bool {
        let poly = obstacle.visibility_polyline();
        let points: Vec<Point> = poly.points().collect();
        if points.len() < 3 {
            return false;
        }

        // Ray-casting (even parity rule)
        let ray_x = location.x();
        let ray_y = location.y();
        let mut crossings = 0u32;
        let n = points.len();

        for i in 0..n {
            let j = (i + 1) % n;
            let pi = points[i];
            let pj = points[j];

            if scan_dir.is_horizontal() {
                if (pi.y() <= ray_y && pj.y() > ray_y)
                    || (pj.y() <= ray_y && pi.y() > ray_y)
                {
                    let t = (ray_y - pi.y()) / (pj.y() - pi.y());
                    if pi.x() + t * (pj.x() - pi.x()) > ray_x {
                        crossings += 1;
                    }
                }
            } else {
                if (pi.x() <= ray_x && pj.x() > ray_x)
                    || (pj.x() <= ray_x && pi.x() > ray_x)
                {
                    let t = (ray_x - pi.x()) / (pj.x() - pi.x());
                    if pi.y() + t * (pj.y() - pi.y()) > ray_y {
                        crossings += 1;
                    }
                }
            }
        }

        crossings % 2 == 1
    }

    // ---------------------------------------------------------------
    // Segment restriction
    // ---------------------------------------------------------------

    /// Test if a segment crosses any obstacle.
    ///
    /// Faithful port of TS `ObstacleTree.SegmentCrossesANonGroupObstacle()`.
    pub fn segment_crosses_an_obstacle(
        &self,
        start_point: Point,
        end_point: Point,
    ) -> bool {
        let (restricted, _) = self.restrict_segment_with_obstacles(
            start_point, end_point,
        );
        !GeomConstants::close(restricted.x(), end_point.x())
            || !GeomConstants::close(restricted.y(), end_point.y())
    }

    /// Restrict a segment to the first obstacle intersection.
    /// Returns (restricted endpoint, optional obstacle index).
    ///
    /// Faithful port of TS `ObstacleTree.RestrictSegmentWithObstacles()`.
    pub fn restrict_segment_with_obstacles(
        &self,
        start_point: Point,
        end_point: Point,
    ) -> (Point, Option<usize>) {
        let seg_bb = Rectangle::from_points(start_point, end_point);
        let candidates = self.query_rect(&seg_bb);

        let mut closest_end = end_point;
        let mut closest_dist_sq = (end_point - start_point).length_squared();
        let mut closest_obstacle: Option<usize> = None;

        for idx in candidates {
            let obs = &self.obstacles[idx];
            let vis_bb = obs.visibility_bounding_box();

            if !seg_bb.intersects(&vis_bb) {
                continue;
            }

            if let Some(intersect) = segment_rect_intersection(
                start_point, end_point, &vis_bb,
            ) {
                let dist_sq = (intersect - start_point).length_squared();
                if dist_sq < closest_dist_sq {
                    closest_dist_sq = dist_sq;
                    closest_end = intersect;
                    closest_obstacle = Some(idx);
                }
            }
        }

        (closest_end, closest_obstacle)
    }

    /// Rebuild the R-tree after modifications.
    pub fn rebuild_rtree(&mut self) {
        self.rtree = Self::build_rtree(&self.obstacles);
        self.graph_box = Self::compute_graph_box(&self.obstacles);
    }
}

/// Find the nearest intersection of a segment with a rectangle's border.
fn segment_rect_intersection(
    start: Point,
    end: Point,
    rect: &Rectangle,
) -> Option<Point> {
    let edges = [
        (Point::new(rect.left(), rect.bottom()), Point::new(rect.left(), rect.top())),
        (Point::new(rect.right(), rect.bottom()), Point::new(rect.right(), rect.top())),
        (Point::new(rect.left(), rect.bottom()), Point::new(rect.right(), rect.bottom())),
        (Point::new(rect.left(), rect.top()), Point::new(rect.right(), rect.top())),
    ];

    let mut closest: Option<Point> = None;
    let mut closest_dist_sq = f64::MAX;

    for (e_start, e_end) in &edges {
        if let Some(p) = segment_segment_intersection(start, end, *e_start, *e_end) {
            let dist_sq = (p - start).length_squared();
            if dist_sq < closest_dist_sq
                && dist_sq > GeomConstants::SQUARE_OF_DISTANCE_EPSILON
            {
                closest_dist_sq = dist_sq;
                closest = Some(p);
            }
        }
    }

    closest
}

/// Compute the intersection point of two line segments.
fn segment_segment_intersection(
    a1: Point,
    a2: Point,
    b1: Point,
    b2: Point,
) -> Option<Point> {
    let d1 = a2 - a1;
    let d2 = b2 - b1;
    let cross = Point::cross(d1, d2);

    if cross.abs() < 1e-12 {
        return None; // Parallel
    }

    let d = b1 - a1;
    let t = Point::cross(d, d2) / cross;
    let u = Point::cross(d, d1) / cross;

    if t >= -1e-10 && t <= 1.0 + 1e-10 && u >= -1e-10 && u <= 1.0 + 1e-10 {
        let t_clamped = t.clamp(0.0, 1.0);
        Some(Point::new(
            a1.x() + t_clamped * d1.x(),
            a1.y() + t_clamped * d1.y(),
        ))
    } else {
        None
    }
}
