use super::obstacle::Obstacle;
use super::obstacle_tree_overlap::resolve_overlaps;
use super::shape::Shape;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use rstar::{PointDistance, RTree, RTreeObject, AABB};

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
pub struct ObstacleTree {
    pub obstacles: Vec<Obstacle>,
    rtree: RTree<ObstacleEnvelope>,
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
}

/// Sentinel offset margin applied to the graph bounding box.
/// Matches TS ScanSegment.SentinelOffset = 1.0.
const SENTINEL_OFFSET: f64 = 1.0;
