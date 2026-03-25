use rstar::{RTree, RTreeObject, AABB, PointDistance};
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use super::obstacle::Obstacle;
use super::shape::Shape;

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
        let obstacles: Vec<Obstacle> = shapes.iter().enumerate()
            .map(|(i, s)| Obstacle::from_shape(s, padding, i))
            .collect();

        let envelopes: Vec<ObstacleEnvelope> = obstacles.iter().map(|obs| {
            let bb = obs.padded_bounding_box();
            ObstacleEnvelope {
                obstacle_index: obs.index,
                min: [bb.left(), bb.bottom()],
                max: [bb.right(), bb.top()],
            }
        }).collect();

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
        self.rtree.locate_all_at_point(&[p.x(), p.y()])
            .map(|env| env.obstacle_index)
            .collect()
    }

    /// Find all obstacles whose padded bounding box intersects the rectangle.
    pub fn query_rect(&self, rect: &Rectangle) -> Vec<usize> {
        let aabb = AABB::from_corners(
            [rect.left(), rect.bottom()],
            [rect.right(), rect.top()],
        );
        self.rtree.locate_in_envelope_intersecting(&aabb)
            .map(|env| env.obstacle_index)
            .collect()
    }

    /// Get obstacle by index.
    pub fn obstacle(&self, index: usize) -> &Obstacle {
        &self.obstacles[index]
    }
}
