use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::geometry::polyline::Polyline;
use super::shape::Shape;

/// Internal obstacle representation with padded boundary.
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// Index of this obstacle (for cross-referencing).
    pub index: usize,
    /// The original shape.
    pub shape: Shape,
    /// The padded bounding box.
    padded_bbox: Rectangle,
    /// The padded polyline (closed rectangle, 4 points).
    padded_polyline: Polyline,
    /// Padding distance.
    pub padding: f64,
}

impl Obstacle {
    pub fn from_shape(shape: &Shape, padding: f64, index: usize) -> Self {
        let bb = shape.bounding_box();
        let padded = Rectangle::new(
            bb.left() - padding,
            bb.bottom() - padding,
            bb.right() + padding,
            bb.top() + padding,
        );

        // Create closed polyline: 4 corners clockwise from bottom-left
        let mut poly = Polyline::new();
        poly.add_point(padded.left_bottom());
        poly.add_point(padded.left_top());
        poly.add_point(padded.right_top());
        poly.add_point(padded.right_bottom());
        poly.set_closed(true);

        Self {
            index,
            shape: shape.clone(),
            padded_bbox: padded,
            padded_polyline: poly,
            padding,
        }
    }

    pub fn padded_bounding_box(&self) -> &Rectangle {
        &self.padded_bbox
    }

    pub fn padded_polyline(&self) -> &Polyline {
        &self.padded_polyline
    }

    /// Get the 4 padded corner points.
    pub fn padded_corners(&self) -> [Point; 4] {
        [
            self.padded_bbox.left_bottom(),
            self.padded_bbox.left_top(),
            self.padded_bbox.right_top(),
            self.padded_bbox.right_bottom(),
        ]
    }
}
