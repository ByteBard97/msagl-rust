use crate::geometry::rectangle::Rectangle;

/// User-facing shape input. Currently only rectangles.
#[derive(Clone, Debug)]
pub struct Shape {
    /// Bounding box of the shape.
    bounding_box: Rectangle,
}

impl Shape {
    /// Create a rectangular shape at (x, y) with given width and height.
    /// (x, y) is the left-bottom corner.
    pub fn rectangle(x: f64, y: f64, width: f64, height: f64) -> Self {
        Self {
            bounding_box: Rectangle::new(x, y, x + width, y + height),
        }
    }

    /// Create from center point and dimensions.
    pub fn rectangle_centered(cx: f64, cy: f64, width: f64, height: f64) -> Self {
        let hw = width / 2.0;
        let hh = height / 2.0;
        Self {
            bounding_box: Rectangle::new(cx - hw, cy - hh, cx + hw, cy + hh),
        }
    }

    pub fn bounding_box(&self) -> &Rectangle {
        &self.bounding_box
    }
}
