use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::geometry::polyline::Polyline;
use super::shape::Shape;
use super::obstacle_side::{ObstacleSide, SideType};

/// Internal obstacle representation with padded boundary.
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// Index of this obstacle (for cross-referencing).
    pub index: usize,
    /// Ordinal for scanline tiebreaking.
    pub ordinal: usize,
    /// The original shape.
    pub shape: Shape,
    /// The padded bounding box.
    padded_bbox: Rectangle,
    /// The padded polyline (closed rectangle, 4 points).
    padded_polyline: Polyline,
    /// Padding distance.
    pub padding: f64,
    /// Whether this obstacle is a sentinel boundary marker.
    is_sentinel: bool,
    /// Active low side (set during sweep).
    active_low_side: Option<ObstacleSide>,
    /// Active high side (set during sweep).
    active_high_side: Option<ObstacleSide>,
}

impl Obstacle {
    /// Ordinal reserved for sentinel obstacles.
    pub const FIRST_SENTINEL_ORDINAL: usize = 1;
    /// Starting ordinal for non-sentinel obstacles.
    pub const FIRST_NON_SENTINEL_ORDINAL: usize = 10;

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
            ordinal: Self::FIRST_NON_SENTINEL_ORDINAL + index,
            shape: shape.clone(),
            padded_bbox: padded,
            padded_polyline: poly,
            padding,
            is_sentinel: false,
            active_low_side: None,
            active_high_side: None,
        }
    }

    /// Create a sentinel obstacle used as a boundary marker.
    pub fn create_sentinel(a: Point, b: Point, ordinal: usize) -> Self {
        let bbox = Rectangle::from_points(a, b);

        let mut poly = Polyline::new();
        poly.add_point(a);
        poly.add_point(b);

        // Use a dummy shape backed by the same bbox.
        let shape = Shape::rectangle(bbox.left(), bbox.bottom(), bbox.width(), bbox.height());

        Self {
            index: 0,
            ordinal,
            shape,
            padded_bbox: bbox,
            padded_polyline: poly,
            padding: 0.0,
            is_sentinel: true,
            active_low_side: None,
            active_high_side: None,
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

    pub fn ordinal(&self) -> usize {
        self.ordinal
    }

    pub fn is_sentinel(&self) -> bool {
        self.is_sentinel
    }

    pub fn active_low_side(&self) -> Option<&ObstacleSide> {
        self.active_low_side.as_ref()
    }

    pub fn active_high_side(&self) -> Option<&ObstacleSide> {
        self.active_high_side.as_ref()
    }

    /// Create the initial active sides for sweep-line processing.
    ///
    /// For a **horizontal scan** (sweeping vertically):
    /// - Low side = left side (low X): vertical line from bottom-left to top-left.
    /// - High side = right side (high X): vertical line from top-right to bottom-right.
    ///
    /// For a **vertical scan** (sweeping horizontally):
    /// - Low side = bottom side (low Y): horizontal line from left-bottom to right-bottom.
    /// - High side = top side (high Y): horizontal line from right-top to left-top.
    pub fn create_initial_sides(&mut self, is_horizontal_scan: bool) {
        let bb = &self.padded_bbox;
        if is_horizontal_scan {
            // Low = left (low X): bottom-left → top-left
            self.active_low_side = Some(ObstacleSide::new(
                SideType::Low,
                bb.left_bottom(),
                bb.left_top(),
                self.ordinal,
            ));
            // High = right (high X): top-right → bottom-right
            self.active_high_side = Some(ObstacleSide::new(
                SideType::High,
                bb.right_top(),
                bb.right_bottom(),
                self.ordinal,
            ));
        } else {
            // Low = bottom (low Y): left-bottom → right-bottom
            self.active_low_side = Some(ObstacleSide::new(
                SideType::Low,
                bb.left_bottom(),
                bb.right_bottom(),
                self.ordinal,
            ));
            // High = top (high Y): right-top → left-top
            self.active_high_side = Some(ObstacleSide::new(
                SideType::High,
                bb.right_top(),
                bb.left_top(),
                self.ordinal,
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn obstacle_gets_ordinal() {
        let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
        let obs = Obstacle::from_shape(&shape, 4.0, 3);
        assert_eq!(obs.ordinal(), Obstacle::FIRST_NON_SENTINEL_ORDINAL + 3);
    }

    #[test]
    fn obstacle_initially_has_no_active_sides() {
        let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
        let obs = Obstacle::from_shape(&shape, 4.0, 0);
        assert!(obs.active_low_side().is_none());
        assert!(obs.active_high_side().is_none());
    }

    #[test]
    fn create_sides_horizontal_scan() {
        let shape = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
        let mut obs = Obstacle::from_shape(&shape, 4.0, 0);
        obs.create_initial_sides(true); // horizontal scan
        let low = obs.active_low_side().unwrap();
        let high = obs.active_high_side().unwrap();
        // Horizontal scan: low side is left (low X), high side is right (high X)
        assert_eq!(low.side_type(), SideType::Low);
        assert_eq!(high.side_type(), SideType::High);
        // Low side should be at left edge of padded box (10 - 4 = 6)
        assert!((low.start().x() - 6.0).abs() < 1e-6);
        // High side should be at right edge (10 + 100 + 4 = 114)
        assert!((high.start().x() - 114.0).abs() < 1e-6);
    }

    #[test]
    fn create_sides_vertical_scan() {
        let shape = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
        let mut obs = Obstacle::from_shape(&shape, 4.0, 0);
        obs.create_initial_sides(false); // vertical scan
        let low = obs.active_low_side().unwrap();
        let high = obs.active_high_side().unwrap();
        // Vertical scan: low side is bottom (low Y), high side is top (high Y)
        assert_eq!(low.side_type(), SideType::Low);
        assert_eq!(high.side_type(), SideType::High);
        // Low side should be at bottom edge (20 - 4 = 16)
        assert!((low.start().y() - 16.0).abs() < 1e-6);
        // High side should be at top edge (20 + 50 + 4 = 74)
        assert!((high.start().y() - 74.0).abs() < 1e-6);
    }

    #[test]
    fn sentinel_creation() {
        let sentinel = Obstacle::create_sentinel(
            Point::new(-100.0, -100.0),
            Point::new(100.0, -100.0),
            Obstacle::FIRST_SENTINEL_ORDINAL,
        );
        assert!(sentinel.is_sentinel());
        assert_eq!(sentinel.ordinal(), Obstacle::FIRST_SENTINEL_ORDINAL);
    }

    #[test]
    fn from_shape_preserves_existing_api() {
        let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
        let obs = Obstacle::from_shape(&shape, 4.0, 0);
        // Existing API must still work
        assert_eq!(obs.index, 0);
        let bb = obs.padded_bounding_box();
        assert!((bb.left() - (-4.0)).abs() < 1e-6);
        assert!((bb.right() - 104.0).abs() < 1e-6);
        let corners = obs.padded_corners();
        assert_eq!(corners.len(), 4);
    }
}
