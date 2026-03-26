use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;

/// User-facing shape input for the router.
///
/// Wraps a closed Polyline boundary. Rectangles are the common case
/// but any closed polyline is accepted.
///
/// Ported from: routing/shape.ts -- `class Shape`
/// The TS Shape stores `BoundaryCurve: ICurve`. We store `Polyline` directly
/// since we only support polyline boundaries (not arcs/curves).
#[derive(Clone, Debug)]
pub struct Shape {
    boundary: Polyline,
    bounding_box: Rectangle,
}

impl Shape {
    /// Create a shape from a closed polyline boundary.
    ///
    /// Matches TS: `new Shape(boundaryCurve)`
    pub fn from_polyline(polyline: Polyline) -> Self {
        debug_assert!(
            polyline.is_closed(),
            "Shape boundary must be a closed polyline"
        );
        let bounding_box = polyline.bounding_box();
        Self {
            boundary: polyline,
            bounding_box,
        }
    }

    /// Create a rectangular shape at (x, y) with given width and height.
    /// (x, y) is the left-bottom corner.
    pub fn rectangle(x: f64, y: f64, width: f64, height: f64) -> Self {
        let mut poly = Polyline::new();
        // Clockwise from bottom-left (matching TS convention)
        poly.add_point(Point::new(x, y));
        poly.add_point(Point::new(x, y + height));
        poly.add_point(Point::new(x + width, y + height));
        poly.add_point(Point::new(x + width, y));
        poly.set_closed(true);
        Self::from_polyline(poly)
    }

    /// Create from center point and dimensions.
    pub fn rectangle_centered(cx: f64, cy: f64, width: f64, height: f64) -> Self {
        let hw = width / 2.0;
        let hh = height / 2.0;
        Self::rectangle(cx - hw, cy - hh, width, height)
    }

    /// The boundary polyline.
    ///
    /// Matches TS: `Shape.BoundaryCurve`
    pub fn boundary_polyline(&self) -> &Polyline {
        &self.boundary
    }

    pub fn bounding_box(&self) -> &Rectangle {
        &self.bounding_box
    }
}
