use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;

/// Number of segments used to approximate a circle as a polygon.
/// C# RefineEllipse creates ~8 points; 16 gives smooth enough results
/// for rectilinear routing while keeping vertex count manageable.
const CIRCLE_SEGMENTS: usize = 16;

/// User-facing shape input for the router.
///
/// Wraps a closed Polyline boundary. Rectangles are the common case
/// but any closed polyline is accepted.
///
/// Ported from: routing/shape.ts -- `class Shape`
/// The TS Shape stores `BoundaryCurve: ICurve`. We store `Polyline` directly
/// since we only support polyline boundaries (not arcs/curves).
///
/// C# `Shape.IsGroup` → `is_group: bool` (true when the shape has children).
/// C# `Shape.IsTransparent` → `is_transparent: bool` (edges may cross boundary).
#[derive(Clone, Debug)]
pub struct Shape {
    boundary: Polyline,
    bounding_box: Rectangle,
    /// Whether this shape was constructed as a rectangle (4 axis-aligned sides).
    is_rect: bool,
    /// Whether this shape is a group (contains child shapes).
    ///
    /// Matches C# `Shape.IsGroup` (true when `Children.Count > 0`).
    is_group: bool,
    /// Whether edges are allowed to cross through this shape's boundary.
    ///
    /// Matches C# `Shape.IsTransparent`.
    is_transparent: bool,
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
            is_rect: false,
            is_group: false,
            is_transparent: false,
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
        let bounding_box = poly.bounding_box();
        Self {
            boundary: poly,
            bounding_box,
            is_rect: true,
            is_group: false,
            is_transparent: false,
        }
    }

    /// Create from center point and dimensions.
    pub fn rectangle_centered(cx: f64, cy: f64, width: f64, height: f64) -> Self {
        let hw = width / 2.0;
        let hh = height / 2.0;
        Self::rectangle(cx - hw, cy - hh, width, height)
    }

    /// Create a polygon shape from a list of vertex points.
    ///
    /// The points should be in clockwise order for consistency with the
    /// C# convention. The polyline is automatically closed.
    ///
    /// Matches C#: `CurveFactory.CreateCurveFromPoints(points)`
    pub fn polygon(points: &[Point]) -> Self {
        debug_assert!(
            points.len() >= 3,
            "Polygon must have at least 3 vertices"
        );
        let mut poly = Polyline::new();
        for &p in points {
            poly.add_point(p);
        }
        poly.set_closed(true);

        // Ensure clockwise winding (C# convention).
        // Signed area > 0 means counterclockwise; reverse if so.
        let signed_area = polygon_signed_area(points);
        if signed_area > 0.0 {
            // Reverse: rebuild in opposite order
            let mut reversed = Polyline::new();
            for &p in points.iter().rev() {
                reversed.add_point(p);
            }
            reversed.set_closed(true);
            Self::from_polyline(reversed)
        } else {
            Self::from_polyline(poly)
        }
    }

    /// Create a circle approximation as an N-gon.
    ///
    /// Matches C# `Ellipse` → `RefineEllipse` → polyline conversion.
    /// Uses `CIRCLE_SEGMENTS` (16) vertices evenly spaced around the circle.
    pub fn circle(center: Point, radius: f64) -> Self {
        debug_assert!(radius > 0.0, "Circle radius must be positive");
        let mut poly = Polyline::new();
        // Generate vertices clockwise (C# convention: start at right, go clockwise)
        for i in 0..CIRCLE_SEGMENTS {
            let angle =
                -2.0 * std::f64::consts::PI * (i as f64) / (CIRCLE_SEGMENTS as f64);
            let x = center.x() + radius * angle.cos();
            let y = center.y() + radius * angle.sin();
            poly.add_point(Point::new(x, y));
        }
        poly.set_closed(true);
        Self::from_polyline(poly)
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

    /// Whether this shape was constructed as a rectangle.
    pub fn is_rect(&self) -> bool {
        self.is_rect
    }

    /// Create a group shape — a shape that contains child shapes.
    ///
    /// Matches C# `Shape.IsGroup` (true when `Children.Count > 0`).
    /// Returns a new `Shape` with `is_group = true`.
    pub fn group(boundary: Polyline) -> Self {
        let mut s = Self::from_polyline(boundary);
        s.is_group = true;
        s
    }

    /// Create a rectangular group shape.
    pub fn rectangle_group(x: f64, y: f64, width: f64, height: f64) -> Self {
        let mut s = Self::rectangle(x, y, width, height);
        s.is_group = true;
        s
    }

    /// Whether this shape is a group (contains child shapes).
    ///
    /// Matches C# `Shape.IsGroup`.
    pub fn is_group(&self) -> bool {
        self.is_group
    }

    /// Whether edges may cross through this shape's boundary.
    ///
    /// Matches C# `Shape.IsTransparent`.
    pub fn is_transparent(&self) -> bool {
        self.is_transparent
    }

    /// Set whether edges may cross through this shape's boundary.
    ///
    /// Matches C# `Shape.IsTransparent` setter.
    pub fn set_transparent(&mut self, v: bool) {
        self.is_transparent = v;
    }
}

/// Compute the signed area of a polygon (positive = counterclockwise).
fn polygon_signed_area(pts: &[Point]) -> f64 {
    let n = pts.len();
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += pts[i].x() * pts[j].y();
        area -= pts[j].x() * pts[i].y();
    }
    area / 2.0
}
