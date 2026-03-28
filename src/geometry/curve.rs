use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;

/// A straight line segment between two points.
///
/// C# equivalent: `Microsoft.Msagl.Core.Geometry.Curves.LineSegment`
/// Used by `ObstaclePortEntrance.MaxVisibilitySegment` and
/// `TransientGraphUtility.ExtendEdgeChain`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LineSegment {
    /// The start point of the segment.
    /// C# LineSegment: Start
    pub start: Point,
    /// The end point of the segment.
    /// C# LineSegment: End
    pub end: Point,
}

impl LineSegment {
    /// Create a new line segment from `start` to `end`.
    pub fn new(start: Point, end: Point) -> Self {
        Self { start, end }
    }
}

/// A segment within a compound curve.
#[derive(Clone, Debug)]
pub enum CurveSegment {
    Line {
        from: Point,
        to: Point,
    },
    Arc {
        from: Point,
        to: Point,
        center: Point,
        ccw: bool,
    },
}

impl CurveSegment {
    pub fn start(&self) -> Point {
        match self {
            CurveSegment::Line { from, .. } => *from,
            CurveSegment::Arc { from, .. } => *from,
        }
    }

    pub fn end(&self) -> Point {
        match self {
            CurveSegment::Line { to, .. } => *to,
            CurveSegment::Arc { to, .. } => *to,
        }
    }

    pub fn bounding_box(&self) -> Rectangle {
        match self {
            CurveSegment::Line { from, to } => Rectangle::from_points(*from, *to),
            CurveSegment::Arc {
                from, to, center, ..
            } => {
                let mut bb = Rectangle::from_points(*from, *to);
                bb.add_point(*center);
                bb
            }
        }
    }
}

/// A compound curve consisting of connected line segments and arcs.
#[derive(Clone, Debug)]
pub struct Curve {
    segments: Vec<CurveSegment>,
}

impl Curve {
    pub fn new() -> Self {
        Self {
            segments: Vec::new(),
        }
    }

    pub fn add_line(&mut self, from: Point, to: Point) {
        self.segments.push(CurveSegment::Line { from, to });
    }

    pub fn add_arc(&mut self, from: Point, to: Point, center: Point, ccw: bool) {
        self.segments.push(CurveSegment::Arc {
            from,
            to,
            center,
            ccw,
        });
    }

    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    pub fn start(&self) -> Point {
        self.segments.first().expect("empty curve").start()
    }

    pub fn end(&self) -> Point {
        self.segments.last().expect("empty curve").end()
    }

    pub fn segments(&self) -> impl Iterator<Item = &CurveSegment> {
        self.segments.iter()
    }

    pub fn bounding_box(&self) -> Rectangle {
        let mut bb = Rectangle::empty();
        for seg in &self.segments {
            bb.add_rect(&seg.bounding_box());
        }
        bb
    }

    pub fn from_polyline(poly: &Polyline) -> Self {
        let mut curve = Curve::new();
        let points: Vec<Point> = poly.points().collect();
        for pair in points.windows(2) {
            curve.add_line(pair[0], pair[1]);
        }
        if poly.is_closed() && points.len() >= 2 {
            curve.add_line(*points.last().unwrap(), points[0]);
        }
        curve
    }
}

impl Default for Curve {
    fn default() -> Self {
        Self::new()
    }
}
