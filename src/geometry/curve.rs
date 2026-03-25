use crate::geometry::point::Point;

#[derive(Clone, Debug)]
pub enum CurveSegment {
    Line { from: Point, to: Point },
    Arc { from: Point, to: Point, center: Point, ccw: bool },
}

#[derive(Clone, Debug)]
pub struct Curve {
    segments: Vec<CurveSegment>,
}

impl Curve {
    pub fn new() -> Self { Self { segments: Vec::new() } }
}

impl Default for Curve {
    fn default() -> Self { Self::new() }
}
