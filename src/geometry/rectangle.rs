use crate::geometry::point::Point;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rectangle {
    left: f64,
    bottom: f64,
    right: f64,
    top: f64,
}

impl Rectangle {
    pub fn new(left: f64, bottom: f64, right: f64, top: f64) -> Self {
        Self { left, bottom, right, top }
    }
    pub fn empty() -> Self {
        Self { left: 0.0, right: -1.0, bottom: 0.0, top: -1.0 }
    }
    pub fn is_empty(&self) -> bool { self.right < self.left }
    pub fn from_point(p: Point) -> Self {
        Self { left: p.x(), bottom: p.y(), right: p.x(), top: p.y() }
    }
    pub fn add_point(&mut self, p: Point) {
        if self.is_empty() {
            *self = Self::from_point(p);
        } else {
            self.left = self.left.min(p.x());
            self.right = self.right.max(p.x());
            self.bottom = self.bottom.min(p.y());
            self.top = self.top.max(p.y());
        }
    }
}
