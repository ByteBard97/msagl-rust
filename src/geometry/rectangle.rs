use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;

/// Axis-aligned bounding box with left/bottom/right/top bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rectangle {
    left: f64,
    bottom: f64,
    right: f64,
    top: f64,
}

impl Rectangle {
    /// Create from explicit bounds: left, bottom, right, top.
    ///
    /// Panics in debug mode if bounds are inverted (left > right or bottom > top).
    pub fn new(left: f64, bottom: f64, right: f64, top: f64) -> Self {
        debug_assert!(left <= right && bottom <= top,
            "Rectangle::new called with inverted bounds: left={left}, right={right}, bottom={bottom}, top={top}");
        Self {
            left,
            bottom,
            right,
            top,
        }
    }

    pub fn from_points(a: Point, b: Point) -> Self {
        Self {
            left: a.x().min(b.x()),
            bottom: a.y().min(b.y()),
            right: a.x().max(b.x()),
            top: a.y().max(b.y()),
        }
    }

    pub fn from_point(p: Point) -> Self {
        Self {
            left: p.x(),
            bottom: p.y(),
            right: p.x(),
            top: p.y(),
        }
    }

    pub fn empty() -> Self {
        Self {
            left: 0.0,
            right: -1.0,
            bottom: 0.0,
            top: -1.0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.right < self.left
    }

    pub fn left(&self) -> f64 {
        self.left
    }
    pub fn bottom(&self) -> f64 {
        self.bottom
    }
    pub fn right(&self) -> f64 {
        self.right
    }
    pub fn top(&self) -> f64 {
        self.top
    }
    pub fn width(&self) -> f64 {
        self.right - self.left
    }
    pub fn height(&self) -> f64 {
        self.top - self.bottom
    }

    pub fn center(&self) -> Point {
        Point::new(
            (self.left + self.right) / 2.0,
            (self.bottom + self.top) / 2.0,
        )
    }

    pub fn left_bottom(&self) -> Point {
        Point::new(self.left, self.bottom)
    }
    pub fn right_top(&self) -> Point {
        Point::new(self.right, self.top)
    }
    pub fn left_top(&self) -> Point {
        Point::new(self.left, self.top)
    }
    pub fn right_bottom(&self) -> Point {
        Point::new(self.right, self.bottom)
    }

    pub fn contains(&self, p: Point) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        p.x() >= self.left - eps
            && p.x() <= self.right + eps
            && p.y() >= self.bottom - eps
            && p.y() <= self.top + eps
    }

    pub fn contains_with_padding(&self, p: Point, padding: f64) -> bool {
        p.x() >= self.left - padding
            && p.x() <= self.right + padding
            && p.y() >= self.bottom - padding
            && p.y() <= self.top + padding
    }

    pub fn contains_rect(&self, other: &Rectangle) -> bool {
        self.contains(other.left_bottom()) && self.contains(other.right_top())
    }

    pub fn intersects(&self, other: &Rectangle) -> bool {
        self.intersects_on_x(other) && self.intersects_on_y(other)
    }

    pub fn intersects_on_x(&self, other: &Rectangle) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        !(other.left > self.right + eps || other.right < self.left - eps)
    }

    pub fn intersects_on_y(&self, other: &Rectangle) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        !(other.bottom > self.top + eps || other.top < self.bottom - eps)
    }

    pub fn intersection(&self, other: &Rectangle) -> Rectangle {
        if !self.intersects(other) {
            return Rectangle::empty();
        }
        Rectangle::new(
            self.left.max(other.left),
            self.bottom.max(other.bottom),
            self.right.min(other.right),
            self.top.min(other.top),
        )
    }

    pub fn add_point(&mut self, p: Point) {
        if self.is_empty() {
            *self = Rectangle::from_point(p);
        } else {
            self.left = self.left.min(p.x());
            self.right = self.right.max(p.x());
            self.bottom = self.bottom.min(p.y());
            self.top = self.top.max(p.y());
        }
    }

    pub fn add_rect(&mut self, other: &Rectangle) {
        if other.is_empty() {
            return;
        }
        if self.is_empty() {
            *self = *other;
        } else {
            self.left = self.left.min(other.left);
            self.right = self.right.max(other.right);
            self.bottom = self.bottom.min(other.bottom);
            self.top = self.top.max(other.top);
        }
    }

    pub fn pad(&mut self, padding: f64) {
        self.left -= padding;
        self.bottom -= padding;
        self.right += padding;
        self.top += padding;
    }
}
