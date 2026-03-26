use ordered_float::OrderedFloat;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::ops::{Add, Div, Mul, Neg, Sub};

use crate::geometry::point_comparer::GeomConstants;

/// Triangle orientation result, matching C# `TriangleOrientation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriangleOrientation {
    Counterclockwise,
    Clockwise,
    Collinear,
}

#[derive(Clone, Copy)]
pub struct Point {
    x: OrderedFloat<f64>,
    y: OrderedFloat<f64>,
}

impl Point {
    pub const ORIGIN: Point = Point {
        x: OrderedFloat(0.0),
        y: OrderedFloat(0.0),
    };

    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x: OrderedFloat(x),
            y: OrderedFloat(y),
        }
    }

    #[inline]
    pub fn x(&self) -> f64 {
        self.x.into_inner()
    }

    #[inline]
    pub fn y(&self) -> f64 {
        self.y.into_inner()
    }

    #[inline]
    pub fn length(&self) -> f64 {
        (self.x() * self.x() + self.y() * self.y()).sqrt()
    }

    #[inline]
    pub fn length_squared(&self) -> f64 {
        self.x() * self.x() + self.y() * self.y()
    }

    #[inline]
    pub fn l1(&self) -> f64 {
        self.x().abs() + self.y().abs()
    }

    #[inline]
    pub fn dot(&self, other: Point) -> f64 {
        self.x() * other.x() + self.y() * other.y()
    }

    #[inline]
    pub fn cross(a: Point, b: Point) -> f64 {
        a.x() * b.y() - a.y() * b.x()
    }

    pub fn normalize(&self) -> Point {
        let len = self.length();
        debug_assert!(
            len > GeomConstants::DISTANCE_EPSILON,
            "cannot normalize zero-length vector"
        );
        Point::new(self.x() / len, self.y() / len)
    }

    #[inline]
    pub fn rotate90_ccw(&self) -> Point {
        Point::new(-self.y(), self.x())
    }

    #[inline]
    pub fn rotate90_cw(&self) -> Point {
        Point::new(self.y(), -self.x())
    }

    pub fn rotate(&self, angle: f64) -> Point {
        let c = angle.cos();
        let s = angle.sin();
        Point::new(c * self.x() - s * self.y(), s * self.x() + c * self.y())
    }

    #[inline]
    pub fn close_to(&self, other: Point) -> bool {
        (*self - other).length() <= GeomConstants::DISTANCE_EPSILON
    }

    #[inline]
    pub fn close_to_with_eps(&self, other: Point, eps: f64) -> bool {
        (*self - other).length() <= eps
    }

    #[inline]
    pub fn middle(a: Point, b: Point) -> Point {
        (a + b) / 2.0
    }

    /// Round a point's coordinates to the standard geometric precision.
    /// Matches C# `ApproximateComparer.Round(Point)`.
    #[inline]
    pub fn round(p: Point) -> Point {
        Point::new(GeomConstants::round(p.x()), GeomConstants::round(p.y()))
    }

    pub fn line_line_intersection(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
        let ba = b - a;
        let dc = d - c;
        let ca = c - a;
        let denom = ba.x() * dc.y() - ba.y() * dc.x();
        if denom.abs() < GeomConstants::TOLERANCE {
            return None;
        }
        let t = (ca.x() * dc.y() - ca.y() * dc.x()) / denom;
        Some(a + ba * t)
    }

    #[inline]
    pub fn signed_doubled_triangle_area(a: Point, b: Point, c: Point) -> f64 {
        Point::cross(b - a, c - a)
    }

    /// Determine triangle orientation using `DISTANCE_EPSILON` threshold.
    /// Faithful port of C# `Point.GetTriangleOrientation`.
    #[inline]
    pub fn get_triangle_orientation(
        corner_a: Point,
        corner_b: Point,
        corner_c: Point,
    ) -> TriangleOrientation {
        let area = Point::signed_doubled_triangle_area(corner_a, corner_b, corner_c);
        if area > GeomConstants::DISTANCE_EPSILON {
            TriangleOrientation::Counterclockwise
        } else if area < -GeomConstants::DISTANCE_EPSILON {
            TriangleOrientation::Clockwise
        } else {
            TriangleOrientation::Collinear
        }
    }

    /// Determine triangle orientation using `INTERSECTION_EPSILON` threshold.
    /// Faithful port of C# `Point.GetTriangleOrientationWithIntersectionEpsilon`.
    #[inline]
    pub fn get_triangle_orientation_with_intersection_epsilon(
        corner_a: Point,
        corner_b: Point,
        corner_c: Point,
    ) -> TriangleOrientation {
        let area = Point::signed_doubled_triangle_area(corner_a, corner_b, corner_c);
        if area > GeomConstants::INTERSECTION_EPSILON {
            TriangleOrientation::Counterclockwise
        } else if area < -GeomConstants::INTERSECTION_EPSILON {
            TriangleOrientation::Clockwise
        } else {
            TriangleOrientation::Collinear
        }
    }
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}
impl Eq for Point {}

impl Hash for Point {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
    }
}

impl Ord for Point {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.x.cmp(&other.x).then(self.y.cmp(&other.y))
    }
}

impl PartialOrd for Point {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x(), self.y())
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x(), self.y())
    }
}

impl Add for Point {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x() + rhs.x(), self.y() + rhs.y())
    }
}

impl Sub for Point {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x() - rhs.x(), self.y() - rhs.y())
    }
}

impl Mul<f64> for Point {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x() * rhs, self.y() * rhs)
    }
}

impl Mul<Point> for f64 {
    type Output = Point;
    #[inline]
    fn mul(self, rhs: Point) -> Point {
        Point::new(self * rhs.x(), self * rhs.y())
    }
}

impl Div<f64> for Point {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f64) -> Self {
        Self::new(self.x() / rhs, self.y() / rhs)
    }
}

impl Neg for Point {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x(), -self.y())
    }
}
