use crate::geometry::point::Point;

/// Whether this is a low (bottom/left) or high (top/right) side of an obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SideType {
    Low,
    High,
}

/// A side of an obstacle boundary, with slope tracking for scanline intersection.
///
/// Ported from BasicObstacleSide.ts + LowObstacleSide/HighObstacleSide.
/// In the TS, LowObstacleSide traverses clockwise for horizontal scan (counter-clockwise for vertical),
/// while HighObstacleSide does the opposite. In Rust we capture this as start/end points + SideType.
#[derive(Debug, Clone)]
pub struct ObstacleSide {
    side_type: SideType,
    start: Point,
    end: Point,
    obstacle_ordinal: usize,
    slope: f64,
    slope_inverse: f64,
}

impl ObstacleSide {
    pub fn new(side_type: SideType, start: Point, end: Point, obstacle_ordinal: usize) -> Self {
        let dx = end.x() - start.x();
        let dy = end.y() - start.y();
        let slope = if dx.abs() < 1e-10 {
            f64::INFINITY
        } else {
            dy / dx
        };
        let slope_inverse = if dy.abs() < 1e-10 {
            f64::INFINITY
        } else {
            dx / dy
        };
        Self {
            side_type,
            start,
            end,
            obstacle_ordinal,
            slope,
            slope_inverse,
        }
    }

    pub fn side_type(&self) -> SideType {
        self.side_type
    }

    pub fn start(&self) -> Point {
        self.start
    }

    pub fn end(&self) -> Point {
        self.end
    }

    pub fn obstacle_ordinal(&self) -> usize {
        self.obstacle_ordinal
    }

    pub fn slope(&self) -> f64 {
        self.slope
    }

    pub fn slope_inverse(&self) -> f64 {
        self.slope_inverse
    }

    /// Compute the scanline intersection point at a given perpendicular coordinate.
    ///
    /// For horizontal scan (sweeping vertically): given a Y value, compute the X where this side
    /// crosses it.
    /// For vertical scan (sweeping horizontally): given an X value, compute the Y where this side
    /// crosses it.
    pub fn scanline_intersection(&self, perp_coord: f64, is_horizontal_scan: bool) -> f64 {
        if is_horizontal_scan {
            if self.slope.is_infinite() || self.slope.abs() < 1e-10 {
                self.start.x()
            } else {
                self.start.x() + (perp_coord - self.start.y()) * self.slope_inverse
            }
        } else if self.slope_inverse.is_infinite() || self.slope_inverse.abs() < 1e-10 {
            self.start.y()
        } else {
            self.start.y() + (perp_coord - self.start.x()) * self.slope
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::point::Point;

    #[test]
    fn low_side_has_correct_slope() {
        let side = ObstacleSide::new(
            SideType::Low,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            0,
        );
        // Horizontal side: slope = dy/dx = 0/10 = 0
        assert!((side.slope() - 0.0).abs() < 1e-10);
        assert_eq!(side.side_type(), SideType::Low);
    }

    #[test]
    fn high_side_stores_type() {
        let side = ObstacleSide::new(
            SideType::High,
            Point::new(0.0, 10.0),
            Point::new(10.0, 10.0),
            1,
        );
        assert_eq!(side.side_type(), SideType::High);
        assert_eq!(side.obstacle_ordinal(), 1);
    }

    #[test]
    fn vertical_side_slope_is_infinite() {
        let side = ObstacleSide::new(
            SideType::Low,
            Point::new(5.0, 0.0),
            Point::new(5.0, 10.0),
            0,
        );
        assert!(side.slope().is_infinite());
    }

    #[test]
    fn scanline_intersection_vertical_side() {
        // Vertical side at x=5
        let side = ObstacleSide::new(
            SideType::Low,
            Point::new(5.0, 0.0),
            Point::new(5.0, 10.0),
            0,
        );
        // Horizontal scan: given any Y, X should be 5.0
        assert!((side.scanline_intersection(3.0, true) - 5.0).abs() < 1e-10);
        assert!((side.scanline_intersection(7.0, true) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn scanline_intersection_horizontal_side() {
        // Horizontal side at y=10
        let side = ObstacleSide::new(
            SideType::High,
            Point::new(0.0, 10.0),
            Point::new(20.0, 10.0),
            0,
        );
        // Vertical scan: given any X, Y should be 10.0
        assert!((side.scanline_intersection(5.0, false) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn scanline_intersection_diagonal_side() {
        // Diagonal side from (0,0) to (10,10), slope = 1
        let side = ObstacleSide::new(
            SideType::Low,
            Point::new(0.0, 0.0),
            Point::new(10.0, 10.0),
            0,
        );
        assert!((side.slope() - 1.0).abs() < 1e-10);
        // Horizontal scan at y=5: x should be 5
        assert!((side.scanline_intersection(5.0, true) - 5.0).abs() < 1e-10);
        // Vertical scan at x=5: y should be 5
        assert!((side.scanline_intersection(5.0, false) - 5.0).abs() < 1e-10);
    }
}
