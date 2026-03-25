use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use std::cmp::Ordering;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    North,
    East,
}

#[derive(Clone, Copy, Debug)]
pub struct ScanDirection {
    pub dir: Direction,
}

impl ScanDirection {
    pub fn horizontal() -> Self { Self { dir: Direction::East } }
    pub fn vertical() -> Self { Self { dir: Direction::North } }

    #[inline]
    pub fn coord(&self, p: Point) -> f64 {
        match self.dir {
            Direction::East => p.x(),
            Direction::North => p.y(),
        }
    }

    #[inline]
    pub fn perp_coord(&self, p: Point) -> f64 {
        match self.dir {
            Direction::East => p.y(),
            Direction::North => p.x(),
        }
    }

    pub fn compare(&self, a: Point, b: Point) -> Ordering {
        let perp = GeomConstants::compare(self.perp_coord(a), self.perp_coord(b));
        if perp != Ordering::Equal { return perp; }
        GeomConstants::compare(self.coord(a), self.coord(b))
    }

    pub fn compare_perp(&self, a: Point, b: Point) -> Ordering {
        GeomConstants::compare(self.perp_coord(a), self.perp_coord(b))
    }

    pub fn compare_scan(&self, a: Point, b: Point) -> Ordering {
        GeomConstants::compare(self.coord(a), self.coord(b))
    }

    pub fn is_flat(&self, start: Point, end: Point) -> bool {
        GeomConstants::close(self.perp_coord(start), self.perp_coord(end))
    }

    pub fn is_perpendicular(&self, start: Point, end: Point) -> bool {
        GeomConstants::close(self.coord(start), self.coord(end))
    }

    pub fn make_point(&self, coord: f64, perp_coord: f64) -> Point {
        match self.dir {
            Direction::East => Point::new(coord, perp_coord),
            Direction::North => Point::new(perp_coord, coord),
        }
    }

    pub fn is_horizontal(&self) -> bool {
        self.dir == Direction::East
    }

    pub fn is_vertical(&self) -> bool {
        self.dir == Direction::North
    }

    pub fn perpendicular(&self) -> Self {
        match self.dir {
            Direction::East => Self::vertical(),
            Direction::North => Self::horizontal(),
        }
    }

    pub fn min(&self, a: Point, b: Point) -> Point {
        if self.compare(a, b).is_le() { a } else { b }
    }

    pub fn max(&self, a: Point, b: Point) -> Point {
        if self.compare(a, b).is_ge() { a } else { b }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::point::Point;

    #[test]
    fn horizontal_scan_is_horizontal() {
        let dir = ScanDirection::horizontal();
        assert!(dir.is_horizontal());
        assert!(!dir.is_vertical());
    }

    #[test]
    fn vertical_scan_is_vertical() {
        let dir = ScanDirection::vertical();
        assert!(dir.is_vertical());
        assert!(!dir.is_horizontal());
    }

    #[test]
    fn perpendicular_swaps_direction() {
        let h = ScanDirection::horizontal();
        let v = h.perpendicular();
        assert!(v.is_vertical());
        assert!(v.perpendicular().is_horizontal());
    }

    #[test]
    fn min_max_returns_correct_point() {
        let dir = ScanDirection::horizontal();
        let a = Point::new(5.0, 10.0);
        let b = Point::new(5.0, 20.0);
        // Horizontal scan: perp is Y. a has lower Y, so a is "less"
        assert_eq!(dir.min(a, b), a);
        assert_eq!(dir.max(a, b), b);
    }
}
