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
}
