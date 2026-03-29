use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::routing::compass_direction::CompassDirection;
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
    pub fn horizontal() -> Self {
        Self {
            dir: Direction::East,
        }
    }
    pub fn vertical() -> Self {
        Self {
            dir: Direction::North,
        }
    }

    /// Return the scan direction for a compass direction.
    ///
    /// Faithful port of C# `ScanDirection.GetInstance(outDir)`.
    /// East/West → horizontal (coord = x); North/South → vertical (coord = y).
    pub fn for_compass(dir: CompassDirection) -> Self {
        match dir {
            CompassDirection::East | CompassDirection::West => Self::horizontal(),
            CompassDirection::North | CompassDirection::South => Self::vertical(),
        }
    }

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
        if perp != Ordering::Equal {
            return perp;
        }
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
        if self.compare(a, b).is_le() {
            a
        } else {
            b
        }
    }

    pub fn max(&self, a: Point, b: Point) -> Point {
        if self.compare(a, b).is_ge() {
            a
        } else {
            b
        }
    }
}
