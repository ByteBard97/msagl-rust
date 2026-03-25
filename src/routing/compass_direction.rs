use crate::geometry::point::Point;

/// Four compass directions for VertexEntry[4] indexing and path search.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CompassDirection {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
}

impl CompassDirection {
    pub fn all() -> [Self; 4] {
        [Self::North, Self::East, Self::South, Self::West]
    }

    pub fn index(self) -> usize {
        self as usize
    }

    pub fn opposite(self) -> Self {
        match self {
            Self::North => Self::South,
            Self::East => Self::West,
            Self::South => Self::North,
            Self::West => Self::East,
        }
    }

    pub fn left(self) -> Self {
        match self {
            Self::North => Self::West,
            Self::East => Self::North,
            Self::South => Self::East,
            Self::West => Self::South,
        }
    }

    pub fn right(self) -> Self {
        match self {
            Self::North => Self::East,
            Self::East => Self::South,
            Self::South => Self::West,
            Self::West => Self::North,
        }
    }

    /// Determine the compass direction from `from` to `to`.
    ///
    /// For rectilinear graphs, edges are axis-aligned. Returns `None` if
    /// the points are identical (distance < 1e-12).
    /// On a diagonal tie, horizontal direction is preferred.
    pub fn from_points(from: Point, to: Point) -> Option<Self> {
        let dx = to.x() - from.x();
        let dy = to.y() - from.y();

        if dx.abs() > dy.abs() {
            if dx > 0.0 { Some(Self::East) } else { Some(Self::West) }
        } else if dy.abs() > dx.abs() {
            if dy > 0.0 { Some(Self::North) } else { Some(Self::South) }
        } else if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
            None
        } else {
            // Diagonal tie-break: prefer horizontal
            if dx > 0.0 { Some(Self::East) } else { Some(Self::West) }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compass_direction_has_four_variants() {
        let all = CompassDirection::all();
        assert_eq!(all.len(), 4);
    }

    #[test]
    fn compass_direction_opposite() {
        assert_eq!(CompassDirection::North.opposite(), CompassDirection::South);
        assert_eq!(CompassDirection::East.opposite(), CompassDirection::West);
        assert_eq!(CompassDirection::South.opposite(), CompassDirection::North);
        assert_eq!(CompassDirection::West.opposite(), CompassDirection::East);
    }

    #[test]
    fn compass_direction_to_index() {
        let indices: Vec<usize> = CompassDirection::all().iter().map(|d| d.index()).collect();
        assert_eq!(indices, vec![0, 1, 2, 3]);
    }

    #[test]
    fn compass_direction_left_right_turns() {
        assert_eq!(CompassDirection::North.left(), CompassDirection::West);
        assert_eq!(CompassDirection::North.right(), CompassDirection::East);
        assert_eq!(CompassDirection::East.left(), CompassDirection::North);
        assert_eq!(CompassDirection::East.right(), CompassDirection::South);
    }

    #[test]
    fn from_points_axis_aligned() {
        let origin = Point::new(0.0, 0.0);
        assert_eq!(CompassDirection::from_points(origin, Point::new(10.0, 0.0)), Some(CompassDirection::East));
        assert_eq!(CompassDirection::from_points(origin, Point::new(-10.0, 0.0)), Some(CompassDirection::West));
        assert_eq!(CompassDirection::from_points(origin, Point::new(0.0, 10.0)), Some(CompassDirection::North));
        assert_eq!(CompassDirection::from_points(origin, Point::new(0.0, -10.0)), Some(CompassDirection::South));
    }

    #[test]
    fn from_points_identical_returns_none() {
        let p = Point::new(5.0, 5.0);
        assert_eq!(CompassDirection::from_points(p, p), None);
    }

    #[test]
    fn from_points_diagonal_prefers_horizontal() {
        let origin = Point::new(0.0, 0.0);
        // Equal dx and dy — horizontal wins
        assert_eq!(CompassDirection::from_points(origin, Point::new(5.0, 5.0)), Some(CompassDirection::East));
        assert_eq!(CompassDirection::from_points(origin, Point::new(-5.0, -5.0)), Some(CompassDirection::West));
    }
}
