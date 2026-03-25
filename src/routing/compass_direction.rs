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

    /// Convert from index back to direction.
    /// Matches TS `CompassVector.ToIndex` inverse.
    pub fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Self::North),
            1 => Some(Self::East),
            2 => Some(Self::South),
            3 => Some(Self::West),
            _ => None,
        }
    }

    /// Check if two directions are parallel (same or opposite).
    /// Matches TS `CompassVector.DirectionsAreParallel`.
    pub fn is_parallel(self, other: Self) -> bool {
        self == other || self == other.opposite()
    }

    /// Determine the compass direction from `from` to `to`.
    ///
    /// Matches TS `CompassVector.DirectionFromPointToPoint` /
    /// `CompassVector.VectorDirectionPP` for pure (axis-aligned) directions.
    ///
    /// Returns `None` if the points are identical (distance < epsilon).
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

    /// Convert direction to a unit point vector.
    /// Matches TS `CompassVector.toPoint(dir)`.
    pub fn to_point(self) -> Point {
        match self {
            Self::North => Point::new(0.0, 1.0),
            Self::East => Point::new(1.0, 0.0),
            Self::South => Point::new(0.0, -1.0),
            Self::West => Point::new(-1.0, 0.0),
        }
    }
}