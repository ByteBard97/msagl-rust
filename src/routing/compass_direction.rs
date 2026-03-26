use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;

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
        } else if dx.abs() < GeomConstants::SQUARE_OF_DISTANCE_EPSILON && dy.abs() < GeomConstants::SQUARE_OF_DISTANCE_EPSILON {
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

    /// Convert to the TS/C# bitflag value for this direction.
    /// N=1, E=2, S=4, W=8 (matching TS `Direction` enum).
    pub fn to_flag(self) -> u8 {
        match self {
            Self::North => 1,
            Self::East => 2,
            Self::South => 4,
            Self::West => 8,
        }
    }
}

/// A set of directions represented as a bitflag, matching the TS/C# `Direction` enum.
///
/// TS uses `Direction` as a bitmask: None=0, North=1, East=2, South=4, West=8.
/// Compound directions are bitwise OR: NE=3, NW=9, SE=6, SW=12, NESW=15.
///
/// This type faithfully ports that pattern for the heuristic bend estimation
/// in SsstRectilinearPath.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirectionFlags(pub u8);

impl DirectionFlags {
    pub const NONE: Self = Self(0);
    pub const NORTH: Self = Self(1);
    pub const EAST: Self = Self(2);
    pub const SOUTH: Self = Self(4);
    pub const WEST: Self = Self(8);
    pub const ALL: Self = Self(15); // N|E|S|W

    /// Create from a single CompassDirection.
    pub fn from_direction(dir: CompassDirection) -> Self {
        Self(dir.to_flag())
    }

    /// Create from Option<CompassDirection> -- None maps to DirectionFlags::NONE.
    pub fn from_option(dir: Option<CompassDirection>) -> Self {
        match dir {
            Some(d) => Self::from_direction(d),
            None => Self::NONE,
        }
    }

    pub fn is_none(self) -> bool {
        self.0 == 0
    }

    pub fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    pub fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    pub fn intersect(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    pub fn remove(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    /// Whether this is a single pure direction (exactly one bit set).
    /// Matches TS `CompassVector.IsPureDirection`.
    pub fn is_pure(self) -> bool {
        self.0 != 0 && (self.0 & (self.0 - 1)) == 0
    }

    /// Compute VectorDirection from a point vector.
    /// Matches TS `CompassVector.VectorDirection(vectorToTarget)`.
    /// Returns a DirectionFlags that may be compound (e.g. NE if both x>0 and y>0).
    pub fn vector_direction(dx: f64, dy: f64) -> Self {
        let eps = 1e-6;
        let mut r = Self::NONE;
        if dx > eps {
            r = r.union(Self::EAST);
        } else if dx < -eps {
            r = r.union(Self::WEST);
        }
        if dy > eps {
            r = r.union(Self::NORTH);
        } else if dy < -eps {
            r = r.union(Self::SOUTH);
        }
        r
    }

    /// Left turn of a pure direction.
    /// Matches TS `SsstRectilinearPath.Left`.
    pub fn left(self) -> Self {
        match self.0 {
            0 => Self::NONE,
            1 => Self::WEST,  // N -> W
            2 => Self::NORTH, // E -> N
            4 => Self::EAST,  // S -> E
            8 => Self::SOUTH, // W -> S
            _ => Self::NONE,  // not pure
        }
    }

    /// Right turn of a pure direction.
    /// Matches TS `SsstRectilinearPath.Right`.
    pub fn right(self) -> Self {
        match self.0 {
            0 => Self::NONE,
            1 => Self::EAST,  // N -> E
            2 => Self::SOUTH, // E -> S
            4 => Self::WEST,  // S -> W
            8 => Self::NORTH, // W -> N
            _ => Self::NONE,  // not pure
        }
    }
}

/// The `AddOneTurn` lookup table from TS/C# SsstRectilinearPath.
/// Index is the Direction bitflag value (0..15).
/// Value is the direction set after adding one turn.
pub const ADD_ONE_TURN: [u8; 16] = [
    0,  // 0: None
    11, // 1: N -> N|E|W = 1|2|8 = 11
    7,  // 2: E -> N|E|S = 1|2|4 = 7
    15, // 3: N|E
    14, // 4: S -> E|S|W = 2|4|8 = 14
    15, // 5
    15, // 6
    15, // 7
    13, // 8: W -> N|S|W = 1|4|8 = 13
    15, // 9
    15, // 10
    15, // 11
    15, // 12
    15, // 13
    15, // 14
    15, // 15
];