use crate::geometry::point::Point;

/// Approximate distance epsilon matching C# `ApproximateComparer.DistanceEpsilon`.
const DISTANCE_EPSILON: f64 = 1e-6;
const HALF_EPSILON: f64 = DISTANCE_EPSILON / 2.0;

/// Four compass directions for `VertexEntry` indexing and path search.
///
/// This enum is used for indexing into the `vertex_entries[4]` array on each vertex.
/// Values 0..3 are stable array indices.
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

    /// Determine the pure compass direction from `from` to `to`.
    ///
    /// For rectilinear graphs, edges are axis-aligned. Returns `None` if
    /// the points are identical (distance < epsilon).
    /// On a diagonal tie, horizontal direction is preferred.
    pub fn from_points(from: Point, to: Point) -> Option<Self> {
        let dx = to.x() - from.x();
        let dy = to.y() - from.y();

        if dx.abs() > dy.abs() {
            if dx > 0.0 {
                Some(Self::East)
            } else {
                Some(Self::West)
            }
        } else if dy.abs() > dx.abs() {
            if dy > 0.0 {
                Some(Self::North)
            } else {
                Some(Self::South)
            }
        } else if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
            None
        } else {
            // Diagonal tie-break: prefer horizontal
            if dx > 0.0 {
                Some(Self::East)
            } else {
                Some(Self::West)
            }
        }
    }

    /// Convert to the bitflag-based `Direction` type.
    pub fn to_direction(self) -> Direction {
        match self {
            Self::North => Direction::NORTH,
            Self::East => Direction::EAST,
            Self::South => Direction::SOUTH,
            Self::West => Direction::WEST,
        }
    }

    /// Convert from a pure `Direction` flag. Panics if compound or None.
    pub fn from_direction(dir: Direction) -> Self {
        match dir.0 {
            1 => Self::North,
            2 => Self::East,
            4 => Self::South,
            8 => Self::West,
            _ => panic!("from_direction called with non-pure direction: {:?}", dir),
        }
    }
}

// ---------------------------------------------------------------------------
// Direction — bitflag type matching C# `[Flags] enum Direction`
// ---------------------------------------------------------------------------

/// Bitflag direction type matching C# `Direction`.
///
/// Values: None=0, North=1, East=2, South=4, West=8.
/// Compound directions use bitwise OR: NE = North|East = 3, etc.
/// Used in the bend estimation heuristic with the `ADD_ONE_TURN` table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Direction(pub u8);

impl Direction {
    pub const NONE: Direction = Direction(0);
    pub const NORTH: Direction = Direction(1);
    pub const EAST: Direction = Direction(2);
    pub const SOUTH: Direction = Direction(4);
    pub const WEST: Direction = Direction(8);
    /// All four cardinal directions.
    pub const ALL: Direction = Direction(1 | 2 | 4 | 8); // 15

    /// Check if this direction is "pure" — exactly one cardinal bit set.
    pub fn is_pure(self) -> bool {
        matches!(self.0, 1 | 2 | 4 | 8)
    }

    /// Check if this direction is None (no bits set).
    pub fn is_none(self) -> bool {
        self.0 == 0
    }

    /// Bitwise OR of two directions.
    pub fn union(self, other: Direction) -> Direction {
        Direction(self.0 | other.0)
    }

    /// Bitwise AND of two directions.
    pub fn intersect(self, other: Direction) -> Direction {
        Direction(self.0 & other.0)
    }

    /// Bitwise complement within the 4-bit direction space.
    pub fn complement(self) -> Direction {
        Direction(!self.0 & 0x0F)
    }

    /// Check if `direction` is fully contained in `self`.
    /// Matches C# `IsInDirs`: `direction == (direction & self)`.
    pub fn contains(self, direction: Direction) -> bool {
        direction.0 == (direction.0 & self.0)
    }

    /// Rotate a pure direction left (counter-clockwise).
    /// Matches C# `SsstRectilinearPath.Left`.
    pub fn left(self) -> Direction {
        match self.0 {
            0 => Direction::NONE,
            1 => Direction::WEST,  // N -> W
            2 => Direction::NORTH, // E -> N
            4 => Direction::EAST,  // S -> E
            8 => Direction::SOUTH, // W -> S
            _ => panic!("left() called on non-pure direction: {:?}", self),
        }
    }

    /// Rotate a pure direction right (clockwise).
    /// Matches C# `SsstRectilinearPath.Right`.
    pub fn right(self) -> Direction {
        match self.0 {
            0 => Direction::NONE,
            1 => Direction::EAST,  // N -> E
            2 => Direction::SOUTH, // E -> S
            4 => Direction::WEST,  // S -> W
            8 => Direction::NORTH, // W -> N
            _ => panic!("right() called on non-pure direction: {:?}", self),
        }
    }

    /// Compute the (possibly compound) direction from point `a` to point `b`.
    ///
    /// Matches C# `CompassVector.VectorDirection(a, b)` / `DirectionsFromPointToPoint`.
    pub fn from_point_to_point(a: Point, b: Point) -> Direction {
        let dx = b.x() - a.x();
        let dy = b.y() - a.y();

        let mut r = Direction::NONE;
        if dx > HALF_EPSILON {
            r = Direction::EAST;
        } else if -dx > HALF_EPSILON {
            r = Direction::WEST;
        }
        if dy > HALF_EPSILON {
            r = r.union(Direction::NORTH);
        } else if -dy > HALF_EPSILON {
            r = r.union(Direction::SOUTH);
        }
        r
    }

    /// Compute the (possibly compound) direction from a vector.
    ///
    /// Matches C# `CompassVector.VectorDirection(Point d)`.
    pub fn from_vector(dx: f64, dy: f64) -> Direction {
        let mut r = Direction::NONE;
        if dx > DISTANCE_EPSILON {
            r = Direction::EAST;
        } else if dx < -DISTANCE_EPSILON {
            r = Direction::WEST;
        }
        if dy > DISTANCE_EPSILON {
            r = r.union(Direction::NORTH);
        } else if dy < -DISTANCE_EPSILON {
            r = r.union(Direction::SOUTH);
        }
        r
    }

    /// Compute pure direction from point to point. Panics if result is compound.
    /// Matches C# `CompassVector.PureDirectionFromPointToPoint`.
    pub fn pure_from_point_to_point(a: Point, b: Point) -> Direction {
        let dir = Self::from_point_to_point(a, b);
        debug_assert!(dir.is_pure(), "Impure direction from {:?} to {:?}: {:?}", a, b, dir);
        dir
    }

    /// Convert a pure Direction to an array index (0..3).
    /// Matches C# `CompassVector.ToIndex`.
    pub fn to_index(self) -> usize {
        match self.0 {
            1 => 0, // North
            2 => 1, // East
            4 => 2, // South
            8 => 3, // West
            _ => panic!("to_index called on non-pure direction: {:?}", self),
        }
    }

    /// Convert a pure Direction to `CompassDirection`.
    pub fn to_compass(self) -> CompassDirection {
        CompassDirection::from_direction(self)
    }

    /// The `AddOneTurn` lookup table from C#.
    ///
    /// Indexed by the direction bits (0..15). Each entry gives the set of
    /// directions reachable with one additional turn from the input direction set.
    pub fn add_one_turn(self) -> Direction {
        ADD_ONE_TURN[self.0 as usize]
    }
}

/// Implements `|` for Direction.
impl std::ops::BitOr for Direction {
    type Output = Direction;
    fn bitor(self, rhs: Direction) -> Direction {
        Direction(self.0 | rhs.0)
    }
}

/// Implements `&` for Direction.
impl std::ops::BitAnd for Direction {
    type Output = Direction;
    fn bitand(self, rhs: Direction) -> Direction {
        Direction(self.0 & rhs.0)
    }
}

/// Implements `!` for Direction (complement within 4-bit space).
impl std::ops::Not for Direction {
    type Output = Direction;
    fn not(self) -> Direction {
        Direction(!self.0 & 0x0F)
    }
}

/// Implements `|=` for Direction.
impl std::ops::BitOrAssign for Direction {
    fn bitor_assign(&mut self, rhs: Direction) {
        self.0 |= rhs.0;
    }
}

/// Implements `&=` for Direction.
impl std::ops::BitAndAssign for Direction {
    fn bitand_assign(&mut self, rhs: Direction) {
        self.0 &= rhs.0;
    }
}

/// The `AddOneTurn` table from C# `SsstRectilinearPath`.
///
/// Index is the direction bitflag value (0..15).
/// Value is the set of directions reachable with one additional turn.
const ADD_ONE_TURN: [Direction; 16] = [
    Direction(0),  // 0  = None -> None
    Direction(1 | 2 | 8),  // 1  = N -> N|E|W
    Direction(1 | 2 | 4),  // 2  = E -> E|N|S
    Direction(15), // 3  = E|N -> all
    Direction(2 | 4 | 8),  // 4  = S -> E|S|W
    Direction(15), // 5  = S|N -> all
    Direction(15), // 6  = S|E -> all
    Direction(15), // 7
    Direction(1 | 4 | 8),  // 8  = W -> N|S|W (=13)
    Direction(15), // 9
    Direction(15), // 10
    Direction(15), // 11
    Direction(15), // 12
    Direction(15), // 13
    Direction(15), // 14
    Direction(15), // 15
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direction_bitflags_match_csharp() {
        assert_eq!(Direction::NONE.0, 0);
        assert_eq!(Direction::NORTH.0, 1);
        assert_eq!(Direction::EAST.0, 2);
        assert_eq!(Direction::SOUTH.0, 4);
        assert_eq!(Direction::WEST.0, 8);
    }

    #[test]
    fn direction_union_and_intersect() {
        let ne = Direction::NORTH | Direction::EAST;
        assert_eq!(ne.0, 3);
        assert!(ne.contains(Direction::NORTH));
        assert!(ne.contains(Direction::EAST));
        assert!(!ne.contains(Direction::SOUTH));

        let n = ne & Direction::NORTH;
        assert_eq!(n, Direction::NORTH);
    }

    #[test]
    fn direction_is_pure() {
        assert!(Direction::NORTH.is_pure());
        assert!(Direction::EAST.is_pure());
        assert!(Direction::SOUTH.is_pure());
        assert!(Direction::WEST.is_pure());
        assert!(!Direction::NONE.is_pure());
        assert!(!(Direction::NORTH | Direction::EAST).is_pure());
    }

    #[test]
    fn direction_left_right() {
        assert_eq!(Direction::NORTH.left(), Direction::WEST);
        assert_eq!(Direction::NORTH.right(), Direction::EAST);
        assert_eq!(Direction::EAST.left(), Direction::NORTH);
        assert_eq!(Direction::EAST.right(), Direction::SOUTH);
    }

    #[test]
    fn direction_from_point_to_point_pure() {
        let a = Point::new(0.0, 0.0);
        let b = Point::new(10.0, 0.0);
        assert_eq!(Direction::from_point_to_point(a, b), Direction::EAST);
    }

    #[test]
    fn direction_from_point_to_point_compound() {
        let a = Point::new(0.0, 0.0);
        let b = Point::new(10.0, 5.0);
        let dir = Direction::from_point_to_point(a, b);
        assert_eq!(dir, Direction::EAST | Direction::NORTH);
        assert!(!dir.is_pure());
    }

    #[test]
    fn add_one_turn_table_matches_csharp() {
        // N -> N|E|W
        assert_eq!(Direction::NORTH.add_one_turn().0, 1 | 2 | 8);
        // E -> E|N|S
        assert_eq!(Direction::EAST.add_one_turn().0, 1 | 2 | 4);
        // S -> E|S|W
        assert_eq!(Direction::SOUTH.add_one_turn().0, 2 | 4 | 8);
        // W -> N|S|W
        assert_eq!(Direction::WEST.add_one_turn().0, 1 | 4 | 8);
    }

    #[test]
    fn compass_direction_to_direction_roundtrip() {
        for cd in CompassDirection::all() {
            let dir = cd.to_direction();
            let back = dir.to_compass();
            assert_eq!(cd, back);
        }
    }

    #[test]
    fn direction_to_index_matches_compass() {
        assert_eq!(Direction::NORTH.to_index(), 0);
        assert_eq!(Direction::EAST.to_index(), 1);
        assert_eq!(Direction::SOUTH.to_index(), 2);
        assert_eq!(Direction::WEST.to_index(), 3);
    }

    #[test]
    fn direction_complement() {
        let ne = Direction::NORTH | Direction::EAST;
        let sw = !ne;
        assert_eq!(sw, Direction::SOUTH | Direction::WEST);
    }
}
