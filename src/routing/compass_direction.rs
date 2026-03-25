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
}
