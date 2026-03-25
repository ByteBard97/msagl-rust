use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::scan_direction::ScanDirection;

/// A side currently active on the scan line.
#[derive(Clone, Debug)]
pub struct ActiveSide {
    pub obstacle_index: usize,
    /// The perpendicular coordinate where this side intersects the current sweep position.
    pub perp_coord: f64,
    /// Start point of the side.
    pub start: Point,
    /// End point of the side.
    pub end: Point,
    pub is_low_side: bool,
}

/// Key for ordering sides on the scan line.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct SideKey {
    perp_coord: OrderedFloat<f64>,
    obstacle_ordinal: usize,
}

/// Maintains the set of obstacle sides currently intersecting the sweep line.
pub struct RectilinearScanLine {
    scan_direction: ScanDirection,
    sides: BTreeMap<SideKey, ActiveSide>,
}

impl RectilinearScanLine {
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self {
            scan_direction,
            sides: BTreeMap::new(),
        }
    }

    /// Insert an obstacle side at the given perpendicular coordinate.
    pub fn insert(&mut self, side: ActiveSide) {
        let key = SideKey {
            perp_coord: OrderedFloat(GeomConstants::round(side.perp_coord)),
            obstacle_ordinal: side.obstacle_index,
        };
        self.sides.insert(key, side);
    }

    /// Remove an obstacle side.
    pub fn remove(&mut self, obstacle_index: usize, perp_coord: f64) {
        let key = SideKey {
            perp_coord: OrderedFloat(GeomConstants::round(perp_coord)),
            obstacle_ordinal: obstacle_index,
        };
        self.sides.remove(&key);
    }

    /// Find the nearest side below (lower perpendicular coordinate) the given coordinate.
    pub fn low_neighbor(&self, perp_coord: f64) -> Option<&ActiveSide> {
        let key = SideKey {
            perp_coord: OrderedFloat(GeomConstants::round(perp_coord)),
            obstacle_ordinal: 0,
        };
        self.sides.range(..key).next_back().map(|(_, s)| s)
    }

    /// Find the nearest side above (higher perpendicular coordinate) the given coordinate.
    pub fn high_neighbor(&self, perp_coord: f64) -> Option<&ActiveSide> {
        let key = SideKey {
            perp_coord: OrderedFloat(GeomConstants::round(perp_coord)),
            obstacle_ordinal: usize::MAX,
        };
        self.sides.range(key..).next().map(|(_, s)| s)
    }

    /// Number of active sides.
    pub fn len(&self) -> usize {
        self.sides.len()
    }

    pub fn is_empty(&self) -> bool {
        self.sides.is_empty()
    }
}

// Suppress unused field warning for scan_direction — will be used in Phase 3 sweep.
impl RectilinearScanLine {
    #[allow(dead_code)]
    pub fn scan_direction(&self) -> ScanDirection {
        self.scan_direction
    }
}
