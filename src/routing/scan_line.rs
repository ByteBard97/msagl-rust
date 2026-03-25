use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use crate::geometry::point_comparer::GeomConstants;
use super::obstacle_side::{ObstacleSide, SideType};
use super::scan_direction::ScanDirection;

/// Key for ordering sides on the scan line.
/// Ordered by: scan-perpendicular coordinate → side type (High before Low at same position) → obstacle ordinal.
///
/// Public so callers can hold a handle to a specific position in the scanline
/// (matching TS `RBNode<BasicObstacleSide>` usage in `Find`, `NextLow`, `NextHigh`).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct SideKey {
    /// The perpendicular coordinate where this side sits.
    coord: OrderedFloat<f64>,
    /// High sides sort before Low sides at the same coordinate.
    /// High = 0, Low = 1, so High comes first in BTreeMap ordering.
    side_type_order: u8,
    /// Obstacle ordinal for final tiebreaking.
    obstacle_ordinal: usize,
}

impl SideKey {
    fn new(coord: f64, side: &ObstacleSide) -> Self {
        Self {
            coord: OrderedFloat(GeomConstants::round(coord)),
            side_type_order: match side.side_type() {
                SideType::High => 0,
                SideType::Low => 1,
            },
            obstacle_ordinal: side.obstacle_ordinal(),
        }
    }
}

/// Maintains the set of obstacle sides currently intersecting the sweep line.
pub struct RectilinearScanLine {
    scan_direction: ScanDirection,
    sides: BTreeMap<SideKey, ObstacleSide>,
}

impl RectilinearScanLine {
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self {
            scan_direction,
            sides: BTreeMap::new(),
        }
    }

    pub fn scan_direction(&self) -> ScanDirection {
        self.scan_direction
    }

    /// Insert an obstacle side. The key coordinate is the side's position perpendicular to the sweep.
    pub fn insert(&mut self, side: ObstacleSide) {
        let coord = self.side_coordinate(&side);
        let key = SideKey::new(coord, &side);
        self.sides.insert(key, side);
    }

    /// Remove an obstacle side.
    pub fn remove(&mut self, side: &ObstacleSide) {
        let coord = self.side_coordinate(side);
        let key = SideKey::new(coord, side);
        self.sides.remove(&key);
    }

    /// Find both neighbors (low and high) around the given coordinate.
    ///
    /// Returns `(low_neighbor, high_neighbor)` where low is the nearest side at a strictly
    /// smaller coordinate and high is the nearest side at a strictly larger coordinate.
    pub fn find_neighbors(&self, coord: f64) -> (Option<&ObstacleSide>, Option<&ObstacleSide>) {
        // Low probe: coord with side_type_order=0, ordinal=0 — everything below this is low.
        let probe_low = SideKey {
            coord: OrderedFloat(GeomConstants::round(coord)),
            side_type_order: 0,
            obstacle_ordinal: 0,
        };
        let low = self.sides.range(..probe_low).next_back().map(|(_, s)| s);

        // High probe: coord with side_type_order=1, ordinal=MAX — everything from here up is high.
        let probe_high = SideKey {
            coord: OrderedFloat(GeomConstants::round(coord)),
            side_type_order: 1,
            obstacle_ordinal: usize::MAX,
        };
        let high = self.sides.range(probe_high..).next().map(|(_, s)| s);

        (low, high)
    }

    /// Find the nearest side below (lower coordinate) the given coordinate.
    pub fn low_neighbor(&self, coord: f64) -> Option<&ObstacleSide> {
        self.find_neighbors(coord).0
    }

    /// Find the nearest side above (higher coordinate) the given coordinate.
    pub fn high_neighbor(&self, coord: f64) -> Option<&ObstacleSide> {
        self.find_neighbors(coord).1
    }

    /// Number of active sides.
    pub fn len(&self) -> usize {
        self.sides.len()
    }

    pub fn is_empty(&self) -> bool {
        self.sides.is_empty()
    }

    /// Find a specific side in the scanline and return its key.
    ///
    /// Matches TS: `RectilinearScanLine.Find(side)` which returns an `RBNode`.
    /// Returns `None` if the side is not present.
    pub fn find(&self, side: &ObstacleSide) -> Option<SideKey> {
        let coord = self.side_coordinate(side);
        let key = SideKey::new(coord, side);
        if self.sides.contains_key(&key) {
            Some(key)
        } else {
            None
        }
    }

    /// Return the side at the given key, if present.
    pub fn get(&self, key: &SideKey) -> Option<&ObstacleSide> {
        self.sides.get(key)
    }

    /// Return the next side in the low (decreasing) direction from the given key.
    ///
    /// Matches TS: `RectilinearScanLine.NextLowR(sideNode)` which returns
    /// `SideTree.previous(sideNode)`.
    pub fn next_low(&self, key: &SideKey) -> Option<(&SideKey, &ObstacleSide)> {
        self.sides.range(..key).next_back()
    }

    /// Return the next side in the high (increasing) direction from the given key.
    ///
    /// Matches TS: `RectilinearScanLine.NextHighR(sideNode)` which returns
    /// `SideTree.next(sideNode)`.
    pub fn next_high(&self, key: &SideKey) -> Option<(&SideKey, &ObstacleSide)> {
        // range is exclusive on the left for `(key..)` — we need to skip past the
        // key itself. Use a range that starts strictly after key.
        use std::ops::Bound;
        self.sides.range((Bound::Excluded(key.clone()), Bound::Unbounded)).next()
    }

    /// Return all sides currently on the scanline, ordered by their scan coordinate.
    ///
    /// Used by the VG generator to enumerate all gaps between consecutive sides
    /// and emit full-width scan segments at each event row.
    pub fn all_sides_ordered(&self) -> Vec<&ObstacleSide> {
        self.sides.values().collect()
    }

    /// Compute the coordinate used for ordering this side on the scanline.
    ///
    /// For horizontal scan (sweeping in Y): sides are vertical edges, keyed by their X position.
    /// For vertical scan (sweeping in X): sides are horizontal edges, keyed by their Y position.
    fn side_coordinate(&self, side: &ObstacleSide) -> f64 {
        if self.scan_direction.is_horizontal() {
            side.start().x()
        } else {
            side.start().y()
        }
    }
}
