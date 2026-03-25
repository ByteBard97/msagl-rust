use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use crate::geometry::point_comparer::GeomConstants;
use super::obstacle_side::{ObstacleSide, SideType};
use super::scan_direction::ScanDirection;

/// Key for ordering sides on the scan line.
/// Ordered by: scan-perpendicular coordinate → side type (High before Low at same position) → obstacle ordinal.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct SideKey {
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
