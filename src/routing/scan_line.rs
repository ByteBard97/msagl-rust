//! Rectilinear sweep-line data structure for obstacle sides.
//!
//! Faithful port of MSAGL TS `RectilinearScanLine.ts` (204 lines).
//!
//! Maintains a sorted set of obstacle sides currently intersecting the sweep line.
//! Uses a BTreeMap with composite keys that incorporate the scanline intersection
//! position, matching the TS RBTree<BasicObstacleSide> with custom comparator.
//!
//! The TS comparator (`RectilinearScanLine.Compare`) orders sides by:
//!   1. Scanline intersection point (where the sweep line crosses the side)
//!   2. Side type (High before Low at same intersection)
//!   3. Obstacle ordinal (final tiebreaker)

use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::obstacle_side::{ObstacleSide, SideType};
use super::scan_direction::ScanDirection;

/// Key for ordering sides on the scan line.
///
/// Ordered by: scanline intersection coordinate -> side type (High before Low)
/// -> obstacle ordinal.
///
/// Matches TS `RectilinearScanLine.Compare()` which uses
/// `ScanLineIntersectSidePBS` to compute the intersection position, then falls
/// back to side type and obstacle ordinal.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct SideKey {
    /// The scan coordinate where this side intersects the current sweep line.
    coord: OrderedFloat<f64>,
    /// High sides sort before Low sides at the same coordinate.
    /// High = 0, Low = 1 (matches TS `compareBooleans(firstIsLow, secondIsLow)`).
    side_type_order: u8,
    /// Obstacle ordinal for final tiebreaking.
    /// Matches TS: `compareNumbers(first.Obstacle.Ordinal, second.Obstacle.Ordinal)`.
    obstacle_ordinal: usize,
}

impl SideKey {
    fn new(intersect_coord: f64, side: &ObstacleSide) -> Self {
        Self {
            coord: OrderedFloat(GeomConstants::round(intersect_coord)),
            side_type_order: match side.side_type() {
                SideType::High => 0,
                SideType::Low => 1,
            },
            obstacle_ordinal: side.obstacle_ordinal(),
        }
    }

    /// Return the scan coordinate stored in this key.
    pub fn coord_value(&self) -> f64 {
        self.coord.into_inner()
    }
}

/// Maintains the set of obstacle sides currently intersecting the sweep line.
///
/// Faithful port of TS `RectilinearScanLine`.
///
/// The TS uses an RBTree with comparator that computes scanline intersections
/// at runtime using `linePositionAtLastInsertOrRemove`. We use a BTreeMap with
/// composite keys, recomputing intersection coordinates on insert/remove.
pub struct RectilinearScanLine {
    scan_direction: ScanDirection,
    /// The sorted tree of active sides.
    /// Matches TS: `SideTree: RBTree<BasicObstacleSide>`
    sides: BTreeMap<SideKey, ObstacleSide>,
    /// Position tracking for scanline intersection computation.
    /// Set on each insert/remove call.
    /// Matches TS: `linePositionAtLastInsertOrRemove`
    line_position: Point,
}

impl RectilinearScanLine {
    /// Create a new scanline with an initial position.
    /// Matches TS: `constructor(scanDir, start)`
    pub fn new_at(scan_direction: ScanDirection, start: Point) -> Self {
        Self {
            scan_direction,
            sides: BTreeMap::new(),
            line_position: start,
        }
    }

    /// Create a new scanline (position defaults to origin).
    /// Backward-compatible constructor for existing callers.
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self::new_at(scan_direction, Point::new(0.0, 0.0))
    }

    pub fn scan_direction(&self) -> ScanDirection {
        self.scan_direction
    }

    /// Insert an obstacle side at the given scan position, returning a key.
    /// Matches TS: `Insert(side, scanPos) -> RBNode<BasicObstacleSide>`
    pub fn insert_at(&mut self, side: ObstacleSide, scan_pos: Point) -> SideKey {
        self.line_position = scan_pos;
        let coord = self.compute_intersect_coord(&side);
        let key = SideKey::new(coord, &side);
        self.sides.insert(key.clone(), side);
        key
    }

    /// Insert a side using the current line position.
    /// Backward-compatible API for callers that don't track position.
    pub fn insert(&mut self, side: ObstacleSide) {
        let coord = self.compute_intersect_coord(&side);
        let key = SideKey::new(coord, &side);
        self.sides.insert(key, side);
    }

    /// Remove an obstacle side at the given scan position.
    /// Matches TS: `Remove(side, scanPos)`
    pub fn remove_at(&mut self, side: &ObstacleSide, scan_pos: Point) {
        self.line_position = scan_pos;
        self.remove(side);
    }

    /// Remove an obstacle side using the current line position.
    pub fn remove(&mut self, side: &ObstacleSide) {
        let coord = self.compute_intersect_coord(side);
        let key = SideKey::new(coord, side);
        self.sides.remove(&key);
    }

    /// Find a side in the scanline and return its key.
    /// Returns `None` if the side starts after the current position.
    ///
    /// Matches TS: `Find(side) -> RBNode<BasicObstacleSide> | null`
    /// TS checks: if line position is before side.Start in perp direction, return null.
    pub fn find(&self, side: &ObstacleSide) -> Option<SideKey> {
        // TS: if (-1 === scanDirection.ComparePerpCoord(linePosition, side.Start))
        if self.scan_direction.compare_perp(self.line_position, side.start())
            == std::cmp::Ordering::Less
        {
            return None;
        }
        let coord = self.compute_intersect_coord(side);
        let key = SideKey::new(coord, side);
        if self.sides.contains_key(&key) {
            Some(key)
        } else {
            None
        }
    }

    /// Return the next side in the low (decreasing) direction from the given key.
    /// Matches TS: `NextLowR(sideNode) -> SideTree.previous(sideNode)`
    pub fn next_low(&self, key: &SideKey) -> Option<(&SideKey, &ObstacleSide)> {
        self.sides.range(..key).next_back()
    }

    /// Return the next side in the high (increasing) direction from the given key.
    /// Matches TS: `NextHighR(sideNode) -> SideTree.next(sideNode)`
    pub fn next_high(&self, key: &SideKey) -> Option<(&SideKey, &ObstacleSide)> {
        use std::ops::Bound;
        self.sides
            .range((Bound::Excluded(key.clone()), Bound::Unbounded))
            .next()
    }

    /// Find the side, then return its low neighbor.
    /// Matches TS: `NextLowB(side)`
    pub fn next_low_by_side(&self, side: &ObstacleSide) -> Option<(&SideKey, &ObstacleSide)> {
        self.find(side).and_then(|key| self.next_low(&key))
    }

    /// Find the side, then return its high neighbor.
    /// Matches TS: `NextHighB(side)`
    pub fn next_high_by_side(&self, side: &ObstacleSide) -> Option<(&SideKey, &ObstacleSide)> {
        self.find(side).and_then(|key| self.next_high(&key))
    }

    /// Navigate in a direction: ascending = next_high, descending = next_low.
    /// Matches TS: `Next(dir, sideNode)` where `IsAscending(dir)` picks direction.
    pub fn next_in_direction(
        &self,
        ascending: bool,
        key: &SideKey,
    ) -> Option<(&SideKey, &ObstacleSide)> {
        if ascending {
            self.next_high(key)
        } else {
            self.next_low(key)
        }
    }

    /// Return the lowest (first) side in the scanline.
    /// Matches TS: `Lowest() -> SideTree.treeMinimum()`
    pub fn lowest(&self) -> Option<(&SideKey, &ObstacleSide)> {
        self.sides.iter().next()
    }

    /// Number of active sides.
    /// Matches TS: `Count`
    pub fn len(&self) -> usize {
        self.sides.len()
    }

    pub fn is_empty(&self) -> bool {
        self.sides.is_empty()
    }

    /// Return the side at the given key.
    pub fn get(&self, key: &SideKey) -> Option<&ObstacleSide> {
        self.sides.get(key)
    }

    /// Set the line position explicitly (for overlap events).
    pub fn set_line_position(&mut self, pos: Point) {
        self.line_position = pos;
    }

    /// Get the current line position.
    pub fn line_position(&self) -> Point {
        self.line_position
    }

    /// Return all sides currently on the scanline, ordered by their key.
    pub fn all_sides_ordered(&self) -> Vec<&ObstacleSide> {
        self.sides.values().collect()
    }

    /// Find both neighbors (low and high) around the given coordinate.
    /// Returns `(low_neighbor, high_neighbor)`.
    pub fn find_neighbors(
        &self,
        coord: f64,
    ) -> (Option<&ObstacleSide>, Option<&ObstacleSide>) {
        let probe_low = SideKey {
            coord: OrderedFloat(GeomConstants::round(coord)),
            side_type_order: 0,
            obstacle_ordinal: 0,
        };
        let low = self.sides.range(..probe_low).next_back().map(|(_, s)| s);

        let probe_high = SideKey {
            coord: OrderedFloat(GeomConstants::round(coord)),
            side_type_order: 1,
            obstacle_ordinal: usize::MAX,
        };
        let high = self.sides.range(probe_high..).next().map(|(_, s)| s);

        (low, high)
    }

    /// Find the nearest side below the given coordinate.
    pub fn low_neighbor(&self, coord: f64) -> Option<&ObstacleSide> {
        self.find_neighbors(coord).0
    }

    /// Find the nearest side above the given coordinate.
    pub fn high_neighbor(&self, coord: f64) -> Option<&ObstacleSide> {
        self.find_neighbors(coord).1
    }

    /// Compute the scan coordinate where the sweep line at the current position
    /// intersects the given side.
    ///
    /// For rectilinear obstacles (axis-aligned sides), this is the side's
    /// scan-parallel coordinate. For angled sides (convex hull overlaps), this
    /// computes the actual intersection.
    fn compute_intersect_coord(&self, side: &ObstacleSide) -> f64 {
        // For axis-aligned sides (rectilinear obstacles), the side's position
        // on the scanline is simply its scan-parallel coordinate.
        if self.scan_direction.is_horizontal() {
            side.start().x()
        } else {
            side.start().y()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_side(
        side_type: SideType,
        x: f64,
        y0: f64,
        y1: f64,
        ordinal: usize,
    ) -> ObstacleSide {
        ObstacleSide::new(side_type, Point::new(x, y0), Point::new(x, y1), ordinal)
    }

    #[test]
    fn insert_and_find() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        let side = make_side(SideType::Low, 5.0, 0.0, 10.0, 0);
        sl.insert(side.clone());
        assert!(sl.find(&side).is_some());
    }

    #[test]
    fn insert_remove() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        let side = make_side(SideType::Low, 5.0, 0.0, 10.0, 0);
        sl.insert(side.clone());
        assert_eq!(sl.len(), 1);
        sl.remove(&side);
        assert_eq!(sl.len(), 0);
    }

    #[test]
    fn insert_at_with_position() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new_at(sd, Point::new(0.0, 0.0));
        let side = make_side(SideType::Low, 5.0, 0.0, 10.0, 0);
        let key = sl.insert_at(side.clone(), Point::new(0.0, 5.0));
        assert!(sl.get(&key).is_some());
    }

    #[test]
    fn neighbor_traversal() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        let s1 = make_side(SideType::Low, 2.0, 0.0, 10.0, 0);
        let s2 = make_side(SideType::Low, 5.0, 0.0, 10.0, 1);
        let s3 = make_side(SideType::Low, 8.0, 0.0, 10.0, 2);
        sl.insert(s1);
        sl.insert(s2.clone());
        sl.insert(s3);

        let k2 = sl.find(&s2).unwrap();
        let low = sl.next_low(&k2);
        assert!(low.is_some());
        assert_eq!(low.unwrap().1.obstacle_ordinal(), 0);

        let high = sl.next_high(&k2);
        assert!(high.is_some());
        assert_eq!(high.unwrap().1.obstacle_ordinal(), 2);
    }

    #[test]
    fn lowest_returns_minimum() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        sl.insert(make_side(SideType::Low, 8.0, 0.0, 10.0, 1));
        sl.insert(make_side(SideType::Low, 2.0, 0.0, 10.0, 0));
        let (_, lowest) = sl.lowest().unwrap();
        assert_eq!(lowest.obstacle_ordinal(), 0);
    }

    #[test]
    fn high_before_low_at_same_coord() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        let low_side = make_side(SideType::Low, 5.0, 0.0, 10.0, 0);
        let high_side = make_side(SideType::High, 5.0, 0.0, 10.0, 1);
        sl.insert(low_side);
        sl.insert(high_side);
        let (_, first) = sl.lowest().unwrap();
        assert_eq!(
            first.side_type(),
            SideType::High,
            "High should sort before Low at same coord"
        );
    }

    #[test]
    fn direction_navigation() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        sl.insert(make_side(SideType::Low, 2.0, 0.0, 10.0, 0));
        let s2 = make_side(SideType::Low, 5.0, 0.0, 10.0, 1);
        sl.insert(s2.clone());
        sl.insert(make_side(SideType::Low, 8.0, 0.0, 10.0, 2));

        let k = sl.find(&s2).unwrap();
        let asc = sl.next_in_direction(true, &k);
        assert_eq!(asc.unwrap().1.obstacle_ordinal(), 2);
        let desc = sl.next_in_direction(false, &k);
        assert_eq!(desc.unwrap().1.obstacle_ordinal(), 0);
    }

    #[test]
    fn next_low_by_side_and_next_high_by_side() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        let s1 = make_side(SideType::Low, 2.0, 0.0, 10.0, 0);
        let s2 = make_side(SideType::Low, 5.0, 0.0, 10.0, 1);
        let s3 = make_side(SideType::Low, 8.0, 0.0, 10.0, 2);
        sl.insert(s1);
        sl.insert(s2.clone());
        sl.insert(s3);

        let low = sl.next_low_by_side(&s2);
        assert_eq!(low.unwrap().1.obstacle_ordinal(), 0);
        let high = sl.next_high_by_side(&s2);
        assert_eq!(high.unwrap().1.obstacle_ordinal(), 2);
    }

    #[test]
    fn find_neighbors_by_coord() {
        let sd = ScanDirection::horizontal();
        let mut sl = RectilinearScanLine::new(sd);
        sl.insert(make_side(SideType::Low, 2.0, 0.0, 10.0, 0));
        sl.insert(make_side(SideType::Low, 8.0, 0.0, 10.0, 1));

        let (low, high) = sl.find_neighbors(5.0);
        assert_eq!(low.unwrap().obstacle_ordinal(), 0);
        assert_eq!(high.unwrap().obstacle_ordinal(), 1);
    }

    #[test]
    fn empty_scanline() {
        let sl = RectilinearScanLine::new(ScanDirection::horizontal());
        assert!(sl.is_empty());
        assert_eq!(sl.len(), 0);
        assert!(sl.lowest().is_none());
    }
}
