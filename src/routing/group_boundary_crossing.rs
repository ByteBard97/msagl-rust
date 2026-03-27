//! Group boundary crossing types for rectilinear edge routing.
//!
//! Faithful port of C# `GroupBoundaryCrossing.cs`, `PointAndCrossings.cs`,
//! `PointAndCrossingsList.cs`, and `GroupBoundaryCrossingMap.cs`.

use super::compass_direction::CompassDirection;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use std::collections::HashMap;

// ── GroupBoundaryCrossing ──────────────────────────────────────────────────

/// Width of a group boundary crossing edge.
///
/// Matches C# `GroupBoundaryCrossing.BoundaryWidth = ApproximateComparer.DistanceEpsilon`.
pub const BOUNDARY_WIDTH: f64 = GeomConstants::DISTANCE_EPSILON;

/// A record of crossing a group obstacle boundary.
///
/// Faithful port of C# `GroupBoundaryCrossing.cs`.
/// Fields:
/// - `obstacle_idx`: index into the obstacle array for the group obstacle crossed.
/// - `direction_to_inside`: compass direction pointing toward the interior of the group.
#[derive(Clone, Debug, PartialEq)]
pub struct GroupBoundaryCrossing {
    pub obstacle_idx: usize,
    pub direction_to_inside: CompassDirection,
}

impl GroupBoundaryCrossing {
    pub fn new(obstacle_idx: usize, direction_to_inside: CompassDirection) -> Self {
        Self { obstacle_idx, direction_to_inside }
    }

    /// Compute the interior vertex point just inside the group boundary.
    ///
    /// Matches C# `GroupBoundaryCrossing.GetInteriorVertexPoint(outerVertex)`:
    /// ```csharp
    /// return ApproximateComparer.Round(
    ///     outerVertex + CompassVector.ToPoint(DirectionToInside) * BoundaryWidth);
    /// ```
    pub fn get_interior_vertex_point(&self, outer_vertex: Point) -> Point {
        let delta = compass_direction_to_point(self.direction_to_inside) * BOUNDARY_WIDTH;
        Point::round(outer_vertex + delta)
    }
}

/// Convert a compass direction to a unit offset point.
///
/// Matches C# `CompassVector.ToPoint(Direction)`.
fn compass_direction_to_point(dir: CompassDirection) -> Point {
    match dir {
        CompassDirection::North => Point::new(0.0, 1.0),
        CompassDirection::South => Point::new(0.0, -1.0),
        CompassDirection::East => Point::new(1.0, 0.0),
        CompassDirection::West => Point::new(-1.0, 0.0),
    }
}

// ── PointAndCrossings ─────────────────────────────────────────────────────

/// A location and the group boundary crossings at that location.
///
/// Faithful port of C# `PointAndCrossings.cs`.
#[derive(Clone, Debug)]
pub struct PointAndCrossings {
    pub location: Point,
    pub crossings: Vec<GroupBoundaryCrossing>,
}

impl PointAndCrossings {
    pub fn new(location: Point, crossings: Vec<GroupBoundaryCrossing>) -> Self {
        Self { location, crossings }
    }
}

// ── PointAndCrossingsList ────────────────────────────────────────────────

/// An ordered list of `PointAndCrossings` with a cursor for sequential popping.
///
/// Faithful port of C# `PointAndCrossingsList.cs`.
///
/// C# fields:
/// - `items: List<PointAndCrossings>` — ordered by location.
/// - `index: int` — cursor (items before `index` have been popped).
#[derive(Clone, Debug, Default)]
pub struct PointAndCrossingsList {
    items: Vec<PointAndCrossings>,
    index: usize,
}

impl PointAndCrossingsList {
    /// Create an empty list.
    pub fn new() -> Self {
        Self { items: Vec::new(), index: 0 }
    }

    /// Return total number of items (not remaining from cursor).
    ///
    /// Matches C# `Count` property.
    pub fn count(&self) -> usize {
        self.items.len()
    }

    /// Reset the cursor to the beginning.
    ///
    /// Matches C# `Reset()`.
    pub fn reset(&mut self) {
        self.index = 0;
    }

    /// Pop the next item (advance cursor and return item at old cursor).
    ///
    /// Matches C# `Pop()` → `return items[index++]`.
    ///
    /// # Panics
    /// Panics if no items remain (cursor is at end).
    pub fn pop(&mut self) -> PointAndCrossings {
        let item = self.items[self.index].clone();
        self.index += 1;
        item
    }

    /// Return true if the current item's location is at or before `comparand`.
    ///
    /// Matches C# `CurrentIsBeforeOrAt(Point comparand)`:
    /// `index < Count && PointComparer.Compare(items[index].Location, comparand) <= 0`
    pub fn current_is_before_or_at(&self, comparand: Point) -> bool {
        self.index < self.items.len()
            && compare_points(self.items[self.index].location, comparand)
                != std::cmp::Ordering::Greater
    }

    /// Add a location + crossings pair to the list.
    ///
    /// Matches C# `Add(Point intersect, List<GroupBoundaryCrossing> crossings)`.
    pub fn add(&mut self, intersect: Point, crossings: Vec<GroupBoundaryCrossing>) {
        self.items.push(PointAndCrossings::new(intersect, crossings));
    }

    /// Sorted merge of two lists, deduplicating on equal location.
    ///
    /// Matches C# `MergeFrom(PointAndCrossingsList other)`.
    /// After merge, the cursor is reset to 0.
    pub fn merge_from(&mut self, other: &PointAndCrossingsList) {
        if other.items.is_empty() {
            return;
        }
        if self.items.is_empty() {
            self.items = other.items.clone();
            self.index = 0;
            return;
        }
        // Sorted merge, deduplicating on equal location.
        let mut merged: Vec<PointAndCrossings> = Vec::with_capacity(
            self.items.len() + other.items.len(),
        );
        let mut a = 0usize;
        let mut b = 0usize;
        while a < self.items.len() && b < other.items.len() {
            let ord = compare_points(self.items[a].location, other.items[b].location);
            match ord {
                std::cmp::Ordering::Less => {
                    merged.push(self.items[a].clone());
                    a += 1;
                }
                std::cmp::Ordering::Equal => {
                    // On equal location: keep self's entry (dedup).
                    merged.push(self.items[a].clone());
                    a += 1;
                    b += 1;
                }
                std::cmp::Ordering::Greater => {
                    merged.push(other.items[b].clone());
                    b += 1;
                }
            }
        }
        while a < self.items.len() {
            merged.push(self.items[a].clone());
            a += 1;
        }
        while b < other.items.len() {
            merged.push(other.items[b].clone());
            b += 1;
        }
        self.items = merged;
        self.index = 0;
    }

    /// Filter items to those where `start <= location <= end`, then reset cursor.
    ///
    /// Matches C# `Trim(Point start, Point end)`.
    /// If `start > end` (by compare_points), they are swapped first.
    pub fn trim(&mut self, start: Point, end: Point) {
        let (lo, hi) = if compare_points(start, end) != std::cmp::Ordering::Greater {
            (start, end)
        } else {
            (end, start)
        };
        self.items.retain(|pac| {
            compare_points(lo, pac.location) != std::cmp::Ordering::Greater
                && compare_points(pac.location, hi) != std::cmp::Ordering::Greater
        });
        self.index = 0;
    }

    /// Filter crossings by direction and return them as a Vec.
    ///
    /// Matches C# `static ToCrossingArray(List<GroupBoundaryCrossing> crossings, Direction dirToInside)`.
    /// Returns crossings whose `direction_to_inside == dir_to_inside`.
    /// An empty Vec corresponds to C# returning `null`.
    pub fn to_crossing_array(
        crossings: &[GroupBoundaryCrossing],
        dir_to_inside: CompassDirection,
    ) -> Vec<GroupBoundaryCrossing> {
        crossings
            .iter()
            .filter(|c| c.direction_to_inside == dir_to_inside)
            .cloned()
            .collect()
    }

    /// Return a reference to the first item, if any.
    pub fn first(&self) -> Option<&PointAndCrossings> {
        self.items.first()
    }

    /// Return a reference to the last item, if any.
    pub fn last(&self) -> Option<&PointAndCrossings> {
        self.items.last()
    }
}

// ── GroupBoundaryCrossingMap ──────────────────────────────────────────────

/// A map from intersection point to group boundary crossings at that point.
///
/// Faithful port of C# `GroupBoundaryCrossingMap.cs`.
///
/// C# implementation: `Dictionary<Point, List<GroupBoundaryCrossing>>`.
#[derive(Debug, Default)]
pub struct GroupBoundaryCrossingMap {
    map: HashMap<Point, Vec<GroupBoundaryCrossing>>,
}

impl GroupBoundaryCrossingMap {
    pub fn new() -> Self {
        Self { map: HashMap::new() }
    }

    /// Add an intersection to the map.
    ///
    /// Matches C# `AddIntersection(Point intersection, Obstacle groupObstacle, Direction dirToInside)`.
    /// Deduplicates by obstacle_idx: if the obstacle is already in the list for that point,
    /// it is not added again.
    pub fn add_intersection(
        &mut self,
        intersection: Point,
        obstacle_idx: usize,
        dir_to_inside: CompassDirection,
    ) {
        let entry = self.map.entry(intersection).or_default();
        // Dedup by obstacle_idx (matching C# dedup logic).
        if !entry.iter().any(|c| c.obstacle_idx == obstacle_idx) {
            entry.push(GroupBoundaryCrossing::new(obstacle_idx, dir_to_inside));
        }
    }

    /// Clear the map.
    ///
    /// Matches C# `Clear()`.
    pub fn clear(&mut self) {
        self.map.clear();
    }

    /// Collect all intersection points in [start, end] (inclusive), sort them,
    /// and build a `PointAndCrossingsList`.
    ///
    /// Matches C# `GetOrderedListBetween(Point start, Point end)`.
    /// If `start > end` (by compare_points), they are swapped first so the
    /// inclusive test is direction-independent.
    pub fn get_ordered_list_between(
        &self,
        start: Point,
        end: Point,
    ) -> Option<PointAndCrossingsList> {
        if self.map.is_empty() {
            return None;
        }

        let (lo, hi) = if compare_points(start, end) != std::cmp::Ordering::Greater {
            (start, end)
        } else {
            (end, start)
        };

        let mut items: Vec<PointAndCrossings> = self
            .map
            .iter()
            .filter(|(pt, _)| {
                compare_points(lo, **pt) != std::cmp::Ordering::Greater
                    && compare_points(**pt, hi) != std::cmp::Ordering::Greater
            })
            .map(|(pt, crossings)| PointAndCrossings::new(*pt, crossings.clone()))
            .collect();

        if items.is_empty() {
            return None;
        }

        items.sort_by(|a, b| compare_points(a.location, b.location));

        Some(PointAndCrossingsList { items, index: 0 })
    }
}

// ── Point comparison (X-primary, Y-secondary) ────────────────────────────

/// Compare two points: X-primary, Y-secondary.
///
/// Matches C# `PointComparer.Compare(Point a, Point b)`.
/// Uses `GeomConstants::compare` (epsilon-based).
pub fn compare_points(a: Point, b: Point) -> std::cmp::Ordering {
    let x_cmp = GeomConstants::compare(a.x(), b.x());
    if x_cmp != std::cmp::Ordering::Equal {
        return x_cmp;
    }
    GeomConstants::compare(a.y(), b.y())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boundary_width_matches_distance_epsilon() {
        assert!((BOUNDARY_WIDTH - 1e-6).abs() < 1e-12);
    }

    #[test]
    fn get_interior_vertex_point_north() {
        let crossing = GroupBoundaryCrossing::new(0, CompassDirection::North);
        let outer = Point::new(5.0, 10.0);
        let inner = crossing.get_interior_vertex_point(outer);
        // Should be outer + (0, BOUNDARY_WIDTH), rounded.
        assert!((inner.x() - 5.0).abs() < 1e-9);
        assert!((inner.y() - (10.0 + BOUNDARY_WIDTH)).abs() < 1e-9);
    }

    #[test]
    fn crossing_list_pop_and_current_is_before_or_at() {
        let mut list = PointAndCrossingsList::new();
        list.add(Point::new(1.0, 0.0), vec![]);
        list.add(Point::new(2.0, 0.0), vec![]);

        assert!(list.current_is_before_or_at(Point::new(1.0, 0.0)));
        let pac = list.pop();
        assert!((pac.location.x() - 1.0).abs() < 1e-9);
        assert!(list.current_is_before_or_at(Point::new(2.0, 0.0)));
        let _ = list.pop();
        assert!(!list.current_is_before_or_at(Point::new(3.0, 0.0)));
    }

    #[test]
    fn to_crossing_array_filters_by_direction() {
        let crossings = vec![
            GroupBoundaryCrossing::new(0, CompassDirection::North),
            GroupBoundaryCrossing::new(1, CompassDirection::South),
            GroupBoundaryCrossing::new(2, CompassDirection::North),
        ];
        let result =
            PointAndCrossingsList::to_crossing_array(&crossings, CompassDirection::North);
        assert_eq!(result.len(), 2);
        assert!(result.iter().all(|c| c.direction_to_inside == CompassDirection::North));

        let empty =
            PointAndCrossingsList::to_crossing_array(&crossings, CompassDirection::East);
        assert!(empty.is_empty());
    }

    #[test]
    fn crossing_map_add_and_get_ordered() {
        let mut map = GroupBoundaryCrossingMap::new();
        map.add_intersection(Point::new(3.0, 5.0), 0, CompassDirection::East);
        map.add_intersection(Point::new(1.0, 5.0), 1, CompassDirection::West);
        map.add_intersection(Point::new(2.0, 5.0), 2, CompassDirection::East);
        // Dedup: adding same obstacle_idx for same point is a no-op.
        map.add_intersection(Point::new(3.0, 5.0), 0, CompassDirection::East);

        let list = map
            .get_ordered_list_between(Point::new(0.0, 5.0), Point::new(4.0, 5.0))
            .expect("should have items");
        assert_eq!(list.count(), 3);
        assert!((list.items[0].location.x() - 1.0).abs() < 1e-9);
        assert!((list.items[1].location.x() - 2.0).abs() < 1e-9);
        assert!((list.items[2].location.x() - 3.0).abs() < 1e-9);
        // Dedup check: only one crossing at x=3.
        assert_eq!(list.items[2].crossings.len(), 1);
    }

    #[test]
    fn crossing_map_clear() {
        let mut map = GroupBoundaryCrossingMap::new();
        map.add_intersection(Point::new(1.0, 1.0), 0, CompassDirection::North);
        map.clear();
        assert!(map.map.is_empty());
        assert!(map
            .get_ordered_list_between(Point::new(0.0, 0.0), Point::new(2.0, 2.0))
            .is_none());
    }

    #[test]
    fn merge_from_sorted_merge() {
        let mut a = PointAndCrossingsList::new();
        a.add(Point::new(1.0, 0.0), vec![]);
        a.add(Point::new(3.0, 0.0), vec![]);

        let mut b = PointAndCrossingsList::new();
        b.add(Point::new(2.0, 0.0), vec![]);
        b.add(Point::new(4.0, 0.0), vec![]);

        a.merge_from(&b);
        assert_eq!(a.count(), 4);
        let xs: Vec<f64> = a.items.iter().map(|p| p.location.x()).collect();
        assert_eq!(xs, vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn trim_filters_inclusive() {
        let mut list = PointAndCrossingsList::new();
        for x in [0.0f64, 1.0, 2.0, 3.0, 4.0] {
            list.add(Point::new(x, 0.0), vec![]);
        }
        list.trim(Point::new(1.0, 0.0), Point::new(3.0, 0.0));
        assert_eq!(list.count(), 3);
        let xs: Vec<f64> = list.items.iter().map(|p| p.location.x()).collect();
        assert_eq!(xs, vec![1.0, 2.0, 3.0]);
    }
}
