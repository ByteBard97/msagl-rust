//! Scan segments and scan segment tree for rectilinear visibility generation.
//!
//! Faithful port of MSAGL TS `ScanSegment.ts` and `ScanSegmentTree.ts`.
//!
//! A ScanSegment is a single axis-aligned visibility segment created during
//! the sweep-line. The ScanSegmentTree stores them ordered by position,
//! supporting range queries and merge operations.

use ordered_float::OrderedFloat;
use std::cmp::Ordering;
use std::collections::BTreeMap;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VertexId;
use super::scan_direction::ScanDirection;

/// Weight of a scan segment (affects routing preference).
/// Matches TS: NormalWeight=1, ReflectionWeight=5, OverlappedWeight=500.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SegmentWeight {
    Normal,
    Reflection,
    Overlapped,
}

impl SegmentWeight {
    pub fn value(self) -> u32 {
        match self {
            Self::Normal => 1,
            Self::Reflection => 5,
            Self::Overlapped => 500,
        }
    }
}

/// A visibility segment created during sweep.
/// Matches TS `ScanSegment extends SegmentBase`.
#[derive(Clone, Debug)]
pub struct ScanSegment {
    pub start: Point,
    pub end: Point,
    pub weight: SegmentWeight,
    pub is_vertical: bool,
    /// Lowest visibility vertex along this segment.
    pub lowest_vertex: Option<VertexId>,
    /// Highest visibility vertex along this segment.
    pub highest_vertex: Option<VertexId>,
    /// Whether start needs an overlap boundary vertex. TS: `NeedStartOverlapVertex`
    pub need_start_overlap_vertex: bool,
    /// Whether end needs an overlap boundary vertex. TS: `NeedEndOverlapVertex`
    pub need_end_overlap_vertex: bool,
    /// Whether this segment requires a vertex at the overlap boundary (legacy).
    pub needs_overlap_vertex: bool,
}

impl ScanSegment {
    pub fn new(start: Point, end: Point, weight: SegmentWeight, is_vertical: bool) -> Self {
        Self {
            start,
            end,
            weight,
            is_vertical,
            lowest_vertex: None,
            highest_vertex: None,
            need_start_overlap_vertex: false,
            need_end_overlap_vertex: false,
            needs_overlap_vertex: false,
        }
    }

    /// Create a simple segment. Matches TS: `ScanSegment.mk(start, end)`.
    pub fn mk(start: Point, end: Point) -> Self {
        let is_vert = (start.x() - end.x()).abs() < GeomConstants::DISTANCE_EPSILON;
        Self::new(start, end, SegmentWeight::Normal, is_vert)
    }

    /// Update start and end points. Matches TS: `ScanSegment.Update(start, end)`.
    pub fn update(&mut self, start: Point, end: Point) {
        self.start = start;
        self.end = end;
    }

    pub fn contains_coord(&self, coord: f64) -> bool {
        coord >= self.low_coord() - GeomConstants::DISTANCE_EPSILON
            && coord <= self.high_coord() + GeomConstants::DISTANCE_EPSILON
    }

    /// Check if this segment intersects another. Matches TS: `IntersectsSegment`.
    pub fn intersects_segment(&self, other: &ScanSegment) -> bool {
        if self.is_vertical == other.is_vertical {
            GeomConstants::close(self.perp_coord(), other.perp_coord())
                && self.low_coord() <= other.high_coord() + GeomConstants::DISTANCE_EPSILON
                && other.low_coord() <= self.high_coord() + GeomConstants::DISTANCE_EPSILON
        } else {
            let (h, v) = if self.is_vertical { (other, self) } else { (self, other) };
            let vx = v.perp_coord();
            let hy = h.perp_coord();
            vx >= h.low_coord() - GeomConstants::DISTANCE_EPSILON
                && vx <= h.high_coord() + GeomConstants::DISTANCE_EPSILON
                && hy >= v.low_coord() - GeomConstants::DISTANCE_EPSILON
                && hy <= v.high_coord() + GeomConstants::DISTANCE_EPSILON
        }
    }

    pub fn is_reflection(&self) -> bool { self.weight == SegmentWeight::Reflection }
    pub fn is_overlapped(&self) -> bool { self.weight == SegmentWeight::Overlapped }

    pub fn perp_coord(&self) -> f64 {
        if self.is_vertical { self.start.x() } else { self.start.y() }
    }

    pub fn low_coord(&self) -> f64 {
        if self.is_vertical {
            self.start.y().min(self.end.y())
        } else {
            self.start.x().min(self.end.x())
        }
    }

    pub fn high_coord(&self) -> f64 {
        if self.is_vertical {
            self.start.y().max(self.end.y())
        } else {
            self.start.x().max(self.end.x())
        }
    }
}

// =========================================================================
// ScanSegmentTree — Faithful port of TS ScanSegmentTree.ts (311 lines)
// =========================================================================

/// Key for ordering segments in the BTreeMap.
///
/// Matches TS `ScanSegmentTree.Compare()`:
///   1. ScanDirection.Compare(first.Start, second.Start)
///   2. Longer segments first at same start (reversed end)
///   3. Unique ID tiebreaker
#[derive(Clone, Debug)]
struct SegKey {
    perp: OrderedFloat<f64>,
    scan_start: OrderedFloat<f64>,
    /// Negated scan_end for longest-first ordering at same start.
    neg_scan_end: OrderedFloat<f64>,
    id: u64,
}

impl PartialEq for SegKey {
    fn eq(&self, other: &Self) -> bool { self.cmp(other) == Ordering::Equal }
}
impl Eq for SegKey {}
impl PartialOrd for SegKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl Ord for SegKey {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.id == other.id { return Ordering::Equal; }
        self.perp.cmp(&other.perp)
            .then(self.scan_start.cmp(&other.scan_start))
            .then(self.neg_scan_end.cmp(&other.neg_scan_end))
            .then(self.id.cmp(&other.id))
    }
}

/// Collection of scan segments stored in a BTreeMap ordered by position.
///
/// Faithful port of TS `ScanSegmentTree` (311 lines).
pub struct ScanSegmentTree {
    scan_direction: ScanDirection,
    tree: BTreeMap<SegKey, ScanSegment>,
    next_id: u64,
}

impl ScanSegmentTree {
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self { scan_direction, tree: BTreeMap::new(), next_id: 0 }
    }

    pub fn scan_direction(&self) -> ScanDirection { self.scan_direction }

    /// Insert a segment. If an identical segment exists, return false.
    /// Matches TS: `InsertUnique(seg) -> RBNode<ScanSegment>`
    pub fn insert_unique(&mut self, seg: ScanSegment) -> bool {
        if self.find_key_exact(&seg).is_some() {
            return false;
        }
        let key = self.make_key(&seg);
        self.tree.insert(key, seg);
        true
    }

    /// Insert a segment unconditionally.
    pub fn insert(&mut self, seg: ScanSegment) {
        let key = self.make_key(&seg);
        self.tree.insert(key, seg);
    }

    /// Remove a segment from the tree. Matches TS: `Remove(seg)`.
    pub fn remove(&mut self, seg: &ScanSegment) {
        if let Some(key) = self.find_key_exact(seg) {
            self.tree.remove(&key);
        }
    }

    /// Find a segment with exact start and end match.
    /// Matches TS: `Find(start, end)`.
    pub fn find(&self, start: Point, end: Point) -> Option<&ScanSegment> {
        self.tree.values().find(|s| self.pts_match(s.start, start) && self.pts_match(s.end, end))
    }

    /// Find the lowest segment that intersects the perpendicular range [start, end].
    /// Matches TS: `FindLowestIntersector(start, end)`.
    pub fn find_lowest_intersector_range(
        &self, start: Point, end: Point,
    ) -> Option<&ScanSegment> {
        if GeomConstants::close(start.x(), end.x()) && GeomConstants::close(start.y(), end.y()) {
            // Point query
            let scan = self.scan_direction.coord(start);
            for seg in self.tree.values() {
                let s = self.scan_direction.coord(seg.start);
                let e = self.scan_direction.coord(seg.end);
                if s <= scan + GeomConstants::DISTANCE_EPSILON
                    && e >= scan - GeomConstants::DISTANCE_EPSILON
                {
                    return Some(seg);
                }
            }
            return None;
        }
        let lookup = ScanSegment::mk(start, end);
        for seg in self.tree.values() {
            if seg.intersects_segment(&lookup) { return Some(seg); }
            if self.scan_direction.compare(seg.start, end) == Ordering::Greater {
                return None;
            }
        }
        None
    }

    /// Find the highest segment that intersects the perpendicular range.
    /// Matches TS: `FindHighestIntersector(start, end)`.
    pub fn find_highest_intersector_range(
        &self, start: Point, end: Point,
    ) -> Option<&ScanSegment> {
        if GeomConstants::close(start.x(), end.x()) && GeomConstants::close(start.y(), end.y()) {
            let scan = self.scan_direction.coord(start);
            for seg in self.tree.values().rev() {
                let s = self.scan_direction.coord(seg.start);
                let e = self.scan_direction.coord(seg.end);
                if s <= scan + GeomConstants::DISTANCE_EPSILON
                    && e >= scan - GeomConstants::DISTANCE_EPSILON
                {
                    return Some(seg);
                }
            }
            return None;
        }
        let lookup = ScanSegment::mk(start, end);
        for seg in self.tree.values().rev() {
            if seg.intersects_segment(&lookup) { return Some(seg); }
            if self.scan_direction.compare(seg.end, start) == Ordering::Less {
                return None;
            }
        }
        None
    }

    /// Backward-compatible: find lowest by scan coordinate only.
    pub fn find_lowest_intersector(&self, scan_coord: f64) -> Option<&ScanSegment> {
        for seg in self.tree.values() {
            if seg.contains_coord(scan_coord) { return Some(seg); }
        }
        None
    }

    /// Backward-compatible: find highest by scan coordinate only.
    pub fn find_highest_intersector(&self, scan_coord: f64) -> Option<&ScanSegment> {
        for seg in self.tree.values().rev() {
            if seg.contains_coord(scan_coord) { return Some(seg); }
        }
        None
    }

    /// Find a segment containing the given point.
    /// Matches TS: `FindSegmentContainingPoint(location, allowUnfound)`.
    pub fn find_segment_containing_point(
        &self, location: Point, allow_unfound: bool,
    ) -> Option<&ScanSegment> {
        self.find_segment_overlapping_points(location, location, allow_unfound)
    }

    /// Find the first segment overlapping the range [start, end].
    /// Matches TS: `FindSegmentOverlappingPoints(start, end, allowUnfound)`.
    pub fn find_segment_overlapping_points(
        &self, start: Point, end: Point, allow_unfound: bool,
    ) -> Option<&ScanSegment> {
        let start_scan = self.scan_direction.coord(start);
        let end_scan = self.scan_direction.coord(end);

        for seg in self.tree.values() {
            let seg_end = self.scan_direction.coord(seg.end);
            if seg_end >= start_scan - GeomConstants::DISTANCE_EPSILON {
                let seg_start = self.scan_direction.coord(seg.start);
                if seg_start <= end_scan + GeomConstants::DISTANCE_EPSILON {
                    return Some(seg);
                }
            }
        }

        if !allow_unfound {
            debug_assert!(false, "Could not find expected segment");
        }
        None
    }

    /// Find a segment that contains the given point (backward-compatible).
    /// Checks both perpendicular coordinate match and scan range containment.
    pub fn find_containing_point(&self, p: Point) -> Option<&ScanSegment> {
        let perp = GeomConstants::round(self.scan_direction.perp_coord(p));
        let scan = self.scan_direction.coord(p);
        self.tree.values().find(|seg| {
            let seg_perp = GeomConstants::round(self.scan_direction.perp_coord(seg.start));
            if (seg_perp - perp).abs() > GeomConstants::DISTANCE_EPSILON {
                return false;
            }
            seg.contains_coord(scan)
        })
    }

    /// Merge adjacent touching segments with matching overlap status.
    ///
    /// Faithful port of TS `ScanSegmentTree.MergeSegments()` (lines 193-283).
    pub fn merge_segments(&mut self) {
        if self.tree.len() < 2 { return; }

        let segments: Vec<ScanSegment> = self.tree.values().cloned().collect();
        self.tree.clear();
        self.next_id = 0;

        let mut i = 0;
        while i < segments.len() {
            let mut current = segments[i].clone();
            let mut j = i + 1;

            while j < segments.len() {
                let next = &segments[j];
                let cmp = self.scan_direction.compare(next.start, current.end);

                match cmp {
                    Ordering::Greater => break,
                    Ordering::Equal => {
                        if next.weight == current.weight {
                            // Same overlap: merge
                            if self.scan_direction.compare(current.end, next.end) == Ordering::Less {
                                current.update(current.start, next.end);
                            }
                            j += 1;
                        } else {
                            // Different overlap: mark vertices, don't merge
                            current.need_end_overlap_vertex = true;
                            // Can't mutate segments[j] since we consumed into `segments`,
                            // so we insert current now, then handle next in next iteration.
                            break;
                        }
                    }
                    Ordering::Less => {
                        // Overlapping range — merge if same overlap status
                        if current.is_overlapped() == next.is_overlapped() {
                            if self.scan_direction.compare(current.end, next.end) == Ordering::Less {
                                current.update(current.start, next.end);
                            }
                        }
                        j += 1;
                    }
                }
            }

            let key = self.make_key(&current);
            self.tree.insert(key, current);
            i = j;
        }
    }

    pub fn len(&self) -> usize { self.tree.len() }
    pub fn is_empty(&self) -> bool { self.tree.is_empty() }

    /// Iterate over all segments in order. Matches TS: `get Segments()`.
    pub fn all_segments(&self) -> impl Iterator<Item = &ScanSegment> {
        self.tree.values()
    }

    /// Iterate over all segments mutably.
    pub fn all_segments_mut(&mut self) -> impl Iterator<Item = &mut ScanSegment> {
        self.tree.values_mut()
    }

    /// Get all segments at the given perpendicular coordinate.
    pub fn segments_at_coord(&self, perp_coord: f64) -> Vec<&ScanSegment> {
        let rounded = GeomConstants::round(perp_coord);
        self.tree.values()
            .filter(|seg| {
                let sp = GeomConstants::round(self.scan_direction.perp_coord(seg.start));
                (sp - rounded).abs() < GeomConstants::DISTANCE_EPSILON
            })
            .collect()
    }

    fn make_key(&mut self, seg: &ScanSegment) -> SegKey {
        let id = self.next_id;
        self.next_id += 1;
        SegKey {
            perp: OrderedFloat(GeomConstants::round(self.scan_direction.perp_coord(seg.start))),
            scan_start: OrderedFloat(GeomConstants::round(self.scan_direction.coord(seg.start))),
            neg_scan_end: OrderedFloat(GeomConstants::round(-self.scan_direction.coord(seg.end))),
            id,
        }
    }

    fn find_key_exact(&self, seg: &ScanSegment) -> Option<SegKey> {
        for (key, existing) in &self.tree {
            if self.pts_match(existing.start, seg.start)
                && self.pts_match(existing.end, seg.end)
                && existing.weight == seg.weight
            {
                return Some(key.clone());
            }
        }
        None
    }

    fn pts_match(&self, a: Point, b: Point) -> bool {
        GeomConstants::close(a.x(), b.x()) && GeomConstants::close(a.y(), b.y())
    }
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn h_seg(y: f64, x0: f64, x1: f64) -> ScanSegment {
        ScanSegment::new(Point::new(x0, y), Point::new(x1, y), SegmentWeight::Normal, false)
    }

    fn v_seg(x: f64, y0: f64, y1: f64) -> ScanSegment {
        ScanSegment::new(Point::new(x, y0), Point::new(x, y1), SegmentWeight::Normal, true)
    }

    #[test]
    fn insert_unique_deduplicates() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        assert!(tree.insert_unique(h_seg(5.0, 0.0, 10.0)));
        assert!(!tree.insert_unique(h_seg(5.0, 0.0, 10.0)));
        assert_eq!(tree.len(), 1);
    }

    #[test]
    fn insert_unique_different_segments() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 20.0, 30.0));
        assert_eq!(tree.len(), 2);
    }

    #[test]
    fn find_exact_match() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        assert!(tree.find(Point::new(0.0, 5.0), Point::new(10.0, 5.0)).is_some());
        assert!(tree.find(Point::new(0.0, 5.0), Point::new(15.0, 5.0)).is_none());
    }

    #[test]
    fn find_lowest_intersector_vertical_segs() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(v_seg(2.0, 0.0, 10.0));
        tree.insert_unique(v_seg(5.0, 0.0, 10.0));
        tree.insert_unique(v_seg(8.0, 3.0, 7.0));

        let result = tree.find_lowest_intersector_range(
            Point::new(1.0, 5.0), Point::new(9.0, 5.0),
        );
        assert!(result.is_some());
        assert!(GeomConstants::close(result.unwrap().start.x(), 2.0));
    }

    #[test]
    fn find_highest_intersector_vertical_segs() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(v_seg(2.0, 0.0, 10.0));
        tree.insert_unique(v_seg(5.0, 0.0, 10.0));
        tree.insert_unique(v_seg(8.0, 3.0, 7.0));

        let result = tree.find_highest_intersector_range(
            Point::new(1.0, 5.0), Point::new(9.0, 5.0),
        );
        assert!(result.is_some());
        assert!(GeomConstants::close(result.unwrap().start.x(), 8.0));
    }

    #[test]
    fn find_segment_containing_point() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 20.0, 30.0));

        assert!(tree.find_segment_containing_point(Point::new(5.0, 5.0), true).is_some());
        assert!(tree.find_segment_containing_point(Point::new(15.0, 5.0), true).is_none());
    }

    #[test]
    fn merge_touching_same_weight() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 10.0, 20.0));
        tree.merge_segments();
        assert_eq!(tree.len(), 1);
        let seg = tree.all_segments().next().unwrap();
        assert!(GeomConstants::close(seg.start.x().min(seg.end.x()), 0.0));
        assert!(GeomConstants::close(seg.start.x().max(seg.end.x()), 20.0));
    }

    #[test]
    fn merge_non_touching_unchanged() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 15.0, 25.0));
        tree.merge_segments();
        assert_eq!(tree.len(), 2);
    }

    #[test]
    fn merge_touching_different_overlap_marks_vertices() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(ScanSegment::new(
            Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false,
        ));
        tree.insert_unique(ScanSegment::new(
            Point::new(10.0, 5.0), Point::new(20.0, 5.0), SegmentWeight::Overlapped, false,
        ));
        tree.merge_segments();
        // Should remain 2 segments (different overlap)
        assert_eq!(tree.len(), 2);
        let segs: Vec<_> = tree.all_segments().collect();
        let has_end = segs.iter().any(|s| s.need_end_overlap_vertex);
        assert!(has_end, "Should have end overlap vertex marked");
    }

    #[test]
    fn segments_ordered_by_start() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 20.0, 30.0));
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 10.0, 20.0));
        let starts: Vec<f64> = tree.all_segments().map(|s| s.start.x()).collect();
        assert!(starts.windows(2).all(|w| w[0] <= w[1]),
            "Segments should be ordered: {:?}", starts);
    }

    #[test]
    fn longer_segments_first_at_same_start() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(h_seg(5.0, 0.0, 10.0));
        tree.insert(h_seg(5.0, 0.0, 20.0));
        let segs: Vec<_> = tree.all_segments().collect();
        assert_eq!(segs.len(), 2);
        assert!(segs[0].end.x() > segs[1].end.x(),
            "Longer segment should come first at same start");
    }

    #[test]
    fn remove_segment() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        let seg = h_seg(5.0, 0.0, 10.0);
        tree.insert_unique(seg.clone());
        tree.remove(&seg);
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn segments_at_coord_filters() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 20.0, 30.0));
        tree.insert_unique(h_seg(10.0, 0.0, 10.0));
        assert_eq!(tree.segments_at_coord(5.0).len(), 2);
        assert_eq!(tree.segments_at_coord(10.0).len(), 1);
    }

    #[test]
    fn backward_compat_find_lowest() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert_unique(h_seg(5.0, 0.0, 10.0));
        tree.insert_unique(h_seg(5.0, 20.0, 30.0));
        assert!(tree.find_lowest_intersector(5.0).is_some());
        assert!(tree.find_lowest_intersector(15.0).is_none());
    }

    #[test]
    fn intersects_segment_perpendicular() {
        let h = ScanSegment::new(
            Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false,
        );
        let v = ScanSegment::new(
            Point::new(5.0, 0.0), Point::new(5.0, 10.0), SegmentWeight::Normal, true,
        );
        assert!(h.intersects_segment(&v));
        assert!(v.intersects_segment(&h));

        let v_miss = ScanSegment::new(
            Point::new(15.0, 0.0), Point::new(15.0, 10.0), SegmentWeight::Normal, true,
        );
        assert!(!h.intersects_segment(&v_miss));
    }
}
