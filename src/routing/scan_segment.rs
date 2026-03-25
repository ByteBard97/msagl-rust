use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VertexId;
use super::scan_direction::ScanDirection;

/// Weight of a scan segment (affects routing preference).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SegmentWeight {
    Normal,      // Weight = 1 (preferred)
    Reflection,  // Weight = 5 (less preferred)
    Overlapped,  // Weight = 500 (avoid if possible)
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
#[derive(Clone, Debug)]
pub struct ScanSegment {
    pub start: Point,
    pub end: Point,
    pub weight: SegmentWeight,
    pub is_vertical: bool,
    /// Lowest vertex along this segment (set during graph construction).
    pub lowest_vertex: Option<VertexId>,
    /// Highest vertex along this segment.
    pub highest_vertex: Option<VertexId>,
    /// Whether this segment requires a vertex at the overlap boundary.
    pub needs_overlap_vertex: bool,
}

impl ScanSegment {
    /// Create a new segment with explicit direction flag and weight.
    pub fn new(start: Point, end: Point, weight: SegmentWeight, is_vertical: bool) -> Self {
        Self {
            start,
            end,
            weight,
            is_vertical,
            lowest_vertex: None,
            highest_vertex: None,
            needs_overlap_vertex: false,
        }
    }

    /// Check if a point's scan coordinate falls within this segment's range.
    pub fn contains_coord(&self, coord: f64) -> bool {
        coord >= self.low_coord() - GeomConstants::DISTANCE_EPSILON
            && coord <= self.high_coord() + GeomConstants::DISTANCE_EPSILON
    }

    /// Check if this segment is a reflection segment.
    pub fn is_reflection(&self) -> bool {
        self.weight == SegmentWeight::Reflection
    }

    /// Check if this segment is overlapped.
    pub fn is_overlapped(&self) -> bool {
        self.weight == SegmentWeight::Overlapped
    }

    /// Get the perpendicular coordinate (constant for axis-aligned segments).
    pub fn perp_coord(&self) -> f64 {
        if self.is_vertical {
            self.start.x()
        } else {
            self.start.y()
        }
    }

    /// Get the low (start) scan coordinate.
    pub fn low_coord(&self) -> f64 {
        if self.is_vertical {
            self.start.y().min(self.end.y())
        } else {
            self.start.x().min(self.end.x())
        }
    }

    /// Get the high (end) scan coordinate.
    pub fn high_coord(&self) -> f64 {
        if self.is_vertical {
            self.start.y().max(self.end.y())
        } else {
            self.start.x().max(self.end.x())
        }
    }
}

/// Collection of scan segments, stored sorted by perpendicular coordinate.
/// For horizontal segments, keyed by Y. For vertical, keyed by X.
pub struct ScanSegmentTree {
    scan_direction: ScanDirection,
    /// Maps perp_coord -> list of segments at that coordinate.
    segments: BTreeMap<OrderedFloat<f64>, Vec<ScanSegment>>,
}

impl ScanSegmentTree {
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self {
            scan_direction,
            segments: BTreeMap::new(),
        }
    }

    /// Insert a segment. Keyed by its perpendicular coordinate.
    pub fn insert(&mut self, seg: ScanSegment) {
        let key = OrderedFloat(GeomConstants::round(
            self.scan_direction.perp_coord(seg.start)
        ));
        self.segments.entry(key).or_default().push(seg);
    }

    /// Find a segment that contains the given point.
    pub fn find_containing_point(&self, p: Point) -> Option<&ScanSegment> {
        let key = OrderedFloat(GeomConstants::round(
            self.scan_direction.perp_coord(p)
        ));
        let segs = self.segments.get(&key)?;
        let coord = self.scan_direction.coord(p);
        segs.iter().find(|s| {
            let s_low = self.scan_direction.coord(s.start);
            let s_high = self.scan_direction.coord(s.end);
            let (lo, hi) = if s_low <= s_high { (s_low, s_high) } else { (s_high, s_low) };
            coord >= lo - GeomConstants::DISTANCE_EPSILON
                && coord <= hi + GeomConstants::DISTANCE_EPSILON
        })
    }

    /// Iterate over all segments.
    pub fn all_segments(&self) -> impl Iterator<Item = &ScanSegment> {
        self.segments.values().flat_map(|v| v.iter())
    }

    /// Number of segments.
    pub fn len(&self) -> usize {
        self.segments.values().map(|v| v.len()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Find the segment with the lowest perpendicular coordinate whose scan range
    /// contains the given scan coordinate value.
    pub fn find_lowest_intersector(&self, scan_coord: f64) -> Option<&ScanSegment> {
        for segs in self.segments.values() {
            for seg in segs {
                if seg.contains_coord(scan_coord) {
                    return Some(seg);
                }
            }
        }
        None
    }

    /// Find the segment with the highest perpendicular coordinate whose scan range
    /// contains the given scan coordinate value.
    pub fn find_highest_intersector(&self, scan_coord: f64) -> Option<&ScanSegment> {
        for segs in self.segments.values().rev() {
            for seg in segs.iter().rev() {
                if seg.contains_coord(scan_coord) {
                    return Some(seg);
                }
            }
        }
        None
    }

    /// Insert a segment only if no segment with the same start, end, and weight already exists.
    /// Returns `true` if the segment was inserted, `false` if a duplicate was found.
    pub fn insert_unique(&mut self, seg: ScanSegment) -> bool {
        let key = OrderedFloat(GeomConstants::round(
            self.scan_direction.perp_coord(seg.start)
        ));
        let entry = self.segments.entry(key).or_default();
        let is_duplicate = entry.iter().any(|existing| {
            existing.weight == seg.weight
                && (existing.start.x() - seg.start.x()).abs() < GeomConstants::DISTANCE_EPSILON
                && (existing.start.y() - seg.start.y()).abs() < GeomConstants::DISTANCE_EPSILON
                && (existing.end.x() - seg.end.x()).abs() < GeomConstants::DISTANCE_EPSILON
                && (existing.end.y() - seg.end.y()).abs() < GeomConstants::DISTANCE_EPSILON
        });
        if is_duplicate {
            false
        } else {
            entry.push(seg);
            true
        }
    }

    /// Get all segments at the given perpendicular coordinate.
    pub fn segments_at_coord(&self, perp_coord: f64) -> &[ScanSegment] {
        let key = OrderedFloat(GeomConstants::round(perp_coord));
        self.segments.get(&key).map(Vec::as_slice).unwrap_or(&[])
    }

    /// Merge adjacent touching segments at each perpendicular coordinate that share the same weight.
    /// Two segments merge when one's high_coord equals the other's low_coord and weights match.
    pub fn merge_segments(&mut self) {
        for segs in self.segments.values_mut() {
            // Sort by low_coord so adjacent segments are neighbours in the slice.
            segs.sort_by(|a, b| {
                a.low_coord()
                    .partial_cmp(&b.low_coord())
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let mut merged: Vec<ScanSegment> = Vec::with_capacity(segs.len());
            for seg in segs.drain(..) {
                if let Some(last) = merged.last_mut() {
                    let touching = (last.high_coord() - seg.low_coord()).abs()
                        < GeomConstants::DISTANCE_EPSILON;
                    if touching && last.weight == seg.weight {
                        // Extend last segment to cover seg's range.
                        let new_high = seg.high_coord();
                        if last.is_vertical {
                            // Keep perp (x) the same; update the y extent.
                            let px = last.start.x();
                            let low_y = last.low_coord();
                            last.start = Point::new(px, low_y);
                            last.end = Point::new(px, new_high);
                        } else {
                            let py = last.start.y();
                            let low_x = last.low_coord();
                            last.start = Point::new(low_x, py);
                            last.end = Point::new(new_high, py);
                        }
                        continue;
                    }
                }
                merged.push(seg);
            }
            *segs = merged;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_weight_ordering() {
        assert!(SegmentWeight::Normal < SegmentWeight::Reflection);
        assert!(SegmentWeight::Reflection < SegmentWeight::Overlapped);
    }

    #[test]
    fn segment_weight_values() {
        assert_eq!(SegmentWeight::Normal.value(), 1);
        assert_eq!(SegmentWeight::Reflection.value(), 5);
        assert_eq!(SegmentWeight::Overlapped.value(), 500);
    }

    #[test]
    fn segment_perp_coord_vertical() {
        let seg = ScanSegment::new(
            Point::new(10.0, 0.0), Point::new(10.0, 100.0),
            SegmentWeight::Normal, true
        );
        assert!((seg.perp_coord() - 10.0).abs() < 1e-10);
        assert!((seg.low_coord() - 0.0).abs() < 1e-10);
        assert!((seg.high_coord() - 100.0).abs() < 1e-10);
    }

    #[test]
    fn segment_perp_coord_horizontal() {
        let seg = ScanSegment::new(
            Point::new(0.0, 50.0), Point::new(200.0, 50.0),
            SegmentWeight::Normal, false
        );
        assert!((seg.perp_coord() - 50.0).abs() < 1e-10);
        assert!((seg.low_coord() - 0.0).abs() < 1e-10);
        assert!((seg.high_coord() - 200.0).abs() < 1e-10);
    }

    #[test]
    fn contains_coord_within_range() {
        let seg = ScanSegment::new(
            Point::new(10.0, 0.0), Point::new(10.0, 100.0),
            SegmentWeight::Normal, true
        );
        assert!(seg.contains_coord(50.0));
        assert!(seg.contains_coord(0.0));
        assert!(seg.contains_coord(100.0));
        assert!(!seg.contains_coord(-10.0));
        assert!(!seg.contains_coord(110.0));
    }

    #[test]
    fn reflection_and_overlapped_checks() {
        let normal = ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Normal, false);
        let refl = ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Reflection, false);
        let over = ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Overlapped, false);
        assert!(!normal.is_reflection());
        assert!(!normal.is_overlapped());
        assert!(refl.is_reflection());
        assert!(over.is_overlapped());
    }

    // -------------------------------------------------------------------------
    // ScanSegmentTree — new method tests
    // -------------------------------------------------------------------------

    #[test]
    fn find_lowest_intersector_returns_first_crossing() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(0.0, 20.0), Point::new(50.0, 20.0),
            SegmentWeight::Normal, false,
        ));
        // scan_coord 25 is within [0, 50] for both; lowest perp coord is y=10.
        let lowest = tree.find_lowest_intersector(25.0);
        assert!(lowest.is_some());
        assert!((lowest.unwrap().perp_coord() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn find_highest_intersector_returns_last_crossing() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(0.0, 20.0), Point::new(50.0, 20.0),
            SegmentWeight::Normal, false,
        ));
        let highest = tree.find_highest_intersector(25.0);
        assert!(highest.is_some());
        assert!((highest.unwrap().perp_coord() - 20.0).abs() < 1e-10);
    }

    #[test]
    fn find_intersector_returns_none_when_no_crossing() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        assert!(tree.find_lowest_intersector(60.0).is_none());
        assert!(tree.find_highest_intersector(60.0).is_none());
    }

    #[test]
    fn insert_unique_rejects_duplicate() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        let seg = ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        );
        assert!(tree.insert_unique(seg.clone()));
        assert!(!tree.insert_unique(seg));
        assert_eq!(tree.len(), 1);
    }

    #[test]
    fn insert_unique_accepts_different_weight() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        let seg_a = ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        );
        let seg_b = ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Reflection, false,
        );
        assert!(tree.insert_unique(seg_a));
        assert!(tree.insert_unique(seg_b));
        assert_eq!(tree.len(), 2);
    }

    #[test]
    fn segments_at_coord_returns_correct_slice() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(60.0, 10.0), Point::new(100.0, 10.0),
            SegmentWeight::Reflection, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(0.0, 20.0), Point::new(50.0, 20.0),
            SegmentWeight::Normal, false,
        ));
        assert_eq!(tree.segments_at_coord(10.0).len(), 2);
        assert_eq!(tree.segments_at_coord(20.0).len(), 1);
        assert_eq!(tree.segments_at_coord(30.0).len(), 0);
    }

    #[test]
    fn merge_adjacent_segments() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(50.0, 10.0), Point::new(100.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        assert_eq!(tree.len(), 2);
        tree.merge_segments();
        assert_eq!(tree.len(), 1);
        let seg = tree.all_segments().next().unwrap();
        assert!((seg.low_coord() - 0.0).abs() < 1e-10);
        assert!((seg.high_coord() - 100.0).abs() < 1e-10);
    }

    #[test]
    fn merge_does_not_merge_different_weights() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(50.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(50.0, 10.0), Point::new(100.0, 10.0),
            SegmentWeight::Reflection, false,
        ));
        tree.merge_segments();
        assert_eq!(tree.len(), 2);
    }

    #[test]
    fn merge_three_adjacent_same_weight() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 5.0), Point::new(30.0, 5.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(30.0, 5.0), Point::new(60.0, 5.0),
            SegmentWeight::Normal, false,
        ));
        tree.insert(ScanSegment::new(
            Point::new(60.0, 5.0), Point::new(90.0, 5.0),
            SegmentWeight::Normal, false,
        ));
        tree.merge_segments();
        assert_eq!(tree.len(), 1);
        let seg = tree.all_segments().next().unwrap();
        assert!((seg.low_coord() - 0.0).abs() < 1e-10);
        assert!((seg.high_coord() - 90.0).abs() < 1e-10);
    }

    #[test]
    fn merge_non_adjacent_segments_stay_separate() {
        let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
        tree.insert(ScanSegment::new(
            Point::new(0.0, 10.0), Point::new(40.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        // Gap: [40, 60] is empty.
        tree.insert(ScanSegment::new(
            Point::new(60.0, 10.0), Point::new(100.0, 10.0),
            SegmentWeight::Normal, false,
        ));
        tree.merge_segments();
        assert_eq!(tree.len(), 2);
    }
}
