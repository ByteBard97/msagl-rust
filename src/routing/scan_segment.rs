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
}
