use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VertexId;
use super::scan_direction::ScanDirection;

/// Weight of a scan segment (affects routing preference).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SegmentWeight {
    Normal,      // Weight = 1
    Reflection,  // Weight = 5 (less preferred)
}

/// A visibility segment created during sweep.
#[derive(Clone, Debug)]
pub struct ScanSegment {
    pub start: Point,
    pub end: Point,
    pub weight: SegmentWeight,
    /// Lowest vertex along this segment (set during graph construction).
    pub lowest_vertex: Option<VertexId>,
    /// Highest vertex along this segment.
    pub highest_vertex: Option<VertexId>,
}

impl ScanSegment {
    pub fn new(start: Point, end: Point) -> Self {
        Self {
            start,
            end,
            weight: SegmentWeight::Normal,
            lowest_vertex: None,
            highest_vertex: None,
        }
    }

    pub fn with_weight(start: Point, end: Point, weight: SegmentWeight) -> Self {
        Self {
            start,
            end,
            weight,
            lowest_vertex: None,
            highest_vertex: None,
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
