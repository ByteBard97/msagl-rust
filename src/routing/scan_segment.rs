use super::scan_direction::ScanDirection;
use super::static_graph_utility::StaticGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use ordered_float::OrderedFloat;
use std::collections::BTreeMap;

/// Weight of a scan segment (affects routing preference).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SegmentWeight {
    Normal,     // Weight = 1 (preferred)
    Reflection, // Weight = 5 (less preferred)
    Overlapped, // Weight = 500 (avoid if possible)
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

    /// Whether this segment has any visibility vertices.
    /// Matches TS: `ScanSegment.HasVisibility()`
    pub fn has_visibility(&self) -> bool {
        self.lowest_vertex.is_some()
    }

    /// Called when the segment intersector begins processing this segment.
    /// Creates a vertex at the start point.
    ///
    /// Matches TS: `ScanSegment.OnSegmentIntersectorBegin(vg)`
    /// In the full TS, this handles group crossings and overlap vertices.
    /// We skip group crossings (deferred). We always create the start vertex
    /// to ensure connectivity even for segments without crossings; when the
    /// full VG generator (Task 8) is in place, this will be narrowed to the
    /// TS behavior (overlap-only creation).
    pub fn on_intersector_begin(&mut self, graph: &mut VisibilityGraph) {
        // TS: if (!this.AppendGroupCrossingsThroughPoint(vg, this.Start)) {
        //       this.LoadStartOverlapVertexIfNeeded(vg);
        //     }
        // Until the full VG generator produces a dense segment grid, we always
        // create the start vertex to maintain graph connectivity.
        let v = graph.add_vertex(self.start);
        self.append_visibility_vertex(graph, v);
    }

    /// Called when the segment intersector finishes this segment.
    /// Creates a vertex at the end point and connects it to the chain.
    ///
    /// Matches TS: `ScanSegment.OnSegmentIntersectorEnd(vg)`
    pub fn on_intersector_end(&mut self, graph: &mut VisibilityGraph) {
        // TS: this.AppendGroupCrossingsThroughPoint(vg, this.End)
        // (skipped — no groups)

        // TS checks HighestVisibilityVertex == null or IsPureLower before loading end.
        // We always create the end vertex for connectivity (see on_intersector_begin note).
        let should_load_end = match self.highest_vertex {
            None => true,
            Some(hv) => {
                let hvp = graph.point(hv);
                StaticGraphUtility::is_pure_lower(hvp, self.end)
            }
        };
        if should_load_end {
            let v = graph.add_vertex(self.end);
            self.append_visibility_vertex(graph, v);
        }
    }

    /// Append a visibility vertex to this segment's chain.
    /// Creates a unidirectional ascending edge from the current highest vertex to the new vertex.
    ///
    /// Matches C#: `ScanSegment.AppendVisibilityVertex` which calls `AddVisibilityEdge(source, target)`
    /// with the assertion `IsPureLower(source.Point, target.Point)` — edges always go low-to-high.
    /// The path search traverses both out-edges and in-edges, so unidirectional edges provide
    /// full connectivity.
    pub fn append_visibility_vertex(&mut self, graph: &mut VisibilityGraph, vertex: VertexId) {
        if let Some(highest) = self.highest_vertex {
            // C#: if (PointComparer.IsPureLower(newVertex.Point, HighestVisibilityVertex.Point))
            //       return;  // Already have a higher or equal vertex
            let new_point = graph.point(vertex);
            let high_point = graph.point(highest);
            if StaticGraphUtility::is_pure_lower(new_point, high_point) {
                return;
            }

            // Add a single ascending edge (low → high), matching C# AddVisibilityEdge.
            let points_equal = GeomConstants::close(high_point.x(), new_point.x())
                && GeomConstants::close(high_point.y(), new_point.y());
            if highest != vertex && !points_equal {
                let dist = (new_point - high_point).length() * self.weight.value() as f64;
                graph.add_edge(highest, vertex, dist);
            }
        }

        if self.lowest_vertex.is_none() {
            self.lowest_vertex = Some(vertex);
        }
        self.highest_vertex = Some(vertex);
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
            self.scan_direction.perp_coord(seg.start),
        ));
        self.segments.entry(key).or_default().push(seg);
    }

    /// Find a segment that contains the given point.
    pub fn find_containing_point(&self, p: Point) -> Option<&ScanSegment> {
        let key = OrderedFloat(GeomConstants::round(self.scan_direction.perp_coord(p)));
        let segs = self.segments.get(&key)?;
        let coord = self.scan_direction.coord(p);
        segs.iter().find(|s| {
            let s_low = self.scan_direction.coord(s.start);
            let s_high = self.scan_direction.coord(s.end);
            let (lo, hi) = if s_low <= s_high {
                (s_low, s_high)
            } else {
                (s_high, s_low)
            };
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
            self.scan_direction.perp_coord(seg.start),
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

    /// Merge overlapping or adjacent segments at each perpendicular coordinate.
    ///
    /// Faithful port of TS `ScanSegmentTree.MergeSegments()`.
    /// Two segments merge when they share the same weight and either:
    /// - They touch (one's high_coord == the other's low_coord)
    /// - They overlap (one's low_coord < the other's high_coord)
    /// - One is contained within the other
    pub fn merge_segments(&mut self) {
        for segs in self.segments.values_mut() {
            // Sort by low_coord so adjacent/overlapping segments are neighbours.
            segs.sort_by(|a, b| {
                a.low_coord()
                    .partial_cmp(&b.low_coord())
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let mut merged: Vec<ScanSegment> = Vec::with_capacity(segs.len());
            for seg in segs.drain(..) {
                if let Some(last) = merged.last_mut() {
                    // Check if segments touch or overlap
                    let overlaps_or_touches = seg.low_coord()
                        <= last.high_coord() + GeomConstants::DISTANCE_EPSILON;
                    if overlaps_or_touches && last.weight == seg.weight {
                        // Extend last segment to cover seg's range (take max of high_coords).
                        let new_high = seg.high_coord().max(last.high_coord());
                        if last.is_vertical {
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
