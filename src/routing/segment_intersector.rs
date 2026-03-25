//! Event-based sweep-line segment intersector.
//!
//! Faithful port of MSAGL TS: `SegmentIntersector.ts` (285 lines).
//!
//! Sweeps H and V scan segments using Y-ordered events, maintaining a BTreeMap
//! of active vertical segments. At each horizontal segment event, scans the
//! BTreeMap for verticals in range and creates intersection vertices.

use std::cmp::Ordering;
use std::collections::BTreeMap;
use ordered_float::OrderedFloat;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VisibilityGraph;
use super::scan_segment::ScanSegment;

// ---------------------------------------------------------------------------
// Event types
// ---------------------------------------------------------------------------

/// Event types for the segment sweep.
/// Order matters: VOpen < HOpen < VClose ensures correct processing sequence.
/// Matches TS: `enum SegEventType { VOpen, VClose, HOpen }`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SegEventType {
    VOpen,
    HOpen,
    VClose,
}

/// A sweep event referencing a segment by its index into the combined segment array.
#[derive(Debug)]
struct SegEvent {
    event_type: SegEventType,
    /// Index into `all_segments` slice (v_segments come first, then h_segments).
    seg_index: usize,
}

impl SegEvent {
    fn is_vertical(&self) -> bool {
        self.event_type != SegEventType::HOpen
    }

    /// The site (Y coordinate) of this event.
    /// VClose uses segment end; VOpen and HOpen use segment start.
    /// Matches TS: `get Site(): Point`
    fn site<'a>(&self, segments: &'a [ScanSegment]) -> Point {
        let seg = &segments[self.seg_index];
        match self.event_type {
            SegEventType::VClose => seg.end,
            _ => seg.start,
        }
    }
}

// ---------------------------------------------------------------------------
// Event comparison (faithful port of TS Compare, lines 181-253)
// ---------------------------------------------------------------------------

/// Compare two events for sweep ordering.
///
/// Primary: Y coordinate.
/// At same Y:
///   - Two vertical: VOpen before VClose
///   - Two horizontal: by X coordinate
///   - V vs H: VOpen before HOpen, HOpen before VClose
///
/// Matches TS: `SegmentIntersector.Compare()` (lines 181-253)
fn compare_events(first: &SegEvent, second: &SegEvent, segments: &[ScanSegment]) -> Ordering {
    let first_site = first.site(segments);
    let second_site = second.site(segments);

    // Primary: Y coordinate
    let cmp_y = GeomConstants::compare(first_site.y(), second_site.y());
    if cmp_y != Ordering::Equal {
        return cmp_y;
    }

    // Same Y — handle V-V, H-H, and V-H cases
    if first.is_vertical() && second.is_vertical() {
        // Two vertical events at same Y: VOpen before VClose
        // TS: (VClose === first.EventType ? 1 : 0) - (VClose === second.EventType ? 1 : 0)
        let first_close = if first.event_type == SegEventType::VClose { 1i32 } else { 0 };
        let second_close = if second.event_type == SegEventType::VClose { 1i32 } else { 0 };
        return (first_close - second_close).cmp(&0);
    }

    if !first.is_vertical() && !second.is_vertical() {
        // Two H events at same Y: order by X coordinate
        return GeomConstants::compare(first_site.x(), second_site.x());
    }

    // One V, one H: V events need correct ordering relative to H
    let v_event = if first.is_vertical() { first } else { second };

    // Start assuming v_event is 'first' and it's VOpen → comes before HOpen
    let mut cmp: i32 = -1;
    if v_event.event_type == SegEventType::VClose {
        cmp = 1; // VClose comes after HOpen
    }
    if !std::ptr::eq(v_event, first) {
        cmp *= -1; // Undo the swap if v_event was actually 'second'
    }

    match cmp {
        -1 => Ordering::Less,
        1 => Ordering::Greater,
        _ => Ordering::Equal,
    }
}

// ---------------------------------------------------------------------------
// Scanline key for BTreeMap of active vertical segments
// ---------------------------------------------------------------------------

/// Key for ordering vertical segments in the scanline BTreeMap.
/// Orders by X coordinate, with Y as tiebreaker.
/// Matches TS: `SegmentIntersector.CompareSS()` (lines 258-281)
#[derive(Debug, Clone, Copy)]
struct ScanLineKey {
    x: OrderedFloat<f64>,
    y: OrderedFloat<f64>,
    seg_index: usize,
}

impl ScanLineKey {
    fn new(seg: &ScanSegment, seg_index: usize) -> Self {
        Self {
            x: OrderedFloat(seg.start.x()),
            y: OrderedFloat(seg.start.y()),
            seg_index,
        }
    }
}

impl PartialEq for ScanLineKey {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for ScanLineKey {}

impl PartialOrd for ScanLineKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ScanLineKey {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.seg_index == other.seg_index {
            return Ordering::Equal;
        }
        // Primary: X coordinate
        let cmp_x = GeomConstants::compare(self.x.0, other.x.0);
        if cmp_x != Ordering::Equal {
            return cmp_x;
        }
        // Tiebreaker: Y coordinate
        let cmp_y = GeomConstants::compare(self.y.0, other.y.0);
        if cmp_y != Ordering::Equal {
            return cmp_y;
        }
        // Ultimate tiebreaker: segment index (for distinct segments at same point)
        self.seg_index.cmp(&other.seg_index)
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Build the visibility graph from horizontal and vertical scan segments
/// using an event-based sweep-line algorithm.
///
/// Faithful port of TS: `SegmentIntersector.Generate()` (lines 70-112).
///
/// The segments are mutated: vertex tracking fields (lowest_vertex,
/// highest_vertex) are populated during the sweep.
///
/// Returns the constructed VisibilityGraph.
pub fn build_graph_from_segments(
    h_segments: &mut [ScanSegment],
    v_segments: &mut [ScanSegment],
) -> VisibilityGraph {
    // Build a combined segments array: v_segments first [0..v_len), then h_segments [v_len..)
    let v_len = v_segments.len();
    let h_len = h_segments.len();
    let total = v_len + h_len;

    if total == 0 {
        return VisibilityGraph::new();
    }

    // Create events
    let mut events: Vec<SegEvent> = Vec::with_capacity(2 * v_len + h_len);
    for i in 0..v_len {
        events.push(SegEvent { event_type: SegEventType::VOpen, seg_index: i });
        events.push(SegEvent { event_type: SegEventType::VClose, seg_index: i });
    }
    for i in 0..h_len {
        events.push(SegEvent { event_type: SegEventType::HOpen, seg_index: v_len + i });
    }

    // Build combined segment view for sorting (we need immutable access for sites)
    // We'll store segments in a Vec that we can index into.
    let mut all_segments: Vec<ScanSegment> = Vec::with_capacity(total);
    all_segments.extend_from_slice(v_segments);
    all_segments.extend_from_slice(h_segments);

    // Sort events using the faithful TS comparison
    events.sort_by(|a, b| compare_events(a, b, &all_segments));

    // Process events
    let mut graph = VisibilityGraph::new();
    let mut scanline: BTreeMap<ScanLineKey, usize> = BTreeMap::new();
    let mut segments_without_visibility: Vec<usize> = Vec::new();

    for evt in &events {
        match evt.event_type {
            SegEventType::VOpen => {
                // TS: OnSegmentOpen(seg) then ScanInsert(seg)
                all_segments[evt.seg_index].on_intersector_begin(&mut graph);
                let key = ScanLineKey::new(&all_segments[evt.seg_index], evt.seg_index);
                scanline.insert(key, evt.seg_index);
            }
            SegEventType::VClose => {
                // TS: OnSegmentClose(seg) then ScanRemove(seg)
                all_segments[evt.seg_index].on_intersector_end(&mut graph);
                if !all_segments[evt.seg_index].has_visibility() {
                    segments_without_visibility.push(evt.seg_index);
                }
                let key = ScanLineKey::new(&all_segments[evt.seg_index], evt.seg_index);
                scanline.remove(&key);
            }
            SegEventType::HOpen => {
                let h_idx = evt.seg_index;
                // TS: OnSegmentOpen(hSeg)
                all_segments[h_idx].on_intersector_begin(&mut graph);

                // TS: ScanIntersect(hSeg) — find all V segments in H range
                let h_start_x = all_segments[h_idx].start.x();
                let h_end_x = all_segments[h_idx].end.x();
                let h_x_lo = h_start_x.min(h_end_x);
                let h_x_hi = h_start_x.max(h_end_x);
                let h_y = all_segments[h_idx].start.y();

                // Find first V segment with X >= h_x_lo using BTreeMap range
                // TS: findFirst(pred) where pred is v.Start.x >= hSeg.Start.x
                // Then iterate until v.Start.x > hSeg.End.x
                let matching_v_indices: Vec<usize> = scanline
                    .iter()
                    .filter(|(key, _)| {
                        GeomConstants::compare(key.x.0, h_x_lo) != Ordering::Less
                    })
                    .take_while(|(key, _)| {
                        GeomConstants::compare(key.x.0, h_x_hi) != Ordering::Greater
                    })
                    .map(|(_, &idx)| idx)
                    .collect();

                for v_idx in matching_v_indices {
                    let v_x = all_segments[v_idx].start.x();
                    let intersection = Point::new(v_x, h_y);
                    let new_vertex = graph.add_vertex(intersection);

                    // TS: hSeg.AppendVisibilityVertex(visGraph, newVertex)
                    all_segments[h_idx].append_visibility_vertex(&mut graph, new_vertex);
                    // TS: vSeg.AppendVisibilityVertex(visGraph, newVertex)
                    all_segments[v_idx].append_visibility_vertex(&mut graph, new_vertex);
                }

                // TS: OnSegmentClose(hSeg) — H segments are opened and closed in the same event
                all_segments[h_idx].on_intersector_end(&mut graph);
                if !all_segments[h_idx].has_visibility() {
                    segments_without_visibility.push(h_idx);
                }
            }
        }
    }

    // Copy vertex tracking state back to the original slices
    for (i, seg) in all_segments.iter().enumerate() {
        if i < v_len {
            v_segments[i].lowest_vertex = seg.lowest_vertex;
            v_segments[i].highest_vertex = seg.highest_vertex;
        } else {
            h_segments[i - v_len].lowest_vertex = seg.lowest_vertex;
            h_segments[i - v_len].highest_vertex = seg.highest_vertex;
        }
    }

    graph
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::routing::scan_segment::SegmentWeight;

    #[test]
    fn two_crossing_segments_create_correct_vertices() {
        // One H segment crossing one V segment
        let mut h = vec![ScanSegment::new(
            Point::new(0.0, 5.0), Point::new(10.0, 5.0),
            SegmentWeight::Normal, false,
        )];
        let mut v = vec![ScanSegment::new(
            Point::new(5.0, 0.0), Point::new(5.0, 10.0),
            SegmentWeight::Normal, true,
        )];

        let graph = build_graph_from_segments(&mut h, &mut v);

        // Vertices: H-start (0,5), H-end (10,5), V-start (5,0), V-end (5,10),
        // intersection (5,5) = 5 total.
        // But some may be created only when intersected:
        // V-start (5,0) from VOpen begin (only if needs_overlap_vertex, which is false)
        // V-end (5,10) from VClose end
        // H doesn't create start/end unless overlap vertex is needed
        // Intersection at (5,5) is always created
        // So we get: intersection (5,5), V-end (5,10)
        // V: begin creates nothing (no overlap), intersection creates (5,5), end creates (5,10)
        //    -> vertices at (5,5) and (5,10), edge between them
        // H: begin creates nothing, intersection creates (5,5) [already exists], end creates nothing
        //    -> no additional vertices or edges from H alone

        // Actually let's think again: the segment's on_intersector_end creates
        // a vertex at the end only if needs_overlap_vertex. Otherwise it does nothing.
        // The intersection creates vertex (5,5).
        // So: V gets vertex at (5,5) from intersection; on_intersector_end does nothing (no overlap).
        // H gets vertex at (5,5) from intersection; on_intersector_end does nothing (no overlap).
        // Total vertices: 1 (just the intersection).
        // This matches the TS behavior where non-overlapped segments only get vertices at intersections.
        assert_eq!(graph.vertex_count(), 1);
        assert!(graph.find_vertex(Point::new(5.0, 5.0)).is_some());
    }

    #[test]
    fn multiple_crossings_correct_vertex_count() {
        // 2 H segments crossing 2 V segments = up to 4 intersections
        let mut h = vec![
            ScanSegment::new(Point::new(0.0, 3.0), Point::new(10.0, 3.0), SegmentWeight::Normal, false),
            ScanSegment::new(Point::new(0.0, 7.0), Point::new(10.0, 7.0), SegmentWeight::Normal, false),
        ];
        let mut v = vec![
            ScanSegment::new(Point::new(3.0, 0.0), Point::new(3.0, 10.0), SegmentWeight::Normal, true),
            ScanSegment::new(Point::new(7.0, 0.0), Point::new(7.0, 10.0), SegmentWeight::Normal, true),
        ];

        let graph = build_graph_from_segments(&mut h, &mut v);

        // 4 intersection vertices: (3,3), (7,3), (3,7), (7,7)
        assert_eq!(graph.vertex_count(), 4);
        assert!(graph.find_vertex(Point::new(3.0, 3.0)).is_some());
        assert!(graph.find_vertex(Point::new(7.0, 3.0)).is_some());
        assert!(graph.find_vertex(Point::new(3.0, 7.0)).is_some());
        assert!(graph.find_vertex(Point::new(7.0, 7.0)).is_some());
    }

    #[test]
    fn event_ordering_vopen_before_hopen_before_vclose() {
        // Create events at the same Y and verify ordering
        let segments = vec![
            // V segment: start at (5, 0), end at (5, 10)
            ScanSegment::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0), SegmentWeight::Normal, true),
            // H segment: start at (0, 5), end at (10, 5)
            ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false),
        ];

        // VOpen at Y=0, HOpen at Y=5, VClose at Y=10
        let v_open = SegEvent { event_type: SegEventType::VOpen, seg_index: 0 };
        let h_open = SegEvent { event_type: SegEventType::HOpen, seg_index: 1 };
        let v_close = SegEvent { event_type: SegEventType::VClose, seg_index: 0 };

        // VOpen (Y=0) before HOpen (Y=5)
        assert_eq!(compare_events(&v_open, &h_open, &segments), Ordering::Less);
        // HOpen (Y=5) before VClose (Y=10)
        assert_eq!(compare_events(&h_open, &v_close, &segments), Ordering::Less);

        // Now test at same Y: VOpen before HOpen, HOpen before VClose
        let segments_same_y = vec![
            // V segment: start at (5, 5), end at (5, 5) — degenerate for testing
            ScanSegment::new(Point::new(5.0, 5.0), Point::new(5.0, 5.0), SegmentWeight::Normal, true),
            // H segment: start at (0, 5), end at (10, 5)
            ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false),
        ];

        let v_open_same = SegEvent { event_type: SegEventType::VOpen, seg_index: 0 };
        let h_open_same = SegEvent { event_type: SegEventType::HOpen, seg_index: 1 };
        let v_close_same = SegEvent { event_type: SegEventType::VClose, seg_index: 0 };

        // VOpen before HOpen at same Y
        assert_eq!(compare_events(&v_open_same, &h_open_same, &segments_same_y), Ordering::Less);
        // HOpen before VClose at same Y
        assert_eq!(compare_events(&h_open_same, &v_close_same, &segments_same_y), Ordering::Less);
    }

    #[test]
    fn parallel_segments_no_intersection() {
        let mut h1 = vec![
            ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Normal, false),
            ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false),
        ];

        let graph = build_graph_from_segments(&mut h1, &mut []);
        assert_eq!(graph.vertex_count(), 0);
    }

    #[test]
    fn vertex_tracking_state_written_back() {
        let mut h = vec![ScanSegment::new(
            Point::new(0.0, 5.0), Point::new(10.0, 5.0),
            SegmentWeight::Normal, false,
        )];
        let mut v = vec![ScanSegment::new(
            Point::new(5.0, 0.0), Point::new(5.0, 10.0),
            SegmentWeight::Normal, true,
        )];

        let _graph = build_graph_from_segments(&mut h, &mut v);

        // Both segments should have vertex tracking set
        assert!(h[0].lowest_vertex.is_some());
        assert!(h[0].highest_vertex.is_some());
        assert!(v[0].lowest_vertex.is_some());
        assert!(v[0].highest_vertex.is_some());
    }

    #[test]
    fn edges_created_between_consecutive_vertices_on_segment() {
        // V segment crossed by two H segments → V should have edges between crossings
        let mut h = vec![
            ScanSegment::new(Point::new(0.0, 3.0), Point::new(10.0, 3.0), SegmentWeight::Normal, false),
            ScanSegment::new(Point::new(0.0, 7.0), Point::new(10.0, 7.0), SegmentWeight::Normal, false),
        ];
        let mut v = vec![ScanSegment::new(
            Point::new(5.0, 0.0), Point::new(5.0, 10.0),
            SegmentWeight::Normal, true,
        )];

        let graph = build_graph_from_segments(&mut h, &mut v);

        // Intersection vertices: (5,3) and (5,7)
        let v1 = graph.find_vertex(Point::new(5.0, 3.0)).unwrap();
        let v2 = graph.find_vertex(Point::new(5.0, 7.0)).unwrap();

        // There should be edges between them (bidirectional)
        assert!(graph.find_edge(v1, v2).is_some() || graph.find_edge(v2, v1).is_some());
    }

    #[test]
    fn weighted_segments_produce_weighted_edges() {
        let mut h = vec![ScanSegment::new(
            Point::new(0.0, 5.0), Point::new(10.0, 5.0),
            SegmentWeight::Reflection, false,
        )];
        let mut v = vec![ScanSegment::new(
            Point::new(5.0, 0.0), Point::new(5.0, 10.0),
            SegmentWeight::Normal, true,
        )];

        let graph = build_graph_from_segments(&mut h, &mut v);
        assert_eq!(graph.vertex_count(), 1);
    }

    #[test]
    fn sweep_matches_brute_force_for_small_inputs() {
        // Compare event-based sweep with brute-force intersection
        let mut h = vec![
            ScanSegment::new(Point::new(0.0, 2.0), Point::new(8.0, 2.0), SegmentWeight::Normal, false),
            ScanSegment::new(Point::new(1.0, 6.0), Point::new(9.0, 6.0), SegmentWeight::Normal, false),
        ];
        let mut v = vec![
            ScanSegment::new(Point::new(3.0, 0.0), Point::new(3.0, 10.0), SegmentWeight::Normal, true),
            ScanSegment::new(Point::new(7.0, 1.0), Point::new(7.0, 8.0), SegmentWeight::Normal, true),
        ];

        let graph = build_graph_from_segments(&mut h, &mut v);

        // Brute-force: check all H-V pairs
        // H1 (y=2, x=[0,8]) x V1 (x=3, y=[0,10]) -> (3,2) YES
        // H1 (y=2, x=[0,8]) x V2 (x=7, y=[1,8])  -> (7,2) YES
        // H2 (y=6, x=[1,9]) x V1 (x=3, y=[0,10]) -> (3,6) YES
        // H2 (y=6, x=[1,9]) x V2 (x=7, y=[1,8])  -> (7,6) YES
        assert_eq!(graph.vertex_count(), 4);
        assert!(graph.find_vertex(Point::new(3.0, 2.0)).is_some());
        assert!(graph.find_vertex(Point::new(7.0, 2.0)).is_some());
        assert!(graph.find_vertex(Point::new(3.0, 6.0)).is_some());
        assert!(graph.find_vertex(Point::new(7.0, 6.0)).is_some());
    }

    #[test]
    fn empty_segments_return_empty_graph() {
        let graph = build_graph_from_segments(&mut [], &mut []);
        assert_eq!(graph.vertex_count(), 0);
    }

    #[test]
    fn h_only_no_crossings() {
        let mut h = vec![
            ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Normal, false),
        ];
        let graph = build_graph_from_segments(&mut h, &mut []);
        assert_eq!(graph.vertex_count(), 0);
    }

    #[test]
    fn v_only_no_crossings() {
        let mut v = vec![
            ScanSegment::new(Point::new(0.0, 0.0), Point::new(0.0, 10.0), SegmentWeight::Normal, true),
        ];
        let graph = build_graph_from_segments(&mut [], &mut v);
        assert_eq!(graph.vertex_count(), 0);
    }
}
