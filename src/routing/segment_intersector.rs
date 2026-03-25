use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VisibilityGraph;
use super::scan_segment::ScanSegment;

/// Build the visibility graph from horizontal and vertical scan segments.
///
/// For each pair of (horizontal, vertical) segments that cross, adds a vertex
/// at the intersection and edges along both segments.
pub fn build_graph_from_segments(
    graph: &mut VisibilityGraph,
    h_segments: &[ScanSegment],
    v_segments: &[ScanSegment],
) {
    // Step 1: Add edges for each segment's endpoints
    add_segment_edges(graph, h_segments);
    add_segment_edges(graph, v_segments);

    // Step 2: Find intersections between horizontal and vertical segments,
    // splitting existing edges at each crossing point.
    add_intersection_vertices(graph, h_segments, v_segments);
}

/// Add graph edges for each segment (start -> end).
fn add_segment_edges(graph: &mut VisibilityGraph, segments: &[ScanSegment]) {
    for seg in segments {
        if seg.start == seg.end {
            continue;
        }
        let v1 = graph.add_vertex(seg.start);
        let v2 = graph.add_vertex(seg.end);
        let dist = distance(seg.start, seg.end);
        let weight = seg_weight(seg);
        graph.add_edge(v1, v2, dist * weight);
        graph.add_edge(v2, v1, dist * weight);
    }
}

/// Find all crossings between horizontal and vertical segments,
/// add vertices at crossing points, and create edges through them.
fn add_intersection_vertices(
    graph: &mut VisibilityGraph,
    h_segments: &[ScanSegment],
    v_segments: &[ScanSegment],
) {
    // For each horizontal segment, collect all vertical segments that cross it.
    // A horizontal segment has fixed y, spanning [x_lo, x_hi].
    // A vertical segment has fixed x, spanning [y_lo, y_hi].
    // They cross if the vertical's x is within [x_lo, x_hi] and
    // the horizontal's y is within [y_lo, y_hi].

    for h_seg in h_segments {
        let h_y = h_seg.start.y();
        let h_x_lo = h_seg.start.x().min(h_seg.end.x());
        let h_x_hi = h_seg.start.x().max(h_seg.end.x());

        for v_seg in v_segments {
            let v_x = v_seg.start.x();
            let v_y_lo = v_seg.start.y().min(v_seg.end.y());
            let v_y_hi = v_seg.start.y().max(v_seg.end.y());

            let eps = GeomConstants::DISTANCE_EPSILON;

            // Check crossing (strictly interior is optional — endpoints count too)
            if v_x >= h_x_lo - eps && v_x <= h_x_hi + eps
                && h_y >= v_y_lo - eps && h_y <= v_y_hi + eps
            {
                let intersection = Point::new(v_x, h_y);

                // Skip if this is an endpoint of both segments (already connected)
                let is_h_endpoint = intersection == h_seg.start || intersection == h_seg.end;
                let is_v_endpoint = intersection == v_seg.start || intersection == v_seg.end;
                if is_h_endpoint && is_v_endpoint {
                    continue;
                }

                // Add the intersection vertex
                let _vid = graph.add_vertex(intersection);

                // Edges will be created by the segment-chain builder below.
            }
        }
    }

    // Now rebuild edges along each segment, sorted by coordinate.
    // For each segment, gather all vertices that lie on it, sort them,
    // and create edges between consecutive pairs.
    rebuild_segment_chain(graph, h_segments, true);
    rebuild_segment_chain(graph, v_segments, false);
}

/// Remove existing edges along segments and rebuild with intermediate vertices.
fn rebuild_segment_chain(
    graph: &mut VisibilityGraph,
    segments: &[ScanSegment],
    is_horizontal: bool,
) {
    for seg in segments {
        if seg.start == seg.end {
            continue;
        }

        // First remove the direct edge between endpoints
        if let (Some(v1), Some(v2)) = (
            graph.find_vertex(seg.start),
            graph.find_vertex(seg.end),
        ) {
            graph.remove_edge(v1, v2);
            graph.remove_edge(v2, v1);
        }

        // Collect all graph vertices that lie on this segment
        let mut on_segment = collect_vertices_on_segment(graph, seg, is_horizontal);

        // Sort by coordinate along the segment
        if is_horizontal {
            on_segment.sort_by(|a, b| {
                GeomConstants::compare(
                    graph.point(*a).x(),
                    graph.point(*b).x(),
                )
            });
        } else {
            on_segment.sort_by(|a, b| {
                GeomConstants::compare(
                    graph.point(*a).y(),
                    graph.point(*b).y(),
                )
            });
        }

        // Create edges between consecutive vertices
        let weight = seg_weight(seg);
        for pair in on_segment.windows(2) {
            let p1 = graph.point(pair[0]);
            let p2 = graph.point(pair[1]);
            let dist = distance(p1, p2) * weight;
            graph.add_edge(pair[0], pair[1], dist);
            graph.add_edge(pair[1], pair[0], dist);
        }
    }
}

/// Find all vertices in the graph that lie on a segment.
fn collect_vertices_on_segment(
    graph: &VisibilityGraph,
    seg: &ScanSegment,
    is_horizontal: bool,
) -> Vec<crate::visibility::graph::VertexId> {
    use crate::visibility::graph::VertexId;
    let eps = GeomConstants::DISTANCE_EPSILON;
    let mut result = Vec::new();

    if is_horizontal {
        let y = seg.start.y();
        let x_lo = seg.start.x().min(seg.end.x());
        let x_hi = seg.start.x().max(seg.end.x());
        for i in 0..graph.vertex_count() {
            let p = graph.point(VertexId(i));
            if (p.y() - y).abs() <= eps && p.x() >= x_lo - eps && p.x() <= x_hi + eps {
                result.push(VertexId(i));
            }
        }
    } else {
        let x = seg.start.x();
        let y_lo = seg.start.y().min(seg.end.y());
        let y_hi = seg.start.y().max(seg.end.y());
        for i in 0..graph.vertex_count() {
            let p = graph.point(VertexId(i));
            if (p.x() - x).abs() <= eps && p.y() >= y_lo - eps && p.y() <= y_hi + eps {
                result.push(VertexId(i));
            }
        }
    }

    result
}

fn distance(a: Point, b: Point) -> f64 {
    (a - b).length()
}

fn seg_weight(seg: &ScanSegment) -> f64 {
    match seg.weight {
        super::scan_segment::SegmentWeight::Normal => 1.0,
        super::scan_segment::SegmentWeight::Reflection => 5.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::point::Point;

    #[test]
    fn two_crossing_segments_create_intersection() {
        let h = vec![ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0))];
        let v = vec![ScanSegment::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];

        let mut graph = VisibilityGraph::new();
        build_graph_from_segments(&mut graph, &h, &v);

        // Should have 5 vertices: 4 endpoints + 1 intersection at (5, 5)
        assert_eq!(graph.vertex_count(), 5);
        assert!(graph.find_vertex(Point::new(5.0, 5.0)).is_some());
    }

    #[test]
    fn parallel_segments_no_intersection() {
        let h1 = ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
        let h2 = ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0));

        let mut graph = VisibilityGraph::new();
        build_graph_from_segments(&mut graph, &[h1, h2], &[]);

        assert_eq!(graph.vertex_count(), 4);
    }
}
