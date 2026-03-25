use msagl_rust::routing::segment_intersector::build_graph_from_segments;
use msagl_rust::routing::scan_segment::{ScanSegment, SegmentWeight};
use msagl_rust::visibility::graph::VisibilityGraph;
use msagl_rust::Point;

#[test]
fn two_crossing_segments_create_intersection() {
    let h = vec![ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false)];
    let v = vec![ScanSegment::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0), SegmentWeight::Normal, true)];

    let mut graph = VisibilityGraph::new();
    build_graph_from_segments(&mut graph, &h, &v);

    // Should have 5 vertices: 4 endpoints + 1 intersection at (5, 5)
    assert_eq!(graph.vertex_count(), 5);
    assert!(graph.find_vertex(Point::new(5.0, 5.0)).is_some());
}

#[test]
fn parallel_segments_no_intersection() {
    let h1 = ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Normal, false);
    let h2 = ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false);

    let mut graph = VisibilityGraph::new();
    build_graph_from_segments(&mut graph, &[h1, h2], &[]);

    assert_eq!(graph.vertex_count(), 4);
}
