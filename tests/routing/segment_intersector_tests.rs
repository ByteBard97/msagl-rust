use msagl_rust::routing::segment_intersector::build_graph_from_segments;
use msagl_rust::routing::scan_segment::{ScanSegment, SegmentWeight};
use msagl_rust::Point;

#[test]
fn two_crossing_segments_create_intersection() {
    let mut h = vec![ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false)];
    let mut v = vec![ScanSegment::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0), SegmentWeight::Normal, true)];

    let graph = build_graph_from_segments(&mut h, &mut v);

    // V endpoints (5,0), (5,10) + H endpoints (0,5), (10,5) + intersection (5,5) = 5
    assert_eq!(graph.vertex_count(), 5);
    assert!(graph.find_vertex(Point::new(5.0, 5.0)).is_some());
}

#[test]
fn parallel_segments_no_intersection() {
    let mut h = vec![
        ScanSegment::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0), SegmentWeight::Normal, false),
        ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0), SegmentWeight::Normal, false),
    ];

    let graph = build_graph_from_segments(&mut h, &mut []);

    // 2 H segments with start+end each = 4 vertices
    assert_eq!(graph.vertex_count(), 4);
}
