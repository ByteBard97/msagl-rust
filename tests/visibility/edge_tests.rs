use msagl_rust::geometry::point::Point;
use msagl_rust::visibility::edge::VisEdge;
use msagl_rust::visibility::graph::VertexId;

#[test]
fn toll_free_edge_has_zero_weight() {
    let edge = VisEdge::toll_free(VertexId(5), Point::new(10.0, 20.0));
    assert!(edge.is_toll_free);
    assert_eq!(edge.weight, 0.0);
    assert_eq!(edge.target, VertexId(5));
}

#[test]
fn normal_edge_is_not_toll_free() {
    let edge = VisEdge::new(VertexId(3), Point::new(5.0, 6.0), 2.5);
    assert!(!edge.is_toll_free);
    assert_eq!(edge.weight, 2.5);
}

#[test]
fn equality_ignores_toll_free_flag() {
    // Two edges to the same target point are equal regardless of toll-free status.
    let p = Point::new(10.0, 20.0);
    let normal = VisEdge::new(VertexId(7), p, 1.0);
    let free = VisEdge::toll_free(VertexId(7), p);
    assert_eq!(normal, free);
}

#[test]
fn ordering_by_target_point() {
    // Edges are ordered by target point (x first, then y), not by VertexId.
    let a = VisEdge::new(VertexId(99), Point::new(1.0, 0.0), 99.0);
    let b = VisEdge::toll_free(VertexId(1), Point::new(2.0, 0.0));
    assert!(a < b);

    // Same x, different y.
    let c = VisEdge::new(VertexId(5), Point::new(5.0, 1.0), 0.0);
    let d = VisEdge::new(VertexId(2), Point::new(5.0, 2.0), 0.0);
    assert!(c < d);
}
