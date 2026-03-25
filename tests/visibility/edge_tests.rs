use msagl_rust::visibility::edge::VisEdge;
use msagl_rust::visibility::graph::VertexId;

#[test]
fn toll_free_edge_has_zero_weight() {
    let edge = VisEdge::toll_free(VertexId(5));
    assert!(edge.is_toll_free);
    assert_eq!(edge.weight, 0.0);
    assert_eq!(edge.target, VertexId(5));
}

#[test]
fn normal_edge_is_not_toll_free() {
    let edge = VisEdge::new(VertexId(3), 2.5);
    assert!(!edge.is_toll_free);
    assert_eq!(edge.weight, 2.5);
}

#[test]
fn equality_ignores_toll_free_flag() {
    // Two edges to the same target are equal regardless of toll-free status.
    let normal = VisEdge::new(VertexId(7), 1.0);
    let free = VisEdge::toll_free(VertexId(7));
    assert_eq!(normal, free);
}

#[test]
fn ordering_ignores_toll_free_flag() {
    let a = VisEdge::new(VertexId(1), 99.0);
    let b = VisEdge::toll_free(VertexId(2));
    assert!(a < b);
}
