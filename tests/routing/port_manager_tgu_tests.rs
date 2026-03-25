use msagl_rust::routing::port_manager::PortManager;
use msagl_rust::visibility::graph::{VertexId, VisibilityGraph};
use msagl_rust::Point;

/// Build a simple cross-shaped visibility graph:
///
/// ```text
///     (50,100)
///        |
/// (0,50)--(50,50)--(100,50)
///        |
///     (50,0)
/// ```
fn make_cross_graph() -> VisibilityGraph {
    let mut g = VisibilityGraph::new();
    let c = g.add_vertex(Point::new(50.0, 50.0));
    let e = g.add_vertex(Point::new(100.0, 50.0));
    let w = g.add_vertex(Point::new(0.0, 50.0));
    let n = g.add_vertex(Point::new(50.0, 100.0));
    let s = g.add_vertex(Point::new(50.0, 0.0));
    g.add_edge(c, e, 50.0);
    g.add_edge(e, c, 50.0);
    g.add_edge(c, w, 50.0);
    g.add_edge(w, c, 50.0);
    g.add_edge(c, n, 50.0);
    g.add_edge(n, c, 50.0);
    g.add_edge(c, s, 50.0);
    g.add_edge(s, c, 50.0);
    g
}

#[test]
fn splice_connects_to_axis_aligned_neighbors() {
    let mut graph = make_cross_graph();
    // Splice a port at (25, 50) — on the horizontal line between W(0,50) and C(50,50).
    // The port should be connected to both the east neighbor (center at 50,50) and the
    // west neighbor (W at 0,50).
    //
    // Edges are stored in ascending order (lower→higher), so the west connection
    // from port(25,50) to w(0,50) is stored as w→port rather than port→w.
    // Use find_edge_pp which checks both directions.
    let _ = PortManager::splice_port(&mut graph, Point::new(25.0, 50.0));

    let has_east = graph
        .find_edge_pp(Point::new(25.0, 50.0), Point::new(50.0, 50.0))
        .is_some();
    let has_west = graph
        .find_edge_pp(Point::new(25.0, 50.0), Point::new(0.0, 50.0))
        .is_some();

    assert!(has_east, "port should connect east to axis-aligned neighbor (center)");
    assert!(has_west, "port should connect west to axis-aligned neighbor (w)");
}

#[test]
fn unsplice_restores_edge_count() {
    let mut graph = make_cross_graph();
    let initial_edges: usize = (0..graph.vertex_count())
        .map(|i| graph.out_degree(VertexId(i)))
        .sum();

    let mut splice = PortManager::splice_port(&mut graph, Point::new(25.0, 50.0));
    // After splice the edge count should have grown
    let after_splice_edges: usize = (0..graph.vertex_count())
        .map(|i| graph.out_degree(VertexId(i)))
        .sum();
    assert!(
        after_splice_edges > initial_edges,
        "splice should add edges"
    );

    PortManager::unsplice(&mut graph, &mut splice);

    // After unsplice the edge count should be restored
    let final_edges: usize = (0..graph.vertex_count())
        .map(|i| graph.out_degree(VertexId(i)))
        .sum();
    assert_eq!(
        initial_edges, final_edges,
        "edge count should be restored after unsplice"
    );
}
