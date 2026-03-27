use msagl_rust::routing::obstacle_tree::ObstacleTree;
use msagl_rust::routing::port_manager::PortManager;
use msagl_rust::routing::transient_graph_utility::TransientGraphUtility;
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
    let _ = PortManager::splice_port(&mut graph, &mut ObstacleTree::empty(), Point::new(25.0, 50.0));

    let has_east = graph
        .find_edge_pp(Point::new(25.0, 50.0), Point::new(50.0, 50.0))
        .is_some();
    let has_west = graph
        .find_edge_pp(Point::new(25.0, 50.0), Point::new(0.0, 50.0))
        .is_some();

    assert!(
        has_east,
        "port should connect east to axis-aligned neighbor (center)"
    );
    assert!(
        has_west,
        "port should connect west to axis-aligned neighbor (w)"
    );
}

#[test]
fn unsplice_restores_edge_count() {
    let mut graph = make_cross_graph();
    let initial_edges: usize = (0..graph.vertex_count())
        .map(|i| graph.out_degree(VertexId(i)))
        .sum();

    let mut splice = PortManager::splice_port(&mut graph, &mut ObstacleTree::empty(), Point::new(25.0, 50.0));
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

#[test]
fn find_or_add_vertex_on_edge_splits_existing_edge() {
    // Create a simple VG with one vertical edge: (0,0) → (0,10)
    let mut graph = VisibilityGraph::new();
    let v_bottom = graph.add_vertex(Point::new(0.0, 0.0));
    let v_top = graph.add_vertex(Point::new(0.0, 10.0));
    graph.add_edge(v_bottom, v_top, 1.0);

    // Add a vertex at (0, 5) — on the existing edge.
    // After this, (0,5) should have edges to both (0,0) and (0,10).
    let mut tgu = TransientGraphUtility::new();
    let v_mid = tgu.find_or_add_vertex_splitting(
        &mut graph,
        Point::new(0.0, 5.0),
    );

    assert!(graph.out_degree(v_mid) + graph.in_degree(v_mid) > 0,
        "vertex at (0,5) should have edges after splitting");

    // Should be connected to bottom
    let has_bottom = graph.find_edge_pp(Point::new(0.0, 0.0), Point::new(0.0, 5.0)).is_some();
    assert!(has_bottom, "should have edge between (0,0) and (0,5)");

    // Should be connected to top
    let has_top = graph.find_edge_pp(Point::new(0.0, 5.0), Point::new(0.0, 10.0)).is_some();
    assert!(has_top, "should have edge between (0,5) and (0,10)");
}

#[test]
fn split_vertex_on_grid_does_not_break_other_edges() {
    // Create a grid-like VG mimicking a padded obstacle boundary:
    //
    //  (-1,11)-----(11,11)
    //     |            |
    //  (-1,-1)-----(11,-1)
    //
    // With horizontal edges at y=-1 and y=11, vertical edges at x=-1 and x=11.
    let mut graph = VisibilityGraph::new();
    let bl = graph.add_vertex(Point::new(-1.0, -1.0));
    let br = graph.add_vertex(Point::new(11.0, -1.0));
    let tl = graph.add_vertex(Point::new(-1.0, 11.0));
    let tr = graph.add_vertex(Point::new(11.0, 11.0));
    graph.add_edge(bl, br, 1.0); // bottom horizontal
    graph.add_edge(tl, tr, 1.0); // top horizontal
    graph.add_edge(bl, tl, 1.0); // left vertical
    graph.add_edge(br, tr, 1.0); // right vertical

    let initial_edge_count: usize = (0..graph.vertex_count())
        .map(|i| graph.out_degree(VertexId(i)))
        .sum();

    // Add a vertex at (11, 5) — on the right vertical edge between (11,-1) and (11,11).
    let mut tgu = TransientGraphUtility::new();
    let v_mid = tgu.find_or_add_vertex_splitting(&mut graph, Point::new(11.0, 5.0));

    // The new vertex should be connected to both endpoints of the right vertical edge.
    let has_below = graph.find_edge_pp(Point::new(11.0, -1.0), Point::new(11.0, 5.0)).is_some();
    let has_above = graph.find_edge_pp(Point::new(11.0, 5.0), Point::new(11.0, 11.0)).is_some();
    assert!(has_below, "should have edge between (11,-1) and (11,5)");
    assert!(has_above, "should have edge between (11,5) and (11,11)");

    // The other 3 edges should still exist.
    assert!(graph.find_edge_pp(Point::new(-1.0, -1.0), Point::new(11.0, -1.0)).is_some(),
        "bottom horizontal edge should survive");
    assert!(graph.find_edge_pp(Point::new(-1.0, 11.0), Point::new(11.0, 11.0)).is_some(),
        "top horizontal edge should survive");
    assert!(graph.find_edge_pp(Point::new(-1.0, -1.0), Point::new(-1.0, 11.0)).is_some(),
        "left vertical edge should survive");

    // Total edge count should increase by 1 (one edge split into two).
    let final_edge_count: usize = (0..graph.vertex_count())
        .map(|i| graph.out_degree(VertexId(i)))
        .sum();
    assert_eq!(final_edge_count, initial_edge_count + 1,
        "splitting one edge should add exactly 1 edge");
}
