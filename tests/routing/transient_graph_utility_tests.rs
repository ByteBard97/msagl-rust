use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::routing::transient_graph_utility::TransientGraphUtility;
use msagl_rust::visibility::graph::VisibilityGraph;
use msagl_rust::Point;

// -----------------------------------------------------------------------
// find_or_add_vertex
// -----------------------------------------------------------------------

#[test]
fn add_transient_vertex() {
    let mut graph = VisibilityGraph::new();
    let mut tgu = TransientGraphUtility::new();
    let v = tgu.find_or_add_vertex(&mut graph, Point::new(10.0, 20.0));
    assert_eq!(graph.vertex_count(), 1);
    assert_eq!(graph.point(v), Point::new(10.0, 20.0));
    assert_eq!(tgu.added_vertices().len(), 1);
}

#[test]
fn find_or_add_vertex_reuses_existing() {
    let mut graph = VisibilityGraph::new();
    let existing = graph.add_vertex(Point::new(10.0, 20.0));
    let mut tgu = TransientGraphUtility::new();
    let found = tgu.find_or_add_vertex(&mut graph, Point::new(10.0, 20.0));
    assert_eq!(found, existing);
    assert_eq!(graph.vertex_count(), 1);
    // Not tracked as transient since it already existed.
    assert_eq!(tgu.added_vertices().len(), 0);
}

// -----------------------------------------------------------------------
// find_or_add_edge -- no bracket
// -----------------------------------------------------------------------

#[test]
fn add_edge_without_bracket() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(100.0, 0.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge_vv(&mut graph, a, b);
    // Edge should be in ascending order (a->b since a is "lower"/west).
    assert!(graph.find_edge(a, b).is_some(), "should have edge A->B");
}

// -----------------------------------------------------------------------
// bracket detection
// -----------------------------------------------------------------------

#[test]
fn bracket_detection_splits_edge() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, c, 100.0);

    let b = graph.add_vertex(Point::new(50.0, 0.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut graph, a, b, 50.0);

    // A->C should be split into A->B + B->C
    assert!(graph.find_edge(a, b).is_some(), "should have edge A->B");
    assert!(graph.find_edge(b, c).is_some(), "should have edge B->C");
    assert!(graph.find_edge(a, c).is_none(), "should NOT have edge A->C");
}

#[test]
fn bracket_detection_vertical() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(0.0, 100.0));
    graph.add_edge(a, c, 100.0);

    let b = graph.add_vertex(Point::new(0.0, 50.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut graph, a, b, 50.0);

    assert!(graph.find_edge(a, b).is_some(), "should have edge A->B");
    assert!(graph.find_edge(b, c).is_some(), "should have edge B->C");
    assert!(graph.find_edge(a, c).is_none(), "should NOT have edge A->C");
}

#[test]
fn no_bracket_when_target_is_past_existing() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(50.0, 0.0));
    graph.add_edge(a, b, 50.0);

    // c is past b -- no bracket, just a new edge
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut graph, a, c, 100.0);

    // a->b still exists, and we add b->c
    assert!(graph.find_edge(a, b).is_some(), "A->B should still exist");
}

// -----------------------------------------------------------------------
// split_edge
// -----------------------------------------------------------------------

#[test]
fn split_edge_at_midpoint() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, c, 100.0);

    let b = graph.add_vertex(Point::new(50.0, 0.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.split_edge(&mut graph, a, c, b);

    assert!(graph.find_edge(a, b).is_some(), "should have A->B");
    assert!(graph.find_edge(b, c).is_some(), "should have B->C");
    assert!(graph.find_edge(a, c).is_none(), "should NOT have A->C");
}

#[test]
fn split_edge_at_endpoint_is_noop() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, c, 100.0);

    let mut tgu = TransientGraphUtility::new();
    // Split at source -- should be a no-op.
    tgu.split_edge(&mut graph, a, c, a);
    assert!(graph.find_edge(a, c).is_some(), "A->C should still exist");
}

// -----------------------------------------------------------------------
// remove_from_graph
// -----------------------------------------------------------------------

#[test]
fn remove_from_graph_restores_state() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, c, 100.0);

    let b = graph.add_vertex(Point::new(50.0, 0.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut graph, a, b, 50.0);

    // Verify split happened.
    assert!(graph.find_edge(a, c).is_none());

    // Restore.
    tgu.remove_from_graph(&mut graph);
    assert!(graph.find_edge(a, c).is_some(), "A->C should be restored");
    assert!(graph.find_edge(a, b).is_none(), "A->B should be removed");
    assert!(graph.find_edge(b, c).is_none(), "B->C should be removed");
}

#[test]
fn remove_from_graph_clears_tracking() {
    let mut graph = VisibilityGraph::new();
    let mut tgu = TransientGraphUtility::new();
    let _v = tgu.find_or_add_vertex(&mut graph, Point::new(5.0, 5.0));
    assert_eq!(tgu.added_vertices().len(), 1);

    tgu.remove_from_graph(&mut graph);
    assert_eq!(tgu.added_vertices().len(), 0);
    assert_eq!(tgu.added_edges().len(), 0);
}

// -----------------------------------------------------------------------
// connect_vertex_to_target
// -----------------------------------------------------------------------

#[test]
fn connect_collinear_vertices() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(100.0, 0.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.connect_vertex_to_target(&mut graph, a, b, CompassDirection::East, 1.0);
    assert!(graph.find_edge(a, b).is_some());
}

#[test]
fn connect_with_bend_vertex() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(100.0, 50.0));
    let mut tgu = TransientGraphUtility::new();
    // Final edge dir is East -> bend at (source.x, target.y) = (0, 50)
    tgu.connect_vertex_to_target(&mut graph, a, b, CompassDirection::East, 1.0);

    let bend = graph.find_vertex(Point::new(0.0, 50.0));
    assert!(bend.is_some(), "should have created bend vertex at (0, 50)");
    assert_eq!(
        tgu.added_vertices().len(),
        1,
        "bend vertex should be tracked"
    );
}

#[test]
fn connect_same_point_is_noop() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(5.0, 5.0));
    let mut tgu = TransientGraphUtility::new();
    tgu.connect_vertex_to_target(&mut graph, a, a, CompassDirection::East, 1.0);
    assert_eq!(tgu.added_edges().len(), 0);
}

// -----------------------------------------------------------------------
// add_edge_to_target_edge
// -----------------------------------------------------------------------

#[test]
fn add_edge_to_target_edge_creates_vertex_and_splits() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 50.0)); // source
    let t1 = graph.add_vertex(Point::new(50.0, 0.0));
    let t2 = graph.add_vertex(Point::new(50.0, 100.0));
    graph.add_edge(t1, t2, 100.0);

    let mut tgu = TransientGraphUtility::new();
    let intersect = Point::new(50.0, 50.0);
    let v = tgu.add_edge_to_target_edge(&mut graph, a, t1, t2, intersect);

    assert_eq!(graph.point(v), intersect);
    // t1->t2 should be split at v.
    assert!(graph.find_edge(t1, v).is_some(), "t1->v should exist");
    assert!(graph.find_edge(v, t2).is_some(), "v->t2 should exist");
}

// -----------------------------------------------------------------------
// find_perpendicular_or_containing_edge
// -----------------------------------------------------------------------

#[test]
fn find_perpendicular_edge() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(50.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, b, 50.0);
    graph.add_edge(b, c, 50.0);

    // Looking East from a, find the edge that brackets y=0, x=75.
    let result = TransientGraphUtility::find_perpendicular_or_containing_edge(
        &graph,
        a,
        CompassDirection::East,
        Point::new(75.0, 0.0),
    );
    assert!(result.is_some());
    let (src, tgt) = result.unwrap();
    assert_eq!(src, b);
    assert_eq!(tgt, c);
}
