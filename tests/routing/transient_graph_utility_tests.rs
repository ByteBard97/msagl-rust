use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::routing::obstacle_tree::ObstacleTree;
use msagl_rust::routing::router_session::RouterSession;
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::routing::scan_segment::ScanSegmentTree;
use msagl_rust::routing::transient_graph_utility::TransientGraphUtility;
use msagl_rust::visibility::graph::VisibilityGraph;
use msagl_rust::Point;

/// Build a minimal RouterSession from a standalone VisibilityGraph for tests.
fn session_from_graph(graph: VisibilityGraph) -> RouterSession {
    RouterSession {
        vis_graph: graph,
        obstacle_tree: ObstacleTree::empty(),
        h_scan_segments: ScanSegmentTree::new(ScanDirection::horizontal()),
        v_scan_segments: ScanSegmentTree::new(ScanDirection::vertical()),
        padding: 0.0,
    }
}

// -----------------------------------------------------------------------
// find_or_add_vertex
// -----------------------------------------------------------------------

#[test]
fn add_transient_vertex() {
    let mut session = session_from_graph(VisibilityGraph::new());
    let mut tgu = TransientGraphUtility::new();
    let v = tgu.find_or_add_vertex(&mut session, Point::new(10.0, 20.0));
    assert_eq!(session.vis_graph.vertex_count(), 1);
    assert_eq!(session.vis_graph.point(v), Point::new(10.0, 20.0));
    assert_eq!(tgu.added_vertices().len(), 1);
}

#[test]
fn find_or_add_vertex_reuses_existing() {
    let mut graph = VisibilityGraph::new();
    let existing = graph.add_vertex(Point::new(10.0, 20.0));
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    let found = tgu.find_or_add_vertex(&mut session, Point::new(10.0, 20.0));
    assert_eq!(found, existing);
    assert_eq!(session.vis_graph.vertex_count(), 1);
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
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge_vv(&mut session, a, b);
    // Edge should be in ascending order (a->b since a is "lower"/west).
    assert!(session.vis_graph.find_edge(a, b).is_some(), "should have edge A->B");
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
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut session, a, b, 50.0);

    // A->C should be split into A->B + B->C
    assert!(session.vis_graph.find_edge(a, b).is_some(), "should have edge A->B");
    assert!(session.vis_graph.find_edge(b, c).is_some(), "should have edge B->C");
    assert!(session.vis_graph.find_edge(a, c).is_none(), "should NOT have edge A->C");
}

#[test]
fn bracket_detection_vertical() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(0.0, 100.0));
    graph.add_edge(a, c, 100.0);

    let b = graph.add_vertex(Point::new(0.0, 50.0));
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut session, a, b, 50.0);

    assert!(session.vis_graph.find_edge(a, b).is_some(), "should have edge A->B");
    assert!(session.vis_graph.find_edge(b, c).is_some(), "should have edge B->C");
    assert!(session.vis_graph.find_edge(a, c).is_none(), "should NOT have edge A->C");
}

#[test]
fn no_bracket_when_target_is_past_existing() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(50.0, 0.0));
    graph.add_edge(a, b, 50.0);

    // c is past b -- no bracket, just a new edge
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut session, a, c, 100.0);

    // a->b still exists, and we add b->c
    assert!(session.vis_graph.find_edge(a, b).is_some(), "A->B should still exist");
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
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.split_edge(&mut session, a, c, b);

    assert!(session.vis_graph.find_edge(a, b).is_some(), "should have A->B");
    assert!(session.vis_graph.find_edge(b, c).is_some(), "should have B->C");
    assert!(session.vis_graph.find_edge(a, c).is_none(), "should NOT have A->C");
}

#[test]
fn split_edge_at_endpoint_is_noop() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let c = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(a, c, 100.0);

    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    // Split at source -- should be a no-op.
    tgu.split_edge(&mut session, a, c, a);
    assert!(session.vis_graph.find_edge(a, c).is_some(), "A->C should still exist");
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
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.find_or_add_edge(&mut session, a, b, 50.0);

    // Verify split happened.
    assert!(session.vis_graph.find_edge(a, c).is_none());

    // Restore.
    tgu.remove_from_graph(&mut session);
    assert!(session.vis_graph.find_edge(a, c).is_some(), "A->C should be restored");
    assert!(session.vis_graph.find_edge(a, b).is_none(), "A->B should be removed");
    assert!(session.vis_graph.find_edge(b, c).is_none(), "B->C should be removed");
}

#[test]
fn remove_from_graph_clears_tracking() {
    let mut session = session_from_graph(VisibilityGraph::new());
    let mut tgu = TransientGraphUtility::new();
    let _v = tgu.find_or_add_vertex(&mut session, Point::new(5.0, 5.0));
    assert_eq!(tgu.added_vertices().len(), 1);

    tgu.remove_from_graph(&mut session);
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
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.connect_vertex_to_target(&mut session, a, b, CompassDirection::East, 1.0);
    assert!(session.vis_graph.find_edge(a, b).is_some());
}

#[test]
fn connect_with_bend_vertex() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(100.0, 50.0));
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    // Final edge dir is East -> bend at (source.x, target.y) = (0, 50)
    tgu.connect_vertex_to_target(&mut session, a, b, CompassDirection::East, 1.0);

    let bend = session.vis_graph.find_vertex(Point::new(0.0, 50.0));
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
    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    tgu.connect_vertex_to_target(&mut session, a, a, CompassDirection::East, 1.0);
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

    let mut session = session_from_graph(graph);
    let mut tgu = TransientGraphUtility::new();
    let intersect = Point::new(50.0, 50.0);
    let v = tgu.add_edge_to_target_edge(&mut session, a, t1, t2, intersect);

    assert_eq!(session.vis_graph.point(v), intersect);
    // t1->t2 should be split at v.
    assert!(session.vis_graph.find_edge(t1, v).is_some(), "t1->v should exist");
    assert!(session.vis_graph.find_edge(v, t2).is_some(), "v->t2 should exist");
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
