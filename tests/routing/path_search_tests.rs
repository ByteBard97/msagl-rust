use msagl_rust::routing::path_search::{
    PathSearch, DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE,
    manhattan_distance, estimated_bends_to_target,
};
use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::visibility::graph::VisibilityGraph;
use msagl_rust::Point;

#[test]
fn path_search_straight_line() {
    let mut graph = VisibilityGraph::new();
    let v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let v2 = graph.add_vertex(Point::new(10.0, 0.0));
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);

    let search = PathSearch::new(4.0);
    let path = search.find_path(&mut graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0));
    assert!(path.is_some());
    let pts = path.unwrap();
    assert_eq!(pts.len(), 2);
    assert_eq!(pts[0], Point::new(0.0, 0.0));
    assert_eq!(pts[1], Point::new(10.0, 0.0));
}

#[test]
fn path_search_with_bend() {
    let mut graph = VisibilityGraph::new();
    let v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let v2 = graph.add_vertex(Point::new(10.0, 0.0));
    let v3 = graph.add_vertex(Point::new(10.0, 10.0));
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);
    graph.add_edge(v2, v3, 1.0);
    graph.add_edge(v3, v2, 1.0);

    let search = PathSearch::new(4.0);
    let path = search.find_path(&mut graph, Point::new(0.0, 0.0), Point::new(10.0, 10.0));
    assert!(path.is_some());
    let pts = path.unwrap();
    assert_eq!(pts.len(), 3); // (0,0) -> (10,0) -> (10,10)
}

#[test]
fn path_search_no_path() {
    let mut graph = VisibilityGraph::new();
    graph.add_vertex(Point::new(0.0, 0.0));
    graph.add_vertex(Point::new(10.0, 0.0));
    // No edges
    let search = PathSearch::new(4.0);
    assert!(search.find_path(&mut graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0)).is_none());
}

#[test]
fn path_search_multi_hop() {
    let mut graph = VisibilityGraph::new();
    let v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let v2 = graph.add_vertex(Point::new(5.0, 0.0));
    let v3 = graph.add_vertex(Point::new(10.0, 0.0));
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);
    graph.add_edge(v2, v3, 1.0);
    graph.add_edge(v3, v2, 1.0);

    let search = PathSearch::new(4.0);
    let path = search.find_path(&mut graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0));
    assert!(path.is_some());
    // Should skip intermediate collinear point (5,0)
    let pts = path.unwrap();
    assert_eq!(pts.len(), 2); // (0,0) -> (10,0) with collinear skip
}

#[test]
fn path_search_same_source_target() {
    let mut graph = VisibilityGraph::new();
    graph.add_vertex(Point::new(5.0, 5.0));

    let search = PathSearch::new(4.0);
    let path = search.find_path(&mut graph, Point::new(5.0, 5.0), Point::new(5.0, 5.0));
    assert!(path.is_some());
    assert_eq!(path.unwrap().len(), 1);
}

#[test]
fn path_search_prefers_fewer_bends() {
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(5.0, 0.0));
    let c = graph.add_vertex(Point::new(5.0, 5.0));
    let d = graph.add_vertex(Point::new(10.0, 5.0));
    graph.add_edge(a, b, 5.0);
    graph.add_edge(b, a, 5.0);
    graph.add_edge(b, c, 5.0);
    graph.add_edge(c, b, 5.0);
    graph.add_edge(c, d, 5.0);
    graph.add_edge(d, c, 5.0);

    let e = graph.add_vertex(Point::new(10.0, 0.0));
    graph.add_edge(a, e, 10.0);
    graph.add_edge(e, a, 10.0);
    graph.add_edge(e, d, 5.0);
    graph.add_edge(d, e, 5.0);

    let search = PathSearch::new(4.0);
    let path = search.find_path(
        &mut graph,
        Point::new(0.0, 0.0),
        Point::new(10.0, 5.0),
    );
    assert!(path.is_some());
    let pts = path.unwrap();
    assert!(pts.len() <= 3, "Expected path with fewer bends, got {:?}", pts);
}

// -----------------------------------------------------------------------
// Unit tests for cost computation and helpers
// -----------------------------------------------------------------------

#[test]
fn bend_penalty_scales_with_distance() {
    let search = PathSearch::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE);
    let cost_near = search.compute_cost(50.0, 1, 100.0);
    let cost_far  = search.compute_cost(50.0, 1, 1000.0);
    assert!(cost_far > cost_near);
}

#[test]
fn zero_bends_cost_equals_length() {
    let search = PathSearch::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE);
    let cost = search.compute_cost(123.0, 0, 500.0);
    assert!((cost - 123.0).abs() < 1e-10);
}

#[test]
fn compute_cost_matches_formula() {
    let search = PathSearch::new(4.0);
    let cost = search.compute_cost(50.0, 2, 200.0);
    assert!((cost - 66.0).abs() < 1e-10);
}

#[test]
fn manhattan_distance_axis_aligned() {
    let a = Point::new(0.0, 0.0);
    let b = Point::new(3.0, 4.0);
    assert!((manhattan_distance(a, b) - 7.0).abs() < 1e-10);
}

#[test]
fn estimated_bends_already_aligned() {
    let pt = Point::new(0.0, 0.0);
    let tgt = Point::new(10.0, 0.0);
    assert_eq!(estimated_bends_to_target(CompassDirection::East, pt, tgt), 0);
}

#[test]
fn estimated_bends_needs_turn() {
    let pt = Point::new(0.0, 0.0);
    let tgt = Point::new(5.0, 5.0);
    assert_eq!(estimated_bends_to_target(CompassDirection::East, pt, tgt), 1);
}

// -----------------------------------------------------------------------
// Tests for the SsstRectilinearPath direction-aware search
// -----------------------------------------------------------------------

use msagl_rust::routing::path_search::SsstRectilinearPath;

#[test]
fn ssst_straight_path() {
    let mut graph = VisibilityGraph::new();
    let v0 = graph.add_vertex(Point::new(0.0, 0.0));
    let v1 = graph.add_vertex(Point::new(50.0, 0.0));
    let v2 = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v1, v0, 1.0);
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);

    let mut ssst = SsstRectilinearPath::new();
    ssst.bends_importance = 4.0;
    let result = ssst.get_path_with_cost(
        None, v0, 0.0, None, v2, 0.0, f64::MAX, &mut graph,
    );
    assert!(result.is_some());
    let path = SsstRectilinearPath::restore_path_v(ssst.arena(), result, &graph);
    // Collinear points should be reduced to just start and end
    assert_eq!(path.len(), 2);
    assert_eq!(path[0], Point::new(0.0, 0.0));
    assert_eq!(path[1], Point::new(100.0, 0.0));
}

#[test]
fn ssst_path_with_one_bend() {
    let mut graph = VisibilityGraph::new();
    let v0 = graph.add_vertex(Point::new(0.0, 0.0));
    let v1 = graph.add_vertex(Point::new(50.0, 0.0));
    let v2 = graph.add_vertex(Point::new(50.0, 50.0));
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v1, v0, 1.0);
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v1, 1.0);

    let mut ssst = SsstRectilinearPath::new();
    ssst.bends_importance = 4.0;
    let result = ssst.get_path_with_cost(
        None, v0, 0.0, None, v2, 0.0, f64::MAX, &mut graph,
    );
    assert!(result.is_some());
    let path = SsstRectilinearPath::restore_path_v(ssst.arena(), result, &graph);
    assert_eq!(path.len(), 3);
    assert_eq!(path[0], Point::new(0.0, 0.0));
    assert_eq!(path[1], Point::new(50.0, 0.0));
    assert_eq!(path[2], Point::new(50.0, 50.0));
}

#[test]
fn ssst_no_path_returns_none() {
    let mut graph = VisibilityGraph::new();
    let v0 = graph.add_vertex(Point::new(0.0, 0.0));
    let v1 = graph.add_vertex(Point::new(100.0, 0.0));
    // No edges

    let mut ssst = SsstRectilinearPath::new();
    ssst.bends_importance = 4.0;
    let result = ssst.get_path_with_cost(
        None, v0, 0.0, None, v1, 0.0, f64::MAX, &mut graph,
    );
    assert!(result.is_none());
}

#[test]
fn ssst_upper_bound_prunes_search() {
    let mut graph = VisibilityGraph::new();
    let v0 = graph.add_vertex(Point::new(0.0, 0.0));
    let v1 = graph.add_vertex(Point::new(100.0, 0.0));
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v1, v0, 1.0);

    let mut ssst = SsstRectilinearPath::new();
    ssst.bends_importance = 1.0;
    // Set upper bound too low for the path cost
    let result = ssst.get_path_with_cost(
        None, v0, 0.0, None, v1, 0.0, 50.0, &mut graph,
    );
    assert!(result.is_none());
}

#[test]
fn ssst_vertex_entries_cleaned_up_after_search() {
    let mut graph = VisibilityGraph::new();
    let v0 = graph.add_vertex(Point::new(0.0, 0.0));
    let v1 = graph.add_vertex(Point::new(50.0, 0.0));
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v1, v0, 1.0);

    let mut ssst = SsstRectilinearPath::new();
    ssst.bends_importance = 1.0;
    let _ = ssst.get_path_with_cost(
        None, v0, 0.0, None, v1, 0.0, f64::MAX, &mut graph,
    );

    // After search, vertex entries should be cleaned up
    assert!(!graph.has_vertex_entries(v0));
    assert!(!graph.has_vertex_entries(v1));
}
