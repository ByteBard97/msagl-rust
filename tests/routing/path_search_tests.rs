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
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0));
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
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 10.0));
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
    assert!(search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0)).is_none());
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
    let path = search.find_path(&graph, Point::new(0.0, 0.0), Point::new(10.0, 0.0));
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
    let path = search.find_path(&graph, Point::new(5.0, 5.0), Point::new(5.0, 5.0));
    assert!(path.is_some());
    assert_eq!(path.unwrap().len(), 1);
}

#[test]
fn path_search_prefers_fewer_bends() {
    // Create a graph with two paths: short with 2 bends vs slightly longer with 0 bends.
    // With high bend penalty, the straight path should win.
    let mut graph = VisibilityGraph::new();
    let a = graph.add_vertex(Point::new(0.0, 0.0));
    let b = graph.add_vertex(Point::new(5.0, 0.0));
    let c = graph.add_vertex(Point::new(5.0, 5.0));
    let d = graph.add_vertex(Point::new(10.0, 5.0));
    // Straight-ish path: a -> b -> c -> d (2 bends, length ~20)
    graph.add_edge(a, b, 5.0);
    graph.add_edge(b, a, 5.0);
    graph.add_edge(b, c, 5.0);
    graph.add_edge(c, b, 5.0);
    graph.add_edge(c, d, 5.0);
    graph.add_edge(d, c, 5.0);

    // Also add a long straight path: a -> e -> d
    let e = graph.add_vertex(Point::new(10.0, 0.0));
    graph.add_edge(a, e, 10.0);
    graph.add_edge(e, a, 10.0);
    graph.add_edge(e, d, 5.0);
    graph.add_edge(d, e, 5.0);

    // High bend penalty: should prefer a -> e -> d (1 bend, length 15)
    // over a -> b -> c -> d (2 bends, length 15)
    let search = PathSearch::new(4.0);
    let path = search.find_path(
        &graph,
        Point::new(0.0, 0.0),
        Point::new(10.0, 5.0),
    );
    assert!(path.is_some());
    let pts = path.unwrap();
    // The a -> e -> d path has 1 bend: East then North
    assert!(pts.len() <= 3, "Expected path with fewer bends, got {:?}", pts);
}

// -----------------------------------------------------------------------
// Tests extracted from src/routing/path_search.rs
// -----------------------------------------------------------------------

#[test]
fn bend_penalty_scales_with_distance() {
    let search = PathSearch::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE);

    // distance=100 -> bend_cost = 100 * 4/100 = 4 per bend
    // distance=1000 -> bend_cost = 1000 * 4/100 = 40 per bend
    let cost_near = search.compute_cost(50.0, 1, 100.0);
    let cost_far  = search.compute_cost(50.0, 1, 1000.0);
    assert!(cost_far > cost_near, "far bend cost {cost_far} should exceed near bend cost {cost_near}");
}

#[test]
fn zero_bends_cost_equals_length() {
    let search = PathSearch::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE);
    let cost = search.compute_cost(123.0, 0, 500.0);
    assert!((cost - 123.0).abs() < 1e-10);
}

#[test]
fn compute_cost_matches_formula() {
    // bend_cost per bend = 200 * 4/100 = 8
    // total = 50 + 8 * 2 = 66
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
    // Heading East, target is directly East -> 0 bends.
    let pt = Point::new(0.0, 0.0);
    let tgt = Point::new(10.0, 0.0);
    assert_eq!(estimated_bends_to_target(CompassDirection::East, pt, tgt), 0);
}

#[test]
fn estimated_bends_needs_turn() {
    // Heading East but target is North-East -> 1 bend.
    let pt = Point::new(0.0, 0.0);
    let tgt = Point::new(5.0, 5.0);
    assert_eq!(estimated_bends_to_target(CompassDirection::East, pt, tgt), 1);
}
