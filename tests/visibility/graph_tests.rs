use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::routing::vertex_entry::VertexEntryIndex;
use msagl_rust::visibility::graph::{VertexId, VisibilityGraph};
use msagl_rust::Point;

#[test]
fn add_and_find_vertex() {
    let mut g = VisibilityGraph::new();
    let v = g.add_vertex(Point::new(1.0, 2.0));
    assert_eq!(g.find_vertex(Point::new(1.0, 2.0)), Some(v));
    assert_eq!(g.find_vertex(Point::new(3.0, 4.0)), None);
}

#[test]
fn add_edge() {
    let mut g = VisibilityGraph::new();
    let v1 = g.add_vertex(Point::new(0.0, 0.0));
    let v2 = g.add_vertex(Point::new(5.0, 0.0));
    g.add_edge(v1, v2, 1.0);
    assert_eq!(g.out_degree(v1), 1);
    assert_eq!(g.in_degree(v2), 1);
}

#[test]
fn find_or_add_vertex_idempotent() {
    let mut g = VisibilityGraph::new();
    let v1 = g.find_or_add_vertex(Point::new(1.0, 2.0));
    let v2 = g.find_or_add_vertex(Point::new(1.0, 2.0));
    assert_eq!(v1, v2);
    assert_eq!(g.vertex_count(), 1);
}

#[test]
fn remove_edge() {
    let mut g = VisibilityGraph::new();
    let v1 = g.add_vertex(Point::new(0.0, 0.0));
    let v2 = g.add_vertex(Point::new(5.0, 0.0));
    g.add_edge(v1, v2, 1.0);
    g.remove_edge(v1, v2);
    assert_eq!(g.out_degree(v1), 0);
}

#[test]
fn vertex_point() {
    let mut g = VisibilityGraph::new();
    let v = g.add_vertex(Point::new(3.0, 4.0));
    assert_eq!(g.point(v), Point::new(3.0, 4.0));
}

#[test]
fn out_edges_iteration() {
    let mut g = VisibilityGraph::new();
    let v1 = g.add_vertex(Point::new(0.0, 0.0));
    let v2 = g.add_vertex(Point::new(5.0, 0.0));
    let v3 = g.add_vertex(Point::new(0.0, 5.0));
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v1, v3, 1.0);
    let targets: Vec<VertexId> = g.out_edges(v1).map(|e| e.target).collect();
    assert_eq!(targets.len(), 2);
}

#[test]
fn vertex_entries_default_to_none() {
    let mut graph = VisibilityGraph::new();
    let v = graph.add_vertex(Point::new(10.0, 20.0));
    for dir in CompassDirection::all() {
        assert!(graph.vertex_entry(v, dir).is_none());
    }
}

#[test]
fn set_and_get_vertex_entry() {
    let mut graph = VisibilityGraph::new();
    let v = graph.add_vertex(Point::new(10.0, 20.0));
    graph.set_vertex_entry(v, CompassDirection::East, Some(VertexEntryIndex(42)));
    assert_eq!(
        graph.vertex_entry(v, CompassDirection::East),
        Some(VertexEntryIndex(42))
    );
    assert!(graph.vertex_entry(v, CompassDirection::North).is_none());
}

#[test]
fn clear_vertex_entries_resets_all() {
    let mut graph = VisibilityGraph::new();
    let v = graph.add_vertex(Point::new(10.0, 20.0));
    graph.set_vertex_entry(v, CompassDirection::East, Some(VertexEntryIndex(1)));
    graph.set_vertex_entry(v, CompassDirection::North, Some(VertexEntryIndex(2)));
    graph.clear_vertex_entries();
    for dir in CompassDirection::all() {
        assert!(graph.vertex_entry(v, dir).is_none());
    }
}
