use msagl_rust::routing::obstacle_tree::ObstacleTree;
use msagl_rust::routing::path_search::PathSearch;
use msagl_rust::routing::port_manager::PortManager;
use msagl_rust::routing::shape::Shape;
use msagl_rust::routing::router_session::RouterSession;
use msagl_rust::routing::visibility_graph_generator::generate_visibility_graph;
use msagl_rust::visibility::graph::VisibilityGraph;
use msagl_rust::Point;

#[test]
fn splice_adds_edges_to_aligned_vertices() {
    let mut _obstacle_tree = ObstacleTree::empty();
    let mut graph = VisibilityGraph::new();
    let _v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let _v2 = graph.add_vertex(Point::new(10.0, 0.0));
    let _v3 = graph.add_vertex(Point::new(0.0, 10.0));
    graph.add_edge(_v1, _v2, 10.0);
    graph.add_edge(_v1, _v3, 10.0);

    // Splice a point that is horizontally aligned with v1 and v2
    let splice = PortManager::splice_port(&mut graph, &mut _obstacle_tree, Point::new(5.0, 0.0));
    let port_id = splice.port_vertex;

    // Should have added edges to v1 (west, dist=5) and v2 (east, dist=5)
    assert!(graph.out_degree(port_id) > 0);
}

#[test]
fn unsplice_removes_added_edges() {
    let mut _obstacle_tree = ObstacleTree::empty();
    let mut graph = VisibilityGraph::new();
    let _v1 = graph.add_vertex(Point::new(0.0, 0.0));
    let _v2 = graph.add_vertex(Point::new(10.0, 0.0));

    let mut splice = PortManager::splice_port(&mut graph, &mut _obstacle_tree, Point::new(5.0, 0.0));
    let port_id = splice.port_vertex;

    assert!(graph.out_degree(port_id) > 0);

    PortManager::unsplice(&mut graph, &mut splice);
    assert_eq!(graph.out_degree(port_id), 0);
}

#[test]
fn splice_and_route_between_obstacles() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(30.0, 0.0, 10.0, 10.0),
    ];
    let RouterSession { vis_graph: mut graph, obstacle_tree: mut _obstacle_tree, .. } = generate_visibility_graph(&shapes, 2.0);

    // Splice source port (right of obstacle 0) and target (left of obstacle 1)
    let source = Point::new(12.0, 5.0);
    let target = Point::new(28.0, 5.0);

    let mut src_splice = PortManager::splice_port(&mut graph, &mut _obstacle_tree, source);
    let mut tgt_splice = PortManager::splice_port(&mut graph, &mut _obstacle_tree, target);

    let search = PathSearch::new(4.0);
    let path = search.find_path(&graph, source, target);
    assert!(path.is_some(), "should find path between spliced ports");

    let pts = path.unwrap();
    assert_eq!(pts[0], source);
    assert_eq!(*pts.last().unwrap(), target);

    // Cleanup
    PortManager::unsplice(&mut graph, &mut src_splice);
    PortManager::unsplice(&mut graph, &mut tgt_splice);
}

#[test]
fn full_pipeline_shapes_to_path() {
    // Full integration: shapes -> visibility graph -> splice ports -> find path
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0), // obstacle at (0,0)-(10,10)
        Shape::rectangle(20.0, 0.0, 10.0, 10.0), // obstacle at (20,0)-(30,10)
        Shape::rectangle(10.0, 20.0, 10.0, 10.0), // obstacle at (10,20)-(20,30)
    ];
    let padding = 2.0;
    let RouterSession { vis_graph: mut graph, obstacle_tree: mut _obstacle_tree, .. } = generate_visibility_graph(&shapes, padding);

    // Route from right of obstacle 0 to left of obstacle 1
    let source = Point::new(12.0, 5.0);
    let target = Point::new(18.0, 5.0);

    let mut src_splice = PortManager::splice_port(&mut graph, &mut _obstacle_tree, source);
    let mut tgt_splice = PortManager::splice_port(&mut graph, &mut _obstacle_tree, target);

    let search = PathSearch::new(4.0);
    let path = search.find_path(&graph, source, target);
    assert!(path.is_some(), "full pipeline should find a path");

    let pts = path.unwrap();
    assert_eq!(pts[0], source, "path should start at source");
    assert_eq!(*pts.last().unwrap(), target, "path should end at target");
    assert!(pts.len() >= 2, "path should have at least 2 points");

    // Cleanup
    PortManager::unsplice(&mut graph, &mut src_splice);
    PortManager::unsplice(&mut graph, &mut tgt_splice);
}
