use msagl_rust::geometry::point::Point;
use msagl_rust::routing::edge_geometry::EdgeGeometry;
use msagl_rust::routing::port::FloatingPort;
use msagl_rust::routing::rectilinear_edge_router::RectilinearEdgeRouter;
use msagl_rust::routing::shape::Shape;

#[test]
fn smoke_route_one_edge_between_two_boxes() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::rectangle(300.0, 0.0, 100.0, 50.0),
    ];
    let mut router = RectilinearEdgeRouter::new(&shapes);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(50.0, 25.0)),
        FloatingPort::new(1, Point::new(350.0, 25.0)),
    ));
    let result = router.padding(4.0).edge_separation(8.0).run();
    assert_eq!(result.edges.len(), 1, "should route exactly 1 edge");
    let path = &result.edges[0].points;
    assert!(
        path.len() >= 2,
        "path should have at least 2 points, got {}",
        path.len()
    );
    // Verify all segments are axis-aligned (rectilinear)
    for w in path.windows(2) {
        assert!(
            (w[0].x() - w[1].x()).abs() < 1e-6 || (w[0].y() - w[1].y()).abs() < 1e-6,
            "non-rectilinear segment: ({}, {}) -> ({}, {})",
            w[0].x(),
            w[0].y(),
            w[1].x(),
            w[1].y()
        );
    }
}

#[test]
fn smoke_route_three_edges_four_boxes() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 80.0, 40.0),
        Shape::rectangle(200.0, 0.0, 80.0, 40.0),
        Shape::rectangle(0.0, 150.0, 80.0, 40.0),
        Shape::rectangle(200.0, 150.0, 80.0, 40.0),
    ];
    let mut router = RectilinearEdgeRouter::new(&shapes);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(40.0, 20.0)),
        FloatingPort::new(1, Point::new(240.0, 20.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(2, Point::new(40.0, 170.0)),
        FloatingPort::new(3, Point::new(240.0, 170.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(40.0, 20.0)),
        FloatingPort::new(3, Point::new(240.0, 170.0)),
    ));
    let result = router.padding(4.0).edge_separation(8.0).run();
    assert_eq!(result.edges.len(), 3, "should route 3 edges");
    for (i, edge) in result.edges.iter().enumerate() {
        assert!(edge.points.len() >= 2, "edge {} has less than 2 points", i);
        for w in edge.points.windows(2) {
            assert!(
                (w[0].x() - w[1].x()).abs() < 1e-6 || (w[0].y() - w[1].y()).abs() < 1e-6,
                "edge {} has non-rectilinear segment",
                i
            );
        }
    }
}

#[test]
fn smoke_single_edge_no_obstacles_between() {
    // Two boxes with no obstacle between them
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 50.0, 50.0),
        Shape::rectangle(200.0, 0.0, 50.0, 50.0),
    ];
    let mut router = RectilinearEdgeRouter::new(&shapes);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(25.0, 25.0)),
        FloatingPort::new(1, Point::new(225.0, 25.0)),
    ));
    let result = router.padding(4.0).run();
    assert_eq!(result.edges.len(), 1);
    // Path should exist and be reasonable
    let path = &result.edges[0].points;
    assert!(path.len() >= 2);
}
