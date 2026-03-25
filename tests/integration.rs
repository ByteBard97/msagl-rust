use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

#[test]
fn route_single_edge_between_two_boxes() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 50.0, 30.0),
        Shape::rectangle(200.0, 0.0, 50.0, 30.0),
    ];

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(4.0)
        .edge_separation(8.0);

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(54.0, 15.0)),
        FloatingPort::new(1, Point::new(196.0, 15.0)),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 1);
    assert!(result.edges[0].points.len() >= 2);

    let pts = &result.edges[0].points;
    assert!(
        (pts[0].x() - 54.0).abs() < 10.0,
        "start x should be near 54, got {}",
        pts[0].x()
    );
    assert!(
        (pts.last().unwrap().x() - 196.0).abs() < 10.0,
        "end x should be near 196, got {}",
        pts.last().unwrap().x()
    );
}

#[test]
fn route_two_parallel_edges() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 50.0, 30.0),
        Shape::rectangle(200.0, 0.0, 50.0, 30.0),
    ];

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(4.0)
        .edge_separation(8.0);

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(54.0, 10.0)),
        FloatingPort::new(1, Point::new(196.0, 10.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(54.0, 20.0)),
        FloatingPort::new(1, Point::new(196.0, 20.0)),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 2);
    assert!(result.edges[0].points.len() >= 2);
    assert!(result.edges[1].points.len() >= 2);
}

#[test]
fn route_around_obstacle() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 30.0, 30.0),
        Shape::rectangle(60.0, 0.0, 30.0, 60.0),
        Shape::rectangle(120.0, 0.0, 30.0, 30.0),
    ];

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(4.0)
        .edge_separation(8.0);

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(34.0, 15.0)),
        FloatingPort::new(2, Point::new(116.0, 15.0)),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 1);
    let pts = &result.edges[0].points;
    assert!(pts.len() >= 2, "path should exist");
}

#[test]
fn route_no_edges() {
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let mut router = RectilinearEdgeRouter::new(&shapes).padding(2.0);
    let result = router.run();
    assert!(result.edges.is_empty());
}

#[test]
fn route_with_curve_output() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 50.0, 30.0),
        Shape::rectangle(200.0, 0.0, 50.0, 30.0),
    ];

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(4.0)
        .edge_separation(8.0)
        .corner_fit_radius(3.0);

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(54.0, 15.0)),
        FloatingPort::new(1, Point::new(196.0, 15.0)),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 1);
    assert!(result.edges[0].curve.segment_count() > 0);
}

#[test]
fn route_returns_valid_curve_endpoints() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 40.0, 40.0),
        Shape::rectangle(100.0, 100.0, 40.0, 40.0),
    ];

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(4.0)
        .edge_separation(8.0);

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(44.0, 20.0)),
        FloatingPort::new(1, Point::new(96.0, 120.0)),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 1);

    let edge = &result.edges[0];
    assert!(edge.curve.segment_count() > 0);

    // Curve should start near source and end near target
    let start = edge.curve.start();
    let end = edge.curve.end();
    assert!(
        (start.x() - edge.points[0].x()).abs() < 1e-9,
        "curve start should match first point"
    );
    assert!(
        (end.x() - edge.points.last().unwrap().x()).abs() < 1e-9,
        "curve end should match last point"
    );
}
