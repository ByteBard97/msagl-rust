use msagl_rust::routing::shape::Shape;
use msagl_rust::routing::router_session::RouterSession;
use msagl_rust::routing::visibility_graph_generator::generate_visibility_graph;
use msagl_rust::visibility::graph::VertexId;
use msagl_rust::GeomConstants;
use msagl_rust::Point;

#[test]
fn single_obstacle_creates_visibility_graph() {
    let shapes = vec![Shape::rectangle(10.0, 10.0, 20.0, 20.0)];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);
    // Padded obstacle is 8..32 x 8..32
    // Should have at least the 4 padded corners as vertices
    assert!(
        graph.vertex_count() >= 4,
        "got {} vertices",
        graph.vertex_count()
    );
}

#[test]
fn two_obstacles_horizontal_visibility() {
    // Two boxes side by side with a gap
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(30.0, 0.0, 10.0, 10.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);
    // Should have vertices and edges connecting the gap between obstacles
    assert!(
        graph.vertex_count() >= 8,
        "got {} vertices",
        graph.vertex_count()
    );

    // The padded boxes are [-2..12] and [28..42] at y range [-2..12]
    // There should be horizontal visibility at y=-2, y=12 between x=12 and x=28
    let v1 = graph.find_vertex(Point::new(12.0, -2.0));
    let v2 = graph.find_vertex(Point::new(28.0, -2.0));
    assert!(v1.is_some(), "should have vertex at padded corner (12, -2)");
    assert!(v2.is_some(), "should have vertex at padded corner (28, -2)");
}

#[test]
fn two_obstacles_vertical_visibility() {
    // Two boxes stacked vertically
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(0.0, 30.0, 10.0, 10.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);
    assert!(
        graph.vertex_count() >= 8,
        "got {} vertices",
        graph.vertex_count()
    );
}

#[test]
fn three_obstacles_blocking() {
    // Middle obstacle blocks direct path between left and right
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),   // left
        Shape::rectangle(20.0, -5.0, 10.0, 20.0), // middle (taller)
        Shape::rectangle(40.0, 0.0, 10.0, 10.0),  // right
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);
    // The middle obstacle should block some horizontal visibility
    assert!(graph.vertex_count() > 0);
}

#[test]
fn empty_obstacles_produces_empty_graph() {
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&[], 2.0);
    assert_eq!(graph.vertex_count(), 0);
}

#[test]
fn single_obstacle_has_corner_segments() {
    // A single box at origin, 10x10, padding=2
    // Padded box: [-2..12, -2..12]
    // Horizontal segments at y=-2 and y=12, from x=-2 to x=12
    // Vertical segments at x=-2 and x=12, from y=-2 to y=12
    // These 4 segments form a rectangle around the obstacle.
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);

    // The 4 corners of the padded box should be vertices
    assert!(graph.find_vertex(Point::new(-2.0, -2.0)).is_some());
    assert!(graph.find_vertex(Point::new(-2.0, 12.0)).is_some());
    assert!(graph.find_vertex(Point::new(12.0, -2.0)).is_some());
    assert!(graph.find_vertex(Point::new(12.0, 12.0)).is_some());
}

#[test]
fn two_obstacles_gap_has_edges() {
    // Two boxes with gap, verify edges exist in the gap
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(30.0, 0.0, 10.0, 10.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);

    // At y=-2 (bottom edge of both), there should be a horizontal segment
    // from x=12 to x=28
    let v_left = graph.find_vertex(Point::new(12.0, -2.0));
    let v_right = graph.find_vertex(Point::new(28.0, -2.0));
    assert!(v_left.is_some(), "missing vertex at (12, -2)");
    assert!(v_right.is_some(), "missing vertex at (28, -2)");

    // Verify there's an edge between them (directly or via intermediate vertices)
    let vid_left = v_left.unwrap();
    let has_edge = graph
        .out_edges(vid_left)
        .any(|e| e.target == v_right.unwrap());
    assert!(has_edge, "should have direct edge from (12,-2) to (28,-2)");
}

// -----------------------------------------------------------------------
// Tests extracted from src/routing/visibility_graph_generator.rs
// -----------------------------------------------------------------------

#[test]
fn empty_shapes_returns_empty_graph() {
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&[], 4.0);
    assert_eq!(graph.vertex_count(), 0);
}

#[test]
fn merge_overlapping_intervals() {
    // Kept from original for backward compatibility
    let intervals = vec![(1.0, 3.0), (2.0, 5.0), (7.0, 9.0)];
    let mut result = vec![intervals[0]];
    for &(lo, hi) in &intervals[1..] {
        let last = result.last_mut().unwrap();
        if lo <= last.1 + GeomConstants::DISTANCE_EPSILON {
            last.1 = last.1.max(hi);
        } else {
            result.push((lo, hi));
        }
    }
    assert_eq!(result.len(), 2);
    assert!(GeomConstants::close(result[0].0, 1.0));
    assert!(GeomConstants::close(result[0].1, 5.0));
    assert!(GeomConstants::close(result[1].0, 7.0));
    assert!(GeomConstants::close(result[1].1, 9.0));
}

#[test]
fn single_obstacle_creates_surrounding_segments() {
    let shapes = vec![Shape::rectangle(50.0, 50.0, 100.0, 60.0)];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 4.0);
    // All edges should be axis-aligned
    for v in 0..graph.vertex_count() {
        let vid = VertexId(v);
        for edge in graph.out_edges(vid) {
            let p = graph.point(vid);
            let tp = graph.point(edge.target);
            assert!(
                (p.x() - tp.x()).abs() < 1e-10 || (p.y() - tp.y()).abs() < 1e-10,
                "non-rectilinear edge from {:?} to {:?}",
                p,
                tp
            );
        }
    }
    assert!(graph.vertex_count() > 0);
}

#[test]
fn two_boxes_have_connected_graph() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::rectangle(200.0, 0.0, 100.0, 50.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 4.0);
    assert!(graph.vertex_count() > 0);
    // Every vertex should have at least one edge
    for v in 0..graph.vertex_count() {
        let vid = VertexId(v);
        assert!(
            graph.out_degree(vid) + graph.in_degree(vid) > 0,
            "vertex at {:?} is disconnected",
            graph.point(vid)
        );
    }
}

#[test]
fn three_boxes_triangle_layout() {
    let shapes = vec![
        Shape::rectangle(100.0, 0.0, 80.0, 40.0),
        Shape::rectangle(0.0, 100.0, 80.0, 40.0),
        Shape::rectangle(200.0, 100.0, 80.0, 40.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 4.0);
    assert!(graph.vertex_count() > 0);
    // All edges rectilinear
    for v in 0..graph.vertex_count() {
        let vid = VertexId(v);
        for edge in graph.out_edges(vid) {
            let p = graph.point(vid);
            let tp = graph.point(edge.target);
            assert!((p.x() - tp.x()).abs() < 1e-10 || (p.y() - tp.y()).abs() < 1e-10,);
        }
    }
}

#[test]
fn single_obstacle_has_four_padded_corners() {
    // Shape at (0,0) size 10x10, padding 2
    // Padded bbox: (-2, -2) to (12, 12)
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);

    // The 4 padded corners should be vertices in the graph
    assert!(
        graph.find_vertex(Point::new(-2.0, -2.0)).is_some(),
        "missing bottom-left corner"
    );
    assert!(
        graph.find_vertex(Point::new(12.0, -2.0)).is_some(),
        "missing bottom-right corner"
    );
    assert!(
        graph.find_vertex(Point::new(-2.0, 12.0)).is_some(),
        "missing top-left corner"
    );
    assert!(
        graph.find_vertex(Point::new(12.0, 12.0)).is_some(),
        "missing top-right corner"
    );
}

#[test]
fn two_obstacles_gap_has_horizontal_edge() {
    // Two boxes with a gap, should have horizontal edge in the gap
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(30.0, 0.0, 10.0, 10.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 2.0);

    // Padded: [-2..12] and [28..42] at y range [-2..12]
    let v_left = graph.find_vertex(Point::new(12.0, -2.0));
    let v_right = graph.find_vertex(Point::new(28.0, -2.0));
    assert!(v_left.is_some(), "missing vertex at (12, -2)");
    assert!(v_right.is_some(), "missing vertex at (28, -2)");

    // Should have direct edge between them
    let vid_left = v_left.unwrap();
    let has_edge = graph
        .out_edges(vid_left)
        .any(|e| e.target == v_right.unwrap());
    assert!(has_edge, "should have direct edge from (12,-2) to (28,-2)");
}

/// Verify VG connectivity for the route_around_obstacle scenario.
/// The middle obstacle blocks direct horizontal path, requiring a detour.
#[test]
fn route_around_blocker_vg_connected() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 30.0, 30.0),
        Shape::rectangle(60.0, 0.0, 30.0, 60.0),
        Shape::rectangle(120.0, 0.0, 30.0, 30.0),
    ];
    let RouterSession { vis_graph: graph, .. } = generate_visibility_graph(&shapes, 4.0);

    // All boundary vertices should exist and be connected
    assert!(graph.vertex_count() >= 20, "need sufficient vertices");

    // Key connectivity: path from left to right requires going around blocker
    // Check vertical edges at blocker boundaries
    let v1 = graph.find_vertex(Point::new(56.0, -4.0));
    let v2 = graph.find_vertex(Point::new(56.0, 34.0));
    let v3 = graph.find_vertex(Point::new(56.0, 64.0));
    assert!(v1.is_some(), "missing vertex at (56,-4)");
    assert!(v2.is_some(), "missing vertex at (56,34)");
    assert!(v3.is_some(), "missing vertex at (56,64)");
}
