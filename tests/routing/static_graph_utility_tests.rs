use msagl_rust::routing::static_graph_utility::StaticGraphUtility;
use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::visibility::graph::VisibilityGraph;
use msagl_rust::geometry::rectangle::Rectangle;
use msagl_rust::Point;

// -----------------------------------------------------------------------
// edge_direction
// -----------------------------------------------------------------------

#[test]
fn edge_direction_horizontal() {
    let src = Point::new(0.0, 0.0);
    assert_eq!(
        StaticGraphUtility::edge_direction(src, Point::new(10.0, 0.0)),
        Some(CompassDirection::East),
    );
    assert_eq!(
        StaticGraphUtility::edge_direction(src, Point::new(-5.0, 0.0)),
        Some(CompassDirection::West),
    );
}

#[test]
fn edge_direction_vertical() {
    let src = Point::new(0.0, 0.0);
    assert_eq!(
        StaticGraphUtility::edge_direction(src, Point::new(0.0, 7.0)),
        Some(CompassDirection::North),
    );
    assert_eq!(
        StaticGraphUtility::edge_direction(src, Point::new(0.0, -3.0)),
        Some(CompassDirection::South),
    );
}

#[test]
fn edge_direction_identical_points() {
    let p = Point::new(1.0, 1.0);
    assert_eq!(StaticGraphUtility::edge_direction(p, p), None);
}

// -----------------------------------------------------------------------
// find_adjacent_vertex
// -----------------------------------------------------------------------

#[test]
fn find_adjacent_vertex_via_out_edge() {
    let mut g = VisibilityGraph::new();
    let a = g.add_vertex(Point::new(0.0, 0.0));
    let b = g.add_vertex(Point::new(10.0, 0.0));
    g.add_edge(a, b, 10.0);

    assert_eq!(
        StaticGraphUtility::find_adjacent_vertex(&g, a, CompassDirection::East),
        Some(b),
    );
    assert_eq!(
        StaticGraphUtility::find_adjacent_vertex(&g, a, CompassDirection::North),
        None,
    );
}

#[test]
fn find_adjacent_vertex_via_in_edge() {
    // `b` has `a` as an in-edge; traversing West from `b` should find `a`.
    let mut g = VisibilityGraph::new();
    let a = g.add_vertex(Point::new(0.0, 0.0));
    let b = g.add_vertex(Point::new(10.0, 0.0));
    g.add_edge(a, b, 10.0);

    assert_eq!(
        StaticGraphUtility::find_adjacent_vertex(&g, b, CompassDirection::West),
        Some(a),
    );
}

// -----------------------------------------------------------------------
// point_is_in_rectangle_interior
// -----------------------------------------------------------------------

#[test]
fn point_in_rectangle_interior_true() {
    let rect = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(StaticGraphUtility::point_is_in_rectangle_interior(
        Point::new(5.0, 5.0),
        &rect,
    ));
}

#[test]
fn point_on_rectangle_boundary_false() {
    let rect = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    // On the left edge -- not strictly interior.
    assert!(!StaticGraphUtility::point_is_in_rectangle_interior(
        Point::new(0.0, 5.0),
        &rect,
    ));
}

#[test]
fn point_outside_rectangle_false() {
    let rect = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(!StaticGraphUtility::point_is_in_rectangle_interior(
        Point::new(15.0, 5.0),
        &rect,
    ));
}

// -----------------------------------------------------------------------
// is_collinear
// -----------------------------------------------------------------------

#[test]
fn is_collinear_same_x() {
    assert!(StaticGraphUtility::is_collinear(
        Point::new(3.0, 0.0),
        Point::new(3.0, 10.0),
    ));
}

#[test]
fn is_collinear_same_y() {
    assert!(StaticGraphUtility::is_collinear(
        Point::new(0.0, 5.0),
        Point::new(8.0, 5.0),
    ));
}

#[test]
fn is_not_collinear_diagonal() {
    assert!(!StaticGraphUtility::is_collinear(
        Point::new(0.0, 0.0),
        Point::new(1.0, 1.0),
    ));
}
