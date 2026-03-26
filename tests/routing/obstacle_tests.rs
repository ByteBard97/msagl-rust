use msagl_rust::routing::obstacle::Obstacle;
use msagl_rust::routing::obstacle_side::SideType;
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::routing::shape::Shape;
use msagl_rust::Point;

#[test]
fn shape_rectangle_creation() {
    let s = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let bb = s.bounding_box();
    assert_eq!(bb.left(), 10.0);
    assert_eq!(bb.right(), 110.0);
    assert_eq!(bb.bottom(), 20.0);
    assert_eq!(bb.top(), 70.0);
}

#[test]
fn shape_rectangle_centered() {
    let s = Shape::rectangle_centered(50.0, 50.0, 20.0, 10.0);
    let bb = s.bounding_box();
    assert_eq!(bb.left(), 40.0);
    assert_eq!(bb.right(), 60.0);
    assert_eq!(bb.bottom(), 45.0);
    assert_eq!(bb.top(), 55.0);
}

#[test]
fn obstacle_from_shape_with_padding() {
    let s = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&s, 4.0, 0);
    let bb = obs.padded_bounding_box();
    assert_eq!(bb.left(), 6.0);
    assert_eq!(bb.right(), 114.0);
    assert_eq!(bb.bottom(), 16.0);
    assert_eq!(bb.top(), 74.0);
}

#[test]
fn obstacle_padded_polyline_has_four_points() {
    let s = Shape::rectangle(0.0, 0.0, 10.0, 10.0);
    let obs = Obstacle::from_shape(&s, 2.0, 0);
    assert_eq!(obs.padded_polyline().count(), 4);
    assert!(obs.padded_polyline().is_closed());
}

#[test]
fn obstacle_padded_corners() {
    let s = Shape::rectangle(0.0, 0.0, 10.0, 10.0);
    let obs = Obstacle::from_shape(&s, 2.0, 0);
    let corners = obs.padded_corners();
    assert_eq!(corners[0], Point::new(-2.0, -2.0)); // left_bottom
    assert_eq!(corners[1], Point::new(-2.0, 12.0)); // left_top
    assert_eq!(corners[2], Point::new(12.0, 12.0)); // right_top
    assert_eq!(corners[3], Point::new(12.0, -2.0)); // right_bottom
}

// --- Tests extracted from inline #[cfg(test)] in obstacle.rs ---

#[test]
fn obstacle_gets_ordinal() {
    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&shape, 4.0, 3);
    assert_eq!(obs.ordinal(), Obstacle::FIRST_NON_SENTINEL_ORDINAL + 3);
}

#[test]
fn obstacle_initially_has_no_active_sides() {
    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&shape, 4.0, 0);
    assert!(obs.active_low_side().is_none());
    assert!(obs.active_high_side().is_none());
}

#[test]
fn create_sides_horizontal_scan() {
    let shape = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let mut obs = Obstacle::from_shape(&shape, 4.0, 0);
    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    obs.create_initial_sides(open_key, scan_dir);
    let low = obs.active_low_side().unwrap();
    let high = obs.active_high_side().unwrap();
    // Horizontal scan: low side is left (low X), high side is right (high X)
    assert_eq!(low.side_type(), SideType::Low);
    assert_eq!(high.side_type(), SideType::High);
    // Low side should be at left edge of padded box (10 - 4 = 6)
    assert!((low.start().x() - 6.0).abs() < 1e-6);
    // High side should be at right edge (10 + 100 + 4 = 114)
    assert!((high.start().x() - 114.0).abs() < 1e-6);
}

#[test]
fn create_sides_vertical_scan() {
    let shape = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let mut obs = Obstacle::from_shape(&shape, 4.0, 0);
    let scan_dir = ScanDirection::vertical();
    let open_key = obs.get_open_vertex(scan_dir);
    obs.create_initial_sides(open_key, scan_dir);
    let low = obs.active_low_side().unwrap();
    let high = obs.active_high_side().unwrap();
    // Vertical scan: low side is bottom (low Y), high side is top (high Y)
    assert_eq!(low.side_type(), SideType::Low);
    assert_eq!(high.side_type(), SideType::High);
    // Low side should be at bottom edge (20 - 4 = 16)
    assert!((low.start().y() - 16.0).abs() < 1e-6);
    // High side should be at top edge (20 + 50 + 4 = 74)
    assert!((high.start().y() - 74.0).abs() < 1e-6);
}

#[test]
fn sentinel_creation() {
    let sentinel = Obstacle::create_sentinel(
        Point::new(-100.0, -100.0),
        Point::new(100.0, -100.0),
        Obstacle::FIRST_SENTINEL_ORDINAL,
    );
    assert!(sentinel.is_sentinel());
    assert_eq!(sentinel.ordinal(), Obstacle::FIRST_SENTINEL_ORDINAL);
}

#[test]
fn from_shape_preserves_existing_api() {
    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&shape, 4.0, 0);
    // Existing API must still work
    assert_eq!(obs.index, 0);
    let bb = obs.padded_bounding_box();
    assert!((bb.left() - (-4.0)).abs() < 1e-6);
    assert!((bb.right() - 104.0).abs() < 1e-6);
    let corners = obs.padded_corners();
    assert_eq!(corners.len(), 4);
}

// Task 5: ObstacleTree tests
use msagl_rust::routing::obstacle_tree::ObstacleTree;
use msagl_rust::Rectangle;

#[test]
fn obstacle_tree_query_point() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(20.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 4.0);
    let hits = tree.query_point(Point::new(5.0, 5.0));
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0], 0);
}

#[test]
fn obstacle_tree_query_rect() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(20.0, 0.0, 10.0, 10.0),
        Shape::rectangle(50.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 4.0);
    let hits = tree.query_rect(&Rectangle::new(-5.0, -5.0, 35.0, 15.0));
    assert_eq!(hits.len(), 2);
}

#[test]
fn obstacle_tree_no_hits() {
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let tree = ObstacleTree::new(&shapes, 2.0);
    let hits = tree.query_point(Point::new(100.0, 100.0));
    assert!(hits.is_empty());
}

// --- Task 1: Shape polyline boundary tests ---

#[test]
fn shape_from_polyline_has_correct_bounding_box() {
    use msagl_rust::geometry::polyline::Polyline;

    // L-shaped polyline (6 vertices)
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(100.0, 0.0));
    poly.add_point(Point::new(100.0, 50.0));
    poly.add_point(Point::new(50.0, 50.0));
    poly.add_point(Point::new(50.0, 100.0));
    poly.add_point(Point::new(0.0, 100.0));
    poly.set_closed(true);

    let shape = Shape::from_polyline(poly);
    let bb = shape.bounding_box();
    assert!((bb.left() - 0.0).abs() < 1e-10);
    assert!((bb.bottom() - 0.0).abs() < 1e-10);
    assert!((bb.right() - 100.0).abs() < 1e-10);
    assert!((bb.top() - 100.0).abs() < 1e-10);
}

#[test]
fn shape_rectangle_creates_4_point_polyline() {
    let shape = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let poly = shape.boundary_polyline();
    assert_eq!(poly.count(), 4);
    assert!(poly.is_closed());
}

// --- Task 3: Obstacle IsRectangle, GetOpenVertex, Close tests ---

#[test]
fn obstacle_is_rectangle_for_rect_shape() {
    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&shape, 5.0, 0);
    assert!(obs.is_rectangle());
    assert_eq!(obs.padded_polyline().count(), 4);
}

#[test]
fn obstacle_get_open_vertex_finds_lowest() {
    let shape = Shape::rectangle(10.0, 20.0, 80.0, 60.0);
    let obs = Obstacle::from_shape(&shape, 5.0, 0);

    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    let open_point = obs.padded_polyline().point_at(open_key);

    // For H-scan, the open vertex should be the bottom-left corner (lowest Y, then lowest X)
    // Padded bbox: (5, 15) to (95, 85)
    assert!((open_point.y() - 15.0).abs() < 1e-10);
    assert!((open_point.x() - 5.0).abs() < 1e-10);
}

#[test]
fn obstacle_create_initial_sides_from_open_vertex() {
    let shape = Shape::rectangle(10.0, 20.0, 80.0, 60.0);
    let mut obs = Obstacle::from_shape(&shape, 5.0, 0);

    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    obs.create_initial_sides(open_key, scan_dir);

    let low = obs.active_low_side().expect("should have low side");
    let high = obs.active_high_side().expect("should have high side");

    // Both sides start at the open vertex
    assert_eq!(low.start(), obs.padded_polyline().point_at(open_key));
    // High side may have been advanced past a flat bottom, so check it's valid
    assert_eq!(high.side_type(), SideType::High);
}

#[test]
fn obstacle_close_clears_active_sides() {
    let shape = Shape::rectangle(0.0, 0.0, 100.0, 50.0);
    let mut obs = Obstacle::from_shape(&shape, 5.0, 0);
    let scan_dir = ScanDirection::horizontal();
    let open_key = obs.get_open_vertex(scan_dir);
    obs.create_initial_sides(open_key, scan_dir);
    assert!(obs.active_low_side().is_some());

    obs.close();
    assert!(obs.active_low_side().is_none());
    assert!(obs.active_high_side().is_none());
}

#[test]
fn obstacle_get_open_vertex_vertical_scan() {
    let shape = Shape::rectangle(10.0, 20.0, 80.0, 60.0);
    let obs = Obstacle::from_shape(&shape, 5.0, 0);

    let scan_dir = ScanDirection::vertical();
    let open_key = obs.get_open_vertex(scan_dir);
    let open_point = obs.padded_polyline().point_at(open_key);

    // For V-scan, the open vertex should be the bottom-left corner (lowest X, then lowest Y)
    // Padded bbox: (5, 15) to (95, 85)
    assert!((open_point.x() - 5.0).abs() < 1e-10);
    assert!((open_point.y() - 15.0).abs() < 1e-10);
}
