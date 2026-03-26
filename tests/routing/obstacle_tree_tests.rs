use msagl_rust::routing::obstacle_tree::ObstacleTree;
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::routing::shape::Shape;
use msagl_rust::Point;

fn make_tree() -> ObstacleTree {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::rectangle(200.0, 100.0, 80.0, 60.0),
    ];
    ObstacleTree::new(&shapes, 4.0)
}

#[test]
fn graph_box_encompasses_all_obstacles_with_margin() {
    let tree = make_tree();
    let gb = tree.graph_box();
    // Shape 1: padded bbox = (-4, -4) to (104, 54)
    // Shape 2: padded bbox = (196, 96) to (284, 164)
    // Union = (-4, -4) to (284, 164)
    // With sentinel offset (1.0): (-5, -5) to (285, 165)
    assert!((gb.left() - (-5.0)).abs() < 1e-6);
    assert!((gb.bottom() - (-5.0)).abs() < 1e-6);
    assert!((gb.right() - 285.0).abs() < 1e-6);
    assert!((gb.top() - 165.0).abs() < 1e-6);
}

#[test]
fn inside_hit_test_finds_containing_obstacle() {
    let tree = make_tree();
    // Point inside first obstacle (center of shape 1)
    let hit = tree.inside_hit_test(Point::new(50.0, 25.0));
    assert_eq!(hit, Some(0));
}

#[test]
fn inside_hit_test_finds_second_obstacle() {
    let tree = make_tree();
    let hit = tree.inside_hit_test(Point::new(240.0, 130.0));
    assert_eq!(hit, Some(1));
}

#[test]
fn inside_hit_test_returns_none_for_empty_space() {
    let tree = make_tree();
    assert!(tree.inside_hit_test(Point::new(500.0, 500.0)).is_none());
}

#[test]
fn all_obstacles_returns_all() {
    let tree = make_tree();
    assert_eq!(tree.all_obstacles().len(), 2);
}

// --- Overlap detection tests ---

#[test]
fn non_overlapping_obstacles_no_clumps() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(50.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);
    assert!(!tree.obstacle(0).is_overlapped());
    assert!(!tree.obstacle(1).is_overlapped());
    assert!(tree.obstacle(0).is_primary_obstacle());
    assert!(tree.obstacle(1).is_primary_obstacle());
}

#[test]
fn overlapping_obstacles_form_clump() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 20.0, 10.0),
        Shape::rectangle(10.0, 0.0, 20.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);
    assert!(tree.obstacle(0).is_overlapped());
    assert!(tree.obstacle(1).is_overlapped());
    assert_eq!(tree.obstacle(0).clump().len(), 2);
}

#[test]
fn contained_obstacle_forms_clump() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 100.0),
        Shape::rectangle(10.0, 10.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);
    assert!(tree.obstacle(0).is_overlapped());
    assert!(tree.obstacle(1).is_overlapped());
}

#[test]
fn padding_can_cause_overlap() {
    // Two shapes that are 8px apart: padding of 5 each means padded boxes overlap
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(18.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 5.0);
    assert!(tree.obstacle(0).is_overlapped());
    assert!(tree.obstacle(1).is_overlapped());
}

// --- Primary obstacle tests ---

#[test]
fn primary_obstacles_returns_correct_count() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(50.0, 0.0, 10.0, 10.0),
        Shape::rectangle(100.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);
    assert_eq!(tree.primary_obstacles().count(), 3);
}

// --- InsideHitTest with scan direction ---

#[test]
fn point_is_inside_an_obstacle_horizontal() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::rectangle(200.0, 100.0, 80.0, 60.0),
    ];
    let mut tree = ObstacleTree::new(&shapes, 4.0);
    assert!(tree.point_is_inside_an_obstacle(
        Point::new(50.0, 25.0),
        ScanDirection::horizontal(),
    ));
    assert!(!tree.point_is_inside_an_obstacle(
        Point::new(150.0, 80.0),
        ScanDirection::horizontal(),
    ));
}

#[test]
fn intersection_is_inside_another_obstacle() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 100.0),
        Shape::rectangle(50.0, 50.0, 100.0, 100.0),
        Shape::rectangle(200.0, 200.0, 50.0, 50.0),
    ];
    let mut tree = ObstacleTree::new(&shapes, 0.0);

    // Point inside obstacle 2 (which is separate from 0 and 1)
    let result = tree.intersection_is_inside_another_obstacle(
        0, 1,
        Point::new(225.0, 225.0),
        ScanDirection::horizontal(),
    );
    assert!(result);
}

// --- Segment restriction tests ---

#[test]
fn segment_does_not_cross_when_no_obstacle_in_way() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);

    // Segment completely to the right of obstacle
    assert!(!tree.segment_crosses_an_obstacle(
        Point::new(20.0, 5.0),
        Point::new(50.0, 5.0),
    ));
}

#[test]
fn segment_crosses_obstacle() {
    let shapes = vec![
        Shape::rectangle(10.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);

    // Segment going through the obstacle
    assert!(tree.segment_crosses_an_obstacle(
        Point::new(0.0, 5.0),
        Point::new(30.0, 5.0),
    ));
}

#[test]
fn restrict_segment_returns_intersection_point() {
    let shapes = vec![
        Shape::rectangle(10.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);

    let (restricted, obs_idx) = tree.restrict_segment_with_obstacles(
        Point::new(0.0, 5.0),
        Point::new(30.0, 5.0),
    );

    // Should be restricted to the left edge of the obstacle at x=10
    assert!((restricted.x() - 10.0).abs() < 0.1);
    assert!(obs_idx.is_some());
}

// --- Visibility polyline tests ---

#[test]
fn visibility_polyline_returns_padded_polyline_for_normal_obstacle() {
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let tree = ObstacleTree::new(&shapes, 2.0);
    let obs = tree.obstacle(0);

    // visibility_polyline should be the padded polyline when no convex hull
    let vis_bb = obs.visibility_bounding_box();
    assert!((vis_bb.left() - (-2.0)).abs() < 1e-6);
    assert!((vis_bb.right() - 12.0).abs() < 1e-6);
}

#[test]
fn is_rectangle_true_for_rectangular_shapes() {
    let shapes = vec![Shape::rectangle(0.0, 0.0, 10.0, 10.0)];
    let tree = ObstacleTree::new(&shapes, 0.0);
    assert!(tree.obstacle(0).is_rectangle());
}

// --- Query tests ---

#[test]
fn query_rect_finds_intersecting_obstacles() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(50.0, 0.0, 10.0, 10.0),
        Shape::rectangle(100.0, 0.0, 10.0, 10.0),
    ];
    let tree = ObstacleTree::new(&shapes, 0.0);

    let rect = msagl_rust::geometry::rectangle::Rectangle::new(-1.0, -1.0, 55.0, 11.0);
    let hits = tree.query_rect(&rect);
    // Should find obstacles 0 and 1
    assert_eq!(hits.len(), 2);
}

#[test]
fn rebuild_rtree_updates_spatial_index() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 10.0, 10.0),
        Shape::rectangle(50.0, 0.0, 10.0, 10.0),
    ];
    let mut tree = ObstacleTree::new(&shapes, 0.0);

    // Rebuild should succeed without error
    tree.rebuild_rtree();

    // Queries should still work
    assert_eq!(tree.all_obstacles().len(), 2);
    let hit = tree.inside_hit_test(Point::new(5.0, 5.0));
    assert_eq!(hit, Some(0));
}
