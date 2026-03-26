use msagl_rust::routing::obstacle_tree::ObstacleTree;
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
