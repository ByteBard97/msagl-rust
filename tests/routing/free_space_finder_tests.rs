use msagl_rust::routing::nudging::axis_edge::AxisEdge;
use msagl_rust::routing::nudging::free_space_finder::find_free_space;
use msagl_rust::routing::scan_direction::Direction;
use msagl_rust::Point;
use msagl_rust::Rectangle;

// ---------------------------------------------------------------------------
// Obstacle bounds tests
// ---------------------------------------------------------------------------

#[test]
fn bounds_computed_from_obstacles() {
    // Vertical axis edge at x=5, between two flanking obstacles.
    let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
    let obstacles = vec![
        Rectangle::new(0.0, 0.0, 3.0, 10.0), // left obstacle: right edge at x=3
        Rectangle::new(7.0, 0.0, 10.0, 10.0), // right obstacle: left edge at x=7
    ];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert!(
        edges[0].left_bound >= 3.0 - 1e-6,
        "left_bound should be >= 3.0, got {}",
        edges[0].left_bound
    );
    assert!(
        edges[0].right_bound <= 7.0 + 1e-6,
        "right_bound should be <= 7.0, got {}",
        edges[0].right_bound
    );
}

#[test]
fn unobstructed_edge_has_infinite_bounds() {
    let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
    let obstacles: Vec<Rectangle> = vec![];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert_eq!(
        edges[0].left_bound,
        f64::NEG_INFINITY,
        "no obstacles → left_bound stays -inf"
    );
    assert_eq!(
        edges[0].right_bound,
        f64::INFINITY,
        "no obstacles → right_bound stays +inf"
    );
}

#[test]
fn obstacle_only_on_left_constrains_left_bound() {
    let mut edges = vec![AxisEdge::new(Point::new(10.0, 0.0), Point::new(10.0, 10.0))];
    let obstacles = vec![
        Rectangle::new(0.0, 0.0, 6.0, 10.0), // right edge at x=6, left of edge at x=10
    ];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert!(
        edges[0].left_bound >= 6.0 - 1e-6,
        "left_bound constrained by obstacle right edge: {}",
        edges[0].left_bound
    );
    assert_eq!(
        edges[0].right_bound,
        f64::INFINITY,
        "no right obstacle → right_bound unchanged"
    );
}

#[test]
fn obstacle_only_on_right_constrains_right_bound() {
    let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
    let obstacles = vec![
        Rectangle::new(8.0, 0.0, 14.0, 10.0), // left edge at x=8, right of edge at x=5
    ];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert_eq!(
        edges[0].left_bound,
        f64::NEG_INFINITY,
        "no left obstacle → left_bound unchanged"
    );
    assert!(
        edges[0].right_bound <= 8.0 + 1e-6,
        "right_bound constrained by obstacle left edge: {}",
        edges[0].right_bound
    );
}

#[test]
fn non_overlapping_obstacle_does_not_constrain() {
    // Obstacle does not overlap along the axis edge's direction range.
    let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
    // Obstacle is entirely below the edge (y=20..30, edge is y=0..10)
    let obstacles = vec![Rectangle::new(0.0, 20.0, 3.0, 30.0)];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert_eq!(edges[0].left_bound, f64::NEG_INFINITY);
    assert_eq!(edges[0].right_bound, f64::INFINITY);
}

// ---------------------------------------------------------------------------
// Right-neighbor relationship tests
// ---------------------------------------------------------------------------

#[test]
fn two_parallel_overlapping_edges_are_right_neighbors() {
    // Two vertical edges side by side: left at x=5, right at x=10.
    // Both span y=0..10 so they definitely overlap.
    let mut edges = vec![
        AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0)),
        AxisEdge::new(Point::new(10.0, 0.0), Point::new(10.0, 10.0)),
    ];
    let obstacles: Vec<Rectangle> = vec![];
    find_free_space(&mut edges, &obstacles, Direction::North);
    // Edge 0 (left) should have edge 1 as a right neighbor.
    assert!(
        edges[0].right_neighbors.contains(&1),
        "edge at x=5 should have edge at x=10 as right neighbor"
    );
    // Edge 1 (right) should not have edge 0 as a right neighbor.
    assert!(
        !edges[1].right_neighbors.contains(&0),
        "edge at x=10 should NOT have edge at x=5 as right neighbor"
    );
}

#[test]
fn non_overlapping_parallel_edges_not_neighbors() {
    // Two vertical edges side by side but non-overlapping along Y.
    // Left: x=5, y=0..5. Right: x=10, y=10..20. No Y overlap.
    let mut edges = vec![
        AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 5.0)),
        AxisEdge::new(Point::new(10.0, 10.0), Point::new(10.0, 20.0)),
    ];
    let obstacles: Vec<Rectangle> = vec![];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert!(
        edges[0].right_neighbors.is_empty(),
        "non-overlapping edges should not be right neighbors"
    );
}

#[test]
fn horizontal_edges_neighbor_relationship() {
    // Two horizontal edges: one at y=-5 (East direction uses -y as perp),
    // one at y=-10. Both span x=0..20.
    let mut edges = vec![
        AxisEdge::new(Point::new(0.0, -5.0), Point::new(20.0, -5.0)),
        AxisEdge::new(Point::new(0.0, -10.0), Point::new(20.0, -10.0)),
    ];
    let obstacles: Vec<Rectangle> = vec![];
    find_free_space(&mut edges, &obstacles, Direction::East);
    // The edge with higher perp coord (y=-5 → perp = -(-5)=5) should have
    // the edge with lower perp (y=-10 → perp=10) as right neighbor... but
    // East perp is -y, so -(-5)=5 < -(-10)=10. Edge 0 has perp=5, edge 1
    // has perp=10. Edge 0 is "left", edge 1 is "right neighbor".
    assert!(
        edges[0].right_neighbors.contains(&1),
        "edge with smaller perp (y=-5) should have right neighbor (y=-10)"
    );
}

#[test]
fn wrong_direction_edges_not_processed() {
    // Horizontal edge passed to North sweep — should not be constrained.
    let mut edges = vec![AxisEdge::new(Point::new(0.0, 5.0), Point::new(20.0, 5.0))];
    let obstacles = vec![Rectangle::new(2.0, 3.0, 4.0, 7.0)];
    find_free_space(&mut edges, &obstacles, Direction::North);
    // Horizontal edge is Direction::East, not Direction::North.
    // It should be skipped entirely — bounds remain default.
    assert_eq!(edges[0].left_bound, f64::NEG_INFINITY);
    assert_eq!(edges[0].right_bound, f64::INFINITY);
    assert!(edges[0].right_neighbors.is_empty());
}
