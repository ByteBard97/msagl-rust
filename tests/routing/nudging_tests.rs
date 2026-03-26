use msagl_rust::routing::nudging::axis_edge::AxisEdge;
use msagl_rust::routing::nudging::combinatorial_nudger::get_order;
use msagl_rust::routing::nudging::free_space_finder::find_free_space;
use msagl_rust::routing::nudging::nudge_paths;
use msagl_rust::routing::nudging::path_refiner::{deduplicate, refine_paths};
use msagl_rust::routing::nudging::staircase_remover::remove_staircases;
use msagl_rust::routing::scan_direction::Direction;
use msagl_rust::Point;
use msagl_rust::Rectangle;

#[test]
fn nudge_two_parallel_paths() {
    // Two paths sharing the same horizontal segment.
    let mut paths = vec![
        vec![Point::new(0.0, 5.0), Point::new(20.0, 5.0)],
        vec![Point::new(0.0, 5.0), Point::new(20.0, 5.0)],
    ];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 2.0);

    // After nudging, paths should be separated by at least edge_separation.
    let y0 = paths[0][0].y();
    let y1 = paths[1][0].y();
    assert!(
        (y0 - y1).abs() >= 2.0 - 0.5,
        "paths should be separated: y0={}, y1={}",
        y0,
        y1
    );
}

#[test]
fn nudge_preserves_single_path() {
    let mut paths = vec![vec![
        Point::new(0.0, 5.0),
        Point::new(10.0, 5.0),
        Point::new(10.0, 15.0),
    ]];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 2.0);

    // Single path should stay roughly in place.
    assert!(
        (paths[0][0].y() - 5.0).abs() < 1.0,
        "single path y moved too much: {:?}",
        paths[0]
    );
}

#[test]
fn nudge_respects_obstacles() {
    let mut paths = vec![
        vec![Point::new(0.0, 5.0), Point::new(20.0, 5.0)],
        vec![Point::new(0.0, 5.0), Point::new(20.0, 5.0)],
    ];
    // Obstacle below that limits downward movement.
    let obstacles = vec![Rectangle::new(-1.0, 0.0, 21.0, 3.0)];
    nudge_paths(&mut paths, &obstacles, 2.0);

    // Both paths should be above the obstacle (y >= 3).
    for (pi, path) in paths.iter().enumerate() {
        for (i, p) in path.iter().enumerate() {
            assert!(
                p.y() >= 3.0 - 0.5,
                "path[{}] point[{}] {:?} should be above obstacle",
                pi,
                i,
                p
            );
        }
    }
}

#[test]
fn nudge_three_parallel_paths() {
    let mut paths = vec![
        vec![Point::new(0.0, 10.0), Point::new(20.0, 10.0)],
        vec![Point::new(0.0, 10.0), Point::new(20.0, 10.0)],
        vec![Point::new(0.0, 10.0), Point::new(20.0, 10.0)],
    ];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 3.0);

    // All three should be separated.
    let mut ys: Vec<f64> = paths.iter().map(|p| p[0].y()).collect();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!(
        ys[1] - ys[0] >= 3.0 - 0.5,
        "gap01={}, ys={:?}",
        ys[1] - ys[0],
        ys
    );
    assert!(
        ys[2] - ys[1] >= 3.0 - 0.5,
        "gap12={}, ys={:?}",
        ys[2] - ys[1],
        ys
    );
}

#[test]
fn nudge_empty_paths() {
    let mut paths: Vec<Vec<Point>> = vec![];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 2.0);
    assert!(paths.is_empty());
}

#[test]
fn nudge_single_point_paths_no_panic() {
    let mut paths = vec![vec![Point::new(5.0, 5.0)]];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 2.0);
    // Should not panic.
}

#[test]
fn nudge_l_shaped_paths() {
    // Two L-shaped paths that share a vertical segment.
    let mut paths = vec![
        vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 20.0),
        ],
        vec![
            Point::new(0.0, 5.0),
            Point::new(10.0, 5.0),
            Point::new(10.0, 20.0),
        ],
    ];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 2.0);

    // Paths should still have valid rectilinear structure.
    for (pi, path) in paths.iter().enumerate() {
        assert!(
            path.len() >= 2,
            "path[{}] should have at least 2 points: {:?}",
            pi,
            path
        );
    }
}

#[test]
fn nudge_vertical_parallel_paths() {
    // Two vertical paths at the same x.
    let mut paths = vec![
        vec![Point::new(5.0, 0.0), Point::new(5.0, 20.0)],
        vec![Point::new(5.0, 0.0), Point::new(5.0, 20.0)],
    ];
    let obstacles = vec![];
    nudge_paths(&mut paths, &obstacles, 2.0);

    let x0 = paths[0][0].x();
    let x1 = paths[1][0].x();
    assert!(
        (x0 - x1).abs() >= 2.0 - 0.5,
        "vertical paths should be separated: x0={}, x1={}",
        x0,
        x1
    );
}

// === Tests extracted from path_refiner.rs ===

#[test]
fn deduplicate_removes_consecutive_dups() {
    let mut path = vec![
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
    ];
    deduplicate(&mut path);
    assert_eq!(path.len(), 3);
}

#[test]
fn refine_inserts_subdivision_points() {
    // Two horizontal segments on y=0: one from x=0..10, another from x=5..15.
    // After refinement, both should have a vertex at x=5 and x=10.
    let mut paths = vec![
        vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
        vec![Point::new(5.0, 0.0), Point::new(15.0, 0.0)],
    ];
    refine_paths(&mut paths);
    // Path 0 should now have points at x=0, 5, 10
    assert!(paths[0].len() >= 3, "path0 len = {}", paths[0].len());
    // Path 1 should now have points at x=5, 10, 15
    assert!(paths[1].len() >= 3, "path1 len = {}", paths[1].len());
}

// === Tests extracted from combinatorial_nudger.rs ===

#[test]
fn single_path_creates_axis_edges() {
    let paths = vec![vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
    ]];
    let result = get_order(&paths);
    assert_eq!(result.axis_edges.len(), 2);
    assert_eq!(result.path_edges.len(), 2);
    assert!(result.path_first_edges[0].is_some());
}

#[test]
fn two_paths_share_axis_edge() {
    let paths = vec![
        vec![Point::new(0.0, 5.0), Point::new(20.0, 5.0)],
        vec![Point::new(0.0, 5.0), Point::new(20.0, 5.0)],
    ];
    let result = get_order(&paths);
    // Both paths share the same AxisEdge.
    assert_eq!(result.axis_edges.len(), 1);
    assert_eq!(result.path_edges.len(), 2);
    assert_eq!(result.axis_edge_orders[0].len(), 2);
}

// === Tests extracted from staircase_remover.rs ===

#[test]
fn removes_simple_staircase() {
    // Staircase pattern: right, up, right, up, right
    // But we need alternating: E, N, E, N pattern over 5 points.
    let mut paths = vec![vec![
        Point::new(0.0, 0.0),
        Point::new(5.0, 0.0),
        Point::new(5.0, 5.0),
        Point::new(10.0, 5.0),
        Point::new(10.0, 10.0),
    ]];
    let original_len = paths[0].len();
    remove_staircases(&mut paths, &[]);
    assert!(
        paths[0].len() < original_len,
        "staircase should have been collapsed: {:?}",
        paths[0]
    );
}

#[test]
fn does_not_remove_non_staircase() {
    let mut paths = vec![vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
    ]];
    let original = paths[0].clone();
    remove_staircases(&mut paths, &[]);
    assert_eq!(paths[0], original);
}

// === Tests extracted from free_space_finder.rs ===

#[test]
fn obstacle_bounds_vertical_edge() {
    let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
    let obstacles = vec![
        Rectangle::new(0.0, 0.0, 3.0, 10.0),  // left obstacle
        Rectangle::new(7.0, 0.0, 10.0, 10.0), // right obstacle
    ];
    find_free_space(&mut edges, &obstacles, Direction::North);
    assert!(edges[0].left_bound >= 3.0 - 0.1);
    assert!(edges[0].right_bound <= 7.0 + 0.1);
}
