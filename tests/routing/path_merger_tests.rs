use msagl_rust::routing::nudging::path_merger::PathMerger;
use msagl_rust::Point;

#[test]
fn no_cycles_unchanged() {
    let mut paths = vec![vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(20.0, 0.0),
    ]];
    PathMerger::merge_paths(&mut paths);
    assert_eq!(paths[0].len(), 3);
}

#[test]
fn removes_simple_self_cycle() {
    // Path: A → B → C → B → D (B appears twice, so B→C→B loop is removed)
    let mut paths = vec![vec![
        Point::new(0.0, 0.0),   // A
        Point::new(10.0, 0.0),  // B
        Point::new(10.0, 10.0), // C
        Point::new(10.0, 0.0),  // B again
        Point::new(20.0, 0.0),  // D
    ]];
    PathMerger::merge_paths(&mut paths);
    // Should become A → B → D
    assert_eq!(paths[0].len(), 3);
    assert!((paths[0][0].x() - 0.0).abs() < 1e-10);
    assert!((paths[0][1].x() - 10.0).abs() < 1e-10);
    assert!((paths[0][2].x() - 20.0).abs() < 1e-10);
}

#[test]
fn handles_empty_path() {
    let mut paths = vec![vec![]];
    PathMerger::merge_paths(&mut paths);
    assert_eq!(paths[0].len(), 0);
}

#[test]
fn handles_empty_paths_vec() {
    let mut paths: Vec<Vec<Point>> = vec![];
    PathMerger::merge_paths(&mut paths);
    assert!(paths.is_empty());
}

#[test]
fn handles_single_point_path() {
    let mut paths = vec![vec![Point::new(5.0, 5.0)]];
    PathMerger::merge_paths(&mut paths);
    assert_eq!(paths[0].len(), 1);
}

#[test]
fn removes_self_cycle_at_start() {
    // Path: A → B → C → A → D (cycle returns to first point)
    let mut paths = vec![vec![
        Point::new(0.0, 0.0),   // A
        Point::new(10.0, 0.0),  // B
        Point::new(10.0, 10.0), // C
        Point::new(0.0, 0.0),   // A again
        Point::new(20.0, 0.0),  // D
    ]];
    PathMerger::merge_paths(&mut paths);
    // Should become A → D
    assert_eq!(paths[0].len(), 2);
    assert!((paths[0][0].x() - 0.0).abs() < 1e-10);
    assert!((paths[0][0].y() - 0.0).abs() < 1e-10);
    assert!((paths[0][1].x() - 20.0).abs() < 1e-10);
}

#[test]
fn multiple_paths_each_cleaned_independently() {
    // First path has a cycle; second does not.
    let mut paths = vec![
        vec![
            Point::new(0.0, 0.0),
            Point::new(5.0, 0.0),
            Point::new(5.0, 5.0),
            Point::new(5.0, 0.0), // cycle back to second point
            Point::new(10.0, 0.0),
        ],
        vec![
            Point::new(0.0, 1.0),
            Point::new(10.0, 1.0),
            Point::new(20.0, 1.0),
        ],
    ];
    PathMerger::merge_paths(&mut paths);
    // First path: cycle at (5,0) removed → 3 points
    assert_eq!(paths[0].len(), 3, "path0: {:?}", paths[0]);
    // Second path: untouched
    assert_eq!(paths[1].len(), 3);
}

#[test]
fn two_point_path_no_cycle_unchanged() {
    let mut paths = vec![vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)]];
    PathMerger::merge_paths(&mut paths);
    assert_eq!(paths[0].len(), 2);
}
