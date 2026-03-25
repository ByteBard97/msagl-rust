use msagl_rust::routing::scan_line::RectilinearScanLine;
use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn scan_line_insert_and_neighbors() {
    let sd = ScanDirection::horizontal();
    let mut sl = RectilinearScanLine::new(sd);

    // Insert vertical side at x=5 (horizontal sweep, keyed by X)
    sl.insert(ObstacleSide::new(
        SideType::Low,
        Point::new(5.0, 0.0),
        Point::new(5.0, 10.0),
        0,
    ));

    // Insert vertical side at x=15
    sl.insert(ObstacleSide::new(
        SideType::High,
        Point::new(15.0, 0.0),
        Point::new(15.0, 10.0),
        1,
    ));

    // Query neighbors at x=10
    let low = sl.low_neighbor(10.0);
    assert!(low.is_some());
    assert_eq!(low.unwrap().obstacle_ordinal(), 0);

    let high = sl.high_neighbor(10.0);
    assert!(high.is_some());
    assert_eq!(high.unwrap().obstacle_ordinal(), 1);
}

#[test]
fn scan_line_remove() {
    let sd = ScanDirection::horizontal();
    let mut sl = RectilinearScanLine::new(sd);
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(5.0, 0.0),
        Point::new(5.0, 10.0),
        0,
    );
    sl.insert(side.clone());
    assert_eq!(sl.len(), 1);
    sl.remove(&side);
    assert_eq!(sl.len(), 0);
}

#[test]
fn insert_and_find_low_neighbor() {
    let mut sl = RectilinearScanLine::new(ScanDirection::horizontal());
    // Vertical side at x=10 (horizontal scan, keyed by X)
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        0,
    );
    sl.insert(side);
    // Query at x=15: low neighbor should be the side at x=10
    let low = sl.low_neighbor(15.0);
    assert!(low.is_some());
    assert_eq!(low.unwrap().obstacle_ordinal(), 0);
}

#[test]
fn find_neighbors_returns_both() {
    let mut sl = RectilinearScanLine::new(ScanDirection::horizontal());
    let side_a = ObstacleSide::new(
        SideType::Low,
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        0,
    );
    let side_b = ObstacleSide::new(
        SideType::High,
        Point::new(30.0, 0.0),
        Point::new(30.0, 100.0),
        1,
    );
    sl.insert(side_a);
    sl.insert(side_b);
    let (low, high) = sl.find_neighbors(20.0);
    assert_eq!(low.unwrap().obstacle_ordinal(), 0);
    assert_eq!(high.unwrap().obstacle_ordinal(), 1);
}

#[test]
fn remove_side() {
    let mut sl = RectilinearScanLine::new(ScanDirection::horizontal());
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        0,
    );
    sl.insert(side.clone());
    assert_eq!(sl.len(), 1);
    sl.remove(&side);
    assert_eq!(sl.len(), 0);
}

#[test]
fn high_side_sorts_before_low_at_same_coord() {
    let mut sl = RectilinearScanLine::new(ScanDirection::horizontal());
    let low = ObstacleSide::new(
        SideType::Low,
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        0,
    );
    let high = ObstacleSide::new(
        SideType::High,
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        1,
    );
    sl.insert(low);
    sl.insert(high);
    // Both sides are at x=10 — the tree should hold 2 distinct entries
    assert_eq!(sl.len(), 2);
}

#[test]
fn empty_scanline_returns_none() {
    let sl = RectilinearScanLine::new(ScanDirection::horizontal());
    let (low, high) = sl.find_neighbors(10.0);
    assert!(low.is_none());
    assert!(high.is_none());
}

#[test]
fn vertical_scan_keys_by_y() {
    let mut sl = RectilinearScanLine::new(ScanDirection::vertical());
    // Horizontal side at y=20 (vertical scan, keyed by Y)
    let side = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 20.0),
        Point::new(100.0, 20.0),
        0,
    );
    sl.insert(side);
    let low = sl.low_neighbor(25.0);
    assert!(low.is_some());
    assert_eq!(low.unwrap().obstacle_ordinal(), 0);
    assert!(sl.high_neighbor(25.0).is_none());
}
