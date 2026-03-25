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
