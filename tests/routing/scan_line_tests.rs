use msagl_rust::routing::scan_line::{RectilinearScanLine, ActiveSide};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn scan_line_insert_and_neighbors() {
    let sd = ScanDirection::horizontal();
    let mut sl = RectilinearScanLine::new(sd);

    // Insert side at y=5 (perp coord for horizontal sweep)
    sl.insert(ActiveSide {
        obstacle_index: 0,
        perp_coord: 5.0,
        start: Point::new(0.0, 5.0),
        end: Point::new(10.0, 5.0),
        is_low_side: true,
    });

    // Insert side at y=15
    sl.insert(ActiveSide {
        obstacle_index: 1,
        perp_coord: 15.0,
        start: Point::new(0.0, 15.0),
        end: Point::new(10.0, 15.0),
        is_low_side: false,
    });

    // Query neighbors at y=10
    let low = sl.low_neighbor(10.0);
    assert!(low.is_some());
    assert_eq!(low.unwrap().obstacle_index, 0);

    let high = sl.high_neighbor(10.0);
    assert!(high.is_some());
    assert_eq!(high.unwrap().obstacle_index, 1);
}

#[test]
fn scan_line_remove() {
    let sd = ScanDirection::horizontal();
    let mut sl = RectilinearScanLine::new(sd);
    sl.insert(ActiveSide {
        obstacle_index: 0,
        perp_coord: 5.0,
        start: Point::new(0.0, 5.0),
        end: Point::new(10.0, 5.0),
        is_low_side: true,
    });
    assert_eq!(sl.len(), 1);
    sl.remove(0, 5.0);
    assert_eq!(sl.len(), 0);
}
