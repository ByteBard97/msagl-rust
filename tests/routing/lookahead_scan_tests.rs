use msagl_rust::routing::lookahead_scan::LookaheadScan;
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn add_and_find_site() {
    let mut scan = LookaheadScan::new(ScanDirection::horizontal());
    // Horizontal scan: coord() returns X
    scan.add_site(Point::new(10.0, 20.0), 0, 1);
    let found = scan.find_first_in_range(5.0, 15.0);
    assert!(found.is_some());
    assert_eq!(found.unwrap().initial_obstacle, 0);
}

#[test]
fn find_returns_none_outside_range() {
    let mut scan = LookaheadScan::new(ScanDirection::horizontal());
    scan.add_site(Point::new(10.0, 20.0), 0, 1);
    assert!(scan.find_first_in_range(15.0, 25.0).is_none());
}

#[test]
fn remove_site_works() {
    let mut scan = LookaheadScan::new(ScanDirection::horizontal());
    scan.add_site(Point::new(10.0, 20.0), 0, 1);
    scan.remove_site(Point::new(10.0, 20.0));
    assert!(scan.is_empty());
}

#[test]
fn find_all_in_range_returns_multiple() {
    let mut scan = LookaheadScan::new(ScanDirection::horizontal());
    scan.add_site(Point::new(10.0, 20.0), 0, 1);
    scan.add_site(Point::new(15.0, 25.0), 1, 2);
    scan.add_site(Point::new(50.0, 30.0), 2, 3);
    let found = scan.find_all_in_range(5.0, 20.0);
    assert_eq!(found.len(), 2);
}
