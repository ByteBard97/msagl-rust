use msagl_rust::routing::scan_segment::{ScanSegment, ScanSegmentTree};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn scan_segment_creation() {
    let seg = ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0));
    assert_eq!(seg.start, Point::new(0.0, 5.0));
    assert_eq!(seg.end, Point::new(10.0, 5.0));
}

#[test]
fn scan_segment_tree_insert_and_find() {
    let sd = ScanDirection::horizontal();
    let mut tree = ScanSegmentTree::new(sd);
    tree.insert(ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0)));
    assert!(tree.find_containing_point(Point::new(5.0, 5.0)).is_some());
    assert!(tree.find_containing_point(Point::new(5.0, 3.0)).is_none());
}

#[test]
fn scan_segment_tree_count() {
    let sd = ScanDirection::horizontal();
    let mut tree = ScanSegmentTree::new(sd);
    tree.insert(ScanSegment::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0)));
    tree.insert(ScanSegment::new(Point::new(0.0, 10.0), Point::new(10.0, 10.0)));
    assert_eq!(tree.len(), 2);
}
