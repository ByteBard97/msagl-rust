use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::routing::scan_segment::{ScanSegment, ScanSegmentTree, SegmentWeight};
use msagl_rust::Point;

#[test]
fn scan_segment_creation() {
    let seg = ScanSegment::new(
        Point::new(0.0, 5.0),
        Point::new(10.0, 5.0),
        SegmentWeight::Normal,
        false,
    );
    assert_eq!(seg.start, Point::new(0.0, 5.0));
    assert_eq!(seg.end, Point::new(10.0, 5.0));
}

#[test]
fn scan_segment_tree_insert_and_find() {
    let sd = ScanDirection::horizontal();
    let mut tree = ScanSegmentTree::new(sd);
    tree.insert(ScanSegment::new(
        Point::new(0.0, 5.0),
        Point::new(10.0, 5.0),
        SegmentWeight::Normal,
        false,
    ));
    assert!(tree.find_containing_point(Point::new(5.0, 5.0)).is_some());
    assert!(tree.find_containing_point(Point::new(5.0, 3.0)).is_none());
}

#[test]
fn scan_segment_tree_count() {
    let sd = ScanDirection::horizontal();
    let mut tree = ScanSegmentTree::new(sd);
    tree.insert(ScanSegment::new(
        Point::new(0.0, 5.0),
        Point::new(10.0, 5.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(10.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    assert_eq!(tree.len(), 2);
}

#[test]
fn segment_weight_ordering() {
    assert!(SegmentWeight::Normal < SegmentWeight::Reflection);
    assert!(SegmentWeight::Reflection < SegmentWeight::Overlapped);
}

#[test]
fn segment_weight_values() {
    assert_eq!(SegmentWeight::Normal.value(), 1);
    assert_eq!(SegmentWeight::Reflection.value(), 5);
    assert_eq!(SegmentWeight::Overlapped.value(), 500);
}

#[test]
fn segment_perp_coord_vertical() {
    let seg = ScanSegment::new(
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        SegmentWeight::Normal,
        true,
    );
    assert!((seg.perp_coord() - 10.0).abs() < 1e-10);
    assert!((seg.low_coord() - 0.0).abs() < 1e-10);
    assert!((seg.high_coord() - 100.0).abs() < 1e-10);
}

#[test]
fn segment_perp_coord_horizontal() {
    let seg = ScanSegment::new(
        Point::new(0.0, 50.0),
        Point::new(200.0, 50.0),
        SegmentWeight::Normal,
        false,
    );
    assert!((seg.perp_coord() - 50.0).abs() < 1e-10);
    assert!((seg.low_coord() - 0.0).abs() < 1e-10);
    assert!((seg.high_coord() - 200.0).abs() < 1e-10);
}

#[test]
fn contains_coord_within_range() {
    let seg = ScanSegment::new(
        Point::new(10.0, 0.0),
        Point::new(10.0, 100.0),
        SegmentWeight::Normal,
        true,
    );
    assert!(seg.contains_coord(50.0));
    assert!(seg.contains_coord(0.0));
    assert!(seg.contains_coord(100.0));
    assert!(!seg.contains_coord(-10.0));
    assert!(!seg.contains_coord(110.0));
}

#[test]
fn reflection_and_overlapped_checks() {
    let normal = ScanSegment::new(
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        SegmentWeight::Normal,
        false,
    );
    let refl = ScanSegment::new(
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        SegmentWeight::Reflection,
        false,
    );
    let over = ScanSegment::new(
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        SegmentWeight::Overlapped,
        false,
    );
    assert!(!normal.is_reflection());
    assert!(!normal.is_overlapped());
    assert!(refl.is_reflection());
    assert!(over.is_overlapped());
}

// -------------------------------------------------------------------------
// ScanSegmentTree — new method tests
// -------------------------------------------------------------------------

#[test]
fn find_lowest_intersector_returns_first_crossing() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(0.0, 20.0),
        Point::new(50.0, 20.0),
        SegmentWeight::Normal,
        false,
    ));
    // scan_coord 25 is within [0, 50] for both; lowest perp coord is y=10.
    let lowest = tree.find_lowest_intersector(25.0);
    assert!(lowest.is_some());
    assert!((lowest.unwrap().perp_coord() - 10.0).abs() < 1e-10);
}

#[test]
fn find_highest_intersector_returns_last_crossing() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(0.0, 20.0),
        Point::new(50.0, 20.0),
        SegmentWeight::Normal,
        false,
    ));
    let highest = tree.find_highest_intersector(25.0);
    assert!(highest.is_some());
    assert!((highest.unwrap().perp_coord() - 20.0).abs() < 1e-10);
}

#[test]
fn find_intersector_returns_none_when_no_crossing() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    assert!(tree.find_lowest_intersector(60.0).is_none());
    assert!(tree.find_highest_intersector(60.0).is_none());
}

#[test]
fn insert_unique_rejects_duplicate() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    let seg = ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    );
    assert!(tree.insert_unique(seg.clone()));
    assert!(!tree.insert_unique(seg));
    assert_eq!(tree.len(), 1);
}

#[test]
fn insert_unique_accepts_different_weight() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    let seg_a = ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    );
    let seg_b = ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Reflection,
        false,
    );
    assert!(tree.insert_unique(seg_a));
    assert!(tree.insert_unique(seg_b));
    assert_eq!(tree.len(), 2);
}

#[test]
fn segments_at_coord_returns_correct_slice() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(60.0, 10.0),
        Point::new(100.0, 10.0),
        SegmentWeight::Reflection,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(0.0, 20.0),
        Point::new(50.0, 20.0),
        SegmentWeight::Normal,
        false,
    ));
    assert_eq!(tree.segments_at_coord(10.0).len(), 2);
    assert_eq!(tree.segments_at_coord(20.0).len(), 1);
    assert_eq!(tree.segments_at_coord(30.0).len(), 0);
}

#[test]
fn merge_adjacent_segments() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(50.0, 10.0),
        Point::new(100.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    assert_eq!(tree.len(), 2);
    tree.merge_segments();
    assert_eq!(tree.len(), 1);
    let seg = tree.all_segments().next().unwrap();
    assert!((seg.low_coord() - 0.0).abs() < 1e-10);
    assert!((seg.high_coord() - 100.0).abs() < 1e-10);
}

#[test]
fn merge_does_not_merge_different_weights() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(50.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(50.0, 10.0),
        Point::new(100.0, 10.0),
        SegmentWeight::Reflection,
        false,
    ));
    tree.merge_segments();
    assert_eq!(tree.len(), 2);
}

#[test]
fn merge_three_adjacent_same_weight() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 5.0),
        Point::new(30.0, 5.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(30.0, 5.0),
        Point::new(60.0, 5.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.insert(ScanSegment::new(
        Point::new(60.0, 5.0),
        Point::new(90.0, 5.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.merge_segments();
    assert_eq!(tree.len(), 1);
    let seg = tree.all_segments().next().unwrap();
    assert!((seg.low_coord() - 0.0).abs() < 1e-10);
    assert!((seg.high_coord() - 90.0).abs() < 1e-10);
}

#[test]
fn merge_non_adjacent_segments_stay_separate() {
    let mut tree = ScanSegmentTree::new(ScanDirection::horizontal());
    tree.insert(ScanSegment::new(
        Point::new(0.0, 10.0),
        Point::new(40.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    // Gap: [40, 60] is empty.
    tree.insert(ScanSegment::new(
        Point::new(60.0, 10.0),
        Point::new(100.0, 10.0),
        SegmentWeight::Normal,
        false,
    ));
    tree.merge_segments();
    assert_eq!(tree.len(), 2);
}
