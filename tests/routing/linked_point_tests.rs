use msagl_rust::routing::nudging::linked_point::LinkedPointList;
use msagl_rust::Point;

#[test]
fn from_points_creates_linked_list() {
    let points = vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(20.0, 0.0),
    ];
    let (list, start) = LinkedPointList::from_points(&points);
    let collected = list.collect_points(start.unwrap());
    assert_eq!(collected.len(), 3);
    assert_eq!(collected[0], points[0]);
    assert_eq!(collected[2], points[2]);
}

#[test]
fn insert_after_adds_point_in_middle() {
    let points = vec![Point::new(0.0, 0.0), Point::new(20.0, 0.0)];
    let (mut list, start) = LinkedPointList::from_points(&points);
    list.insert_after(start.unwrap(), Point::new(10.0, 0.0));
    let collected = list.collect_points(start.unwrap());
    assert_eq!(collected.len(), 3);
    assert!((collected[1].x() - 10.0).abs() < 1e-10);
}

#[test]
fn empty_list() {
    let (list, start) = LinkedPointList::from_points(&[]);
    assert!(start.is_none());
    assert_eq!(list.len(), 0);
}

#[test]
fn insert_after_preserves_tail() {
    let points = vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(20.0, 0.0),
    ];
    let (mut list, start) = LinkedPointList::from_points(&points);
    let start_idx = start.unwrap();
    // Insert between first and second
    list.insert_after(start_idx, Point::new(5.0, 0.0));
    let collected = list.collect_points(start_idx);
    assert_eq!(collected.len(), 4);
    assert!((collected[0].x() - 0.0).abs() < 1e-10);
    assert!((collected[1].x() - 5.0).abs() < 1e-10);
    assert!((collected[2].x() - 10.0).abs() < 1e-10);
    assert!((collected[3].x() - 20.0).abs() < 1e-10);
}

#[test]
fn len_matches_node_count() {
    let points = vec![
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(2.0, 0.0),
    ];
    let (list, _) = LinkedPointList::from_points(&points);
    assert_eq!(list.len(), 3);
}

#[test]
fn single_point_list() {
    let points = vec![Point::new(5.0, 5.0)];
    let (list, start) = LinkedPointList::from_points(&points);
    let start_idx = start.unwrap();
    assert_eq!(list.len(), 1);
    assert!(list.next(start_idx).is_none());
    let collected = list.collect_points(start_idx);
    assert_eq!(collected.len(), 1);
    assert_eq!(collected[0], points[0]);
}

#[test]
fn node_accessor_returns_correct_data() {
    let points = vec![Point::new(3.0, 7.0), Point::new(9.0, 7.0)];
    let (list, start) = LinkedPointList::from_points(&points);
    let start_idx = start.unwrap();
    let node = list.node(start_idx);
    assert_eq!(node.point, points[0]);
    assert!(node.next.is_some());
}
