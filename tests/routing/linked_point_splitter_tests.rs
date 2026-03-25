use msagl_rust::routing::nudging::linked_point::LinkedPointList;
use msagl_rust::routing::nudging::linked_point_splitter::LinkedPointSplitter;
use msagl_rust::Point;

#[test]
fn finds_hv_crossing() {
    // H segment: (0,50) -> (100,50)
    // V segment: (50,0) -> (50,100)
    // Crossing at (50, 50)
    let (mut h_list, h_start) = LinkedPointList::from_points(&[
        Point::new(0.0, 50.0),
        Point::new(100.0, 50.0),
    ]);
    let (mut v_list, v_start) = LinkedPointList::from_points(&[
        Point::new(50.0, 0.0),
        Point::new(50.0, 100.0),
    ]);

    LinkedPointSplitter::split(
        &mut h_list,
        &[h_start.unwrap()],
        &mut v_list,
        &[v_start.unwrap()],
    );

    let h_points = h_list.collect_points(h_start.unwrap());
    assert_eq!(h_points.len(), 3, "crossing should be inserted into h segment");

    let v_points = v_list.collect_points(v_start.unwrap());
    assert_eq!(v_points.len(), 3, "crossing should be inserted into v segment");

    // Crossing point should be at (50, 50)
    assert!((h_points[1].x() - 50.0).abs() < 1e-6);
    assert!((h_points[1].y() - 50.0).abs() < 1e-6);
    assert!((v_points[1].x() - 50.0).abs() < 1e-6);
    assert!((v_points[1].y() - 50.0).abs() < 1e-6);
}

#[test]
fn no_crossing_when_segments_dont_intersect() {
    // H segment ends at x=40, V segment starts at x=50 — no crossing
    let (mut h_list, h_start) = LinkedPointList::from_points(&[
        Point::new(0.0, 50.0),
        Point::new(40.0, 50.0),
    ]);
    let (mut v_list, v_start) = LinkedPointList::from_points(&[
        Point::new(50.0, 0.0),
        Point::new(50.0, 100.0),
    ]);

    LinkedPointSplitter::split(
        &mut h_list,
        &[h_start.unwrap()],
        &mut v_list,
        &[v_start.unwrap()],
    );

    let h_points = h_list.collect_points(h_start.unwrap());
    assert_eq!(h_points.len(), 2, "no crossing should be added");

    let v_points = v_list.collect_points(v_start.unwrap());
    assert_eq!(v_points.len(), 2, "no crossing should be added");
}

#[test]
fn no_crossing_when_endpoint_touching_only() {
    // H ends exactly at x=50, V starts at x=50 — endpoint touch, not interior crossing
    let (mut h_list, h_start) = LinkedPointList::from_points(&[
        Point::new(0.0, 50.0),
        Point::new(50.0, 50.0),
    ]);
    let (mut v_list, v_start) = LinkedPointList::from_points(&[
        Point::new(50.0, 0.0),
        Point::new(50.0, 100.0),
    ]);

    LinkedPointSplitter::split(
        &mut h_list,
        &[h_start.unwrap()],
        &mut v_list,
        &[v_start.unwrap()],
    );

    // x=50 is the right endpoint of H, so it is NOT a strict interior crossing
    let h_points = h_list.collect_points(h_start.unwrap());
    assert_eq!(h_points.len(), 2, "endpoint touch should not be inserted");
}

#[test]
fn empty_vertical_list_is_noop() {
    let (mut h_list, h_start) = LinkedPointList::from_points(&[
        Point::new(0.0, 50.0),
        Point::new(100.0, 50.0),
    ]);
    let (mut v_list, _) = LinkedPointList::from_points(&[]);

    LinkedPointSplitter::split(&mut h_list, &[h_start.unwrap()], &mut v_list, &[]);

    let h_points = h_list.collect_points(h_start.unwrap());
    assert_eq!(h_points.len(), 2, "empty v list: h unchanged");
}

#[test]
fn multiple_verticals_crossed_by_one_horizontal() {
    // H: (0,50) -> (100,50)
    // V1: (25,0) -> (25,100) and V2: (75,0) -> (75,100)
    // Both crossed at y=50 — each tested in its own fresh list to keep it simple.
    let h_pts = [Point::new(0.0, 50.0), Point::new(100.0, 50.0)];

    // V1 at x=25
    let (mut h_list1, h_start1) = LinkedPointList::from_points(&h_pts);
    let (mut v_list1, v1_start) = LinkedPointList::from_points(&[
        Point::new(25.0, 0.0),
        Point::new(25.0, 100.0),
    ]);
    LinkedPointSplitter::split(
        &mut h_list1,
        &[h_start1.unwrap()],
        &mut v_list1,
        &[v1_start.unwrap()],
    );
    let h1_points = h_list1.collect_points(h_start1.unwrap());
    assert_eq!(h1_points.len(), 3, "one crossing with v1 at x=25");
    assert!((h1_points[1].x() - 25.0).abs() < 1e-6);

    // V2 at x=75
    let (mut h_list2, h_start2) = LinkedPointList::from_points(&h_pts);
    let (mut v_list2, v2_start) = LinkedPointList::from_points(&[
        Point::new(75.0, 0.0),
        Point::new(75.0, 100.0),
    ]);
    LinkedPointSplitter::split(
        &mut h_list2,
        &[h_start2.unwrap()],
        &mut v_list2,
        &[v2_start.unwrap()],
    );
    let h2_points = h_list2.collect_points(h_start2.unwrap());
    assert_eq!(h2_points.len(), 3, "one crossing with v2 at x=75");
    assert!((h2_points[1].x() - 75.0).abs() < 1e-6);
}

#[test]
fn right_to_left_horizontal_also_split() {
    // H goes right-to-left: (100,50) -> (0,50)
    // V: (50,0) -> (50,100)
    let (mut h_list, h_start) = LinkedPointList::from_points(&[
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ]);
    let (mut v_list, v_start) = LinkedPointList::from_points(&[
        Point::new(50.0, 0.0),
        Point::new(50.0, 100.0),
    ]);

    LinkedPointSplitter::split(
        &mut h_list,
        &[h_start.unwrap()],
        &mut v_list,
        &[v_start.unwrap()],
    );

    let h_points = h_list.collect_points(h_start.unwrap());
    assert_eq!(h_points.len(), 3, "RTL horizontal should also be split");

    let v_points = v_list.collect_points(v_start.unwrap());
    assert_eq!(v_points.len(), 3, "vertical should be split");
}
