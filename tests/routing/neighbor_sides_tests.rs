use msagl_rust::routing::neighbor_sides::NeighborSides;
use msagl_rust::routing::obstacle_side::{ObstacleSide, SideType};
use msagl_rust::Point;

#[test]
fn neighbor_sides_default_is_empty() {
    let ns = NeighborSides::new();
    assert!(ns.low_neighbor().is_none());
    assert!(ns.high_neighbor().is_none());
}

#[test]
fn set_and_get_sides() {
    let mut ns = NeighborSides::new();
    let low = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 0.0),
        Point::new(0.0, 10.0),
        0,
    );
    let high = ObstacleSide::new(
        SideType::High,
        Point::new(20.0, 0.0),
        Point::new(20.0, 10.0),
        1,
    );
    ns.set_sides(Some(low), Some(high));
    assert_eq!(ns.low_neighbor().unwrap().obstacle_ordinal(), 0);
    assert_eq!(ns.high_neighbor().unwrap().obstacle_ordinal(), 1);
}

#[test]
fn clear_removes_sides() {
    let mut ns = NeighborSides::new();
    let low = ObstacleSide::new(
        SideType::Low,
        Point::new(0.0, 0.0),
        Point::new(0.0, 10.0),
        0,
    );
    ns.set_sides(Some(low), None);
    ns.clear();
    assert!(ns.low_neighbor().is_none());
}
