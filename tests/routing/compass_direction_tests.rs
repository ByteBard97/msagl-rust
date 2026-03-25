use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::Point;

#[test]
fn compass_direction_has_four_variants() {
    let all = CompassDirection::all();
    assert_eq!(all.len(), 4);
}

#[test]
fn compass_direction_opposite() {
    assert_eq!(CompassDirection::North.opposite(), CompassDirection::South);
    assert_eq!(CompassDirection::East.opposite(), CompassDirection::West);
    assert_eq!(CompassDirection::South.opposite(), CompassDirection::North);
    assert_eq!(CompassDirection::West.opposite(), CompassDirection::East);
}

#[test]
fn compass_direction_to_index() {
    let indices: Vec<usize> = CompassDirection::all().iter().map(|d| d.index()).collect();
    assert_eq!(indices, vec![0, 1, 2, 3]);
}

#[test]
fn compass_direction_left_right_turns() {
    assert_eq!(CompassDirection::North.left(), CompassDirection::West);
    assert_eq!(CompassDirection::North.right(), CompassDirection::East);
    assert_eq!(CompassDirection::East.left(), CompassDirection::North);
    assert_eq!(CompassDirection::East.right(), CompassDirection::South);
}

#[test]
fn from_points_axis_aligned() {
    let origin = Point::new(0.0, 0.0);
    assert_eq!(CompassDirection::from_points(origin, Point::new(10.0, 0.0)), Some(CompassDirection::East));
    assert_eq!(CompassDirection::from_points(origin, Point::new(-10.0, 0.0)), Some(CompassDirection::West));
    assert_eq!(CompassDirection::from_points(origin, Point::new(0.0, 10.0)), Some(CompassDirection::North));
    assert_eq!(CompassDirection::from_points(origin, Point::new(0.0, -10.0)), Some(CompassDirection::South));
}

#[test]
fn from_points_identical_returns_none() {
    let p = Point::new(5.0, 5.0);
    assert_eq!(CompassDirection::from_points(p, p), None);
}

#[test]
fn from_points_diagonal_prefers_horizontal() {
    let origin = Point::new(0.0, 0.0);
    // Equal dx and dy -- horizontal wins
    assert_eq!(CompassDirection::from_points(origin, Point::new(5.0, 5.0)), Some(CompassDirection::East));
    assert_eq!(CompassDirection::from_points(origin, Point::new(-5.0, -5.0)), Some(CompassDirection::West));
}
