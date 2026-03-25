use msagl_rust::routing::shape::Shape;
use msagl_rust::routing::obstacle::Obstacle;
use msagl_rust::Point;

#[test]
fn shape_rectangle_creation() {
    let s = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let bb = s.bounding_box();
    assert_eq!(bb.left(), 10.0);
    assert_eq!(bb.right(), 110.0);
    assert_eq!(bb.bottom(), 20.0);
    assert_eq!(bb.top(), 70.0);
}

#[test]
fn shape_rectangle_centered() {
    let s = Shape::rectangle_centered(50.0, 50.0, 20.0, 10.0);
    let bb = s.bounding_box();
    assert_eq!(bb.left(), 40.0);
    assert_eq!(bb.right(), 60.0);
    assert_eq!(bb.bottom(), 45.0);
    assert_eq!(bb.top(), 55.0);
}

#[test]
fn obstacle_from_shape_with_padding() {
    let s = Shape::rectangle(10.0, 20.0, 100.0, 50.0);
    let obs = Obstacle::from_shape(&s, 4.0, 0);
    let bb = obs.padded_bounding_box();
    assert_eq!(bb.left(), 6.0);
    assert_eq!(bb.right(), 114.0);
    assert_eq!(bb.bottom(), 16.0);
    assert_eq!(bb.top(), 74.0);
}

#[test]
fn obstacle_padded_polyline_has_four_points() {
    let s = Shape::rectangle(0.0, 0.0, 10.0, 10.0);
    let obs = Obstacle::from_shape(&s, 2.0, 0);
    assert_eq!(obs.padded_polyline().count(), 4);
    assert!(obs.padded_polyline().is_closed());
}

#[test]
fn obstacle_padded_corners() {
    let s = Shape::rectangle(0.0, 0.0, 10.0, 10.0);
    let obs = Obstacle::from_shape(&s, 2.0, 0);
    let corners = obs.padded_corners();
    assert_eq!(corners[0], Point::new(-2.0, -2.0)); // left_bottom
    assert_eq!(corners[1], Point::new(-2.0, 12.0)); // left_top
    assert_eq!(corners[2], Point::new(12.0, 12.0)); // right_top
    assert_eq!(corners[3], Point::new(12.0, -2.0)); // right_bottom
}
