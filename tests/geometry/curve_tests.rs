use msagl_rust::{Curve, CurveSegment, Point, Polyline};

#[test]
fn empty_curve() {
    let c = Curve::new();
    assert_eq!(c.segment_count(), 0);
}

#[test]
fn single_line_segment() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    assert_eq!(c.segment_count(), 1);
    assert_eq!(c.start(), Point::new(0.0, 0.0));
    assert_eq!(c.end(), Point::new(5.0, 0.0));
}

#[test]
fn multiple_line_segments() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_line(Point::new(5.0, 0.0), Point::new(5.0, 3.0));
    assert_eq!(c.segment_count(), 2);
    assert_eq!(c.start(), Point::new(0.0, 0.0));
    assert_eq!(c.end(), Point::new(5.0, 3.0));
}

#[test]
fn bounding_box_of_line_segments() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_line(Point::new(5.0, 0.0), Point::new(5.0, 3.0));
    let bb = c.bounding_box();
    assert_eq!(bb.left(), 0.0);
    assert_eq!(bb.bottom(), 0.0);
    assert_eq!(bb.right(), 5.0);
    assert_eq!(bb.top(), 3.0);
}

#[test]
fn from_polyline() {
    let poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(3.0, 0.0),
        Point::new(3.0, 4.0),
    ]);
    let c = Curve::from_polyline(&poly);
    assert_eq!(c.segment_count(), 2);
    assert_eq!(c.start(), Point::new(0.0, 0.0));
    assert_eq!(c.end(), Point::new(3.0, 4.0));
}

#[test]
fn add_arc_segment() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_arc(
        Point::new(5.0, 0.0),
        Point::new(7.0, 2.0),
        Point::new(7.0, 0.0),
        true,
    );
    assert_eq!(c.segment_count(), 2);
}

#[test]
fn iterate_segments() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_line(Point::new(5.0, 0.0), Point::new(5.0, 3.0));
    let segments: Vec<_> = c.segments().collect();
    assert_eq!(segments.len(), 2);
    match &segments[0] {
        CurveSegment::Line { from, to } => {
            assert_eq!(*from, Point::new(0.0, 0.0));
            assert_eq!(*to, Point::new(5.0, 0.0));
        }
        _ => panic!("expected line segment"),
    }
}
