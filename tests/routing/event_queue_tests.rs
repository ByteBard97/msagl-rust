use msagl_rust::routing::event_queue::{EventQueue, SweepEvent};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

#[test]
fn event_queue_ordering() {
    let sd = ScanDirection::horizontal(); // sweep left-to-right, perp=Y
    let mut q = EventQueue::new(sd);
    // Events at y=10, y=5, y=20
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 10.0), obstacle_index: 0 });
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 5.0), obstacle_index: 1 });
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 20.0), obstacle_index: 2 });
    // Should dequeue in order: y=5, y=10, y=20
    assert_eq!(q.dequeue().unwrap().site().y(), 5.0);
    assert_eq!(q.dequeue().unwrap().site().y(), 10.0);
    assert_eq!(q.dequeue().unwrap().site().y(), 20.0);
}

#[test]
fn event_queue_open_before_close_at_same_position() {
    let sd = ScanDirection::horizontal();
    let mut q = EventQueue::new(sd);
    q.enqueue(SweepEvent::CloseVertex { site: Point::new(5.0, 10.0), obstacle_index: 0 });
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, 10.0), obstacle_index: 1 });
    // At same y=10, OpenVertex should come first
    let first = q.dequeue().unwrap();
    assert!(matches!(first, SweepEvent::OpenVertex { .. }));
}
