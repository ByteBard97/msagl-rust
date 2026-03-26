use msagl_rust::arenas::PolylinePointKey;
use msagl_rust::routing::event_queue::{EventQueue, SweepEvent};
use msagl_rust::routing::scan_direction::ScanDirection;
use msagl_rust::Point;

/// Helper: default vertex key for tests that don't need polyline traversal.
fn vk() -> PolylinePointKey {
    PolylinePointKey::default()
}

#[test]
fn event_queue_ordering() {
    let sd = ScanDirection::horizontal(); // sweep left-to-right, perp=Y
    let mut q = EventQueue::new(sd);
    // Events at y=10, y=5, y=20
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 5.0), obstacle_index: 1, vertex_key: vk() });
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 20.0), obstacle_index: 2, vertex_key: vk() });
    // Should dequeue in order: y=5, y=10, y=20
    assert_eq!(q.dequeue().unwrap().site().y(), 5.0);
    assert_eq!(q.dequeue().unwrap().site().y(), 10.0);
    assert_eq!(q.dequeue().unwrap().site().y(), 20.0);
}

#[test]
fn event_queue_open_before_close_at_same_position() {
    let sd = ScanDirection::horizontal();
    let mut q = EventQueue::new(sd);
    q.enqueue(SweepEvent::CloseVertex { site: Point::new(5.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    q.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, 10.0), obstacle_index: 1, vertex_key: vk() });
    // At same y=10, OpenVertex should come first
    let first = q.dequeue().unwrap();
    assert!(matches!(first, SweepEvent::OpenVertex { .. }));
}

#[test]
fn events_ordered_by_perp_coord_first() {
    let mut queue = EventQueue::new(ScanDirection::horizontal());
    queue.enqueue(SweepEvent::OpenVertex { site: Point::new(5.0, 20.0), obstacle_index: 0, vertex_key: vk() });
    queue.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, 10.0), obstacle_index: 1, vertex_key: vk() });
    // Horizontal scan: perp coord is Y. Y=10 comes before Y=20.
    let first = queue.dequeue().unwrap();
    assert!((first.site().y() - 10.0).abs() < 1e-10);
}

#[test]
fn reflection_events_before_vertex_events_at_same_coord() {
    let mut queue = EventQueue::new(ScanDirection::horizontal());
    queue.enqueue(SweepEvent::OpenVertex { site: Point::new(5.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    queue.enqueue(SweepEvent::LowReflection {
        site: Point::new(3.0, 10.0),
        initial_obstacle: 0,
        reflecting_obstacle: 1,
        low_side_obstacle: None,
        prev_event_index: None,
    });
    let first = queue.dequeue().unwrap();
    assert!(first.is_reflection());
}

#[test]
fn open_before_close_at_same_coord() {
    let mut queue = EventQueue::new(ScanDirection::horizontal());
    queue.enqueue(SweepEvent::CloseVertex { site: Point::new(5.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    queue.enqueue(SweepEvent::OpenVertex { site: Point::new(5.0, 10.0), obstacle_index: 1, vertex_key: vk() });
    let first = queue.dequeue().unwrap();
    assert!(matches!(first, SweepEvent::OpenVertex { .. }));
}

#[test]
fn bend_events_between_open_and_close() {
    let mut queue = EventQueue::new(ScanDirection::horizontal());
    queue.enqueue(SweepEvent::CloseVertex { site: Point::new(5.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    queue.enqueue(SweepEvent::LowBend { site: Point::new(5.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    queue.enqueue(SweepEvent::OpenVertex { site: Point::new(5.0, 10.0), obstacle_index: 0, vertex_key: vk() });
    let first = queue.dequeue().unwrap();
    assert!(matches!(first, SweepEvent::OpenVertex { .. }));
    let second = queue.dequeue().unwrap();
    assert!(matches!(second, SweepEvent::LowBend { .. }));
    let third = queue.dequeue().unwrap();
    assert!(matches!(third, SweepEvent::CloseVertex { .. }));
}

#[test]
fn site_accessor_works_for_all_variants() {
    let events = vec![
        SweepEvent::OpenVertex { site: Point::new(1.0, 2.0), obstacle_index: 0, vertex_key: vk() },
        SweepEvent::CloseVertex { site: Point::new(3.0, 4.0), obstacle_index: 0, vertex_key: vk() },
        SweepEvent::LowBend { site: Point::new(5.0, 6.0), obstacle_index: 0, vertex_key: vk() },
        SweepEvent::HighBend { site: Point::new(7.0, 8.0), obstacle_index: 0, vertex_key: vk() },
        SweepEvent::LowReflection {
            site: Point::new(9.0, 10.0),
            initial_obstacle: 0,
            reflecting_obstacle: 1,
            low_side_obstacle: None,
            prev_event_index: None,
        },
        SweepEvent::HighReflection {
            site: Point::new(11.0, 12.0),
            initial_obstacle: 0,
            reflecting_obstacle: 1,
            high_side_obstacle: None,
            prev_event_index: None,
        },
    ];
    let expected_x = [1.0_f64, 3.0, 5.0, 7.0, 9.0, 11.0];
    for (event, &ex) in events.iter().zip(expected_x.iter()) {
        assert!((event.site().x() - ex).abs() < 1e-10);
    }
}
