use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::routing::vertex_entry::{VertexEntry, VertexEntryIndex};
use msagl_rust::visibility::graph::VertexId;

#[test]
fn vertex_entry_new() {
    let entry = VertexEntry::new(VertexId(5), CompassDirection::East, None, 100.0, 0, 100.0);
    assert_eq!(entry.vertex, VertexId(5));
    assert_eq!(entry.direction, CompassDirection::East);
    assert!(entry.previous_entry.is_none());
    assert!((entry.length - 100.0).abs() < 1e-10);
    assert_eq!(entry.number_of_bends, 0);
    assert!(!entry.is_closed);
}

#[test]
fn vertex_entry_with_previous() {
    let entry = VertexEntry::new(
        VertexId(10),
        CompassDirection::North,
        Some(VertexEntryIndex(3)),
        200.0,
        1,
        208.0,
    );
    assert_eq!(entry.previous_entry, Some(VertexEntryIndex(3)));
    assert_eq!(entry.number_of_bends, 1);
}

#[test]
fn vertex_entry_reset() {
    let mut entry = VertexEntry::new(VertexId(5), CompassDirection::East, None, 100.0, 0, 100.0);
    entry.reset(Some(VertexEntryIndex(7)), 80.0, 1, 84.0);
    assert_eq!(entry.previous_entry, Some(VertexEntryIndex(7)));
    assert!((entry.length - 80.0).abs() < 1e-10);
    assert!((entry.cost - 84.0).abs() < 1e-10);
}
