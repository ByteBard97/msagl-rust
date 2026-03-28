use msagl_rust::routing::compass_direction::Direction;
use msagl_rust::routing::vertex_entry::{VertexEntry, VertexEntryIndex};
use msagl_rust::visibility::graph::VertexId;

#[test]
fn vertex_entry_new() {
    let entry = VertexEntry::new(VertexId(5), Direction::EAST, None, 100.0, 0, 100.0);
    assert_eq!(entry.vertex, VertexId(5));
    assert_eq!(entry.direction, Direction::EAST);
    assert!(entry.previous_entry.is_none());
    assert!((entry.length - 100.0).abs() < 1e-10);
    assert_eq!(entry.number_of_bends, 0);
    assert!(!entry.is_closed);
}

#[test]
fn vertex_entry_with_previous() {
    let entry = VertexEntry::new(
        VertexId(10),
        Direction::NORTH,
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
    let mut entry = VertexEntry::new(VertexId(5), Direction::EAST, None, 100.0, 0, 100.0);
    entry.reset_entry(Some(VertexEntryIndex(7)), 80.0, 1, 84.0);
    assert_eq!(entry.previous_entry, Some(VertexEntryIndex(7)));
    assert!((entry.length - 80.0).abs() < 1e-10);
    assert!((entry.cost - 84.0).abs() < 1e-10);
}

#[test]
fn vertex_entry_source_has_none_direction() {
    // Source entries use Direction::NONE (no incoming direction).
    let entry = VertexEntry::new(VertexId(0), Direction::NONE, None, 0.0, 0, 0.0);
    assert!(entry.direction.is_none());
    assert!(entry.previous_entry.is_none());
}

#[test]
fn vertex_entry_is_closed_default_false() {
    let entry = VertexEntry::new(VertexId(1), Direction::SOUTH, None, 50.0, 0, 50.0);
    assert!(!entry.is_closed);
}
