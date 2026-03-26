use msagl_rust::routing::vertex_entry::{VertexEntry, VertexEntryIndex, VertexEntryArena};
use msagl_rust::routing::compass_direction::CompassDirection;
use msagl_rust::visibility::graph::VertexId;

#[test]
fn vertex_entry_new_source() {
    // Source entry has no direction and no previous entry
    let entry = VertexEntry::new(VertexId(5), None, None, 0.0, 0, 0.0);
    assert_eq!(entry.vertex, VertexId(5));
    assert_eq!(entry.direction, None);
    assert!(entry.previous_entry.is_none());
    assert!((entry.length - 0.0).abs() < 1e-10);
    assert_eq!(entry.number_of_bends, 0);
    assert!(!entry.is_closed);
}

#[test]
fn vertex_entry_new_with_direction() {
    let entry = VertexEntry::new(
        VertexId(5),
        Some(CompassDirection::East),
        None,
        100.0,
        0,
        100.0,
    );
    assert_eq!(entry.vertex, VertexId(5));
    assert_eq!(entry.direction, Some(CompassDirection::East));
    assert!((entry.length - 100.0).abs() < 1e-10);
}

#[test]
fn vertex_entry_with_previous() {
    let entry = VertexEntry::new(
        VertexId(10),
        Some(CompassDirection::North),
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
    let mut entry = VertexEntry::new(
        VertexId(5),
        Some(CompassDirection::East),
        None,
        100.0,
        0,
        100.0,
    );
    entry.reset_entry(Some(VertexEntryIndex(7)), 80.0, 1, 84.0);
    assert_eq!(entry.previous_entry, Some(VertexEntryIndex(7)));
    assert!((entry.length - 80.0).abs() < 1e-10);
    assert!((entry.cost - 84.0).abs() < 1e-10);
    // direction and vertex unchanged
    assert_eq!(entry.direction, Some(CompassDirection::East));
    assert_eq!(entry.vertex, VertexId(5));
}

#[test]
fn arena_create_and_get() {
    let mut arena = VertexEntryArena::new();
    assert!(arena.is_empty());

    let idx0 = arena.create_entry(VertexId(0), None, None, 0.0, 0, 0.0);
    assert_eq!(idx0, VertexEntryIndex(0));
    assert_eq!(arena.len(), 1);

    let idx1 = arena.create_entry(
        VertexId(1),
        Some(CompassDirection::East),
        Some(idx0),
        10.0,
        0,
        10.0,
    );
    assert_eq!(idx1, VertexEntryIndex(1));
    assert_eq!(arena.len(), 2);

    let e0 = arena.get(idx0);
    assert_eq!(e0.vertex, VertexId(0));
    assert_eq!(e0.direction, None);

    let e1 = arena.get(idx1);
    assert_eq!(e1.vertex, VertexId(1));
    assert_eq!(e1.direction, Some(CompassDirection::East));
    assert_eq!(e1.previous_entry, Some(idx0));
}

#[test]
fn arena_previous_vertex() {
    let mut arena = VertexEntryArena::new();

    let idx0 = arena.create_entry(VertexId(5), None, None, 0.0, 0, 0.0);
    assert_eq!(arena.previous_vertex(idx0), None);

    let idx1 = arena.create_entry(
        VertexId(10),
        Some(CompassDirection::South),
        Some(idx0),
        20.0,
        0,
        20.0,
    );
    assert_eq!(arena.previous_vertex(idx1), Some(VertexId(5)));
}

#[test]
fn arena_clear() {
    let mut arena = VertexEntryArena::new();
    arena.create_entry(VertexId(0), None, None, 0.0, 0, 0.0);
    arena.create_entry(VertexId(1), Some(CompassDirection::East), None, 5.0, 0, 5.0);
    assert_eq!(arena.len(), 2);

    arena.clear();
    assert_eq!(arena.len(), 0);
    assert!(arena.is_empty());
}

#[test]
fn direction_index_matches_vertex_entries_array() {
    assert_eq!(CompassDirection::North.index(), 0);
    assert_eq!(CompassDirection::East.index(), 1);
    assert_eq!(CompassDirection::South.index(), 2);
    assert_eq!(CompassDirection::West.index(), 3);
}
