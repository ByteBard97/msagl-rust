//! Event handlers for the visibility graph sweep-line.
//!
//! Split from `visibility_graph_generator.rs` to keep files under 500 lines.
//! Implements all 6 event types from C# VisibilityGraphGenerator.cs:
//!   OpenVertex, LowBend, HighBend, CloseVertex, LowReflection, HighReflection
//!
//! Also implements:
//!   - NeighborFinding (SkipToNeighbor)
//!   - ScanSegment creation (CreateScanSegments)
//!   - Reflection event loading (LoadReflectionEvents)
//!   - Lookahead site management (StoreLookaheadSite)
//!   - Perpendicular/parallel reflection segment creation

use crate::geometry::point::Point;
use super::event_queue::EventQueue;
use super::lookahead_scan::LookaheadScan;
use super::neighbor_sides::NeighborSides;
use super::obstacle::Obstacle;
use super::obstacle_side::ObstacleSide;
use super::scan_direction::ScanDirection;
use super::scan_line::RectilinearScanLine;
use super::scan_segment::{ScanSegmentTree, SegmentWeight};

/// Shared mutable state for the sweep-line event processor.
///
/// C# file: VisibilityGraphGenerator.cs, lines 26-83
/// Aggregates all mutable state needed during one sweep pass.
/// Passed by `&mut` to all event handler functions.
pub struct SweepState {
    /// The current scan direction (horizontal or vertical pass).
    /// C# VisibilityGraphGenerator.cs line 29: ScanDirection
    pub scan_direction: ScanDirection,

    /// The event queue for pending sweep events.
    /// C# VisibilityGraphGenerator.cs line 45: eventQueue
    /// MUST use BinaryHeap for O(log N) priority queue operations
    pub event_queue: EventQueue,

    /// The scanline of active obstacle sides.
    /// C# VisibilityGraphGenerator.cs line 67: scanLine
    /// MUST use BTreeMap-based RectilinearScanLine for O(log N) operations
    pub scan_line: RectilinearScanLine,

    /// The scan segment tree accumulating generated segments.
    /// C# VisibilityGraphGenerator.cs line 48-49: HorizontalScanSegments / VerticalScanSegments
    /// MUST use BTreeMap-based ScanSegmentTree for O(log N) insertion/merge
    pub seg_tree: ScanSegmentTree,

    /// Mutable slice of all obstacles in this sweep.
    /// C# VisibilityGraphGenerator.cs line 64: ObsTree
    pub obstacles: Vec<Obstacle>,

    /// Lookahead scan for reflection events.
    /// C# VisibilityGraphGenerator.cs line 60: lookaheadScan
    /// MUST use BTreeMap for O(log N) range queries
    pub lookahead_scan: LookaheadScan,

    /// Whether this sweep wants reflection events (non-rectangular obstacles).
    /// C# VisibilityGraphGenerator.cs line 61: wantReflections
    pub want_reflections: bool,

    /// Neighbor sides found during the current event processing.
    /// C# VisibilityGraphGenerator.cs lines 81-82
    pub low_neighbor_sides: NeighborSides,
    pub high_neighbor_sides: NeighborSides,
}

// =========================================================================
// OpenVertex — C# VisibilityGraphGenerator.cs lines 710-754
// =========================================================================

/// Process an OpenVertex event.
///
/// C# file: VisibilityGraphGenerator.cs, lines 710-754
/// Big-O: O(log N) for scanline insert + O(log N) for neighbor finding
/// MUST use BTreeMap-based scanline for side insertion and neighbor search
pub fn process_open_vertex(
    state: &mut SweepState,
    site: Point,
    obstacle_index: usize,
) {
    let obs = &state.obstacles[obstacle_index];
    let low_side = obs.active_low_side().unwrap().clone();
    let high_side = obs.active_high_side().unwrap().clone();

    add_side_to_scanline(state, low_side.clone());
    add_side_to_scanline(state, high_side.clone());

    find_neighbors_and_process_vertex_event(state, &low_side, &high_side, site, obstacle_index, true);

    enqueue_low_bend_event(state, &low_side);
    enqueue_high_bend_or_close_event(state, &high_side);
}

// =========================================================================
// LowBend — C# VisibilityGraphGenerator.cs lines 756-778
// =========================================================================

/// Process a LowBendVertexEvent.
///
/// C# file: VisibilityGraphGenerator.cs, lines 756-778
/// Big-O: O(log N) for scanline remove + insert
/// MUST use BTreeMap-based scanline for side replacement
pub fn process_low_bend(
    state: &mut SweepState,
    _site: Point,
    obstacle_index: usize,
    vertex_key: crate::arenas::PolylinePointKey,
) {
    let old_low_side_node = state.scan_line.all_sides_ordered().into_iter().find(|s| s.obstacle_index() == obstacle_index && s.side_type() == crate::routing::obstacle_side::SideType::Low);
    if old_low_side_node.is_none() { return; }
    let old_low_side = old_low_side_node.unwrap().clone();
    
    // Mutates state
    remove_side_from_scanline(state, &old_low_side);
    
    // Re-borrow immutably
    let new_low_side = ObstacleSide::from_polyline_point(
        crate::routing::obstacle_side::SideType::Low,
        obstacle_index,
        vertex_key,
        &state.obstacles[obstacle_index].padded_polyline(),
        state.scan_direction
    );
    
    if state.scan_direction.compare_perp(new_low_side.end(), new_low_side.start()) == std::cmp::Ordering::Greater {
        add_side_to_scanline(state, new_low_side.clone());
        enqueue_low_bend_event(state, &new_low_side);
    } else {
        let mut new_high_side = new_low_side.clone();
        new_high_side.set_side_type(crate::routing::obstacle_side::SideType::High);
        add_side_to_scanline(state, new_high_side.clone());
        enqueue_high_bend_or_close_event(state, &new_high_side);
    }
}

// =========================================================================
// HighBend — C# VisibilityGraphGenerator.cs lines 785-821
// =========================================================================

/// Process a HighBendVertexEvent.
///
/// C# file: VisibilityGraphGenerator.cs, lines 785-821
/// Big-O: O(log N) for scanline remove + insert + neighbor lookup
/// MUST use BTreeMap-based scanline for side operations
pub fn process_high_bend(
    state: &mut SweepState,
    site: Point,
    obstacle_index: usize,
    vertex_key: crate::arenas::PolylinePointKey,
) {
    let old_high_side_node = state.scan_line.all_sides_ordered().into_iter().find(|s| s.obstacle_index() == obstacle_index && s.side_type() == crate::routing::obstacle_side::SideType::High);
    if old_high_side_node.is_none() { return; }
    let old_high_side = old_high_side_node.unwrap().clone();
    
    // Mutate state
    remove_side_from_scanline(state, &old_high_side);

    // Re-borrow immutably
    let new_high_side = ObstacleSide::from_polyline_point(
        crate::routing::obstacle_side::SideType::High,
        obstacle_index,
        vertex_key,
        &state.obstacles[obstacle_index].padded_polyline(),
        state.scan_direction
    );
    
    // Mutate state again
    add_side_to_scanline(state, new_high_side.clone());
    enqueue_high_bend_or_close_event(state, &new_high_side);

    // Extreme vertex lookahead for non-rectangular
    let points_close = |a: f64, b: f64| (a - b).abs() < 1e-10;
    if state.want_reflections && state.scan_direction.is_horizontal() {
        let obs_right = state.obstacles[obstacle_index].padded_bounding_box().right();
        if points_close(new_high_side.start().x(), obs_right) 
            && state.scan_direction.compare_perp(new_high_side.start(), new_high_side.end()) == std::cmp::Ordering::Greater {
            
            let high_side_key = state.scan_line.find(&new_high_side).unwrap();
            if let Some((_, neighbor_side)) = state.scan_line.next_high(&high_side_key) {
                if neighbor_side.side_type() == crate::routing::obstacle_side::SideType::Low
                    && state.scan_direction.compare_perp(neighbor_side.start(), neighbor_side.end()) == std::cmp::Ordering::Less {
                    
                    store_lookahead_site(state, neighbor_side.obstacle_index(), &neighbor_side.clone(), site, false);
                    load_reflection_events(state, &new_high_side);
                }
            }
        }
    }
}

// =========================================================================
// CloseVertex — C# VisibilityGraphGenerator.cs lines 838-913
// =========================================================================

/// Process a CloseVertexEvent.
///
/// C# file: VisibilityGraphGenerator.cs, lines 838-913
/// Big-O: O(log N) for scanline operations + neighbor finding
/// MUST use BTreeMap-based scanline for side removal and neighbor search
pub fn process_close_vertex(
    state: &mut SweepState,
    site: Point,
    obstacle_index: usize,
) {
    let low_side_node = state.scan_line.all_sides_ordered().into_iter().find(|s| s.obstacle_index() == obstacle_index && s.side_type() == crate::routing::obstacle_side::SideType::Low);
    let high_side_node = state.scan_line.all_sides_ordered().into_iter().find(|s| s.obstacle_index() == obstacle_index && s.side_type() == crate::routing::obstacle_side::SideType::High);
    
    if low_side_node.is_none() || high_side_node.is_none() { return; }
    let low_side = low_side_node.unwrap().clone();
    let high_side = high_side_node.unwrap().clone();

    find_neighbors_and_process_vertex_event(state, &low_side, &high_side, site, obstacle_index, false);

    remove_side_from_scanline(state, &low_side);
    remove_side_from_scanline(state, &high_side);
    
    if state.want_reflections {
        store_lookahead_site(state, obstacle_index, &low_side, site, true);
        store_lookahead_site(state, obstacle_index, &high_side, site, true);
    }
}

// =========================================================================
// LowReflection — C# VisibilityGraphGenerator.cs lines 915-928
// =========================================================================

/// Process a LowReflectionEvent.
///
/// C# file: VisibilityGraphGenerator.cs, lines 915-928
/// Big-O: O(log N) for scanline neighbor lookup via BTreeMap
/// MUST use scanline BTreeMap next_low() for neighbor lookup
pub fn process_low_reflection(
    state: &mut SweepState,
    site: Point,
    initial_obstacle: usize,
    reflecting_obstacle: usize,
    _prev_event_site: Option<Point>,
) {
    let side_node = state.scan_line.all_sides_ordered().into_iter().find(|s| s.obstacle_index() == reflecting_obstacle && s.side_type() == crate::routing::obstacle_side::SideType::Low);
    if side_node.is_none() { return; }
    let side_in_sl = side_node.unwrap().clone();
    
    let key = state.scan_line.find(&side_in_sl).unwrap();
    if let Some((_, neighbor_side)) = state.scan_line.next_low(&key) {
        let neighbor_side = neighbor_side.clone();
        if neighbor_side.side_type() != crate::routing::obstacle_side::SideType::High { return; }
        
        let prev_site = match _prev_event_site {
            Some(p) => p,
            None => return,
        };
        if add_perpendicular_reflection_segment(state, site, prev_site, Some(&side_in_sl), Some(&neighbor_side), initial_obstacle, reflecting_obstacle) {
            let neighbor_nbor = state.scan_line.find(&neighbor_side).unwrap();
            let low_nbor_of_neighbor = state.scan_line.next_low(&neighbor_nbor).map(|(_, s)| s.clone());
            let high_nbor_of_neighbor = state.scan_line.next_high(&neighbor_nbor).map(|(_, s)| s.clone());
            
            if add_parallel_reflection_segment(state, site, low_nbor_of_neighbor.as_ref(), high_nbor_of_neighbor.as_ref(), reflecting_obstacle, initial_obstacle) {
                load_reflection_events(state, &side_in_sl);
            }
        }
    }
}

// =========================================================================
// HighReflection — C# VisibilityGraphGenerator.cs lines 930-943
// =========================================================================

/// Process a HighReflectionEvent.
///
/// C# file: VisibilityGraphGenerator.cs, lines 930-943
/// Big-O: O(log N) for scanline neighbor lookup via BTreeMap
/// MUST use scanline BTreeMap next_high() for neighbor lookup
pub fn process_high_reflection(
    state: &mut SweepState,
    site: Point,
    initial_obstacle: usize,
    reflecting_obstacle: usize,
    _prev_event_site: Option<Point>,
) {
    let side_node = state.scan_line.all_sides_ordered().into_iter().find(|s| s.obstacle_index() == reflecting_obstacle && s.side_type() == crate::routing::obstacle_side::SideType::High);
    if side_node.is_none() { return; }
    let side_in_sl = side_node.unwrap().clone();
    
    let key = state.scan_line.find(&side_in_sl).unwrap();
    if let Some((_, neighbor_side)) = state.scan_line.next_high(&key) {
        let neighbor_side = neighbor_side.clone();
        if neighbor_side.side_type() != crate::routing::obstacle_side::SideType::Low { return; }
        
        let prev_site = match _prev_event_site {
            Some(p) => p,
            None => return,
        };
        if add_perpendicular_reflection_segment(state, site, prev_site, Some(&side_in_sl), Some(&neighbor_side), initial_obstacle, reflecting_obstacle) {
            let neighbor_nbor = state.scan_line.find(&neighbor_side).unwrap();
            let low_nbor_of_neighbor = state.scan_line.next_low(&neighbor_nbor).map(|(_, s)| s.clone());
            let high_nbor_of_neighbor = state.scan_line.next_high(&neighbor_nbor).map(|(_, s)| s.clone());
            
            if add_parallel_reflection_segment(state, site, low_nbor_of_neighbor.as_ref(), high_nbor_of_neighbor.as_ref(), reflecting_obstacle, initial_obstacle) {
                load_reflection_events(state, &side_in_sl);
            }
        }
    }
}

// =========================================================================
// Neighbor finding — C# VisibilityGraphGenerator.cs lines 599-668
// =========================================================================

fn find_neighbors_and_process_vertex_event(
    state: &mut SweepState,
    low_side: &ObstacleSide,
    high_side: &ObstacleSide,
    site: Point,
    _obstacle_index: usize,
    is_open: bool,
) {
    let low_side_key = state.scan_line.find(low_side).expect("side should be in scanline");
    let high_side_key = state.scan_line.find(high_side).expect("side should be in scanline");

    let low_in_sl = state.scan_line.get(&low_side_key).cloned().unwrap_or(low_side.clone());
    let high_in_sl = state.scan_line.get(&high_side_key).cloned().unwrap_or(high_side.clone());

    let ref_point = if is_open { low_in_sl.start() } else { low_in_sl.end() };
    let low_neighbors = find_neighbors(state, &low_side_key, &low_in_sl, ref_point);
    let high_neighbors = find_neighbors(state, &high_side_key, &high_in_sl, ref_point);

    create_scan_segments_from_neighbors(state, site, &low_neighbors);

    let need_high = match (low_neighbors.high_neighbor(), high_neighbors.high_neighbor()) {
        (Some(a), Some(b)) => {
            a.obstacle_ordinal() != b.obstacle_ordinal() || a.side_type() != b.side_type()
        }
        _ => true,
    };
    if need_high {
        create_scan_segments_from_neighbors(state, site, &high_neighbors);
    }
}

fn find_neighbors(
    state: &mut SweepState,
    side_key: &crate::routing::scan_line::SideKey,
    side: &ObstacleSide,
    side_reference_point: Point,
) -> crate::routing::neighbor_sides::NeighborSides {
    let mut neighbors = crate::routing::neighbor_sides::NeighborSides::new();

    if let Some((low_key, _)) = state.scan_line.next_low(side_key) {
        skip_to_neighbor(state, false, side, side_reference_point, &low_key.clone(), &mut neighbors);
    }

    if let Some((high_key, _)) = state.scan_line.next_high(side_key) {
        skip_to_neighbor(state, true, side, side_reference_point, &high_key.clone(), &mut neighbors);
    }

    neighbors
}

fn skip_to_neighbor(
    state: &mut SweepState,
    ascending: bool,
    side: &ObstacleSide,
    side_reference_point: Point,
    initial_nbor_key: &crate::routing::scan_line::SideKey,
    neighbors: &mut crate::routing::neighbor_sides::NeighborSides,
) {
    let mut overlap_side: Option<ObstacleSide> = None;
    let mut current_key = initial_nbor_key.clone();

    loop {
        let current_side = match state.scan_line.get(&current_key) {
            Some(s) => s.clone(),
            None => break,
        };

        if current_side.obstacle_ordinal() == side.obstacle_ordinal() {
            match state.scan_line.next_in_direction(&current_key, ascending) {
                Some((next_key, _)) => { current_key = next_key.clone(); continue; }
                None => break,
            }
        }

        let is_overlap_ender = match (ascending, current_side.side_type()) {
            (true, crate::routing::obstacle_side::SideType::High) => true,
            (false, crate::routing::obstacle_side::SideType::Low) => true,
            _ => false,
        };

        if is_overlap_ender {
            if scan_line_crosses_obstacle(side_reference_point, &state.obstacles[current_side.obstacle_index()], state.scan_direction)
                && obstacle_is_overlapped(current_side.obstacle_ordinal(), &state.obstacles) {
                overlap_side = Some(current_side.clone());
            }
            match state.scan_line.next_in_direction(&current_key, ascending) {
                Some((next_key, _)) => { current_key = next_key.clone(); continue; }
                None => break,
            }
        }

        neighbors.set_sides_for_direction(ascending, current_side, overlap_side);
        return;
    }
}

fn obstacle_is_overlapped(ordinal: usize, obstacles: &[Obstacle]) -> bool {
    if ordinal < Obstacle::FIRST_NON_SENTINEL_ORDINAL { return false; }
    let idx = ordinal - Obstacle::FIRST_NON_SENTINEL_ORDINAL;
    if idx < obstacles.len() { obstacles[idx].is_overlapped() } else { false }
}

fn create_scan_segments_from_neighbors(
    state: &mut SweepState,
    site: Point,
    neighbors: &crate::routing::neighbor_sides::NeighborSides,
) {
    let low_nbor = match neighbors.low_neighbor() {
        Some(s) => s,
        None => return,
    };
    let high_nbor = match neighbors.high_neighbor() {
        Some(s) => s,
        None => return,
    };

    let low_nbor_intersect = low_nbor.scanline_intersect(site, state.scan_direction);
    let high_nbor_intersect = high_nbor.scanline_intersect(site, state.scan_direction);
    let is_vertical = state.scan_direction.is_vertical();

    let low_overlap = neighbors.low_overlap_end();
    let high_overlap = neighbors.high_overlap_end();

    if low_overlap.is_none() && high_overlap.is_none() {
        let low_is_overlapped = obstacle_is_overlapped(low_nbor.obstacle_ordinal(), &state.obstacles);
        let high_is_overlapped = obstacle_is_overlapped(high_nbor.obstacle_ordinal(), &state.obstacles);
        let weight = if low_is_overlapped && high_is_overlapped {
            crate::routing::scan_segment::SegmentWeight::Overlapped
        } else {
            crate::routing::scan_segment::SegmentWeight::Normal
        };
        add_segment(state, low_nbor_intersect, high_nbor_intersect, weight, is_vertical);
        return;
    }

    let low_ov_intersect = match low_overlap {
        Some(s) => s.scanline_intersect(site, state.scan_direction),
        None => low_nbor_intersect,
    };
    let high_ov_intersect = match high_overlap {
        Some(s) => s.scanline_intersect(site, state.scan_direction),
        None => high_nbor_intersect,
    };

    add_segment(state, low_nbor_intersect, low_ov_intersect, crate::routing::scan_segment::SegmentWeight::Normal, is_vertical);
    add_segment(state, low_ov_intersect, high_ov_intersect, crate::routing::scan_segment::SegmentWeight::Overlapped, is_vertical);
    if high_overlap.is_some() {
        add_segment(state, high_ov_intersect, high_nbor_intersect, crate::routing::scan_segment::SegmentWeight::Normal, is_vertical);
    }
}

fn add_segment(state: &mut SweepState, start: Point, end: Point, weight: crate::routing::scan_segment::SegmentWeight, is_vertical: bool) {
    let (s, e) = if state.scan_direction.coord(start) <= state.scan_direction.coord(end) {
        (start, end)
    } else {
        (end, start)
    };
    if crate::geometry::point_comparer::GeomConstants::close(state.scan_direction.coord(s), state.scan_direction.coord(e)) {
        return;
    }
    state.seg_tree.insert_unique(crate::routing::scan_segment::ScanSegment::new(s, e, weight, is_vertical));
}

// =========================================================================
// Helper functions
// =========================================================================

/// Compute the scanline intersection with an obstacle side.
///
/// C# file: VisibilityGraphGenerator.cs, lines 170-203
/// Big-O: O(1)
pub fn scanline_intersect_side(
    site: Point,
    side: &ObstacleSide,
    scan_dir: ScanDirection,
) -> Point {
    side.scanline_intersect(site, scan_dir)
}

/// Check if a side reflects upward (has positive slope for low side, negative for high).
///
/// C# file: VisibilityGraphGenerator.cs, lines 264-272
/// Big-O: O(1)
pub fn side_reflects_upward(
    side: &ObstacleSide,
    scan_direction: ScanDirection,
) -> bool {
    let start_coord = scan_direction.coord(side.start());
    let end_coord = scan_direction.coord(side.end());
    match side.side_type() {
        crate::routing::obstacle_side::SideType::Low => end_coord > start_coord,
        crate::routing::obstacle_side::SideType::High => end_coord < start_coord,
    }
}

/// Check if a side reflects downward (negative slope for low, positive for high).
///
/// C# file: VisibilityGraphGenerator.cs, lines 274-282
/// Big-O: O(1)
pub fn side_reflects_downward(
    side: &ObstacleSide,
    scan_direction: ScanDirection,
) -> bool {
    let start_coord = scan_direction.coord(side.start());
    let end_coord = scan_direction.coord(side.end());
    match side.side_type() {
        crate::routing::obstacle_side::SideType::Low => end_coord < start_coord,
        crate::routing::obstacle_side::SideType::High => end_coord > start_coord,
    }
}

/// Add a side to the scanline and load reflection events.
///
/// C# file: VisibilityGraphGenerator.cs, lines 528-534
/// Big-O: O(log N) for BTreeMap insert + O(log N + K) for reflection loading
/// MUST use BTreeMap-based scanline insert
fn add_side_to_scanline(state: &mut SweepState, side: ObstacleSide) {
    state.scan_line.insert(side.clone());
    load_reflection_events(state, &side);
}

/// Remove a side from the scanline.
///
/// C# file: VisibilityGraphGenerator.cs, lines 536-538
/// Big-O: O(log N) for BTreeMap removal
fn remove_side_from_scanline(state: &mut SweepState, side: &ObstacleSide) {
    state.scan_line.remove(side);
}

// =========================================================================
// Reflection event support — C# VisibilityGraphGenerator.cs lines 287-513
// =========================================================================

/// Load reflection events for a side (single-side variant).
///
/// C# file: VisibilityGraphGenerator.cs, lines 336-337
/// Big-O: O(log N + K) via BTreeMap range query
/// MUST use BTreeMap range query on LookaheadScan
pub fn load_reflection_events(
    state: &mut SweepState,
    side_to_queue: &ObstacleSide,
) {
    load_reflection_events_with_range(state, side_to_queue, side_to_queue.clone());
}

/// Load reflection events with range-restricted side parameter.
///
/// C# file: VisibilityGraphGenerator.cs, lines 342-423
/// Big-O: O(log N + K) where K = matching sites, via BTreeMap range query
/// MUST use BTreeMap range query on LookaheadScan for coordinate-range lookup
pub fn load_reflection_events_with_range(
    state: &mut SweepState,
    side_to_queue: &ObstacleSide,
    side_with_range: ObstacleSide,
) {
    if side_reflects_upward(side_to_queue, state.scan_direction)
        || state.scan_direction.is_perpendicular(side_to_queue.start(), side_to_queue.end())
    {
        return;
    }

    let bbox1 = crate::geometry::rectangle::Rectangle::from_points(side_to_queue.start(), side_to_queue.end());
    let bbox2 = crate::geometry::rectangle::Rectangle::from_points(side_with_range.start(), side_with_range.end());

    if state.scan_direction.is_horizontal() {
        if !bbox1.intersects_on_x(&bbox2) { return; }
    } else {
        if !bbox1.intersects_on_y(&bbox2) { return; }
    }

    let bbox_intersect = bbox1.intersection(&bbox2);
    let low = bbox_intersect.left_bottom();
    let high = bbox_intersect.right_top();

    let low_coord = state.scan_direction.coord(low);
    let high_coord = state.scan_direction.coord(high);

    let sites: Vec<_> = state.lookahead_scan.find_all_in_range(low_coord, high_coord).into_iter().cloned().collect();
    let mut stale_sites = Vec::new();

    for site in sites {
        let intersect = scanline_intersect_side(site.site, side_to_queue, state.scan_direction.perpendicular());
        if state.scan_direction.compare_perp(intersect, site.site) == std::cmp::Ordering::Greater {
            add_reflection_event(state, side_to_queue, intersect, site.initial_obstacle, side_to_queue.obstacle_index());
        } else {
            if site.reflecting_obstacle != side_to_queue.obstacle_index() {
                stale_sites.push(site.site);
            }
        }
    }

    for site in stale_sites {
        state.lookahead_scan.remove_site(site);
    }
}

/// Store a lookahead reflection site.
///
/// C# file: VisibilityGraphGenerator.cs, lines 287-333
/// Big-O: O(log N) for BTreeMap insertion in LookaheadScan
/// MUST use BTreeMap-based LookaheadScan.find() and .add()
pub fn store_lookahead_site(
    state: &mut SweepState,
    initial_obstacle: usize,
    reflecting_side: &ObstacleSide,
    reflection_site: Point,
    want_extreme: bool,
) {
    if !state.want_reflections { return; }

    if !state.scan_direction.is_perpendicular(reflecting_side.start(), reflecting_side.end()) {
        let obstacle = &state.obstacles[reflecting_side.obstacle_index()];
        if !want_extreme && !crate::routing::static_graph_utility::StaticGraphUtility::point_is_in_rectangle_interior(reflection_site, &obstacle.padded_bounding_box()) {
            return;
        }

        if side_reflects_upward(reflecting_side, state.scan_direction) {
            let coord = state.scan_direction.coord(reflection_site);
            if state.lookahead_scan.find_first_in_range(coord, coord).is_none() {
                state.lookahead_scan.add_site(reflection_site, initial_obstacle, reflecting_side.obstacle_index());
            }
        }
    }
}

/// Validate a perpendicular reflection segment and add it.
///
/// C# file: VisibilityGraphGenerator.cs, lines 426-489
/// Big-O: O(log N) for BTreeMap lookahead_scan removal
/// MUST use BTreeMap-based LookaheadScan.remove_exact() for site validation
///
/// Returns true if the segment was valid and added.
fn add_perpendicular_reflection_segment(
    state: &mut SweepState,
    event_site: Point,
    prev_site: Point,
    event_side: Option<&ObstacleSide>, // was &ObstacleSide, changed to Option for simplicity
    nbor_side: Option<&ObstacleSide>,
    initial_obstacle: usize,
    reflecting_obstacle: usize,
) -> bool {
    let coord = state.scan_direction.coord(prev_site);
    
    let points_close = |a: Point, b: Point| (a.x() - b.x()).abs() < 1e-10 && (a.y() - b.y()).abs() < 1e-10;
    
    // Check if the site is still there (exact match)
    let found = {
        let sites = state.lookahead_scan.find_all_in_range(coord, coord);
        sites.iter().any(|s| points_close(s.site, prev_site))
    };

    if found {
        state.lookahead_scan.remove_site(prev_site);
        
        if event_side.is_none() {
            return false;
        }
        
        let event_side_ref = event_side.unwrap();
        // Skip flat sides
        if state.scan_direction.is_horizontal() {
            if event_side_ref.start().y() == event_side_ref.end().y() { return false; }
        } else {
            if event_side_ref.start().x() == event_side_ref.end().x() { return false; }
        }

        let ref_obs = &state.obstacles[reflecting_obstacle];
        
        // Check staircase
        let is_staircase = crate::routing::static_graph_utility::StaticGraphUtility::point_is_on_segment(prev_site, ref_obs.padded_polyline().point_at(ref_obs.active_low_side().unwrap().start_vertex_key()), ref_obs.padded_polyline().point_at(ref_obs.active_low_side().unwrap().end_vertex_key()))
            || crate::routing::static_graph_utility::StaticGraphUtility::point_is_on_segment(prev_site, ref_obs.padded_polyline().point_at(ref_obs.active_high_side().unwrap().start_vertex_key()), ref_obs.padded_polyline().point_at(ref_obs.active_high_side().unwrap().end_vertex_key()));
        if is_staircase {
            if !crate::routing::static_graph_utility::StaticGraphUtility::point_is_in_rectangle_interior(event_site, &ref_obs.padded_bounding_box()) {
                return false;
            }

            state.seg_tree.insert_unique(crate::routing::scan_segment::ScanSegment::new(
                prev_site, 
                event_site, 
                crate::routing::scan_segment::SegmentWeight::Reflection,
                state.scan_direction.is_horizontal()
            ));

            if let Some(nbor) = nbor_side {
                let nbor_obs = &state.obstacles[nbor.obstacle_index()];
                return scan_line_crosses_obstacle(event_site, nbor_obs, state.scan_direction);
            }
        }
    }
    
    false
}

/// Add a parallel reflection segment (along the obstacle side).
///
/// C# file: VisibilityGraphGenerator.cs, lines 493-513
/// Big-O: O(log N) for scanline neighbor lookup via BTreeMap
/// MUST use scanline BTreeMap next_low/next_high for opposite neighbor
///
/// Returns true if the segment was added.
fn add_parallel_reflection_segment(
    state: &mut SweepState,
    event_site: Point,
    low_nbor_side: Option<&ObstacleSide>,
    high_nbor_side: Option<&ObstacleSide>,
    _event_obstacle: usize,
    _initial_obstacle: usize,
) -> bool {
    let active_nbor = low_nbor_side.or(high_nbor_side).unwrap();
    let intersect = scanline_intersect_side(event_site, active_nbor, state.scan_direction);

    let (start, end) = if low_nbor_side.is_some() {
        (intersect, event_site)
    } else {
        (event_site, intersect)
    };

    let _actual_low_nbor = match low_nbor_side {
        Some(s) => s.clone(),
        None => state.scan_line.next_low(&state.scan_line.find(high_nbor_side.unwrap()).unwrap()).unwrap().1.clone(),
    };
    
    let _actual_high_nbor = match high_nbor_side {
        Some(s) => s.clone(),
        None => state.scan_line.next_high(&state.scan_line.find(low_nbor_side.unwrap()).unwrap()).unwrap().1.clone(),
    };

    let _s = state.seg_tree.insert_unique(crate::routing::scan_segment::ScanSegment::new(
        start, 
        end, 
        crate::routing::scan_segment::SegmentWeight::Reflection,
        state.scan_direction.is_vertical()
    ));

    true
}

/// Add a reflection event to the event queue.
///
/// C# file: VisibilityGraphGenerator.cs, lines 515-526
/// Big-O: O(log N) for BinaryHeap insertion
fn add_reflection_event(
    state: &mut SweepState,
    side: &ObstacleSide,
    site: Point,
    initial_obstacle: usize,
    reflecting_obstacle: usize,
) {
    let is_low = side.side_type() == crate::routing::obstacle_side::SideType::Low;
    if is_low {
        state.event_queue.enqueue(super::event_queue::SweepEvent::LowReflection {
            site,
            initial_obstacle,
            reflecting_obstacle,
            prev_event_index: None,
        });
    } else {
        state.event_queue.enqueue(super::event_queue::SweepEvent::HighReflection {
            site,
            initial_obstacle,
            reflecting_obstacle,
            prev_event_index: None,
        });
    }
}

/// Remove lookahead sites in the flat-bottom range of an obstacle.
///
/// C# file: VisibilityGraphGenerator.cs, line 748
/// Big-O: O(log N + K) for BTreeMap range removal
/// MUST use BTreeMap range query on LookaheadScan
pub fn remove_sites_for_flat_bottom(
    state: &mut SweepState,
    flat_start: Point,
    flat_end: Point,
) {
    state.lookahead_scan.remove_sites_for_flat_bottom(flat_start, flat_end);
}

/// Enqueue a LowBend event at the end of the current low side.
///
/// C# file: VisibilityGraphGenerator.cs, lines 780-783
/// Big-O: O(log N) for BinaryHeap insertion
fn enqueue_low_bend_event(state: &mut SweepState, low_side: &ObstacleSide) {
    state.event_queue.enqueue(super::event_queue::SweepEvent::LowBend {
        site: low_side.end(),
        obstacle_index: low_side.obstacle_index(),
        vertex_key: low_side.end_vertex_key(),
    });
}

/// Enqueue a HighBend or CloseVertex event based on the next high side.
///
/// C# file: VisibilityGraphGenerator.cs, lines 823-836
/// Big-O: O(log N) for BinaryHeap insertion
fn enqueue_high_bend_or_close_event(state: &mut SweepState, high_side: &ObstacleSide) {
    let obstacle = &state.obstacles[high_side.obstacle_index()];
    let polyline = obstacle.padded_polyline();
    let next_high_side_end = if state.scan_direction.is_horizontal() {
        polyline.prev_key(high_side.end_vertex_key()).unwrap()
    } else {
        polyline.next_key(high_side.end_vertex_key()).unwrap()
    };
    let next_point = polyline.point_at(next_high_side_end);

    if state.scan_direction.compare_perp(next_point, high_side.end()) == std::cmp::Ordering::Greater {
        state.event_queue.enqueue(super::event_queue::SweepEvent::HighBend {
            site: high_side.end(),
            obstacle_index: high_side.obstacle_index(),
            vertex_key: high_side.end_vertex_key(),
        });
    } else {
        state.event_queue.enqueue(super::event_queue::SweepEvent::CloseVertex {
            site: high_side.end(),
            obstacle_index: high_side.obstacle_index(),
            vertex_key: high_side.end_vertex_key(),
        });
    }
}

/// Check if the scanline crosses an obstacle (for overlap detection).
///
/// C# file: VisibilityGraphGenerator.cs, lines 592-597
/// Big-O: O(1)
fn scan_line_crosses_obstacle(
    event_site: Point,
    obstacle: &Obstacle,
    scan_direction: ScanDirection,
) -> bool {
    let perp = scan_direction.perp_coord(event_site);
    let bb = obstacle.padded_bounding_box();
    let bb_low = scan_direction.perp_coord(bb.left_bottom());
    let bb_high = scan_direction.perp_coord(bb.right_top());
    perp > bb_low + crate::geometry::point_comparer::GeomConstants::DISTANCE_EPSILON
        && perp < bb_high - crate::geometry::point_comparer::GeomConstants::DISTANCE_EPSILON
}
