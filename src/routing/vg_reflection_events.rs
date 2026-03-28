//! Reflection event support for the visibility graph sweep-line.
//!
//! Split from `vg_event_processing.rs` to keep files under 500 lines.
//! Implements reflection event loading and lookahead site management
//! from C# VisibilityGraphGenerator.cs lines 287-526.

use crate::geometry::point::Point;
use super::obstacle_side::ObstacleSide;
use super::vg_event_processing::{SweepState, scanline_intersect_side, side_reflects_upward, scan_line_crosses_obstacle};

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
pub(crate) fn add_perpendicular_reflection_segment(
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
pub(crate) fn add_parallel_reflection_segment(
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
pub(crate) fn add_reflection_event(
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

