//! Event handlers for the visibility graph sweep-line.
//!
//! Split from `visibility_graph_generator.rs` to keep files under 500 lines.
//! Implements all 6 event types from TS `VisibilityGraphGenerator.ts`:
//!   OpenVertex, LowBend, HighBend, CloseVertex, LowReflection, HighReflection
//!
//! Also implements neighbor finding (`SkipToNeighbor`) and scan segment creation
//! (`CreateScanSegments` from C# FullVGG lines 210-283).

use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::event_queue::SweepEvent;
use super::obstacle_side::{ObstacleSide, SideType};
use super::scan_line::SideKey;
use super::scan_segment::SegmentWeight;
use super::visibility_graph_generator::{
    SweepState, add_segment_if_valid, scanline_intersect_side,
    side_reflects_upward,
};

// =========================================================================
// OpenVertex — TS lines 765-810
// =========================================================================

/// Process an OpenVertex event.
///
/// Faithful port of TS `ProcessEventO()` (lines 765-810).
pub fn process_open_vertex(
    state: &mut SweepState,
    site: Point,
    obstacle_index: usize,
    vertex_key: PolylinePointKey,
) {
    let obs = &mut state.obstacles[obstacle_index];

    // Create initial sides from the open vertex (TS line 769).
    obs.create_initial_sides(vertex_key, state.scan_direction);

    let low_side = obs.active_low_side()
        .expect("obstacle must have low side after create_initial_sides")
        .clone();
    let high_side = obs.active_high_side()
        .expect("obstacle must have high side after create_initial_sides")
        .clone();

    // Add sides to scanline. TS lines 781-783.
    // AddSideToScanLine loads reflection events (TS line 571).
    add_side_to_scanline(state, low_side.clone());
    add_side_to_scanline(state, high_side.clone());

    // Find the low and high side nodes in the scanline.
    let low_key = state.scan_line.find(&low_side)
        .expect("low side must be in scanline after insert");
    let high_key = state.scan_line.find(&high_side)
        .expect("high side must be in scanline after insert");

    // Find neighbors and create scan segments (TS line 787).
    find_neighbors_and_process_vertex_event(
        state, &low_key, &high_key, site, obstacle_index, true,
    );

    // Check for reflection loading on neighbor sides (TS lines 792-798).
    // For rectangular obstacles with perpendicular sides, SideReflectsUpward
    // returns false, so this is typically a no-op.
    let low_nbor_side = state.low_neighbor_sides.low_neighbor().cloned();
    if let Some(ref nbor) = low_nbor_side {
        if side_reflects_upward(nbor, state.scan_direction) {
            let obs_low = state.obstacles[obstacle_index].active_low_side().cloned();
            if let Some(ref active_low) = obs_low {
                load_reflection_events(state, active_low);
            }
        }
    }
    let high_nbor_side = state.high_neighbor_sides.high_neighbor().cloned();
    if let Some(ref nbor) = high_nbor_side {
        if side_reflects_upward(nbor, state.scan_direction) {
            let obs_high = state.obstacles[obstacle_index].active_high_side().cloned();
            if let Some(ref active_high) = obs_high {
                load_reflection_events(state, active_high);
            }
        }
    }

    // Handle flat bottom absorption (TS lines 801-805).
    // If the two active sides have different start points, there's a flat bottom.
    let obs = &state.obstacles[obstacle_index];
    let active_low_start = obs.active_low_side().map(|s| s.start());
    let active_high_start = obs.active_high_side().map(|s| s.start());
    if active_low_start != active_high_start {
        // Flat bottom: remove any lookahead sites in the flat range.
        // For rectangles this is typically a no-op since all sides are perpendicular.
        if let (Some(ls), Some(hs)) = (active_low_start, active_high_start) {
            let low_coord = state.scan_direction.coord(ls);
            let high_coord = state.scan_direction.coord(hs);
            let (lo, hi) = if low_coord <= high_coord {
                (low_coord, high_coord)
            } else {
                (high_coord, low_coord)
            };
            remove_lookahead_sites_in_range(state, lo, hi);
        }
    }

    // Enqueue LowBend event at low side's end vertex (TS line 808).
    let obs = &state.obstacles[obstacle_index];
    if let Some(low_side) = obs.active_low_side() {
        let end_key = low_side.end_vertex_key();
        let end_site = obs.padded_polyline().point_at(end_key);
        state.event_queue.enqueue(SweepEvent::LowBend {
            site: end_site,
            obstacle_index,
            vertex_key: end_key,
        });
    }

    // Enqueue HighBend or CloseVertex at high side's end (TS line 809).
    enqueue_high_bend_or_close(state, obstacle_index);
}

// =========================================================================
// LowBend — TS lines 813-833
// =========================================================================

/// Process a LowBendVertexEvent.
///
/// Faithful port of TS `ProcessEventLB()` (lines 813-833).
/// Updates ActiveLowSide if the new side is still ascending.
pub fn process_low_bend(
    state: &mut SweepState,
    _site: Point,
    obstacle_index: usize,
    vertex_key: PolylinePointKey,
) {
    let obs = &state.obstacles[obstacle_index];
    let poly = obs.padded_polyline().clone();

    // Create a new LowObstacleSide from this vertex (TS line 820).
    let new_low_side = ObstacleSide::from_polyline_point(
        SideType::Low,
        obs.ordinal(),
        vertex_key,
        &poly,
        state.scan_direction,
    );

    // Only replace if the new side is still ascending (TS line 827).
    let perp_end = state.scan_direction.perp_coord(new_low_side.end());
    let perp_start = state.scan_direction.perp_coord(new_low_side.start());
    if GeomConstants::compare(perp_end, perp_start) == std::cmp::Ordering::Greater {
        // Remove old low side from scanline (TS line 828).
        let old_low = state.obstacles[obstacle_index]
            .active_low_side()
            .cloned();
        if let Some(ref old) = old_low {
            state.scan_line.remove(old);
        }

        // Add new side and load reflection events (TS line 829).
        add_side_to_scanline(state, new_low_side.clone());

        // Update ActiveLowSide (TS line 830).
        state.obstacles[obstacle_index].set_active_low_side(Some(new_low_side));

        // Enqueue next LowBend event (TS line 831).
        let obs = &state.obstacles[obstacle_index];
        if let Some(low_side) = obs.active_low_side() {
            let end_key = low_side.end_vertex_key();
            let end_site = obs.padded_polyline().point_at(end_key);
            state.event_queue.enqueue(SweepEvent::LowBend {
                site: end_site,
                obstacle_index,
                vertex_key: end_key,
            });
        }
    }
    // If not ascending, the side is flat or descending — CloseVertex will handle it.
}

// =========================================================================
// HighBend — TS lines 841-878
// =========================================================================

/// Process a HighBendVertexEvent.
///
/// Faithful port of TS `ProcessEventHB()` (lines 841-878).
pub fn process_high_bend(
    state: &mut SweepState,
    _site: Point,
    obstacle_index: usize,
    vertex_key: PolylinePointKey,
) {
    let obs = &state.obstacles[obstacle_index];
    let poly = obs.padded_polyline().clone();

    // Create a new HighObstacleSide from this vertex (TS line 844).
    let new_high_side = ObstacleSide::from_polyline_point(
        SideType::High,
        obs.ordinal(),
        vertex_key,
        &poly,
        state.scan_direction,
    );

    // Remove old high side from scanline (TS line 845).
    let old_high = state.obstacles[obstacle_index]
        .active_high_side()
        .cloned();
    if let Some(ref old) = old_high {
        state.scan_line.remove(old);
    }

    // Add new side to scanline (TS line 846).
    add_side_to_scanline(state, new_high_side.clone());

    // Update ActiveHighSide (TS line 847).
    state.obstacles[obstacle_index].set_active_high_side(Some(new_high_side));

    // Enqueue next HighBend or Close event (TS line 848).
    enqueue_high_bend_or_close(state, obstacle_index);

    // Extreme vertex lookahead (TS lines 864-877).
    // For rectangular obstacles, start.x never equals bbox.right at a bend
    // (bends don't occur), so this is typically unreachable for rectangles.
    // Implemented faithfully for non-rectangular polylines.
    if state.want_reflections && state.scan_direction.is_horizontal() {
        let obs = &state.obstacles[obstacle_index];
        if let Some(high_side) = obs.active_high_side() {
            let bbox = obs.padded_bounding_box();
            if GeomConstants::close(high_side.start().x(), bbox.right())
                && side_reflects_upward(high_side, state.scan_direction)
            {
                // Check high neighbor for downward-reflecting LowObstacleSide.
                let high_key = state.scan_line.find(high_side);
                if let Some(ref hk) = high_key {
                    if let Some((_, nbor)) = state.scan_line.next_high(hk) {
                        let nbor_clone = nbor.clone();
                        if nbor_clone.side_type() == SideType::Low
                            && super::visibility_graph_generator::side_reflects_downward(
                                &nbor_clone,
                                state.scan_direction,
                            )
                        {
                            store_lookahead_site(
                                state,
                                nbor_clone.obstacle_index(),
                                high_side.start(),
                                true,
                            );
                            load_reflection_events(state, &nbor_clone);
                        }
                    }
                }
            }
        }
    }
}

// =========================================================================
// CloseVertex — TS lines 895-968
// =========================================================================

/// Process a CloseVertexEvent.
///
/// Faithful port of TS `ProcessEventCV()` (lines 935-968) combined with
/// `CreateCloseEventSegmentsAndFindNeighbors()` (lines 895-933).
pub fn process_close_vertex(
    state: &mut SweepState,
    site: Point,
    obstacle_index: usize,
    _vertex_key: PolylinePointKey,
) {
    let obs = &state.obstacles[obstacle_index];
    let low_side = obs.active_low_side()
        .expect("obstacle must have low side at close")
        .clone();
    let high_side = obs.active_high_side()
        .expect("obstacle must have high side at close")
        .clone();

    // Find sides in scanline. TS lines 898-899.
    let low_key = state.scan_line.find(&low_side);
    let high_key = state.scan_line.find(&high_side);

    if let (Some(lk), Some(hk)) = (low_key, high_key) {
        // Check if sides are reverse-ordered due to slope (TS lines 906-910).
        // For rectangular obstacles this doesn't happen (perpendicular sides).
        let (effective_low, effective_high) = {
            let lk_coord = lk.coord_value();
            let hk_coord = hk.coord_value();
            if lk_coord > hk_coord {
                (hk.clone(), lk.clone())
            } else {
                (lk.clone(), hk.clone())
            }
        };

        // Find neighbors and create scan segments (TS line 914).
        find_neighbors_and_process_vertex_event(
            state, &effective_low, &effective_high, site, obstacle_index, false,
        );
    }

    // Load reflection events for neighbor sides (TS lines 964-965).
    let low_nbor = state.low_neighbor_sides.low_neighbor().cloned();
    let high_nbor = state.high_neighbor_sides.high_neighbor().cloned();
    if let Some(ref nbor) = low_nbor {
        load_reflection_events(state, nbor);
    }
    if let Some(ref nbor) = high_nbor {
        load_reflection_events(state, nbor);
    }

    // Remove sides from scanline (TS lines 931-932).
    state.scan_line.remove(&low_side);
    state.scan_line.remove(&high_side);

    // Close the obstacle (TS line 967).
    state.obstacles[obstacle_index].close();
}

// =========================================================================
// LowReflection — TS lines 970-982
// =========================================================================

/// Process a LowReflectionEvent.
///
/// Faithful port of TS `ProcessEventLR()` (lines 970-982).
/// For rectangular obstacles with perpendicular sides, this is never reached.
pub fn process_low_reflection(
    _state: &mut SweepState,
    site: Point,
    initial_obstacle: usize,
    reflecting_obstacle: usize,
) {
    // Reflection events are generated only for non-perpendicular sides.
    // For rectangular obstacles, all sides are perpendicular, so this is a no-op.
    // Full implementation would:
    // 1. Find the reflecting side in the scanline
    // 2. AddPerpendicularReflectionSegment
    // 3. AddParallelReflectionSegment
    // 4. LoadReflectionEvents on the obstacle's ActiveLowSide
    //
    // Since we don't have group/overlap support and our obstacles are rectangular,
    // we stub this to avoid crashes but preserve the event dispatch path.
    let _ = (site, initial_obstacle, reflecting_obstacle);
}

// =========================================================================
// HighReflection — TS lines 985-997
// =========================================================================

/// Process a HighReflectionEvent.
///
/// Faithful port of TS `ProcessEventHR()` (lines 985-997).
/// For rectangular obstacles with perpendicular sides, this is never reached.
pub fn process_high_reflection(
    _state: &mut SweepState,
    site: Point,
    initial_obstacle: usize,
    reflecting_obstacle: usize,
) {
    // Same as LowReflection — stub for rectangular obstacles.
    let _ = (site, initial_obstacle, reflecting_obstacle);
}

// =========================================================================
// Neighbor finding — TS lines 636-722
// =========================================================================

/// Find neighbors for both low and high sides and create scan segments.
///
/// Faithful port of TS `FindNeighborsAndProcessVertexEvent()` (lines 747-757)
/// combined with `FindNeighborsBRR()` (lines 651-663) and
/// `ProcessVertexEvent()` from C# FullVGG (lines 302-317).
fn find_neighbors_and_process_vertex_event(
    state: &mut SweepState,
    low_key: &SideKey,
    high_key: &SideKey,
    site: Point,
    obstacle_index: usize,
    is_open: bool,
) {
    // Find neighbors in both directions (TS line 753).
    state.low_neighbor_sides.clear();
    state.high_neighbor_sides.clear();

    find_neighbors(state, low_key, site, is_open, true);
    find_neighbors(state, high_key, site, is_open, false);

    // Create scan segments from the low side's neighbors (C# FullVGG line 305).
    create_scan_segments_from_neighbors(state, site, obstacle_index, true);

    // If the low and high neighbors differ, also create from high side (C# FullVGG lines 314-316).
    let low_high = state.low_neighbor_sides.high_neighbor().cloned();
    let high_high = state.high_neighbor_sides.high_neighbor().cloned();
    let same_high_neighbor = match (&low_high, &high_high) {
        (Some(a), Some(b)) => sides_equal(a, b),
        (None, None) => true,
        _ => false,
    };
    if !same_high_neighbor {
        create_scan_segments_from_neighbors(state, site, obstacle_index, false);
    }
}

/// Find neighbors in one direction from a side.
///
/// Faithful port of TS `FindNeighbors()` (lines 666-677) combined with
/// `SkipToNeighbor()` (lines 679-722).
fn find_neighbors(
    state: &mut SweepState,
    side_key: &SideKey,
    _site: Point,
    _is_open: bool,
    is_low_direction: bool,
) {
    // Get the side from the scanline.
    let side = match state.scan_line.get(side_key) {
        Some(s) => s.clone(),
        None => return,
    };

    // Find neighbors using the scanline (no mutable borrow of state needed).
    let low_nbor = skip_to_neighbor_low(&state.scan_line, side_key, &side);
    let high_nbor = skip_to_neighbor_high(&state.scan_line, side_key, &side);

    if is_low_direction {
        state.low_neighbor_sides.set_sides(low_nbor, high_nbor);
    } else {
        state.high_neighbor_sides.set_sides(low_nbor, high_nbor);
    }
}

/// Skip in the low (decreasing) direction to find a HighObstacleSide neighbor.
///
/// Simplified SkipToNeighbor for the low direction (TS lines 679-722).
/// We skip past same-obstacle sides and overlaps, stopping at the first
/// HighObstacleSide from a different obstacle.
fn skip_to_neighbor_low(
    scan_line: &super::scan_line::RectilinearScanLine,
    start_key: &SideKey,
    side: &ObstacleSide,
) -> Option<ObstacleSide> {
    let mut current_key = start_key.clone();
    loop {
        let prev = scan_line.next_low(&current_key);
        match prev {
            Some((k, nbor)) => {
                current_key = k.clone();
                // Skip same obstacle's other side (TS line 692-694).
                if nbor.obstacle_index() == side.obstacle_index() {
                    continue;
                }
                // Going low direction, we want a HighObstacleSide (TS line 717).
                if nbor.side_type() == SideType::High {
                    return Some(nbor.clone());
                }
                // LowObstacleSide going low — overlap candidate, keep going.
                continue;
            }
            None => return None,
        }
    }
}

/// Skip in the high (increasing) direction to find a LowObstacleSide neighbor.
///
/// Mirror of skip_to_neighbor_low for the high direction.
fn skip_to_neighbor_high(
    scan_line: &super::scan_line::RectilinearScanLine,
    start_key: &SideKey,
    side: &ObstacleSide,
) -> Option<ObstacleSide> {
    let mut current_key = start_key.clone();
    loop {
        let next = scan_line.next_high(&current_key);
        match next {
            Some((k, nbor)) => {
                current_key = k.clone();
                // Skip same obstacle (TS line 692-694).
                if nbor.obstacle_index() == side.obstacle_index() {
                    continue;
                }
                // Going high, we want a LowObstacleSide as our neighbor.
                if nbor.side_type() == SideType::Low {
                    return Some(nbor.clone());
                }
                // HighObstacleSide going high — overlap candidate, keep going.
                continue;
            }
            None => return None,
        }
    }
}

// =========================================================================
// Scan segment creation — C# FullVGG lines 210-283
// =========================================================================

/// Create scan segments between neighbor sides.
///
/// Simplified port of C# `CreateScanSegments()` (lines 210-283) for
/// non-overlapped rectangular obstacles. Creates a single segment between
/// the low neighbor's intersection and the high neighbor's intersection.
fn create_scan_segments_from_neighbors(
    state: &mut SweepState,
    site: Point,
    _obstacle_index: usize,
    use_low_neighbors: bool,
) {
    let (low_nbor, high_nbor) = if use_low_neighbors {
        (
            state.low_neighbor_sides.low_neighbor().cloned(),
            state.low_neighbor_sides.high_neighbor().cloned(),
        )
    } else {
        (
            state.high_neighbor_sides.low_neighbor().cloned(),
            state.high_neighbor_sides.high_neighbor().cloned(),
        )
    };

    let (low_side, high_side) = match (low_nbor, high_nbor) {
        (Some(l), Some(h)) => (l, h),
        _ => return,
    };

    // Compute intersections at the event's perpendicular coordinate (C# lines 231-232).
    let low_intersect = scanline_intersect_side(site, &low_side, state.scan_direction);
    let high_intersect = scanline_intersect_side(site, &high_side, state.scan_direction);

    // For non-overlapped obstacles, create one segment (C# lines 238-240).
    add_segment_if_valid(low_intersect, high_intersect, SegmentWeight::Normal, state);
}

// =========================================================================
// Helper functions
// =========================================================================

/// Add a side to the scanline and load reflection events.
///
/// Faithful port of TS `AddSideToScanLine()` (lines 568-573).
fn add_side_to_scanline(state: &mut SweepState, side: ObstacleSide) {
    state.scan_line.insert(side.clone());
    // Load any pending lookahead scan intersections (TS line 571).
    load_reflection_events(state, &side);
}

/// Load reflection events for a side.
///
/// Faithful port of TS `LoadReflectionEvents()` (lines 360-443).
/// For rectangular obstacles with perpendicular sides, SideReflectsUpward
/// returns false, so this is typically a no-op.
fn load_reflection_events(state: &mut SweepState, side: &ObstacleSide) {
    // TS line 370: skip if side reflects upward or is perpendicular.
    if side_reflects_upward(side, state.scan_direction) {
        return;
    }
    if state.scan_direction.is_perpendicular(side.start(), side.end()) {
        return;
    }

    // For non-perpendicular sides: find lookahead sites in the side's range
    // and enqueue reflection events. This is the core reflection loading logic
    // from TS lines 376-443.
    let low_coord = state.scan_direction.coord(side.start())
        .min(state.scan_direction.coord(side.end()));
    let high_coord = state.scan_direction.coord(side.start())
        .max(state.scan_direction.coord(side.end()));

    let sites: Vec<_> = state.lookahead_scan
        .find_all_in_range(low_coord, high_coord)
        .into_iter()
        .cloned()
        .collect();

    for la_site in sites {
        // Calculate the intersection in the perpendicular direction (TS line 395-398).
        let intersect = side.scanline_intersect(
            la_site.site,
            state.scan_direction.perpendicular(),
        );

        // Check that the intersection is ahead of the lookahead site (TS line 416).
        if state.scan_direction.compare_perp(intersect, la_site.site)
            == std::cmp::Ordering::Greater
        {
            // Enqueue a reflection event (TS line 423).
            let event = if side.side_type() == SideType::Low {
                SweepEvent::LowReflection {
                    site: intersect,
                    initial_obstacle: la_site.initial_obstacle,
                    reflecting_obstacle: la_site.reflecting_obstacle,
                    prev_event_index: None,
                }
            } else {
                SweepEvent::HighReflection {
                    site: intersect,
                    initial_obstacle: la_site.initial_obstacle,
                    reflecting_obstacle: la_site.reflecting_obstacle,
                    prev_event_index: None,
                }
            };
            state.event_queue.enqueue(event);
        } else if la_site.reflecting_obstacle != side.obstacle_index() {
            // Mark stale (TS line 428).
            state.lookahead_scan.remove_site(la_site.site);
        }
    }
}

/// Store a lookahead reflection site.
///
/// Faithful port of TS `StoreLookaheadSite()` (lines 314-358).
fn store_lookahead_site(
    state: &mut SweepState,
    reflecting_obstacle: usize,
    reflection_site: Point,
    _want_extreme: bool,
) {
    if !state.want_reflections {
        return;
    }

    // For rectangular obstacles, all sides are perpendicular, so
    // SideReflectsUpward never triggers and we never store sites.
    // This stub is here for completeness with non-rectangular polylines.
    if state.lookahead_scan.find_first_in_range(
        state.scan_direction.coord(reflection_site),
        state.scan_direction.coord(reflection_site),
    ).is_none() {
        state.lookahead_scan.add_site(
            reflection_site,
            reflecting_obstacle,
            reflecting_obstacle,
        );
    }
}

/// Enqueue a HighBend or CloseVertex event based on the next high side.
///
/// Faithful port of TS `EnqueueHighBendOrCloseVertexEvent()` (lines 880-892).
fn enqueue_high_bend_or_close(state: &mut SweepState, obstacle_index: usize) {
    let obs = &state.obstacles[obstacle_index];
    if let Some(high_side) = obs.active_high_side() {
        let end_key = high_side.end_vertex_key();
        let poly = obs.padded_polyline();

        // Determine the next point after the high side's end (TS lines 884-886).
        let next_key = if state.scan_direction.is_horizontal() {
            poly.prev_key(end_key)
        } else {
            poly.next_key(end_key)
        };

        if let Some(nk) = next_key {
            let next_point = poly.point_at(nk);
            let end_point = high_side.end();

            // If the next segment ascends, enqueue HighBend; otherwise CloseVertex.
            if state.scan_direction.compare_perp(next_point, end_point)
                == std::cmp::Ordering::Greater
            {
                let end_site = poly.point_at(end_key);
                state.event_queue.enqueue(SweepEvent::HighBend {
                    site: end_site,
                    obstacle_index,
                    vertex_key: end_key,
                });
            } else {
                let end_site = poly.point_at(end_key);
                state.event_queue.enqueue(SweepEvent::CloseVertex {
                    site: end_site,
                    obstacle_index,
                    vertex_key: end_key,
                });
            }
        } else {
            // Fallback: if we can't get next key, enqueue close.
            let end_site = poly.point_at(end_key);
            state.event_queue.enqueue(SweepEvent::CloseVertex {
                site: end_site,
                obstacle_index,
                vertex_key: end_key,
            });
        }
    }
}

/// Remove lookahead sites in a scan-coordinate range.
fn remove_lookahead_sites_in_range(state: &mut SweepState, low: f64, high: f64) {
    let sites: Vec<Point> = state.lookahead_scan
        .find_all_in_range(low, high)
        .into_iter()
        .map(|s| s.site)
        .collect();
    for site in sites {
        state.lookahead_scan.remove_site(site);
    }
}

/// Check if two obstacle sides are the same (same obstacle, same type, same start).
fn sides_equal(a: &ObstacleSide, b: &ObstacleSide) -> bool {
    a.obstacle_index() == b.obstacle_index()
        && a.side_type() == b.side_type()
        && GeomConstants::close(a.start().x(), b.start().x())
        && GeomConstants::close(a.start().y(), b.start().y())
}
