use super::event_queue::{EventQueue, SweepEvent};
use super::neighbor_sides::NeighborSides;
use super::obstacle::Obstacle;
use super::obstacle_side::{ObstacleSide, SideType};
use super::obstacle_tree::ObstacleTree;
use super::scan_direction::ScanDirection;
use super::scan_line::RectilinearScanLine;
use super::scan_segment::{ScanSegment, ScanSegmentTree, SegmentWeight};
use super::segment_intersector::build_graph_from_segments;
use super::shape::Shape;
use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VisibilityGraph;

/// Sentinel offset margin applied beyond the graph bounding box.
/// Matches C# VisibilityGraphGenerator.SentinelOffset = 1.0.
const SENTINEL_OFFSET: f64 = 1.0;

/// Generate a visibility graph from rectangular shapes.
///
/// Uses a two-pass event-driven sweep-line algorithm:
///   Pass 1 (horizontal scan, vertical sweep): creates horizontal segments
///   Pass 2 (vertical scan, horizontal sweep): creates vertical segments
/// Then intersects H and V segments to build the visibility graph.
///
/// Mirrors FullVisibilityGraphGenerator.cs / VisibilityGraphGenerator.ts from MSAGL.
pub fn generate_visibility_graph(shapes: &[Shape], padding: f64) -> VisibilityGraph {
    if shapes.is_empty() {
        return VisibilityGraph::new();
    }

    let obstacle_tree = ObstacleTree::new(shapes, padding);
    let graph_box = obstacle_tree.graph_box();

    // Pass 1: Horizontal scan (vertical sweep, creates horizontal segments)
    let mut h_segments = run_sweep(&obstacle_tree, ScanDirection::horizontal(), &graph_box);

    // Pass 2: Vertical scan (horizontal sweep, creates vertical segments)
    let mut v_segments = run_sweep(&obstacle_tree, ScanDirection::vertical(), &graph_box);

    // Intersect to build the visibility graph
    build_graph_from_segments(&mut h_segments, &mut v_segments)
}

/// Run one sweep pass (horizontal or vertical) and return the resulting scan segments.
fn run_sweep(
    tree: &ObstacleTree,
    scan_direction: ScanDirection,
    graph_box: &crate::geometry::rectangle::Rectangle,
) -> Vec<ScanSegment> {
    let mut scan_line = RectilinearScanLine::new(scan_direction);
    let mut seg_tree = ScanSegmentTree::new(scan_direction);

    // Create and insert sentinels into the scanline (not the event queue).
    insert_sentinels(&mut scan_line, scan_direction, graph_box);

    // Create event queue and enqueue OpenVertex events for all obstacles.
    let mut event_queue = EventQueue::new(scan_direction);
    let mut obstacles: Vec<Obstacle> = tree.obstacles.clone();
    enqueue_open_events(&mut event_queue, &mut obstacles, scan_direction);

    // Process all events
    process_events(
        &mut event_queue,
        &mut scan_line,
        &mut seg_tree,
        &mut obstacles,
        scan_direction,
    );

    // Merge adjacent/overlapping segments with the same weight
    seg_tree.merge_segments();

    // Collect all segments
    seg_tree.all_segments().cloned().collect()
}

/// Insert sentinel obstacle sides at the graph boundaries.
fn insert_sentinels(
    scan_line: &mut RectilinearScanLine,
    scan_direction: ScanDirection,
    graph_box: &crate::geometry::rectangle::Rectangle,
) {
    let offset = SENTINEL_OFFSET;

    if scan_direction.is_horizontal() {
        let low_x = graph_box.left() - offset;
        let low_sentinel = ObstacleSide::new(
            SideType::High,
            Point::new(low_x, graph_box.top() + offset),
            Point::new(low_x, graph_box.bottom() - offset),
            Obstacle::FIRST_SENTINEL_ORDINAL,
        );
        scan_line.insert(low_sentinel);

        let high_x = graph_box.right() + offset;
        let high_sentinel = ObstacleSide::new(
            SideType::Low,
            Point::new(high_x, graph_box.bottom() - offset),
            Point::new(high_x, graph_box.top() + offset),
            Obstacle::FIRST_SENTINEL_ORDINAL + 1,
        );
        scan_line.insert(high_sentinel);
    } else {
        let low_y = graph_box.bottom() - offset;
        let low_sentinel = ObstacleSide::new(
            SideType::High,
            Point::new(graph_box.right() + offset, low_y),
            Point::new(graph_box.left() - offset, low_y),
            Obstacle::FIRST_SENTINEL_ORDINAL,
        );
        scan_line.insert(low_sentinel);

        let high_y = graph_box.top() + offset;
        let high_sentinel = ObstacleSide::new(
            SideType::Low,
            Point::new(graph_box.left() - offset, high_y),
            Point::new(graph_box.right() + offset, high_y),
            Obstacle::FIRST_SENTINEL_ORDINAL + 1,
        );
        scan_line.insert(high_sentinel);
    }
}

/// Enqueue OpenVertex events for all obstacles.
fn enqueue_open_events(
    queue: &mut EventQueue,
    obstacles: &mut [Obstacle],
    scan_direction: ScanDirection,
) {
    for obs in obstacles.iter_mut() {
        let open_key = obs.get_open_vertex(scan_direction);
        let open_site = obs.padded_polyline().point_at(open_key);
        obs.create_initial_sides(open_key, scan_direction);
        queue.enqueue(SweepEvent::OpenVertex {
            site: open_site,
            obstacle_index: obs.index,
            vertex_key: open_key,
        });
    }
}

/// Process all events from the queue.
fn process_events(
    queue: &mut EventQueue,
    scan_line: &mut RectilinearScanLine,
    seg_tree: &mut ScanSegmentTree,
    obstacles: &mut [Obstacle],
    scan_direction: ScanDirection,
) {
    while let Some(event) = queue.dequeue() {
        match event {
            SweepEvent::OpenVertex {
                site,
                obstacle_index,
                ..
            } => {
                process_open_vertex(
                    site,
                    obstacle_index,
                    queue,
                    scan_line,
                    seg_tree,
                    obstacles,
                    scan_direction,
                );
            }
            SweepEvent::CloseVertex {
                site,
                obstacle_index,
                ..
            } => {
                process_close_vertex(
                    site,
                    obstacle_index,
                    scan_line,
                    seg_tree,
                    obstacles,
                    scan_direction,
                );
            }
            _ => {}
        }
    }
}

/// Process an OpenVertex event: add sides to scanline, find neighbors, create segments.
///
/// Uses SkipToNeighbor to find the actual visibility neighbors of each side,
/// then creates scan segments between them (with overlap handling).
fn process_open_vertex(
    site: Point,
    obstacle_index: usize,
    queue: &mut EventQueue,
    scan_line: &mut RectilinearScanLine,
    seg_tree: &mut ScanSegmentTree,
    obstacles: &[Obstacle],
    scan_direction: ScanDirection,
) {
    let obs = &obstacles[obstacle_index];
    let low_side = obs
        .active_low_side()
        .expect("obstacle should have low side after create_initial_sides")
        .clone();
    let high_side = obs
        .active_high_side()
        .expect("obstacle should have high side after create_initial_sides")
        .clone();

    scan_line.insert(low_side.clone());
    scan_line.insert(high_side.clone());

    let low_side_key = scan_line.find(&low_side).expect("just inserted low side");
    let high_side_key = scan_line.find(&high_side).expect("just inserted high side");

    let low_neighbors = find_neighbors_for_side(
        scan_line, &low_side_key, &low_side,
        low_side.start(),
        scan_direction, obstacles,
    );
    let high_neighbors = find_neighbors_for_side(
        scan_line, &high_side_key, &high_side,
        high_side.start(),
        scan_direction, obstacles,
    );

    create_scan_segments_from_neighbors(site, &low_neighbors, seg_tree, scan_direction, obstacles);

    let need_high = match (low_neighbors.high_neighbor(), high_neighbors.high_neighbor()) {
        (Some(a), Some(b)) => {
            a.obstacle_ordinal() != b.obstacle_ordinal() || a.side_type() != b.side_type()
        }
        _ => true,
    };
    if need_high {
        create_scan_segments_from_neighbors(site, &high_neighbors, seg_tree, scan_direction, obstacles);
    }

    let bb = obs.padded_bounding_box();
    let close_site = if scan_direction.is_horizontal() {
        bb.left_top()
    } else {
        bb.right_bottom()
    };
    queue.enqueue(SweepEvent::CloseVertex {
        site: close_site,
        obstacle_index,
        vertex_key: PolylinePointKey::default(),
    });
}

/// Process a CloseVertex event: find neighbors, create segments, remove sides.
fn process_close_vertex(
    site: Point,
    obstacle_index: usize,
    scan_line: &mut RectilinearScanLine,
    seg_tree: &mut ScanSegmentTree,
    obstacles: &[Obstacle],
    scan_direction: ScanDirection,
) {
    let obs = &obstacles[obstacle_index];
    let low_side = obs.active_low_side().expect("obstacle should have low side").clone();
    let high_side = obs.active_high_side().expect("obstacle should have high side").clone();

    let mut low_side_key = scan_line.find(&low_side).expect("low side should be in scanline");
    let mut high_side_key = scan_line.find(&high_side).expect("high side should be in scanline");

    if low_side_key > high_side_key {
        std::mem::swap(&mut low_side_key, &mut high_side_key);
    }

    let low_in_sl = scan_line.get(&low_side_key).cloned().unwrap_or(low_side.clone());
    let high_in_sl = scan_line.get(&high_side_key).cloned().unwrap_or(high_side.clone());

    let low_neighbors = find_neighbors_for_side(
        scan_line, &low_side_key, &low_in_sl,
        low_in_sl.end(),
        scan_direction, obstacles,
    );
    let high_neighbors = find_neighbors_for_side(
        scan_line, &high_side_key, &high_in_sl,
        high_in_sl.end(),
        scan_direction, obstacles,
    );

    create_scan_segments_from_neighbors(site, &low_neighbors, seg_tree, scan_direction, obstacles);

    let need_high = match (low_neighbors.high_neighbor(), high_neighbors.high_neighbor()) {
        (Some(a), Some(b)) => {
            a.obstacle_ordinal() != b.obstacle_ordinal() || a.side_type() != b.side_type()
        }
        _ => true,
    };
    if need_high {
        create_scan_segments_from_neighbors(site, &high_neighbors, seg_tree, scan_direction, obstacles);
    }

    scan_line.remove(&low_side);
    scan_line.remove(&high_side);
}

// ---------------------------------------------------------------------------
// SkipToNeighbor traversal
// ---------------------------------------------------------------------------

/// Find neighbors for one side of an obstacle using SkipToNeighbor.
fn find_neighbors_for_side(
    scan_line: &RectilinearScanLine,
    side_key: &super::scan_line::SideKey,
    side: &ObstacleSide,
    side_reference_point: Point,
    scan_direction: ScanDirection,
    obstacles: &[Obstacle],
) -> NeighborSides {
    let mut neighbors = NeighborSides::new();

    if let Some((low_key, _)) = scan_line.next_low(side_key) {
        skip_to_neighbor(
            scan_line, false, side, side_reference_point,
            &low_key.clone(), &mut neighbors, scan_direction, obstacles,
        );
    }

    if let Some((high_key, _)) = scan_line.next_high(side_key) {
        skip_to_neighbor(
            scan_line, true, side, side_reference_point,
            &high_key.clone(), &mut neighbors, scan_direction, obstacles,
        );
    }

    neighbors
}

/// Walk the scanline to find the actual neighbor, tracking overlap boundaries.
///
/// Faithful port of TS `SkipToNeighbor` (lines 679-722).
/// Uses `scan_line_crosses_obstacle` to detect geometric overlap boundaries,
/// and `obstacle_is_overlapped` to confirm the obstacle is in a clump.
fn skip_to_neighbor(
    scan_line: &RectilinearScanLine,
    ascending: bool,
    side: &ObstacleSide,
    side_reference_point: Point,
    initial_nbor_key: &super::scan_line::SideKey,
    neighbors: &mut NeighborSides,
    scan_direction: ScanDirection,
    obstacles: &[Obstacle],
) {
    let mut overlap_side: Option<ObstacleSide> = None;
    let mut current_key = initial_nbor_key.clone();

    loop {
        let current_side = match scan_line.get(&current_key) {
            Some(s) => s.clone(),
            None => break,
        };

        // Skip the opposite side of the same obstacle
        if current_side.obstacle_ordinal() == side.obstacle_ordinal() {
            match scan_line.next_in_direction(&current_key, ascending) {
                Some((next_key, _)) => { current_key = next_key.clone(); continue; }
                None => break,
            }
        }

        // Check for overlap-ending obstacle: a side whose type means we're
        // leaving an obstacle in this traversal direction.
        let is_overlap_ender = match (ascending, current_side.side_type()) {
            (true, SideType::High) => true,
            (false, SideType::Low) => true,
            _ => false,
        };

        if is_overlap_ender {
            // Only record as overlap if the scanline geometrically crosses through
            // this obstacle AND the obstacle is marked as overlapped (in a clump).
            if scan_line_crosses_obstacle(
                side_reference_point, &current_side, scan_direction, obstacles,
            ) && obstacle_is_overlapped(current_side.obstacle_ordinal(), obstacles) {
                overlap_side = Some(current_side);
            }
            match scan_line.next_in_direction(&current_key, ascending) {
                Some((next_key, _)) => { current_key = next_key.clone(); continue; }
                None => break,
            }
        }

        // Found the neighbor
        neighbors.set_sides_for_direction(ascending, current_side, overlap_side);
        return;
    }
}

/// Check whether the event site's perpendicular coordinate is strictly inside
/// the obstacle's bounding box.
fn scan_line_crosses_obstacle(
    event_site: Point,
    nbor_side: &ObstacleSide,
    scan_direction: ScanDirection,
    obstacles: &[Obstacle],
) -> bool {
    if is_sentinel_side(nbor_side) {
        return false;
    }
    let obs_ordinal = nbor_side.obstacle_ordinal();
    let obs = match obstacles.iter().find(|o| o.ordinal() == obs_ordinal) {
        Some(o) => o,
        None => return false,
    };
    let bb = obs.padded_bounding_box();
    let perp = scan_direction.perp_coord(event_site);
    let bb_low = scan_direction.perp_coord(bb.left_bottom());
    let bb_high = scan_direction.perp_coord(bb.right_top());
    perp > bb_low + GeomConstants::DISTANCE_EPSILON
        && perp < bb_high - GeomConstants::DISTANCE_EPSILON
}

/// Check whether an obstacle with the given ordinal is overlapped (in a clump).
///
/// An overlapped obstacle's interior should be traversable (at high cost)
/// so that paths can route through overlap regions. Non-overlapped obstacle
/// interiors are impassable.
fn obstacle_is_overlapped(ordinal: usize, obstacles: &[Obstacle]) -> bool {
    if ordinal < Obstacle::FIRST_NON_SENTINEL_ORDINAL {
        return false; // sentinel
    }
    let idx = ordinal - Obstacle::FIRST_NON_SENTINEL_ORDINAL;
    if idx < obstacles.len() {
        obstacles[idx].is_overlapped()
    } else {
        false
    }
}

fn is_sentinel_side(side: &ObstacleSide) -> bool {
    side.obstacle_ordinal() < Obstacle::FIRST_NON_SENTINEL_ORDINAL
}

// ---------------------------------------------------------------------------
// Segment creation from neighbors
// ---------------------------------------------------------------------------

/// Create scan segments from neighbor information.
///
/// Faithful port of C# `FullVisibilityGraphGenerator.CreateScanSegments()`.
/// Creates up to 3 segments: Normal, Overlapped, Normal — where the overlap
/// region corresponds to obstacle interiors that have been marked as overlapped.
fn create_scan_segments_from_neighbors(
    site: Point,
    neighbors: &NeighborSides,
    seg_tree: &mut ScanSegmentTree,
    scan_direction: ScanDirection,
    obstacles: &[Obstacle],
) {
    let low_nbor = match neighbors.low_neighbor() {
        Some(s) => s,
        None => return,
    };
    let high_nbor = match neighbors.high_neighbor() {
        Some(s) => s,
        None => return,
    };

    let low_nbor_intersect = low_nbor.scanline_intersect(site, scan_direction);
    let high_nbor_intersect = high_nbor.scanline_intersect(site, scan_direction);
    let is_vertical = scan_direction.is_vertical();

    let low_overlap = neighbors.low_overlap_end();
    let high_overlap = neighbors.high_overlap_end();

    if low_overlap.is_none() && high_overlap.is_none() {
        // No overlaps detected during SkipToNeighbor traversal.
        // Check if both neighbors belong to overlapped obstacles —
        // if so, the entire span between them is an overlap region.
        let low_is_overlapped = obstacle_is_overlapped(low_nbor.obstacle_ordinal(), obstacles);
        let high_is_overlapped = obstacle_is_overlapped(high_nbor.obstacle_ordinal(), obstacles);
        let weight = if low_is_overlapped && high_is_overlapped {
            SegmentWeight::Overlapped
        } else {
            SegmentWeight::Normal
        };
        add_segment(
            low_nbor_intersect, high_nbor_intersect,
            weight, is_vertical, seg_tree, scan_direction,
        );
        return;
    }

    // Compute overlap boundary intersections
    let low_ov_intersect = match low_overlap {
        Some(s) => s.scanline_intersect(site, scan_direction),
        None => low_nbor_intersect,
    };
    let high_ov_intersect = match high_overlap {
        Some(s) => s.scanline_intersect(site, scan_direction),
        None => high_nbor_intersect,
    };

    // Normal from low neighbor to overlap start
    add_segment(
        low_nbor_intersect, low_ov_intersect,
        SegmentWeight::Normal, is_vertical, seg_tree, scan_direction,
    );
    // Overlapped region
    add_segment(
        low_ov_intersect, high_ov_intersect,
        SegmentWeight::Overlapped, is_vertical, seg_tree, scan_direction,
    );
    // Normal from overlap end to high neighbor
    if high_overlap.is_some() {
        add_segment(
            high_ov_intersect, high_nbor_intersect,
            SegmentWeight::Normal, is_vertical, seg_tree, scan_direction,
        );
    }
}

/// Add a segment to the tree if start and end differ.
fn add_segment(
    start: Point,
    end: Point,
    weight: SegmentWeight,
    is_vertical: bool,
    seg_tree: &mut ScanSegmentTree,
    scan_direction: ScanDirection,
) {
    let (s, e) = if scan_direction.coord(start) <= scan_direction.coord(end) {
        (start, end)
    } else {
        (end, start)
    };
    if GeomConstants::close(scan_direction.coord(s), scan_direction.coord(e)) {
        return;
    }
    seg_tree.insert_unique(ScanSegment::new(s, e, weight, is_vertical));
}
