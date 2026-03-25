use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VisibilityGraph;
use super::event_queue::{EventQueue, SweepEvent};
use super::obstacle::Obstacle;
use super::obstacle_side::{ObstacleSide, SideType};
use super::obstacle_tree::ObstacleTree;
use super::scan_direction::ScanDirection;
use super::scan_line::RectilinearScanLine;
use super::scan_segment::{ScanSegment, ScanSegmentTree, SegmentWeight};
use super::segment_intersector::build_graph_from_segments;
use super::shape::Shape;

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
    let h_segments = run_sweep(&obstacle_tree, ScanDirection::horizontal(), &graph_box);

    // Pass 2: Vertical scan (horizontal sweep, creates vertical segments)
    let v_segments = run_sweep(&obstacle_tree, ScanDirection::vertical(), &graph_box);

    // Intersect to build the visibility graph
    let mut graph = VisibilityGraph::new();
    build_graph_from_segments(&mut graph, &h_segments, &v_segments);
    graph
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

    // Merge adjacent segments with the same weight
    seg_tree.merge_segments();

    // Collect all segments
    seg_tree.all_segments().cloned().collect()
}

/// Insert sentinel obstacle sides at the graph boundaries.
///
/// For horizontal scan: sentinels are vertical lines at far left and far right.
/// For vertical scan: sentinels are horizontal lines at far bottom and far top.
fn insert_sentinels(
    scan_line: &mut RectilinearScanLine,
    scan_direction: ScanDirection,
    graph_box: &crate::geometry::rectangle::Rectangle,
) {
    let offset = SENTINEL_OFFSET;

    if scan_direction.is_horizontal() {
        // Horizontal scan (sweeping in Y): sentinels are vertical lines.
        // Low sentinel at far left (its High side faces inward).
        let low_x = graph_box.left() - offset;
        let low_sentinel = ObstacleSide::new(
            SideType::High,
            Point::new(low_x, graph_box.top() + offset),
            Point::new(low_x, graph_box.bottom() - offset),
            Obstacle::FIRST_SENTINEL_ORDINAL,
        );
        scan_line.insert(low_sentinel);

        // High sentinel at far right (its Low side faces inward).
        let high_x = graph_box.right() + offset;
        let high_sentinel = ObstacleSide::new(
            SideType::Low,
            Point::new(high_x, graph_box.bottom() - offset),
            Point::new(high_x, graph_box.top() + offset),
            Obstacle::FIRST_SENTINEL_ORDINAL + 1,
        );
        scan_line.insert(high_sentinel);
    } else {
        // Vertical scan (sweeping in X): sentinels are horizontal lines.
        // Low sentinel at far bottom.
        let low_y = graph_box.bottom() - offset;
        let low_sentinel = ObstacleSide::new(
            SideType::High,
            Point::new(graph_box.right() + offset, low_y),
            Point::new(graph_box.left() - offset, low_y),
            Obstacle::FIRST_SENTINEL_ORDINAL,
        );
        scan_line.insert(low_sentinel);

        // High sentinel at far top.
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
///
/// For horizontal scan: open at bottom Y of padded bbox.
/// For vertical scan: open at left X of padded bbox.
fn enqueue_open_events(
    queue: &mut EventQueue,
    obstacles: &mut [Obstacle],
    scan_direction: ScanDirection,
) {
    let is_horizontal = scan_direction.is_horizontal();
    for obs in obstacles.iter_mut() {
        obs.create_initial_sides(is_horizontal);
        let bb = obs.padded_bounding_box();
        // For both horizontal scan (sweep in Y) and vertical scan (sweep in X),
        // the open event is at left_bottom — the corner with the lowest perp coordinate.
        let open_site = bb.left_bottom();
        queue.enqueue(SweepEvent::OpenVertex {
            site: open_site,
            obstacle_index: obs.index,
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
            // For rectangular obstacles, bend and reflection events don't occur.
            _ => {}
        }
    }
}

/// Process an OpenVertex event: add sides to scanline, create segments in gaps.
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

    // Insert sides into scanline
    scan_line.insert(low_side.clone());
    scan_line.insert(high_side.clone());

    // Find neighbors and create segments
    let low_coord = side_scan_coord(&low_side, scan_direction);
    let high_coord = side_scan_coord(&high_side, scan_direction);

    create_scan_segments_at_event(
        site, low_coord, high_coord, scan_line, seg_tree, scan_direction,
    );

    // Enqueue CloseVertex event at the top of this obstacle
    let bb = obs.padded_bounding_box();
    let close_site = if scan_direction.is_horizontal() {
        bb.left_top() // highest Y
    } else {
        bb.right_bottom() // highest X (right side)
    };
    queue.enqueue(SweepEvent::CloseVertex {
        site: close_site,
        obstacle_index,
    });
}

/// Process a CloseVertex event: create segments in the gaps being opened, then remove sides.
fn process_close_vertex(
    site: Point,
    obstacle_index: usize,
    scan_line: &mut RectilinearScanLine,
    seg_tree: &mut ScanSegmentTree,
    obstacles: &[Obstacle],
    scan_direction: ScanDirection,
) {
    let obs = &obstacles[obstacle_index];
    let low_side = obs
        .active_low_side()
        .expect("obstacle should have low side")
        .clone();
    let high_side = obs
        .active_high_side()
        .expect("obstacle should have high side")
        .clone();

    let low_coord = side_scan_coord(&low_side, scan_direction);
    let high_coord = side_scan_coord(&high_side, scan_direction);

    // Create segments in the gaps (same as open, but before removing sides)
    create_scan_segments_at_event(
        site, low_coord, high_coord, scan_line, seg_tree, scan_direction,
    );

    // Remove sides from scanline
    scan_line.remove(&low_side);
    scan_line.remove(&high_side);
}

/// Create scan segments at an event by looking at the gaps between the obstacle's
/// sides and their neighbors on the scanline.
///
/// This creates up to 3 segments:
/// 1. From low neighbor to obstacle's low side (gap on the low side)
/// 2. Across the obstacle itself (only if obstacle is overlapped — skipped for now)
/// 3. From obstacle's high side to high neighbor (gap on the high side)
fn create_scan_segments_at_event(
    site: Point,
    low_coord: f64,
    high_coord: f64,
    scan_line: &RectilinearScanLine,
    seg_tree: &mut ScanSegmentTree,
    scan_direction: ScanDirection,
) {
    let perp = scan_direction.perp_coord(site);

    // Find the low neighbor (below the obstacle's low side)
    let low_nbor_coord = scan_line
        .low_neighbor(low_coord)
        .map(|s| side_scan_coord_from_side(s, scan_direction));

    // Find the high neighbor (above the obstacle's high side)
    let high_nbor_coord = scan_line
        .high_neighbor(high_coord)
        .map(|s| side_scan_coord_from_side(s, scan_direction));

    // Segment from low neighbor to obstacle's low side
    if let Some(ln_coord) = low_nbor_coord {
        add_segment_if_valid(
            ln_coord, low_coord, perp, scan_direction, seg_tree,
        );
    }

    // Segment from obstacle's high side to high neighbor
    if let Some(hn_coord) = high_nbor_coord {
        add_segment_if_valid(
            high_coord, hn_coord, perp, scan_direction, seg_tree,
        );
    }
}

/// Add a scan segment if start != end.
fn add_segment_if_valid(
    start_coord: f64,
    end_coord: f64,
    perp: f64,
    scan_direction: ScanDirection,
    seg_tree: &mut ScanSegmentTree,
) {
    if !GeomConstants::close(start_coord, end_coord) && end_coord > start_coord {
        let start = scan_direction.make_point(start_coord, perp);
        let end = scan_direction.make_point(end_coord, perp);
        let is_vertical = scan_direction.is_vertical();
        seg_tree.insert_unique(ScanSegment::new(
            start,
            end,
            SegmentWeight::Normal,
            is_vertical,
        ));
    }
}

/// Get the scan-parallel coordinate of an obstacle side.
/// For horizontal scan: this is the X coordinate of the side.
/// For vertical scan: this is the Y coordinate of the side.
fn side_scan_coord(side: &ObstacleSide, scan_direction: ScanDirection) -> f64 {
    scan_direction.coord(side.start())
}

/// Same as side_scan_coord but takes a reference.
fn side_scan_coord_from_side(side: &ObstacleSide, scan_direction: ScanDirection) -> f64 {
    scan_direction.coord(side.start())
}