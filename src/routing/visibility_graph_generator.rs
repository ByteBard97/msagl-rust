use super::event_queue::{EventQueue, SweepEvent};
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
/// Faithful port of TS `EnqueueBottomVertexEvents()`.
/// Uses `GetOpenVertex` to find the lowest polyline vertex, then
/// calls `CreateInitialSides` from that vertex.
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
        site,
        low_coord,
        high_coord,
        scan_line,
        seg_tree,
        scan_direction,
        obstacles,
    );

    // Enqueue CloseVertex event at the top of this obstacle
    let bb = obs.padded_bounding_box();
    let close_site = if scan_direction.is_horizontal() {
        bb.left_top() // highest Y
    } else {
        bb.right_bottom() // highest X (right side)
    };
    // TODO: Once polyline-based obstacles are implemented, use the actual
    // close vertex key from polyline traversal instead of default.
    queue.enqueue(SweepEvent::CloseVertex {
        site: close_site,
        obstacle_index,
        vertex_key: PolylinePointKey::default(),
    });
}

/// Process a CloseVertex event: create segments before AND after removing sides.
///
/// Before removal: segments fill the gaps with the obstacle's sides still present.
/// After removal: segments fill the newly opened gap where the obstacle was.
/// Both are needed because other obstacles may still be on the scanline and
/// create different gap patterns before vs. after removal.
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

    // Create segments with obstacle sides still present.
    create_scan_segments_at_event(
        site,
        low_coord,
        high_coord,
        scan_line,
        seg_tree,
        scan_direction,
        obstacles,
    );

    // Remove sides from scanline
    scan_line.remove(&low_side);
    scan_line.remove(&high_side);

    // Create segments again after removal — the gap where the obstacle was
    // is now open, creating new/larger segments.
    create_scan_segments_at_event(
        site,
        low_coord,
        high_coord,
        scan_line,
        seg_tree,
        scan_direction,
        obstacles,
    );
}

/// Create scan segments at an event.
///
/// Faithful port of C# FullVisibilityGraphGenerator.CreateScanSegments
/// (lines 210-283). Creates normal segments in open-space gaps between
/// obstacles, and overlapped segments through overlap regions where
/// multiple obstacle interiors intersect.
///
/// Uses a depth counter to track how many obstacle interiors we're inside:
/// - depth == 0: open space → Normal segment
/// - depth >= 2: overlap region → Overlapped segment
/// - depth == 1 inside an overlapped (clumped) obstacle → Overlapped segment
/// - depth == 1 inside a non-overlapped obstacle → no segment
///
/// A Low side increments depth (entering an obstacle). A High side
/// decrements depth (leaving). Sentinels are excluded from depth tracking.
fn create_scan_segments_at_event(
    site: Point,
    _low_coord: f64,
    _high_coord: f64,
    scan_line: &RectilinearScanLine,
    seg_tree: &mut ScanSegmentTree,
    scan_direction: ScanDirection,
    obstacles: &[Obstacle],
) {
    let perp = scan_direction.perp_coord(site);
    let all_sides = scan_line.all_sides_ordered();

    if all_sides.len() < 2 {
        return;
    }

    // Track depth: how many obstacle interiors we're inside.
    // At depth 0, we're in open space. At depth >= 1, we're inside at least
    // one obstacle. At depth >= 2, we're in an overlap region.
    let mut depth: i32 = 0;
    // Track whether we're inside any overlapped (clumped) obstacles.
    // When depth == 1 inside a clumped obstacle, we still generate
    // an overlapped segment for traversability.
    let mut overlapped_depth: i32 = 0;

    for pair in all_sides.windows(2) {
        let left = pair[0];
        let right = pair[1];

        // Update depth based on the left side type.
        // Sentinels don't affect depth.
        if !is_sentinel_side(left) {
            let is_overlapped_obs = obstacle_is_overlapped(left.obstacle_ordinal(), obstacles);
            match left.side_type() {
                SideType::Low => {
                    depth += 1;
                    if is_overlapped_obs {
                        overlapped_depth += 1;
                    }
                }
                SideType::High => {
                    depth -= 1;
                    if is_overlapped_obs {
                        overlapped_depth -= 1;
                    }
                }
            }
        }

        let start_coord = side_scan_coord_from_side(left, scan_direction);
        let end_coord = side_scan_coord_from_side(right, scan_direction);

        if GeomConstants::close(start_coord, end_coord) || end_coord <= start_coord {
            continue;
        }

        let start = scan_direction.make_point(start_coord, perp);
        let end = scan_direction.make_point(end_coord, perp);
        let is_vertical = scan_direction.is_vertical();

        if depth <= 0 {
            // Open space: normal segment
            seg_tree.insert_unique(ScanSegment::new(
                start,
                end,
                SegmentWeight::Normal,
                is_vertical,
            ));
        } else if depth >= 2 {
            // Overlap region (inside 2+ obstacles): overlapped segment.
            seg_tree.insert_unique(ScanSegment::new(
                start,
                end,
                SegmentWeight::Overlapped,
                is_vertical,
            ));
        } else if depth == 1 && overlapped_depth > 0 {
            // Inside exactly one obstacle, but it's an overlapped obstacle.
            // Create an overlapped segment for traversability through the
            // clump's interior. This matches C# behavior where overlapped
            // obstacle interiors are traversable at high cost.
            seg_tree.insert_unique(ScanSegment::new(
                start,
                end,
                SegmentWeight::Overlapped,
                is_vertical,
            ));
        }
        // depth == 1 with overlapped_depth == 0: inside a non-overlapped
        // obstacle. No segment created — paths should go around.
    }
}

/// Check whether an obstacle with the given ordinal is overlapped (in a clump).
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

/// Check if an obstacle side belongs to a sentinel (boundary marker).
fn is_sentinel_side(side: &ObstacleSide) -> bool {
    side.obstacle_ordinal() < Obstacle::FIRST_NON_SENTINEL_ORDINAL
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
