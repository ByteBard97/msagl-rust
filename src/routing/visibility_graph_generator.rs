use super::event_queue::{EventQueue, SweepEvent};
use super::neighbor_sides::NeighborSides;
use super::obstacle::Obstacle;
use super::obstacle_side::{ObstacleSide, SideType};
use super::obstacle_tree::ObstacleTree;
use super::router_session::RouterSession;
use super::scan_direction::ScanDirection;
use super::scan_line::RectilinearScanLine;
use super::scan_segment::{ScanSegment, ScanSegmentTree, SegmentWeight};
use super::segment_intersector::build_graph_from_segments;
use super::shape::Shape;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::VisibilityGraph;

/// Sentinel offset margin applied beyond the graph bounding box.
/// Matches C# VisibilityGraphGenerator.SentinelOffset = 1.0.
const SENTINEL_OFFSET: f64 = 1.0;

/// Builds and rebuilds the visibility graph inside a `RouterSession`.
///
/// The generator itself is stateless (no fields); all output lives on the session.
/// This matches C# `FullVisibilityGraphGenerator` / TS `VisibilityGraphGenerator`.
///
/// # Refactor 2
/// Previously a free function (`generate_visibility_graph`); now a struct so that
/// `RectilinearEdgeRouter` can hold it as an owned field and call `generate` whenever
/// the obstacle set changes (`add_shape` / `remove_shape`).
pub struct VisibilityGraphGenerator;

impl VisibilityGraphGenerator {
    /// Run both sweep passes and populate `session.vis_graph`, `session.h_scan_segments`,
    /// and `session.v_scan_segments` from the obstacles already stored in
    /// `session.obstacle_tree`.
    ///
    /// Replaces the entire VG on `session`; safe to call more than once (on obstacle change).
    pub fn generate(&self, session: &mut RouterSession) {
        if session.obstacle_tree.obstacles.is_empty() {
            session.vis_graph = VisibilityGraph::new();
            session.h_scan_segments = ScanSegmentTree::new(ScanDirection::horizontal());
            session.v_scan_segments = ScanSegmentTree::new(ScanDirection::vertical());
            return;
        }

        let graph_box = session.obstacle_tree.graph_box();

        // Pass 1: Horizontal scan (vertical sweep, creates horizontal segments)
        let h_scan_segments =
            run_sweep(&session.obstacle_tree, ScanDirection::horizontal(), &graph_box);

        // Pass 2: Vertical scan (horizontal sweep, creates vertical segments)
        let v_scan_segments =
            run_sweep(&session.obstacle_tree, ScanDirection::vertical(), &graph_box);

        // Intersect H and V segments to build the visibility graph.
        let mut h_segs: Vec<ScanSegment> = h_scan_segments.all_segments().cloned().collect();
        let mut v_segs: Vec<ScanSegment> = v_scan_segments.all_segments().cloned().collect();

        session.vis_graph = build_graph_from_segments(&mut h_segs, &mut v_segs);
        session.h_scan_segments = h_scan_segments;
        session.v_scan_segments = v_scan_segments;
    }
}

/// Convenience constructor: build a `RouterSession` from scratch and run both sweep passes.
///
/// Kept for tests and one-off callers; internally creates a `VisibilityGraphGenerator`
/// and calls `generate`.
pub fn generate_visibility_graph(shapes: &[Shape], padding: f64) -> RouterSession {
    let obstacle_tree = if shapes.is_empty() {
        ObstacleTree::empty()
    } else {
        ObstacleTree::new(shapes, padding)
    };

    let mut session = RouterSession {
        vis_graph: VisibilityGraph::new(),
        obstacle_tree,
        h_scan_segments: ScanSegmentTree::new(ScanDirection::horizontal()),
        v_scan_segments: ScanSegmentTree::new(ScanDirection::vertical()),
        padding,
    };

    VisibilityGraphGenerator.generate(&mut session);
    session
}

/// Run one sweep pass (horizontal or vertical) and return the resulting ScanSegmentTree.
///
/// The tree is preserved (not flattened to a Vec) so the session can hold it for
/// PortManager lookups after the sweep completes.
fn run_sweep(
    tree: &ObstacleTree,
    scan_direction: ScanDirection,
    graph_box: &crate::geometry::rectangle::Rectangle,
) -> ScanSegmentTree {
    let mut scan_line = RectilinearScanLine::new(scan_direction);
    let seg_tree = ScanSegmentTree::new(scan_direction);

    insert_sentinels(&mut scan_line, scan_direction, graph_box);

    let mut event_queue = EventQueue::new(scan_direction);
    let mut obstacles: Vec<Obstacle> = tree.obstacles.clone();
    enqueue_open_events(&mut event_queue, &mut obstacles, scan_direction);

    let mut state = crate::routing::vg_event_processing::SweepState {
        scan_line,
        event_queue,
        seg_tree,
        lookahead_scan: crate::routing::lookahead_scan::LookaheadScan::new(scan_direction),
        obstacles,
        scan_direction,
        want_reflections: true,
        low_neighbor_sides: crate::routing::neighbor_sides::NeighborSides::new(),
        high_neighbor_sides: crate::routing::neighbor_sides::NeighborSides::new(),
    };

    process_events(&mut state);

    state.seg_tree.merge_segments();

    state.seg_tree
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

/// Process all events from the queue using `vg_event_processing`.
fn process_events(state: &mut crate::routing::vg_event_processing::SweepState) {
    let mut prev_event_site: Option<Point> = None;
    
    while let Some(event) = state.event_queue.dequeue() {
        let site = match &event {
            SweepEvent::OpenVertex { site, .. } => *site,
            SweepEvent::CloseVertex { site, .. } => *site,
            SweepEvent::LowBend { site, .. } => *site,
            SweepEvent::HighBend { site, .. } => *site,
            SweepEvent::LowReflection { site, .. } => *site,
            SweepEvent::HighReflection { site, .. } => *site,
        };

        match event {
            SweepEvent::OpenVertex { site, obstacle_index, .. } => {
                crate::routing::vg_event_processing::process_open_vertex(state, site, obstacle_index);
            }
            SweepEvent::CloseVertex { site, obstacle_index, .. } => {
                crate::routing::vg_event_processing::process_close_vertex(state, site, obstacle_index);
            }
            SweepEvent::LowBend { site, obstacle_index, vertex_key } => {
                crate::routing::vg_event_processing::process_low_bend(state, site, obstacle_index, vertex_key);
            }
            SweepEvent::HighBend { site, obstacle_index, vertex_key } => {
                crate::routing::vg_event_processing::process_high_bend(state, site, obstacle_index, vertex_key);
            }
            SweepEvent::LowReflection { site, initial_obstacle, reflecting_obstacle, .. } => {
                crate::routing::vg_event_processing::process_low_reflection(state, site, initial_obstacle, reflecting_obstacle, prev_event_site);
            }
            SweepEvent::HighReflection { site, initial_obstacle, reflecting_obstacle, .. } => {
                crate::routing::vg_event_processing::process_high_reflection(state, site, initial_obstacle, reflecting_obstacle, prev_event_site);
            }
        }
        
        prev_event_site = Some(site);
    }
}


