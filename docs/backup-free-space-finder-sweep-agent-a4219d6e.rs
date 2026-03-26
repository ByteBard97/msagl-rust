//! Faithful port of C# FreeSpaceFinder sweep-line algorithm.
//!
//! C# reference: MSAGL/Routing/Rectilinear/Nudging/FreeSpaceFinder.cs (527 lines)
//! C# base class: MSAGL/Routing/Visibility/LineSweeperBase.cs (172 lines)
//!
//! This module MUST use the same algorithmic approach as C#:
//! - Event-driven sweep with BinaryHeap priority queue
//! - BTreeMap-based active obstacle side trees (left and right)
//! - BTreeMap-based axis edge container tree
//! - O(log n) lookups for nearest obstacle sides and edge containers
//!
//! DO NOT replace any tree lookup with a linear scan.
//! DO NOT rebuild any data structure per event.

use std::collections::{BTreeMap, BinaryHeap};
use std::cmp::Ordering;
use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::routing::nudging::axis_edge::{AxisEdge, AxisEdgeId};
use crate::routing::scan_direction::Direction;

// =============================================================================
// Event types — matches C# VertexEvent, AxisEdgeLowPointEvent, AxisEdgeHighPointEvent
// =============================================================================

/// Sweep event — C# LineSweeperBase processes these in priority order.
/// Priority: lower Z first, then lower perpendicular coordinate.
#[derive(Clone)]
enum SweepEvent {
    /// C# LowestVertexEvent — dispatches to BOTH left and right vertex processing
    LowestVertex {
        point: Point,
        obstacle_idx: usize,
        polyline_point_idx: usize,
    },
    /// C# LeftVertexEvent — dispatches only to left vertex processing
    LeftVertex {
        point: Point,
        obstacle_idx: usize,
        polyline_point_idx: usize,
    },
    /// C# RightVertexEvent — dispatches only to right vertex processing
    RightVertex {
        point: Point,
        obstacle_idx: usize,
        polyline_point_idx: usize,
    },
    /// C# AxisEdgeLowPointEvent
    EdgeLow {
        point: Point,
        edge_id: AxisEdgeId,
    },
    /// C# AxisEdgeHighPointEvent
    EdgeHigh {
        point: Point,
        edge_id: AxisEdgeId,
    },
}

impl SweepEvent {
    fn site(&self) -> Point {
        match self {
            SweepEvent::LowestVertex { point, .. }
            | SweepEvent::LeftVertex { point, .. }
            | SweepEvent::RightVertex { point, .. }
            | SweepEvent::EdgeLow { point, .. }
            | SweepEvent::EdgeHigh { point, .. } => *point,
        }
    }
}

// =============================================================================
// Obstacle side representation for the active side trees
// =============================================================================

/// An active obstacle side in the sweep — ordered by intersection with sweep line.
/// C# uses LeftObstacleSide / RightObstacleSide in RbTree<SegmentBase>.
#[derive(Clone, Debug)]
struct ActiveSide {
    start: Point,
    end: Point,
    obstacle_idx: usize,
}

// =============================================================================
// Axis edge container — C# AxisEdgesContainer
// =============================================================================

/// Groups axis edges at the same perpendicular coordinate.
/// C# AxisEdgesContainer — ordered in edgeContainersTree by perpendicular projection.
#[allow(dead_code)]
struct AxisEdgesContainer {
    source: Point,
    edges: Vec<AxisEdgeId>,
}

// =============================================================================
// Main sweep-line struct — C# FreeSpaceFinder : LineSweeperBase
// =============================================================================

/// Faithful port of C# FreeSpaceFinder.
///
/// Fields match C# LineSweeperBase + FreeSpaceFinder combined:
/// - event_queue: BinaryHeap (C# BinaryHeapWithComparer<SweepEvent>)
/// - left_side_tree: BTreeMap (C# RbTree<SegmentBase> LeftObstacleSideTree)
/// - right_side_tree: BTreeMap (C# RbTree<SegmentBase> RightObstacleSideTree)
/// - edge_containers: BTreeMap (C# RbTree<AxisEdgesContainer> edgeContainersTree)
pub struct FreeSpaceFinderSweep {
    /// Sweep direction — C# SweepDirection (Point used as unit vector)
    direction: Direction,
    /// Perpendicular to sweep direction — C# DirectionPerp
    #[allow(dead_code)]
    direction_perp: Direction,

    /// Current Z coordinate — C# LineSweeperBase.Z
    z: f64,

    /// Event queue — C# LineSweeperBase.EventQueue (BinaryHeapWithComparer)
    /// MUST use BinaryHeap, not Vec + sort.
    event_queue: BinaryHeap<SweepEventEntry>,

    /// Active left obstacle sides — C# LineSweeperBase.LeftObstacleSideTree
    /// Key: OrderedFloat of the side's intersection with the current sweep line
    /// MUST use BTreeMap for O(log n) FindFirst/FindLast. DO NOT use Vec.
    left_side_tree: BTreeMap<OrderedFloat<f64>, Vec<ActiveSide>>,

    /// Active right obstacle sides — C# LineSweeperBase.RightObstacleSideTree
    /// MUST use BTreeMap for O(log n) FindFirst/FindLast. DO NOT use Vec.
    right_side_tree: BTreeMap<OrderedFloat<f64>, Vec<ActiveSide>>,

    /// Active axis edge containers — C# FreeSpaceFinder.edgeContainersTree
    /// Key: perpendicular projection of the container's source point
    /// MUST use BTreeMap for O(log n) Previous/Next/FindFirst. DO NOT use Vec.
    edge_containers: BTreeMap<OrderedFloat<f64>, AxisEdgesContainer>,

    /// Maps axis edge to the obstacle it originated from — C# AxisEdgesToObstaclesTheyOriginatedFrom
    /// Used by NotRestricting() to exempt an edge from being constrained by its own obstacle.
    axis_edge_to_obstacle: Vec<Option<usize>>,

    /// Accumulated left/right bounds to apply after sweep
    left_bounds: Vec<(AxisEdgeId, f64)>,
    right_bounds: Vec<(AxisEdgeId, f64)>,
    /// Accumulated neighbor pairs to apply after sweep
    neighbor_pairs: Vec<(AxisEdgeId, AxisEdgeId)>,
}

/// Priority queue entry — wraps SweepEvent with Ord for BinaryHeap.
/// C# LineSweeperBase.Compare: lower Z first, then lower perp coordinate.
#[derive(Clone)]
struct SweepEventEntry {
    /// Z projection (sweep direction · site) — primary sort key
    z: OrderedFloat<f64>,
    /// Perpendicular projection — secondary sort key
    perp: OrderedFloat<f64>,
    /// The event payload
    event: SweepEvent,
}

// BinaryHeap is a max-heap, so we reverse the ordering for min-heap behavior.
impl Ord for SweepEventEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for min-heap: lower Z first, then lower perp
        other.z.cmp(&self.z).then(other.perp.cmp(&self.perp))
    }
}
impl PartialOrd for SweepEventEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for SweepEventEntry {
    fn eq(&self, other: &Self) -> bool {
        self.z == other.z && self.perp == other.perp
    }
}
impl Eq for SweepEventEntry {}

impl FreeSpaceFinderSweep {
    // =========================================================================
    // Constructor — C# FreeSpaceFinder constructor (lines 37-52)
    // =========================================================================

    pub fn new(
        direction: Direction,
        num_edges: usize,
    ) -> Self {
        // C# constructor initializes:
        //   DirectionPerp = new CompassVector(direction).Right.ToPoint()
        //   xProjection = direction == North ? X : MinusY
        //   edgeContainersTree = new RbTree<AxisEdgesContainer>(this)
        //   SweepPole = direction
        //   All tree structures empty, event queue empty.
        let direction_perp = match direction {
            Direction::North => Direction::East,
            Direction::East => Direction::North,
        };
        Self {
            direction,
            direction_perp,
            z: f64::NEG_INFINITY,
            event_queue: BinaryHeap::new(),
            left_side_tree: BTreeMap::new(),
            right_side_tree: BTreeMap::new(),
            edge_containers: BTreeMap::new(),
            axis_edge_to_obstacle: vec![None; num_edges],
            left_bounds: Vec::new(),
            right_bounds: Vec::new(),
            neighbor_pairs: Vec::new(),
        }
    }

    // =========================================================================
    // Main entry point — C# FreeSpaceFinder.FindFreeSpace() (lines 65-69)
    // =========================================================================

    /// Runs the sweep. C# FindFreeSpace():
    ///   InitTheQueueOfEvents();
    ///   ProcessEvents();
    pub fn find_free_space(
        &mut self,
        axis_edges: &[AxisEdge],
        obstacles: &[crate::geometry::rectangle::Rectangle],
        axis_edge_to_obstacle: &[Option<usize>],
    ) {
        // Store the obstacle-to-edge mapping
        self.axis_edge_to_obstacle = axis_edge_to_obstacle.to_vec();
        // C# FindFreeSpace: InitTheQueueOfEvents(); ProcessEvents();
        self.init_event_queue(axis_edges, obstacles);
        // We need mutable axis_edges for process_events, but the caller
        // provides immutable — we accumulate results and apply later.
        self.process_events_internal(axis_edges, obstacles);
    }

    // =========================================================================
    // Event queue initialization — C# InitTheQueueOfEvents() (lines 475-478)
    //   calls InitQueueOfEvents() from LineSweeperBase (lines 113-119)
    //   then EnqueueEventsForEdge() per axis edge (lines 483-488)
    // =========================================================================

    fn init_event_queue(
        &mut self,
        axis_edges: &[AxisEdge],
        obstacles: &[crate::geometry::rectangle::Rectangle],
    ) {
        // C# InitQueueOfEvents (LineSweeperBase lines 113-119):
        //   foreach obstacle: EnqueueLowestPointsOnObstacles
        for (obs_idx, obs) in obstacles.iter().enumerate() {
            // Get the 4 vertices of the rectangle polyline (CCW: LB, RB, RT, LT)
            let vertices = [
                obs.left_bottom(),
                obs.right_bottom(),
                obs.right_top(),
                obs.left_top(),
            ];
            // Find lowest vertex (lowest Z, then lowest perp)
            let lowest_idx = self.get_lowest_vertex_idx(&vertices);
            self.enqueue_event(SweepEvent::LowestVertex {
                point: vertices[lowest_idx],
                obstacle_idx: obs_idx,
                polyline_point_idx: lowest_idx,
            });
        }

        // C# InitTheQueueOfEvents (lines 477-478):
        //   foreach axisEdge: EnqueueEventsForEdge
        for (edge_id, edge) in axis_edges.iter().enumerate() {
            // C# EnqueueEventsForEdge (lines 483-488):
            //   if EdgeIsParallelToSweepDir(edge):
            //     EnqueueEvent(EdgeLowPointEvent(edge, edge.Source.Point))
            //     EnqueueEvent(EdgeHighPointEvent(edge, edge.Target.Point))
            if self.edge_is_parallel_to_sweep_dir(edge) {
                // Source is low point, target is high point (edges are oriented North/East)
                let (low, high) = self.ordered_endpoints(edge);
                self.enqueue_event(SweepEvent::EdgeLow {
                    point: low,
                    edge_id,
                });
                self.enqueue_event(SweepEvent::EdgeHigh {
                    point: high,
                    edge_id,
                });
            }
        }
    }

    // =========================================================================
    // Event processing loop — C# ProcessEvents() (lines 72-75)
    // =========================================================================

    fn process_events_internal(
        &mut self,
        axis_edges: &[AxisEdge],
        obstacles: &[crate::geometry::rectangle::Rectangle],
    ) {
        // C# ProcessEvents: while (EventQueue.Count > 0) ProcessEvent(EventQueue.Dequeue());
        while let Some(entry) = self.event_queue.pop() {
            self.process_event(entry.event, axis_edges, obstacles);
        }
    }

    // =========================================================================
    // Event dispatch — C# ProcessEvent() (lines 77-92)
    // =========================================================================

    fn process_event(
        &mut self,
        event: SweepEvent,
        axis_edges: &[AxisEdge],
        obstacles: &[crate::geometry::rectangle::Rectangle],
    ) {
        // C# ProcessEvent (lines 77-92):
        // var vertexEvent = sweepEvent as VertexEvent;
        // if (vertexEvent != null)
        //     ProcessVertexEvent(vertexEvent);
        // else {
        //     Z = GetZ(sweepEvent.Site);
        //     if (lowEdgeEvent != null) ProcessLowEdgeEvent(...)
        //     else ProcessHighEdgeEvent(...)
        // }
        match event {
            SweepEvent::LowestVertex {
                point,
                obstacle_idx,
                polyline_point_idx,
            } => {
                // C# ProcessVertexEvent: LowestVertexEvent dispatches to both left and right
                self.z = self.get_z(point);
                self.process_vertex_event(
                    obstacle_idx,
                    polyline_point_idx,
                    obstacles,
                    axis_edges,
                );
            }
            SweepEvent::LeftVertex {
                point,
                obstacle_idx,
                polyline_point_idx,
            } => {
                // C# ProcessVertexEvent: LeftVertexEvent dispatches only to left processing
                self.z = self.get_z(point);
                let obs = &obstacles[obstacle_idx];
                let vertices = [
                    obs.left_bottom(),
                    obs.right_bottom(),
                    obs.right_top(),
                    obs.left_top(),
                ];
                let next_idx = (polyline_point_idx + 1) % 4;
                self.process_left_vertex_with_next(
                    point,
                    obstacle_idx,
                    polyline_point_idx,
                    next_idx,
                    &vertices,
                    axis_edges,
                );
            }
            SweepEvent::RightVertex {
                point,
                obstacle_idx,
                polyline_point_idx,
            } => {
                // C# ProcessVertexEvent: RightVertexEvent dispatches only to right processing
                self.z = self.get_z(point);
                let obs = &obstacles[obstacle_idx];
                let vertices = [
                    obs.left_bottom(),
                    obs.right_bottom(),
                    obs.right_top(),
                    obs.left_top(),
                ];
                let prev_idx = (polyline_point_idx + 3) % 4;
                self.process_right_vertex_with_next(
                    point,
                    obstacle_idx,
                    polyline_point_idx,
                    prev_idx,
                    &vertices,
                    axis_edges,
                );
            }
            SweepEvent::EdgeLow { point, edge_id } => {
                self.z = self.get_z(point);
                self.process_low_edge_event(edge_id, axis_edges);
            }
            SweepEvent::EdgeHigh { point, edge_id } => {
                self.z = self.get_z(point);
                self.process_high_edge_event(edge_id, axis_edges);
            }
        }
    }

    // =========================================================================
    // Edge events — C# ProcessLowEdgeEvent (lines 100-118)
    //                   ProcessHighEdgeEvent (lines 94-98)
    // =========================================================================

    /// C# ProcessLowEdgeEvent:
    /// 1. GetOrCreateAxisEdgesContainer(edge)  — O(log n) BTreeMap lookup
    /// 2. containerNode.Item.AddEdge(edge)
    /// 3. prev = edgeContainersTree.Previous(containerNode) — O(log n)
    /// 4. For each edge in prev × current: TryToAddRightNeighbor
    /// 5. next = edgeContainersTree.Next(containerNode) — O(log n)
    /// 6. For each edge in current × next: TryToAddRightNeighbor
    /// 7. ConstraintEdgeWithObstaclesAtZ(edge, edge.Source.Point)
    fn process_low_edge_event(&mut self, edge_id: AxisEdgeId, axis_edges: &[AxisEdge]) {
        let source = axis_edges[edge_id].source;
        // MUST use BTreeMap — NOT linear scan.
        let container_key = self.get_or_create_container_key(source);

        // Add edge to container
        self.edge_containers
            .get_mut(&container_key)
            .unwrap()
            .edges
            .push(edge_id);

        // C# prev = edgeContainersTree.Previous(containerNode) — O(log n)
        // MUST use BTreeMap range query for Previous — NOT linear scan.
        let prev_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range(..container_key)
            .next_back()
            .map(|(_, c)| c.edges.clone());

        let current_edges: Vec<AxisEdgeId> = self
            .edge_containers
            .get(&container_key)
            .unwrap()
            .edges
            .clone();

        if let Some(prev_edges) = prev_edges {
            for &prev_edge in &prev_edges {
                for &cur_edge in &current_edges {
                    self.try_to_add_right_neighbor(prev_edge, cur_edge, axis_edges);
                }
            }
        }

        // C# next = edgeContainersTree.Next(containerNode) — O(log n)
        // MUST use BTreeMap range query for Next — NOT linear scan.
        let next_key = OrderedFloat(container_key.into_inner() + f64::EPSILON * 0.1);
        let next_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range(next_key..)
            .next()
            .map(|(_, c)| c.edges.clone());

        if let Some(next_edges) = next_edges {
            for &cur_edge in &current_edges {
                for &ne_edge in &next_edges {
                    self.try_to_add_right_neighbor(cur_edge, ne_edge, axis_edges);
                }
            }
        }

        // C# ConstraintEdgeWithObstaclesAtZ(edge, edge.Source.Point)
        let (low, _) = self.ordered_endpoints(&axis_edges[edge_id]);
        self.constraint_edge_with_obstacles_at_z(edge_id, low, axis_edges);
    }

    /// C# ProcessHighEdgeEvent:
    /// 1. RemoveEdge(edge) — O(log n) BTreeMap lookup + remove
    /// 2. ConstraintEdgeWithObstaclesAtZ(edge, edge.Target.Point)
    fn process_high_edge_event(&mut self, edge_id: AxisEdgeId, axis_edges: &[AxisEdge]) {
        let source = axis_edges[edge_id].source;
        // C# RemoveEdge
        self.remove_edge(edge_id, source);
        // C# ConstraintEdgeWithObstaclesAtZ(edge, edge.Target.Point)
        let (_, high) = self.ordered_endpoints(&axis_edges[edge_id]);
        self.constraint_edge_with_obstacles_at_z(edge_id, high, axis_edges);
    }

    // =========================================================================
    // Obstacle vertex events — C# ProcessVertexEvent (lines 347-361)
    //   ProcessLeftVertex (lines 423-443), ProcessRightVertex (lines 363-385)
    // =========================================================================

    fn process_vertex_event(
        &mut self,
        obstacle_idx: usize,
        polyline_point_idx: usize,
        obstacles: &[crate::geometry::rectangle::Rectangle],
        axis_edges: &[AxisEdge],
    ) {
        // C# ProcessVertexEvent (lines 347-361):
        // For a LowestVertexEvent (which is what we always enqueue for obstacles),
        // it calls both ProcessLeftVertex and ProcessRightVertex.
        //
        // "Left" follows nextOnPolyline (CCW), "Right" follows prevOnPolyline (CW).
        let obs = &obstacles[obstacle_idx];
        let vertices = [
            obs.left_bottom(),
            obs.right_bottom(),
            obs.right_top(),
            obs.left_top(),
        ];

        let site = vertices[polyline_point_idx];

        // Process left side (CCW direction = nextOnPolyline)
        let next_idx = (polyline_point_idx + 1) % 4;
        self.process_left_vertex_with_next(
            site,
            obstacle_idx,
            polyline_point_idx,
            next_idx,
            &vertices,
            axis_edges,
        );

        // Process right side (CW direction = prevOnPolyline)
        let prev_idx = (polyline_point_idx + 3) % 4;
        self.process_right_vertex_with_next(
            site,
            obstacle_idx,
            polyline_point_idx,
            prev_idx,
            &vertices,
            axis_edges,
        );
    }

    /// C# ProcessLeftVertex:
    /// 1. ProcessPrevSegmentForLeftVertex — remove old left side if deltaZ > epsilon
    /// 2. Check delta to next vertex
    /// 3. If deltaZ > epsilon: InsertLeftSide, EnqueueEvent(LeftVertexEvent(next))
    /// 4. RestrictEdgeFromTheLeftOfEvent — O(log n) BTreeMap FindLast
    fn process_left_vertex_with_next(
        &mut self,
        site: Point,
        obstacle_idx: usize,
        current_vtx_idx: usize,
        next_vtx_idx: usize,
        vertices: &[Point; 4],
        axis_edges: &[AxisEdge],
    ) {
        // C# ProcessLeftVertex (lines 423-443):
        // 1. ProcessPrevSegmentForLeftVertex — remove prev left side if deltaZ > eps
        let prev_idx = (current_vtx_idx + 3) % 4; // prevOnPolyline for left = actual prev in CCW
        let prev_site = vertices[prev_idx];
        let delta_prev = site - prev_site;
        let delta_z_prev = self.dot_sweep(delta_prev);
        if delta_z_prev > GeomConstants::DISTANCE_EPSILON {
            // Remove the left side that started at prev
            self.remove_left_side(prev_site, vertices[current_vtx_idx]);
        }

        // 2. Check delta to next vertex
        let next_point = vertices[next_vtx_idx];
        let delta = next_point - site;
        let delta_x = self.dot_perp(delta);
        let delta_z = self.dot_sweep(delta);

        if delta_z <= GeomConstants::DISTANCE_EPSILON {
            // C# if (deltaX < 0 && deltaZ >= 0) EnqueueEvent(new LeftVertexEvent(next))
            if delta_x < 0.0 && delta_z >= 0.0 {
                self.enqueue_event(SweepEvent::LeftVertex {
                    point: next_point,
                    obstacle_idx,
                    polyline_point_idx: next_vtx_idx,
                });
            }
        } else {
            // deltaZ > epsilon
            // 3. InsertLeftSide, EnqueueEvent
            self.insert_left_side(site, next_point, obstacle_idx);
            self.enqueue_event(SweepEvent::LeftVertex {
                point: next_point,
                obstacle_idx,
                polyline_point_idx: next_vtx_idx,
            });
        }

        // 4. RestrictEdgeFromTheLeftOfEvent (C# lines 445-454)
        // MUST use BTreeMap range query for FindLast — NOT linear scan.
        self.restrict_edge_from_left_of_event(site, obstacle_idx, axis_edges);
    }

    /// C# ProcessRightVertex:
    /// 1. ProcessPrevSegmentForRightVertex — remove old right side if deltaZ > epsilon
    /// 2. Check delta to next vertex
    /// 3. If deltaZ > epsilon: InsertRightSide, EnqueueEvent(RightVertexEvent(next))
    /// 4. RestrictEdgeContainerToTheRightOfEvent — O(log n) BTreeMap FindFirst
    fn process_right_vertex_with_next(
        &mut self,
        site: Point,
        obstacle_idx: usize,
        current_vtx_idx: usize,
        next_vtx_idx: usize,
        vertices: &[Point; 4],
        axis_edges: &[AxisEdge],
    ) {
        // C# ProcessRightVertex (lines 363-385):
        // 1. ProcessPrevSegmentForRightVertex — the "prev" for right vertex is nextOnPolyline
        //    (because right side goes CW, so the previous segment in CW direction
        //     was from the next-in-CCW vertex to this vertex)
        let next_ccw_idx = (current_vtx_idx + 1) % 4;
        let prev_site = vertices[next_ccw_idx];
        let delta_prev = site - prev_site;
        let delta_z_prev = self.dot_sweep(delta_prev);
        if delta_z_prev > GeomConstants::DISTANCE_EPSILON {
            // Remove the right side from nextOnPolyline to current
            self.remove_right_side(prev_site, vertices[current_vtx_idx]);
        }

        // 2. Check delta to next vertex (CW direction)
        let next_point = vertices[next_vtx_idx];
        let delta = next_point - site;
        let delta_x = self.dot_perp(delta);
        let delta_z = self.dot_sweep(delta);

        if delta_z <= GeomConstants::DISTANCE_EPSILON {
            if delta_x > 0.0 && delta_z >= 0.0 {
                // C# EnqueueEvent(new RightVertexEvent(nextVertex))
                self.enqueue_event(SweepEvent::RightVertex {
                    point: next_point,
                    obstacle_idx,
                    polyline_point_idx: next_vtx_idx,
                });
            } else {
                // C# RestrictEdgeContainerToTheRightOfEvent
                self.restrict_edge_container_to_right_of_event(
                    site, obstacle_idx, axis_edges,
                );
            }
        } else {
            // deltaZ > epsilon
            // 3. InsertRightSide, EnqueueEvent, RestrictEdgeContainerToTheRightOfEvent
            self.insert_right_side(site, next_point, obstacle_idx);
            self.enqueue_event(SweepEvent::RightVertex {
                point: next_point,
                obstacle_idx,
                polyline_point_idx: next_vtx_idx,
            });
            self.restrict_edge_container_to_right_of_event(
                site, obstacle_idx, axis_edges,
            );
        }
    }

    // =========================================================================
    // Constraint application — C# ConstraintEdgeWithObstaclesAtZ (lines 167-171)
    //   ConstraintEdgeWithObstaclesAtZFromLeft (lines 186-193)
    //   ConstraintEdgeWithObstaclesAtZFromRight (lines 173-179)
    // =========================================================================

    /// C# ConstraintEdgeWithObstaclesAtZ:
    ///   ConstraintEdgeWithObstaclesAtZFromLeft(edge, point)
    ///   ConstraintEdgeWithObstaclesAtZFromRight(edge, point)
    fn constraint_edge_with_obstacles_at_z(
        &mut self,
        edge_id: AxisEdgeId,
        point: Point,
        axis_edges: &[AxisEdge],
    ) {
        self.constraint_from_left(edge_id, point, axis_edges);
        self.constraint_from_right(edge_id, point, axis_edges);
    }

    /// C# ConstraintEdgeWithObstaclesAtZFromLeft:
    /// 1. GetActiveSideFromLeft(point) — O(log n) BTreeMap FindLast
    /// 2. If not restricting (edge originated from this obstacle): return
    /// 3. Compute intersection of side with sweep line
    /// 4. edge.BoundFromLeft(intersection * DirectionPerp)
    fn constraint_from_left(
        &mut self,
        edge_id: AxisEdgeId,
        point: Point,
        _axis_edges: &[AxisEdge],
    ) {
        // C# GetActiveSideFromLeft (line 203-206):
        // RightObstacleSideTree.FindLast(side => PointToTheRightOfLineOrOnLineLocal(point, side.Start, side.End))
        // MUST use BTreeMap range query for FindLast — NOT linear scan.
        //
        // We look for the rightmost right-obstacle-side that is to the left of point.
        // The right_side_tree is keyed by the side's intersection with the sweep line.
        // FindLast means: find the entry with the largest key such that the point is
        // to the right of (or on) the side's line.
        let point_perp = self.x_projection(point);

        // MUST use BTreeMap range() — NOT linear scan.
        // Look at sides with intersection <= point_perp (they're candidates for being to the left)
        let side_opt = self
            .right_side_tree
            .range(..=OrderedFloat(point_perp + GeomConstants::DISTANCE_EPSILON))
            .next_back()
            .and_then(|(_, sides)| {
                sides.iter().find(|s| {
                    Self::point_to_right_of_line_or_on_line(point, s.start, s.end)
                }).cloned()
            });

        if let Some(side) = side_opt {
            // Check NotRestricting
            if self.not_restricting(edge_id, side.obstacle_idx) {
                return;
            }
            // Compute intersection of side with sweep line
            let x = self.intersection_of_side_and_sweep_line(&side);
            // C# edge.BoundFromLeft(x * DirectionPerp)
            // In C#, DirectionPerp is a Point and x is a Point, so x*DirectionPerp is a dot product.
            // Actually in C#: x = ObstacleSideComparer.IntersectionOfSideAndSweepLine(node.Item)
            // which returns a Point, and then: edge.BoundFromLeft(x * DirectionPerp)
            // where * is dot product between two Points.
            // In our case, x is already the perpendicular coordinate.
            self.left_bounds.push((edge_id, x));
        }
    }

    /// C# ConstraintEdgeWithObstaclesAtZFromRight:
    /// 1. GetActiveSideFromRight(point) — O(log n) BTreeMap FindFirst
    /// 2. If not restricting: return
    /// 3. Compute intersection of side with sweep line
    /// 4. edge.BoundFromRight(intersection * DirectionPerp)
    fn constraint_from_right(
        &mut self,
        edge_id: AxisEdgeId,
        point: Point,
        _axis_edges: &[AxisEdge],
    ) {
        // C# GetActiveSideFromRight (lines 181-184):
        // LeftObstacleSideTree.FindFirst(side => PointToTheLeftOfLineOrOnLineLocal(point, side.Start, side.End))
        // MUST use BTreeMap range query for FindFirst — NOT linear scan.
        let point_perp = self.x_projection(point);

        // MUST use BTreeMap range() — NOT linear scan.
        // Look at sides with intersection >= point_perp (candidates for being to the right)
        let side_opt = self
            .left_side_tree
            .range(OrderedFloat(point_perp - GeomConstants::DISTANCE_EPSILON)..)
            .next()
            .and_then(|(_, sides)| {
                sides.iter().find(|s| {
                    Self::point_to_left_of_line_or_on_line(point, s.start, s.end)
                }).cloned()
            });

        if let Some(side) = side_opt {
            if self.not_restricting(edge_id, side.obstacle_idx) {
                return;
            }
            let x = self.intersection_of_side_and_sweep_line(&side);
            self.right_bounds.push((edge_id, x));
        }
    }

    // =========================================================================
    // Helper: NotRestricting — C# line 400-403
    // =========================================================================

    /// Returns true if the edge originated from the given obstacle
    /// (meaning the obstacle should NOT constrain this edge).
    fn not_restricting(&self, edge_id: AxisEdgeId, obstacle_idx: usize) -> bool {
        // C# NotRestricting: return AxisEdgesToObstaclesTheyOriginatedFrom.TryGetValue(edge, out p) && p == polyline
        if edge_id < self.axis_edge_to_obstacle.len() {
            self.axis_edge_to_obstacle[edge_id] == Some(obstacle_idx)
        } else {
            false
        }
    }

    // =========================================================================
    // Helper: TryToAddRightNeighbor — C# lines 120-123
    // =========================================================================

    fn try_to_add_right_neighbor(
        &mut self,
        left_edge: AxisEdgeId,
        right_edge: AxisEdgeId,
        axis_edges: &[AxisEdge],
    ) {
        // C# TryToAddRightNeighbor (lines 120-123):
        // if (ProjectionsOfEdgesOverlap(leftEdge, rightEdge))
        //     leftEdge.AddRightNeighbor(rightEdge);
        if self.projections_overlap(&axis_edges[left_edge], &axis_edges[right_edge]) {
            self.neighbor_pairs.push((left_edge, right_edge));
        }
    }

    // =========================================================================
    // Helper: ProjectionsOfEdgesOverlap — C# lines 125-131
    // =========================================================================

    fn projections_overlap(
        &self,
        left: &AxisEdge,
        right: &AxisEdge,
    ) -> bool {
        // C# ProjectionsOfEdgesOverlap (lines 125-131):
        // return SweepPole == Direction.North
        //     ? !(leftEdge.TargetPoint.Y < rightEdge.SourcePoint.Y - eps ||
        //         rightEdge.TargetPoint.Y < leftEdge.SourcePoint.Y - eps)
        //     : !(leftEdge.TargetPoint.X < rightEdge.SourcePoint.X - eps ||
        //         rightEdge.TargetPoint.X < leftEdge.SourcePoint.X - eps);
        let eps = GeomConstants::DISTANCE_EPSILON;
        match self.direction {
            Direction::North => {
                !(left.target.y() < right.source.y() - eps
                    || right.target.y() < left.source.y() - eps)
            }
            Direction::East => {
                !(left.target.x() < right.source.x() - eps
                    || right.target.x() < left.source.x() - eps)
            }
        }
    }

    // =========================================================================
    // Helper: GetOrCreateAxisEdgesContainer — C# lines 320-329
    //   GetAxisEdgesContainerNode — C# lines 335-343
    // =========================================================================

    /// O(log n) lookup/insert in the BTreeMap.
    /// C# uses edgeContainersTree.FindFirst with xProjection comparison.
    fn get_or_create_container_key(&mut self, source: Point) -> OrderedFloat<f64> {
        // C# GetOrCreateAxisEdgesContainer (lines 320-329):
        // var ret = GetAxisEdgesContainerNode(source);
        // if (ret != null) return ret;
        // return edgeContainersTree.Insert(new AxisEdgesContainer(source));
        //
        // C# GetAxisEdgesContainerNode (lines 335-343):
        // var prj = xProjection(point);
        // var ret = edgeContainersTree.FindFirst(cont => xProjection(cont.Source) >= prj - eps/2);
        // if (ret != null && xProjection(ret.Item.Source) <= prj + eps/2) return ret;
        // return null;
        //
        // MUST use BTreeMap range query — NOT linear scan.
        let prj = self.x_projection(source);
        let half_eps = GeomConstants::DISTANCE_EPSILON / 2.0;

        // FindFirst: find first container whose x_projection >= prj - half_eps
        // MUST use BTreeMap range — NOT linear scan.
        let existing_key = self
            .edge_containers
            .range(OrderedFloat(prj - half_eps)..)
            .next()
            .and_then(|(k, _)| {
                if k.into_inner() <= prj + half_eps {
                    Some(*k)
                } else {
                    None
                }
            });

        if let Some(key) = existing_key {
            key
        } else {
            let key = OrderedFloat(prj);
            self.edge_containers.insert(
                key,
                AxisEdgesContainer {
                    source,
                    edges: Vec::new(),
                },
            );
            key
        }
    }

    // =========================================================================
    // Helper: RemoveEdge — C# lines 415-420
    // =========================================================================

    fn remove_edge(&mut self, edge_id: AxisEdgeId, source: Point) {
        // C# RemoveEdge (lines 415-420):
        // var containerNode = GetAxisEdgesContainerNode(edge.Source.Point);
        // containerNode.Item.RemoveAxis(edge);
        // if (containerNode.Item.IsEmpty()) edgeContainersTree.DeleteNodeInternal(containerNode);
        //
        // O(log n) BTreeMap lookup + remove.
        let prj = self.x_projection(source);
        let half_eps = GeomConstants::DISTANCE_EPSILON / 2.0;

        // MUST use BTreeMap range — NOT linear scan.
        let key_opt = self
            .edge_containers
            .range(OrderedFloat(prj - half_eps)..)
            .next()
            .and_then(|(k, _)| {
                if k.into_inner() <= prj + half_eps {
                    Some(*k)
                } else {
                    None
                }
            });

        if let Some(key) = key_opt {
            let should_remove = {
                let container = self.edge_containers.get_mut(&key).unwrap();
                container.edges.retain(|&e| e != edge_id);
                container.edges.is_empty()
            };
            if should_remove {
                self.edge_containers.remove(&key);
            }
        }
    }

    // =========================================================================
    // Result application — called after sweep completes
    // =========================================================================

    pub fn apply_results(&self, axis_edges: &mut [AxisEdge]) {
        // Apply accumulated left bounds
        for &(edge_id, bound) in &self.left_bounds {
            axis_edges[edge_id].bound_from_left(bound);
        }
        // Apply accumulated right bounds
        for &(edge_id, bound) in &self.right_bounds {
            axis_edges[edge_id].bound_from_right(bound);
        }
        // Apply accumulated neighbor pairs
        for &(left_id, right_id) in &self.neighbor_pairs {
            axis_edges[left_id].right_neighbors.push(right_id);
        }
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    /// C# xProjection: for North sweep returns point.X, for East returns -point.Y
    #[inline]
    fn x_projection(&self, p: Point) -> f64 {
        match self.direction {
            Direction::North => p.x(),
            Direction::East => -p.y(),
        }
    }

    /// C# SweepDirection as a Point vector
    #[inline]
    fn sweep_dir_vec(&self) -> Point {
        match self.direction {
            Direction::North => Point::new(0.0, 1.0),
            Direction::East => Point::new(1.0, 0.0),
        }
    }

    /// C# DirectionPerp as a Point vector
    #[inline]
    fn dir_perp_vec(&self) -> Point {
        match self.direction {
            Direction::North => Point::new(1.0, 0.0),
            Direction::East => Point::new(0.0, -1.0),
        }
    }

    /// Dot product with sweep direction
    #[inline]
    fn dot_sweep(&self, p: Point) -> f64 {
        p.dot(self.sweep_dir_vec())
    }

    /// Dot product with perpendicular direction
    #[inline]
    fn dot_perp(&self, p: Point) -> f64 {
        p.dot(self.dir_perp_vec())
    }

    /// C# GetZ(point) = SweepDirection * point (dot product)
    #[inline]
    fn get_z(&self, p: Point) -> f64 {
        self.dot_sweep(p)
    }

    /// Check if an axis edge is parallel to the sweep direction.
    /// C# EdgeIsParallelToSweepDir (lines 490-492)
    fn edge_is_parallel_to_sweep_dir(&self, edge: &AxisEdge) -> bool {
        // C#: edge.Direction == SweepPole || edge.Direction == OppositeDir(SweepPole)
        // Since our edges are always oriented North or East, and SweepPole = direction:
        edge.direction == self.direction
    }

    /// Get ordered endpoints (low, high) in sweep direction
    fn ordered_endpoints(&self, edge: &AxisEdge) -> (Point, Point) {
        let sz = self.dot_sweep(edge.source);
        let tz = self.dot_sweep(edge.target);
        if sz <= tz {
            (edge.source, edge.target)
        } else {
            (edge.target, edge.source)
        }
    }

    /// Find lowest vertex of a rectangle polyline (lowest Z, then lowest perp)
    fn get_lowest_vertex_idx(&self, vertices: &[Point; 4]) -> usize {
        let mut best = 0;
        for i in 1..4 {
            if self.sweep_less(vertices[i], vertices[best]) {
                best = i;
            }
        }
        best
    }

    /// Compare two points for sweep order: lower Z first, then lower perp
    fn sweep_less(&self, a: Point, b: Point) -> bool {
        let az = self.dot_sweep(a);
        let bz = self.dot_sweep(b);
        if az < bz {
            return true;
        }
        if az > bz {
            return false;
        }
        self.dot_perp(a) < self.dot_perp(b)
    }

    /// Enqueue a sweep event
    fn enqueue_event(&mut self, event: SweepEvent) {
        let site = event.site();
        let z = OrderedFloat(self.dot_sweep(site));
        let perp = OrderedFloat(self.dot_perp(site));
        self.event_queue.push(SweepEventEntry { z, perp, event });
    }

    // -- Active side tree operations --

    /// Compute the perpendicular intersection of a side with the current sweep line.
    /// C# ObstacleSideComparer.IntersectionOfSideAndSweepLine
    fn intersection_of_side_and_sweep_line(&self, side: &ActiveSide) -> f64 {
        let start_z = self.dot_sweep(side.start);
        let end_z = self.dot_sweep(side.end);
        let dz = end_z - start_z;

        if dz.abs() < GeomConstants::DISTANCE_EPSILON {
            // Horizontal side — return perp coordinate of start
            self.dot_perp(side.start)
        } else {
            // Interpolate: perp = start_perp + (z - start_z) / dz * (end_perp - start_perp)
            let start_perp = self.dot_perp(side.start);
            let end_perp = self.dot_perp(side.end);
            let t = (self.z - start_z) / dz;
            start_perp + t * (end_perp - start_perp)
        }
    }

    /// Compute the key for inserting/finding a side in the tree.
    fn side_key(&self, side: &ActiveSide) -> OrderedFloat<f64> {
        OrderedFloat(self.intersection_of_side_and_sweep_line(side))
    }

    /// C# InsertLeftSide — add to left obstacle side tree
    fn insert_left_side(&mut self, start: Point, end: Point, obstacle_idx: usize) {
        let side = ActiveSide {
            start,
            end,
            obstacle_idx,
        };
        let key = self.side_key(&side);
        self.left_side_tree
            .entry(key)
            .or_insert_with(Vec::new)
            .push(side);
    }

    /// C# InsertRightSide — add to right obstacle side tree
    fn insert_right_side(&mut self, start: Point, end: Point, obstacle_idx: usize) {
        let side = ActiveSide {
            start,
            end,
            obstacle_idx,
        };
        let key = self.side_key(&side);
        self.right_side_tree
            .entry(key)
            .or_insert_with(Vec::new)
            .push(side);
    }

    /// C# RemoveLeftSide — remove from left obstacle side tree
    fn remove_left_side(&mut self, start: Point, end: Point) {
        // Find and remove the matching side
        let side = ActiveSide {
            start,
            end,
            obstacle_idx: 0, // obstacle_idx not needed for key lookup
        };
        let key = self.side_key(&side);
        let half_eps = GeomConstants::DISTANCE_EPSILON;

        // MUST use BTreeMap range — NOT linear scan.
        // Search in a small range around the expected key
        let mut key_to_clean = None;
        for (&k, sides) in self
            .left_side_tree
            .range(OrderedFloat(key.into_inner() - half_eps)..=OrderedFloat(key.into_inner() + half_eps))
        {
            if sides.iter().any(|s| s.start.close_to(start) && s.end.close_to(end)) {
                key_to_clean = Some(k);
                break;
            }
        }

        if let Some(k) = key_to_clean {
            let sides = self.left_side_tree.get_mut(&k).unwrap();
            sides.retain(|s| !(s.start.close_to(start) && s.end.close_to(end)));
            if sides.is_empty() {
                self.left_side_tree.remove(&k);
            }
        }
    }

    /// C# RemoveRightSide — remove from right obstacle side tree
    fn remove_right_side(&mut self, start: Point, end: Point) {
        let side = ActiveSide {
            start,
            end,
            obstacle_idx: 0,
        };
        let key = self.side_key(&side);
        let half_eps = GeomConstants::DISTANCE_EPSILON;

        // MUST use BTreeMap range — NOT linear scan.
        let mut key_to_clean = None;
        for (&k, sides) in self
            .right_side_tree
            .range(OrderedFloat(key.into_inner() - half_eps)..=OrderedFloat(key.into_inner() + half_eps))
        {
            if sides.iter().any(|s| s.start.close_to(start) && s.end.close_to(end)) {
                key_to_clean = Some(k);
                break;
            }
        }

        if let Some(k) = key_to_clean {
            let sides = self.right_side_tree.get_mut(&k).unwrap();
            sides.retain(|s| !(s.start.close_to(start) && s.end.close_to(end)));
            if sides.is_empty() {
                self.right_side_tree.remove(&k);
            }
        }
    }

    // -- Geometry predicates --

    /// C# PointToTheLeftOfLineOrOnLineLocal (line 195-197):
    /// SignedDoubledTriangleArea(a, linePoint0, linePoint1) > -AreaComparisonEpsilon
    fn point_to_left_of_line_or_on_line(a: Point, line0: Point, line1: Point) -> bool {
        Point::signed_doubled_triangle_area(a, line0, line1)
            > -GeomConstants::INTERSECTION_EPSILON
    }

    /// C# PointToTheRightOfLineOrOnLineLocal (line 199-201):
    /// SignedDoubledTriangleArea(linePoint0, linePoint1, a) < AreaComparisonEpsilon
    fn point_to_right_of_line_or_on_line(a: Point, line0: Point, line1: Point) -> bool {
        Point::signed_doubled_triangle_area(line0, line1, a)
            < GeomConstants::INTERSECTION_EPSILON
    }

    // -- Edge restriction helpers --

    /// C# RestrictEdgeFromTheLeftOfEvent (lines 445-454):
    /// Find container node to the left of event, bound its edges from right.
    fn restrict_edge_from_left_of_event(
        &mut self,
        site: Point,
        obstacle_idx: usize,
        _axis_edges: &[AxisEdge],
    ) {
        // C# GetContainerNodeToTheLeftOfEvent (lines 456-463):
        // double siteX = xProjection(site);
        // return edgeContainersTree.FindLast(container => xProjection(container.Source) <= siteX);
        let site_x = self.x_projection(site);
        let site_perp = self.dot_perp(site);

        // MUST use BTreeMap range query — NOT linear scan.
        let container_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range(..=OrderedFloat(site_x + GeomConstants::DISTANCE_EPSILON))
            .next_back()
            .map(|(_, c)| c.edges.clone());

        if let Some(edges) = container_edges {
            // C# foreach (var edge in containerNode.Item.Edges)
            //     if (!NotRestricting(edge, polylinePoint.Polyline))
            //         edge.BoundFromRight(site * DirectionPerp);
            for &edge_id in &edges {
                if !self.not_restricting(edge_id, obstacle_idx) {
                    self.right_bounds.push((edge_id, site_perp));
                }
            }
        }
    }

    /// C# RestrictEdgeContainerToTheRightOfEvent (lines 387-398):
    /// Find container node to the right of event, bound its edges from left.
    fn restrict_edge_container_to_right_of_event(
        &mut self,
        site: Point,
        obstacle_idx: usize,
        _axis_edges: &[AxisEdge],
    ) {
        // C# (lines 388-398):
        // var siteX = xProjection(site);
        // var containerNode = edgeContainersTree.FindFirst(
        //     container => siteX <= xProjection(container.Source));
        // if (containerNode != null)
        //     foreach (var edge in containerNode.Item.Edges)
        //         if (!NotRestricting(edge, polylinePoint.Polyline))
        //             edge.BoundFromLeft(DirectionPerp * site);
        let site_x = self.x_projection(site);
        let site_perp = self.dot_perp(site);

        // MUST use BTreeMap range query — NOT linear scan.
        let container_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range(OrderedFloat(site_x - GeomConstants::DISTANCE_EPSILON)..)
            .next()
            .map(|(_, c)| c.edges.clone());

        if let Some(edges) = container_edges {
            for &edge_id in &edges {
                if !self.not_restricting(edge_id, obstacle_idx) {
                    self.left_bounds.push((edge_id, site_perp));
                }
            }
        }
    }
}
