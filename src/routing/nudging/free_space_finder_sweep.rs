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
use std::ops::Bound;
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
pub(super) enum SweepEvent {
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
    pub(super) fn site(&self) -> Point {
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
pub(super) struct ActiveSide {
    pub(super) start: Point,
    pub(super) end: Point,
    pub(super) obstacle_idx: usize,
}

// =============================================================================
// Axis edge container — C# AxisEdgesContainer
// =============================================================================

/// Groups axis edges at the same perpendicular coordinate.
/// C# AxisEdgesContainer — ordered in edgeContainersTree by perpendicular projection.
#[allow(dead_code)]
pub(super) struct AxisEdgesContainer {
    pub(super) source: Point,
    pub(super) edges: Vec<AxisEdgeId>,
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
    pub(super) direction: Direction,
    /// Perpendicular to sweep direction — C# DirectionPerp
    #[allow(dead_code)]
    pub(super) direction_perp: Direction,

    /// Current Z coordinate — C# LineSweeperBase.Z
    pub(super) z: f64,

    /// Event queue — C# LineSweeperBase.EventQueue (BinaryHeapWithComparer)
    /// MUST use BinaryHeap, not Vec + sort.
    pub(super) event_queue: BinaryHeap<SweepEventEntry>,

    /// Active left obstacle sides — C# LineSweeperBase.LeftObstacleSideTree
    /// Key: OrderedFloat of the side's intersection with the current sweep line
    /// MUST use BTreeMap for O(log n) FindFirst/FindLast. DO NOT use Vec.
    pub(super) left_side_tree: BTreeMap<OrderedFloat<f64>, Vec<ActiveSide>>,

    /// Active right obstacle sides — C# LineSweeperBase.RightObstacleSideTree
    /// MUST use BTreeMap for O(log n) FindFirst/FindLast. DO NOT use Vec.
    pub(super) right_side_tree: BTreeMap<OrderedFloat<f64>, Vec<ActiveSide>>,

    /// Active axis edge containers — C# FreeSpaceFinder.edgeContainersTree
    /// Key: perpendicular projection of the container's source point
    /// MUST use BTreeMap for O(log n) Previous/Next/FindFirst. DO NOT use Vec.
    pub(super) edge_containers: BTreeMap<OrderedFloat<f64>, AxisEdgesContainer>,

    /// Maps axis edge to the obstacle it originated from — C# AxisEdgesToObstaclesTheyOriginatedFrom
    /// Used by NotRestricting() to exempt an edge from being constrained by its own obstacle.
    pub(super) axis_edge_to_obstacle: Vec<Option<usize>>,

    /// Accumulated left/right bounds to apply after sweep
    pub(super) left_bounds: Vec<(AxisEdgeId, f64)>,
    pub(super) right_bounds: Vec<(AxisEdgeId, f64)>,
    /// Accumulated neighbor pairs to apply after sweep
    pub(super) neighbor_pairs: Vec<(AxisEdgeId, AxisEdgeId)>,
}

/// Priority queue entry — wraps SweepEvent with Ord for BinaryHeap.
/// C# LineSweeperBase.Compare: lower Z first, then lower perp coordinate.
#[derive(Clone)]
pub(super) struct SweepEventEntry {
    /// Z projection (sweep direction · site) — primary sort key
    pub(super) z: OrderedFloat<f64>,
    /// Perpendicular projection — secondary sort key
    pub(super) perp: OrderedFloat<f64>,
    /// The event payload
    pub(super) event: SweepEvent,
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
            // Get the 4 vertices of the rectangle polyline in CCW order
            // matching C# Polyline convention: LB, LT, RT, RB
            let vertices = [
                obs.left_bottom(),
                obs.left_top(),
                obs.right_top(),
                obs.right_bottom(),
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
                    obs.left_top(),
                    obs.right_top(),
                    obs.right_bottom(),
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
                    obs.left_top(),
                    obs.right_top(),
                    obs.right_bottom(),
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
        // Use Excluded bound to skip the current container_key.
        let next_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range((Bound::Excluded(container_key), Bound::Unbounded))
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
            obs.left_top(),
            obs.right_top(),
            obs.right_bottom(),
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

    // Constraint, helper, and geometry methods are in free_space_finder_sweep_helpers.rs
}
