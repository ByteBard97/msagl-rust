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
    /// Obstacle vertex entering sweep — C# LowestVertexEvent/LeftVertexEvent/RightVertexEvent
    ObstacleVertex {
        point: Point,
        obstacle_idx: usize,
        /// Index of the polyline point within the obstacle's padded polyline
        polyline_point_idx: usize,
    },
    /// Axis edge low (source) point — C# AxisEdgeLowPointEvent
    EdgeLow {
        point: Point,
        edge_id: AxisEdgeId,
    },
    /// Axis edge high (target) point — C# AxisEdgeHighPointEvent
    EdgeHigh {
        point: Point,
        edge_id: AxisEdgeId,
    },
}

impl SweepEvent {
    fn site(&self) -> Point {
        match self {
            SweepEvent::ObstacleVertex { point, .. } => *point,
            SweepEvent::EdgeLow { point, .. } => *point,
            SweepEvent::EdgeHigh { point, .. } => *point,
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
        todo!("Port C# FreeSpaceFinder constructor. Initialize all fields.")
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
        todo!("Port C# FindFreeSpace. Call init_event_queue then process_events.")
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
        todo!("Port C# InitTheQueueOfEvents + InitQueueOfEvents. \
               Enqueue obstacle lowest-point events AND axis edge low/high events.")
    }

    // =========================================================================
    // Event processing loop — C# ProcessEvents() (lines 72-75)
    // =========================================================================

    fn process_events(&mut self, axis_edges: &mut [AxisEdge]) {
        todo!("Port C# ProcessEvents. Dequeue events one at a time, call process_event.")
    }

    // =========================================================================
    // Event dispatch — C# ProcessEvent() (lines 77-92)
    // =========================================================================

    fn process_event(&mut self, event: SweepEvent, axis_edges: &mut [AxisEdge]) {
        todo!("Port C# ProcessEvent. Match on event type: \
               ObstacleVertex → process_vertex_event, \
               EdgeLow → process_low_edge_event, \
               EdgeHigh → process_high_edge_event.")
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
    fn process_low_edge_event(&mut self, edge_id: AxisEdgeId, axis_edges: &mut [AxisEdge]) {
        todo!("Port C# ProcessLowEdgeEvent EXACTLY. \
               Use BTreeMap range queries for Previous/Next — NOT linear scan.")
    }

    /// C# ProcessHighEdgeEvent:
    /// 1. RemoveEdge(edge) — O(log n) BTreeMap lookup + remove
    /// 2. ConstraintEdgeWithObstaclesAtZ(edge, edge.Target.Point)
    fn process_high_edge_event(&mut self, edge_id: AxisEdgeId, axis_edges: &mut [AxisEdge]) {
        todo!("Port C# ProcessHighEdgeEvent EXACTLY.")
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
        axis_edges: &mut [AxisEdge],
    ) {
        todo!("Port C# ProcessVertexEvent. Dispatch to process_left_vertex / process_right_vertex.")
    }

    /// C# ProcessLeftVertex:
    /// 1. ProcessPrevSegmentForLeftVertex — remove old left side if deltaZ > epsilon
    /// 2. Check delta to next vertex
    /// 3. If deltaZ > epsilon: InsertLeftSide, EnqueueEvent(LeftVertexEvent(next))
    /// 4. RestrictEdgeFromTheLeftOfEvent — O(log n) BTreeMap FindLast
    fn process_left_vertex(
        &mut self,
        _site: Point,
        _obstacle_idx: usize,
        axis_edges: &mut [AxisEdge],
    ) {
        todo!("Port C# ProcessLeftVertex EXACTLY.")
    }

    /// C# ProcessRightVertex:
    /// 1. ProcessPrevSegmentForRightVertex — remove old right side if deltaZ > epsilon
    /// 2. Check delta to next vertex
    /// 3. If deltaZ > epsilon: InsertRightSide, EnqueueEvent(RightVertexEvent(next))
    /// 4. RestrictEdgeContainerToTheRightOfEvent — O(log n) BTreeMap FindFirst
    fn process_right_vertex(
        &mut self,
        _site: Point,
        _obstacle_idx: usize,
        axis_edges: &mut [AxisEdge],
    ) {
        todo!("Port C# ProcessRightVertex EXACTLY.")
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
        axis_edges: &mut [AxisEdge],
    ) {
        todo!("Port C# ConstraintEdgeWithObstaclesAtZ. \
               Call constraint_from_left and constraint_from_right.")
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
        axis_edges: &mut [AxisEdge],
    ) {
        todo!("Port C# ConstraintEdgeWithObstaclesAtZFromLeft. \
               MUST use BTreeMap range query for FindLast — NOT linear scan.")
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
        axis_edges: &mut [AxisEdge],
    ) {
        todo!("Port C# ConstraintEdgeWithObstaclesAtZFromRight. \
               MUST use BTreeMap range query for FindFirst — NOT linear scan.")
    }

    // =========================================================================
    // Helper: NotRestricting — C# line 400-403
    // =========================================================================

    /// Returns true if the edge originated from the given obstacle
    /// (meaning the obstacle should NOT constrain this edge).
    fn not_restricting(&self, edge_id: AxisEdgeId, obstacle_idx: usize) -> bool {
        todo!("Port C# NotRestricting. Check axis_edge_to_obstacle map.")
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
        todo!("Port C# TryToAddRightNeighbor. \
               Check ProjectionsOfEdgesOverlap, then record neighbor pair.")
    }

    // =========================================================================
    // Helper: ProjectionsOfEdgesOverlap — C# lines 125-131
    // =========================================================================

    fn projections_overlap(
        &self,
        left: &AxisEdge,
        right: &AxisEdge,
    ) -> bool {
        todo!("Port C# ProjectionsOfEdgesOverlap EXACTLY.")
    }

    // =========================================================================
    // Helper: GetOrCreateAxisEdgesContainer — C# lines 320-329
    //   GetAxisEdgesContainerNode — C# lines 335-343
    // =========================================================================

    /// O(log n) lookup/insert in the BTreeMap.
    /// C# uses edgeContainersTree.FindFirst with xProjection comparison.
    fn get_or_create_container_key(&mut self, source: Point) -> OrderedFloat<f64> {
        todo!("Port C# GetOrCreateAxisEdgesContainer. \
               Use BTreeMap range query — NOT linear scan.")
    }

    // =========================================================================
    // Helper: RemoveEdge — C# lines 415-420
    // =========================================================================

    fn remove_edge(&mut self, edge_id: AxisEdgeId, source: Point) {
        todo!("Port C# RemoveEdge. O(log n) BTreeMap lookup + remove.")
    }

    // =========================================================================
    // Result application — called after sweep completes
    // =========================================================================

    pub fn apply_results(&self, axis_edges: &mut [AxisEdge]) {
        todo!("Apply accumulated left_bounds, right_bounds, and neighbor_pairs to axis_edges.")
    }
}
