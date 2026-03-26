//! Supporting types for the FreeSpaceFinder sweep-line algorithm.
//!
//! Contains: ObstaclePolyline, SweepEvent, AxisEdgesContainer, ObstacleSide,
//! and EventEntry (priority queue wrapper).

use std::cmp::Ordering;
use std::collections::HashSet;

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;

use super::axis_edge::AxisEdgeId;

// ---------------------------------------------------------------------------
// Obstacle polyline representation (rectangles as 4-vertex closed polylines)
// ---------------------------------------------------------------------------

/// Index of an obstacle (into the obstacles slice).
pub(super) type ObstacleId = usize;

/// Index of a vertex within a rectangle-polyline (0..3, wrapping).
pub(super) type VertexIdx = usize;

/// A rectangle represented as a closed 4-vertex polyline for the sweep.
/// Vertices are ordered counter-clockwise: LB, RB, RT, LT.
#[derive(Clone, Debug)]
pub(super) struct ObstaclePolyline {
    pub obstacle_id: ObstacleId,
    pub vertices: [Point; 4],
}

impl ObstaclePolyline {
    pub fn from_rect(obstacle_id: ObstacleId, r: &Rectangle) -> Self {
        // Counter-clockwise: left-bottom, right-bottom, right-top, left-top.
        Self {
            obstacle_id,
            vertices: [r.left_bottom(), r.right_bottom(), r.right_top(), r.left_top()],
        }
    }

    pub fn vertex(&self, idx: VertexIdx) -> Point {
        self.vertices[idx % 4]
    }

    pub fn next_vertex(&self, idx: VertexIdx) -> VertexIdx {
        (idx + 1) % 4
    }

    pub fn prev_vertex(&self, idx: VertexIdx) -> VertexIdx {
        (idx + 3) % 4
    }
}

// ---------------------------------------------------------------------------
// Sweep events
// ---------------------------------------------------------------------------

/// Discriminant for event types processed by the sweep.
#[derive(Debug, Clone)]
pub(super) enum SweepEvent {
    /// The lowest vertex of an obstacle polyline — starts processing that obstacle.
    LowestVertex {
        site: Point,
        obstacle_id: ObstacleId,
        vertex_idx: VertexIdx,
    },
    /// A left-side vertex event (next vertex along the left side of an obstacle).
    LeftVertex {
        site: Point,
        obstacle_id: ObstacleId,
        vertex_idx: VertexIdx,
    },
    /// A right-side vertex event (next vertex along the right side of an obstacle).
    RightVertex {
        site: Point,
        obstacle_id: ObstacleId,
        vertex_idx: VertexIdx,
    },
    /// An axis edge enters the sweep (its low point).
    AxisEdgeLow {
        site: Point,
        axis_edge_id: AxisEdgeId,
    },
    /// An axis edge exits the sweep (its high point).
    AxisEdgeHigh {
        site: Point,
        axis_edge_id: AxisEdgeId,
    },
}

impl SweepEvent {
    pub fn site(&self) -> Point {
        match self {
            SweepEvent::LowestVertex { site, .. }
            | SweepEvent::LeftVertex { site, .. }
            | SweepEvent::RightVertex { site, .. }
            | SweepEvent::AxisEdgeLow { site, .. }
            | SweepEvent::AxisEdgeHigh { site, .. } => *site,
        }
    }
}

// ---------------------------------------------------------------------------
// AxisEdgesContainer: groups axis edges at the same perpendicular coordinate.
// Corresponds to AxisEdgesContainer.ts.
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub(super) struct AxisEdgesContainer {
    #[allow(dead_code)]
    pub source: Point,
    pub edges: HashSet<AxisEdgeId>,
}

impl AxisEdgesContainer {
    pub fn new(source: Point) -> Self {
        Self {
            source,
            edges: HashSet::new(),
        }
    }

    pub fn add_edge(&mut self, edge_id: AxisEdgeId) {
        self.edges.insert(edge_id);
    }

    pub fn remove_edge(&mut self, edge_id: AxisEdgeId) {
        self.edges.remove(&edge_id);
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Active obstacle side: a segment of an obstacle boundary in the sweep.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(super) struct ObstacleSide {
    pub start: Point,
    pub end: Point,
    pub obstacle_id: ObstacleId,
}

impl ObstacleSide {
    pub fn direction(&self) -> Point {
        self.end - self.start
    }
}

// ---------------------------------------------------------------------------
// Event priority queue entry with ordering for min-heap.
// ---------------------------------------------------------------------------

pub(super) struct EventEntry {
    pub z: OrderedFloat<f64>,
    pub perp: OrderedFloat<f64>,
    pub seq: u64,
    pub event: SweepEvent,
}

impl PartialEq for EventEntry {
    fn eq(&self, other: &Self) -> bool {
        self.z == other.z && self.perp == other.perp && self.seq == other.seq
    }
}
impl Eq for EventEntry {}

impl PartialOrd for EventEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for EventEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap: reverse the natural order.
        // Lower z first; if tied, lower perp first; if tied, lower seq first.
        other
            .z
            .cmp(&self.z)
            .then(other.perp.cmp(&self.perp))
            .then(other.seq.cmp(&self.seq))
    }
}

/// TS: GetLowestPoint — find the vertex with the lowest sweep position.
pub(super) fn get_lowest_vertex(
    poly: &ObstaclePolyline,
    sweep_dir: Point,
    dir_perp: Point,
) -> VertexIdx {
    let mut best = 0;
    for i in 1..4 {
        if sweep_less(poly.vertex(i), poly.vertex(best), sweep_dir, dir_perp) {
            best = i;
        }
    }
    best
}

/// TS: Less (ComparePoints) — compare two points by sweep position, then perp.
fn sweep_less(a: Point, b: Point, sweep_dir: Point, dir_perp: Point) -> bool {
    let az = a.dot(sweep_dir);
    let bz = b.dot(sweep_dir);
    if az < bz { return true; }
    if az > bz { return false; }
    a.dot(dir_perp) < b.dot(dir_perp)
}
