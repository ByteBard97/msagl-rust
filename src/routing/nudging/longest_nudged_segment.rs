//! LongestNudgedSegment: groups consecutive parallel PathEdges that move together.

use crate::geometry::point::Point;
use crate::routing::scan_direction::Direction;

use super::path_edge::PathEdgeId;

/// Groups consecutive collinear PathEdges along the nudging direction.
///
/// All edges in the group share the same perpendicular coordinate and
/// are assigned a single solver variable so they move as a unit.
#[derive(Debug, Clone)]
pub struct LongestNudgedSegment {
    /// Solver variable ID for this segment.
    pub id: usize,
    /// Direction of the segment (North or East).
    pub direction: Direction,
    /// PathEdge IDs belonging to this segment.
    pub edges: Vec<PathEdgeId>,
    /// Start point (min in the segment direction).
    pub start: Point,
    /// End point (max in the segment direction).
    pub end: Point,
    /// Whether this segment is pinned (e.g. at a port).
    pub is_fixed: bool,
    /// Ideal position for the solver (perpendicular projection).
    pub ideal_position: f64,
}

impl LongestNudgedSegment {
    pub fn new(id: usize) -> Self {
        Self {
            id,
            direction: Direction::North,
            edges: Vec::new(),
            start: Point::ORIGIN,
            end: Point::ORIGIN,
            is_fixed: false,
            ideal_position: 0.0,
        }
    }

    /// Add an edge and update the segment's start/end bounds.
    pub fn add_edge(&mut self, edge_id: PathEdgeId, source: Point, target: Point) {
        if self.edges.is_empty() {
            let dir = direction_from_points_loose(source, target);
            self.direction = dir;
            self.start = source;
            self.end = source;
        }

        match self.direction {
            Direction::North => {
                self.try_update_north(source);
                self.try_update_north(target);
            }
            Direction::East => {
                self.try_update_east(source);
                self.try_update_east(target);
            }
        }

        self.edges.push(edge_id);
    }

    fn try_update_north(&mut self, p: Point) {
        if p.y() < self.start.y() {
            self.start = p;
        } else if p.y() > self.end.y() {
            self.end = p;
        }
    }

    fn try_update_east(&mut self, p: Point) {
        if p.x() < self.start.x() {
            self.start = p;
        } else if p.x() > self.end.x() {
            self.end = p;
        }
    }

    /// Current position of this segment (perpendicular projection).
    pub fn position(&self) -> f64 {
        match self.direction {
            Direction::North => self.start.x(),
            Direction::East => -self.start.y(),
        }
    }

    /// Maximum width among all edges in this segment.
    #[allow(dead_code)]
    pub fn max_width(&self) -> f64 {
        // Width is stored on PathEdges; the caller must pass it in.
        // For now, return 0 (default edge width).
        0.0
    }
}

/// Like direction_from_points but also handles South/West by normalizing.
fn direction_from_points_loose(source: Point, target: Point) -> Direction {
    let dx = target.x() - source.x();
    let dy = target.y() - source.y();
    if dx.abs() > dy.abs() {
        Direction::East
    } else {
        Direction::North
    }
}
