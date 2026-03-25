use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::geometry::point::Point;
use crate::visibility::graph::{VertexId, VisibilityGraph};

/// Cardinal compass directions for tracking entry into a vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CompassDirection {
    North,
    East,
    South,
    West,
}

const NUM_DIRECTIONS: usize = 4;

impl CompassDirection {
    pub fn index(self) -> usize {
        match self {
            CompassDirection::North => 0,
            CompassDirection::East => 1,
            CompassDirection::South => 2,
            CompassDirection::West => 3,
        }
    }

    /// Determine compass direction from `from` to `to`.
    ///
    /// For rectilinear graphs, edges are axis-aligned. Returns `None` if
    /// the points are identical.
    pub fn from_points(from: Point, to: Point) -> Option<Self> {
        let dx = to.x() - from.x();
        let dy = to.y() - from.y();

        if dx.abs() > dy.abs() {
            if dx > 0.0 { Some(CompassDirection::East) }
            else { Some(CompassDirection::West) }
        } else if dy.abs() > dx.abs() {
            if dy > 0.0 { Some(CompassDirection::North) }
            else { Some(CompassDirection::South) }
        } else if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
            None
        } else {
            // Diagonal tie-break: prefer horizontal
            if dx > 0.0 { Some(CompassDirection::East) }
            else { Some(CompassDirection::West) }
        }
    }

    pub fn opposite(self) -> Self {
        match self {
            CompassDirection::North => CompassDirection::South,
            CompassDirection::East => CompassDirection::West,
            CompassDirection::South => CompassDirection::North,
            CompassDirection::West => CompassDirection::East,
        }
    }
}

/// An entry in the A* search arena.
#[derive(Clone, Debug)]
struct VertexEntry {
    vertex: VertexId,
    direction: CompassDirection,
    cost: f64,
    length: f64,
    bends: u32,
    prev_entry: Option<usize>,
}

/// A priority queue item wrapping an arena index and its f-score.
#[derive(Clone, Debug)]
struct QueueItem {
    f_score: f64,
    arena_index: usize,
}

impl PartialEq for QueueItem {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}
impl Eq for QueueItem {}

impl PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Min-heap: reverse ordering so smallest f_score comes first.
impl Ord for QueueItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.partial_cmp(&self.f_score).unwrap_or(Ordering::Equal)
    }
}

/// Direction-aware A* path search on a visibility graph.
///
/// Tracks entry direction at each vertex so that bends (direction changes)
/// can be penalized. The cost function is:
///   `length_importance * path_length + bends_importance * bend_count`
pub struct PathSearch {
    pub length_importance: f64,
    pub bends_importance: f64,
}

impl PathSearch {
    pub fn new(length_importance: f64, bends_importance: f64) -> Self {
        Self { length_importance, bends_importance }
    }

    fn combined_cost(&self, length: f64, bends: u32) -> f64 {
        self.length_importance * length + self.bends_importance * bends as f64
    }

    /// Find the shortest (bend-penalized) path between two points.
    ///
    /// Returns `None` if no path exists.
    pub fn find_path(
        &self,
        graph: &VisibilityGraph,
        source: Point,
        target: Point,
    ) -> Option<Vec<Point>> {
        let source_id = graph.find_vertex(source)?;
        let target_id = graph.find_vertex(target)?;

        if source_id == target_id {
            return Some(vec![source]);
        }

        let vertex_count = graph.vertex_count();
        // best_cost[vertex_index][direction_index] tracks best cost seen
        let mut best_cost = vec![[f64::INFINITY; NUM_DIRECTIONS]; vertex_count];

        let mut arena: Vec<VertexEntry> = Vec::new();
        let mut heap: BinaryHeap<QueueItem> = BinaryHeap::new();

        // Initialize: seed from source's neighbors
        let neighbors = self.collect_neighbors(graph, source_id);
        for (neighbor_id, edge_weight) in &neighbors {
            let dir = match CompassDirection::from_points(source, graph.point(*neighbor_id)) {
                Some(d) => d,
                None => continue,
            };

            let length = *edge_weight;
            let cost = self.combined_cost(length, 0);
            let h = self.heuristic(graph.point(*neighbor_id), target, dir);

            if cost < best_cost[neighbor_id.0][dir.index()] {
                best_cost[neighbor_id.0][dir.index()] = cost;
                let idx = arena.len();
                arena.push(VertexEntry {
                    vertex: *neighbor_id,
                    direction: dir,
                    cost,
                    length,
                    bends: 0,
                    prev_entry: None,
                });
                heap.push(QueueItem { f_score: cost + h, arena_index: idx });
            }
        }

        while let Some(item) = heap.pop() {
            let entry = arena[item.arena_index].clone();

            // Skip stale entries
            if entry.cost > best_cost[entry.vertex.0][entry.direction.index()] {
                continue;
            }

            // Reached target?
            if entry.vertex == target_id {
                return Some(self.reconstruct_path(
                    &arena, item.arena_index, source, graph,
                ));
            }

            // Expand neighbors (bidirectional traversal)
            let cur_point = graph.point(entry.vertex);
            let neighbors = self.collect_neighbors(graph, entry.vertex);

            for (neighbor_id, edge_weight) in &neighbors {
                // Don't go back to source unless it's the target
                if *neighbor_id == source_id && *neighbor_id != target_id {
                    continue;
                }

                let neighbor_point = graph.point(*neighbor_id);
                let dir = match CompassDirection::from_points(cur_point, neighbor_point) {
                    Some(d) => d,
                    None => continue,
                };

                let new_bends = if dir != entry.direction {
                    entry.bends + 1
                } else {
                    entry.bends
                };
                let new_length = entry.length + edge_weight;
                let new_cost = self.combined_cost(new_length, new_bends);

                if new_cost < best_cost[neighbor_id.0][dir.index()] {
                    best_cost[neighbor_id.0][dir.index()] = new_cost;
                    let h = self.heuristic(neighbor_point, target, dir);
                    let idx = arena.len();
                    arena.push(VertexEntry {
                        vertex: *neighbor_id,
                        direction: dir,
                        cost: new_cost,
                        length: new_length,
                        bends: new_bends,
                        prev_entry: Some(item.arena_index),
                    });
                    heap.push(QueueItem { f_score: new_cost + h, arena_index: idx });
                }
            }
        }

        None
    }

    /// Collect all reachable neighbors from a vertex (out-edges + in-edges reversed).
    fn collect_neighbors(
        &self,
        graph: &VisibilityGraph,
        vertex: VertexId,
    ) -> Vec<(VertexId, f64)> {
        let mut neighbors = Vec::new();

        // Out-edges
        for edge in graph.out_edges(vertex) {
            neighbors.push((edge.target, edge.weight));
        }

        // In-edges (reversed): walk back toward sources
        let in_sources: Vec<VertexId> = graph.vertex(vertex).in_edges.clone();
        for src in in_sources {
            // Find the edge weight from src -> vertex
            let weight = graph.out_edges(src)
                .find(|e| e.target == vertex)
                .map(|e| e.weight)
                .unwrap_or_else(|| {
                    // Fallback: compute Euclidean distance
                    let a = graph.point(src);
                    let b = graph.point(vertex);
                    (a - b).length()
                });
            neighbors.push((src, weight));
        }

        neighbors
    }

    /// Heuristic: Manhattan distance + estimated bends to target.
    fn heuristic(&self, point: Point, target: Point, direction: CompassDirection) -> f64 {
        let dx = (target.x() - point.x()).abs();
        let dy = (target.y() - point.y()).abs();
        let manhattan = dx + dy;
        let est_bends = estimated_bends_to_target(direction, point, target);
        self.combined_cost(manhattan, est_bends)
    }

    /// Walk the prev_entry chain to reconstruct the path, skipping collinear
    /// intermediate points.
    fn reconstruct_path(
        &self,
        arena: &[VertexEntry],
        final_index: usize,
        source: Point,
        graph: &VisibilityGraph,
    ) -> Vec<Point> {
        // Collect all points from target back to first entry after source
        let mut points = Vec::new();
        let mut idx = Some(final_index);

        while let Some(i) = idx {
            let entry = &arena[i];
            points.push(graph.point(entry.vertex));
            idx = entry.prev_entry;
        }

        // Add source at the end (since we walked backward)
        points.push(source);
        points.reverse();

        // Remove collinear intermediate points
        if points.len() <= 2 {
            return points;
        }

        let mut filtered = Vec::with_capacity(points.len());
        filtered.push(points[0]);

        for i in 1..points.len() - 1 {
            let prev = filtered.last().unwrap();
            let curr = &points[i];
            let next = &points[i + 1];

            // Keep point if direction changes (not collinear)
            let dir1 = CompassDirection::from_points(*prev, *curr);
            let dir2 = CompassDirection::from_points(*curr, *next);
            if dir1 != dir2 {
                filtered.push(*curr);
            }
        }

        filtered.push(*points.last().unwrap());
        filtered
    }
}

/// Estimate the number of bends needed to reach target from current direction.
fn estimated_bends_to_target(
    direction: CompassDirection,
    point: Point,
    target: Point,
) -> u32 {
    let dx = target.x() - point.x();
    let dy = target.y() - point.y();

    // If essentially at the target, no bends needed
    if dx.abs() < 1e-9 && dy.abs() < 1e-9 {
        return 0;
    }

    // If aligned on one axis, check if going in the right direction
    let going_toward = match direction {
        CompassDirection::East => dx > 1e-9,
        CompassDirection::West => dx < -1e-9,
        CompassDirection::North => dy > 1e-9,
        CompassDirection::South => dy < -1e-9,
    };

    let need_horizontal = dx.abs() > 1e-9;
    let need_vertical = dy.abs() > 1e-9;

    if !need_horizontal && !need_vertical {
        0
    } else if need_horizontal && need_vertical {
        // Need to go both horizontally and vertically: at least 1 bend
        if going_toward { 1 } else { 2 }
    } else {
        // Only one axis needed
        if going_toward { 0 } else { 2 }
    }
}
