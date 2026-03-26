use std::cmp::Ordering;
use std::collections::BinaryHeap;

use super::compass_direction::CompassDirection;
use crate::geometry::point::Point;
use crate::visibility::graph::{VertexId, VisibilityGraph};

/// Number of compass directions (used for the best-cost table).
const NUM_DIRECTIONS: usize = 4;

/// Default bend penalty as a percentage of source-target Manhattan distance.
/// Matches TS `SsstRectilinearPath.DefaultBendPenaltyAsAPercentageOfDistance = 4`.
pub const DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE: f64 = 4.0;

/// Local arena entry for the A* search.
///
/// Intentionally separate from `vertex_entry::VertexEntry`, which is used
/// by the visibility graph for per-vertex open/closed tracking. This struct
/// is an ephemeral allocation inside the heap arena.
#[derive(Clone, Debug)]
struct SearchEntry {
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
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

/// Direction-aware A* shortest path on a rectilinear visibility graph.
///
/// Corresponds to `SsstRectilinearPath` in the TypeScript source.
///
/// The cost function is:
/// ```text
/// bend_cost  = manhattan(source, target) * bend_penalty_as_percentage / 100
/// total_cost = path_length + bend_cost * number_of_bends
/// ```
///
/// This matches the TS default where each bend costs 4% of the
/// source-to-target Manhattan distance.
pub struct PathSearch {
    /// Bend penalty as a percentage of source-target Manhattan distance.
    /// Default: [`DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE`] (4.0).
    pub bend_penalty_as_percentage: f64,
}

impl PathSearch {
    /// Create a `PathSearch` with the given bend penalty percentage.
    pub fn new(bend_penalty_as_percentage: f64) -> Self {
        Self {
            bend_penalty_as_percentage,
        }
    }

    /// Create a `PathSearch` with the TS default bend penalty (4%).
    pub fn default_penalty() -> Self {
        Self::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE)
    }

    /// Compute the total path cost given length, bend count, and source-target distance.
    ///
    /// Exposed for testing.
    pub fn compute_cost(&self, length: f64, bends: u32, source_target_distance: f64) -> f64 {
        let bend_cost = source_target_distance * self.bend_penalty_as_percentage / 100.0;
        length + bend_cost * bends as f64
    }

    /// Find the shortest (bend-penalized) path between two points in the graph.
    ///
    /// Returns `None` if no path exists between `source` and `target`.
    pub fn find_path(
        &self,
        graph: &VisibilityGraph,
        source: Point,
        target: Point,
    ) -> Option<Vec<Point>> {
        let source_id = graph.find_vertex(source)?;
        let target_id = graph.find_vertex(target)?;

        if source_id == target_id {
            return Some(vec![source, target]);
        }

        let source_target_distance = manhattan_distance(source, target);
        let vertex_count = graph.vertex_count();

        // best_cost[vertex_index][direction_index] — lowest cost seen per (vertex, direction).
        let mut best_cost = vec![[f64::INFINITY; NUM_DIRECTIONS]; vertex_count];

        let mut arena: Vec<SearchEntry> = Vec::new();
        let mut heap: BinaryHeap<QueueItem> = BinaryHeap::new();

        // Seed the heap from source's neighbors.
        let initial_neighbors = self.collect_neighbors(graph, source_id);
        for (neighbor_id, edge_weight) in &initial_neighbors {
            let dir = match CompassDirection::from_points(source, graph.point(*neighbor_id)) {
                Some(d) => d,
                None => continue,
            };

            let length = *edge_weight;
            let cost = self.compute_cost(length, 0, source_target_distance);
            let h = self.heuristic(
                graph.point(*neighbor_id),
                target,
                dir,
                source_target_distance,
            );

            if cost < best_cost[neighbor_id.0][dir.index()] {
                best_cost[neighbor_id.0][dir.index()] = cost;
                let idx = arena.len();
                arena.push(SearchEntry {
                    vertex: *neighbor_id,
                    direction: dir,
                    cost,
                    length,
                    bends: 0,
                    prev_entry: None,
                });
                heap.push(QueueItem {
                    f_score: cost + h,
                    arena_index: idx,
                });
            }
        }

        while let Some(item) = heap.pop() {
            let entry = arena[item.arena_index].clone();

            // Skip stale entries.
            if entry.cost > best_cost[entry.vertex.0][entry.direction.index()] {
                continue;
            }

            // Reached target.
            if entry.vertex == target_id {
                return Some(self.reconstruct_path(&arena, item.arena_index, source, graph));
            }

            let cur_point = graph.point(entry.vertex);
            let neighbors = self.collect_neighbors(graph, entry.vertex);

            for (neighbor_id, edge_weight) in &neighbors {
                // Don't backtrack to source (unless it's also the target).
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
                let new_cost = self.compute_cost(new_length, new_bends, source_target_distance);

                if new_cost < best_cost[neighbor_id.0][dir.index()] {
                    best_cost[neighbor_id.0][dir.index()] = new_cost;
                    let h = self.heuristic(neighbor_point, target, dir, source_target_distance);
                    let idx = arena.len();
                    arena.push(SearchEntry {
                        vertex: *neighbor_id,
                        direction: dir,
                        cost: new_cost,
                        length: new_length,
                        bends: new_bends,
                        prev_entry: Some(item.arena_index),
                    });
                    heap.push(QueueItem {
                        f_score: new_cost + h,
                        arena_index: idx,
                    });
                }
            }
        }

        None
    }

    /// Collect all reachable neighbors from a vertex (out-edges + in-edges reversed).
    fn collect_neighbors(&self, graph: &VisibilityGraph, vertex: VertexId) -> Vec<(VertexId, f64)> {
        let mut neighbors = Vec::new();

        for edge in graph.out_edges(vertex) {
            neighbors.push((edge.target, edge.weight));
        }

        let in_sources: Vec<VertexId> = graph.vertex(vertex).in_edges.clone();
        for src in in_sources {
            let weight = graph
                .out_edges(src)
                .find(|e| e.target == vertex)
                .map(|e| e.weight)
                .unwrap_or_else(|| {
                    let a = graph.point(src);
                    let b = graph.point(vertex);
                    (a - b).length()
                });
            neighbors.push((src, weight));
        }

        neighbors
    }

    /// A* heuristic: Manhattan distance to target plus estimated bend cost.
    fn heuristic(
        &self,
        point: Point,
        target: Point,
        direction: CompassDirection,
        source_target_distance: f64,
    ) -> f64 {
        let dx = (target.x() - point.x()).abs();
        let dy = (target.y() - point.y()).abs();
        let manhattan = dx + dy;
        let est_bends = estimated_bends_to_target(direction, point, target);
        let bend_cost = source_target_distance * self.bend_penalty_as_percentage / 100.0;
        manhattan + bend_cost * est_bends as f64
    }

    /// Walk the `prev_entry` chain to reconstruct the path, discarding
    /// collinear intermediate waypoints.
    fn reconstruct_path(
        &self,
        arena: &[SearchEntry],
        final_index: usize,
        source: Point,
        graph: &VisibilityGraph,
    ) -> Vec<Point> {
        let mut points = Vec::new();
        let mut idx = Some(final_index);

        while let Some(i) = idx {
            let entry = &arena[i];
            points.push(graph.point(entry.vertex));
            idx = entry.prev_entry;
        }

        points.push(source);
        points.reverse();

        if points.len() <= 2 {
            return points;
        }

        let mut filtered = Vec::with_capacity(points.len());
        filtered.push(points[0]);

        for i in 1..points.len() - 1 {
            let prev = *filtered.last().unwrap();
            let curr = points[i];
            let next = points[i + 1];

            let dir1 = CompassDirection::from_points(prev, curr);
            let dir2 = CompassDirection::from_points(curr, next);
            if dir1 != dir2 {
                filtered.push(curr);
            }
        }

        filtered.push(*points.last().unwrap());
        filtered
    }
}

/// Compute the Manhattan distance between two points.
pub fn manhattan_distance(a: Point, b: Point) -> f64 {
    (b.x() - a.x()).abs() + (b.y() - a.y()).abs()
}

/// Estimate the number of bends needed to reach `target` from `point` in `direction`.
pub fn estimated_bends_to_target(direction: CompassDirection, point: Point, target: Point) -> u32 {
    let dx = target.x() - point.x();
    let dy = target.y() - point.y();

    if dx.abs() < 1e-9 && dy.abs() < 1e-9 {
        return 0;
    }

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
        if going_toward {
            1
        } else {
            2
        }
    } else if going_toward {
        0
    } else {
        2
    }
}
