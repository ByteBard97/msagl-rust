//! Rectilinear path search.
//!
//! Contains both:
//! - `SsstRectilinearPath`: Faithful port of TS/C# SsstRectilinearPath using
//!   VertexEntry[4] direction tracking, NextNeighbor[3] ordering, and
//!   direction-aware heuristic bend estimation.
//! - `PathSearch`: The working search used by the router. Treats the graph
//!   as undirected (both in-edges and out-edges), which matches the Rust VG
//!   where edges are stored in ascending direction only.

use std::collections::BinaryHeap;
use std::cmp::Ordering;

use crate::geometry::point::Point;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use super::compass_direction::{CompassDirection, DirectionFlags, ADD_ONE_TURN};
use super::vertex_entry::{VertexEntryArena, VertexEntryIndex};

/// Default bend penalty as a percentage of source-target Manhattan distance.
/// Matches TS `SsstRectilinearPath.DefaultBendPenaltyAsAPercentageOfDistance = 4`.
pub const DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE: f64 = 4.0;

/// Weight for overlapped scan segments. Matches TS `ScanSegment.OverlappedWeight = 500`.
pub const OVERLAPPED_WEIGHT: f64 = 500.0;

/// Epsilon for coordinate comparison. Matches TS `closeDistEps`.
const CLOSE_DIST_EPS: f64 = 1e-6;

/// Number of compass directions.
const NUM_DIRECTIONS: usize = 4;

// ===========================================================================
// SsstRectilinearPath — faithful port of TS/C# direction-aware A*
// ===========================================================================

/// NextNeighbor slot for the 3-way neighbor ordering.
/// Slot layout: [0]=other bend, [1]=preferred bend (right), [2]=straight.
#[derive(Clone)]
struct NextNeighbor {
    vertex: Option<VertexId>,
    weight: f64,
}

impl NextNeighbor {
    fn new() -> Self { Self { vertex: None, weight: f64::NAN } }
    fn set(&mut self, v: VertexId, w: f64) { self.vertex = Some(v); self.weight = w; }
    fn clear(&mut self) { self.vertex = None; self.weight = f64::NAN; }
}

/// Priority queue item for the SsstRectilinearPath search.
#[derive(Clone, Debug)]
struct SsstQueueItem {
    priority: f64,
    entry_index: VertexEntryIndex,
}
impl PartialEq for SsstQueueItem { fn eq(&self, o: &Self) -> bool { self.priority == o.priority } }
impl Eq for SsstQueueItem {}
impl PartialOrd for SsstQueueItem { fn partial_cmp(&self, o: &Self) -> Option<Ordering> { Some(self.cmp(o)) } }
impl Ord for SsstQueueItem {
    fn cmp(&self, o: &Self) -> Ordering { o.priority.partial_cmp(&self.priority).unwrap_or(Ordering::Equal) }
}

/// Single source single target rectilinear path search.
///
/// Faithful port of `SsstRectilinearPath` from TS/C#. Uses VertexEntry[4]
/// per vertex for direction tracking, NextNeighbor[3] optimization, and
/// heuristic with estimated bends to target.
pub struct SsstRectilinearPath {
    pub length_importance: f64,
    pub bends_importance: f64,
    target: VertexId,
    source: VertexId,
    entry_directions_to_target: DirectionFlags,
    upper_bound_on_cost: f64,
    source_cost_adjustment: f64,
    target_cost_adjustment: f64,
    queue: BinaryHeap<SsstQueueItem>,
    visited_vertices: Vec<VertexId>,
    next_neighbors: [NextNeighbor; 3],
    arena: VertexEntryArena,
}

impl Default for SsstRectilinearPath {
    fn default() -> Self { Self::new() }
}

impl SsstRectilinearPath {
    pub fn new() -> Self {
        Self {
            length_importance: 1.0, bends_importance: 1.0,
            target: VertexId(0), source: VertexId(0),
            entry_directions_to_target: DirectionFlags::NONE,
            upper_bound_on_cost: f64::INFINITY,
            source_cost_adjustment: 0.0, target_cost_adjustment: 0.0,
            queue: BinaryHeap::new(), visited_vertices: Vec::new(),
            next_neighbors: [NextNeighbor::new(), NextNeighbor::new(), NextNeighbor::new()],
            arena: VertexEntryArena::new(),
        }
    }

    fn combined_cost(&self, length: f64, bends: u32) -> f64 {
        self.length_importance * length + self.bends_importance * bends as f64
    }

    fn total_cost_from_source(&self, length: f64, bends: u32) -> f64 {
        self.combined_cost(length, bends) + self.source_cost_adjustment
    }

    pub fn multistage_adjusted_cost_bound(&self, best_cost: f64) -> f64 {
        if best_cost.is_finite() { best_cost + self.bends_importance } else { best_cost }
    }

    fn heuristic_distance(&self, point: Point, entry_dir: Option<CompassDirection>, graph: &VisibilityGraph) -> f64 {
        let tp = graph.point(self.target);
        let dx = tp.x() - point.x();
        let dy = tp.y() - point.y();
        if dx.abs() < CLOSE_DIST_EPS && dy.abs() < CLOSE_DIST_EPS {
            return self.target_cost_adjustment;
        }
        let dir_to_target = DirectionFlags::vector_direction(dx, dy);
        let entry_flags = match entry_dir {
            None => DirectionFlags::ALL,
            Some(d) => DirectionFlags::from_direction(d),
        };
        let bends = self.get_number_of_bends(entry_flags, dir_to_target);
        self.combined_cost(manhattan_distance(point, tp), bends) + self.target_cost_adjustment
    }

    fn get_number_of_bends(&self, entry_dir: DirectionFlags, dir_to_target: DirectionFlags) -> u32 {
        if dir_to_target.is_pure() {
            self.get_bends_pure(entry_dir, dir_to_target)
        } else {
            Self::get_bends_not_pure(dir_to_target, entry_dir, self.entry_directions_to_target)
        }
    }

    fn get_bends_pure(&self, entry_dir: DirectionFlags, dir_to_target: DirectionFlags) -> u32 {
        if entry_dir.contains(dir_to_target) {
            if self.entry_directions_to_target.contains(dir_to_target) { return 0; }
            if self.entry_directions_to_target.contains(dir_to_target.left())
                || self.entry_directions_to_target.contains(dir_to_target.right()) { return 2; }
            return 4;
        }
        let expanded = DirectionFlags(ADD_ONE_TURN[entry_dir.0 as usize]);
        self.get_bends_pure(expanded, dir_to_target) + 1
    }

    fn get_bends_not_pure(dtt: DirectionFlags, ed: DirectionFlags, edtt: DirectionFlags) -> u32 {
        let a = dtt.intersect(ed);
        if a.is_none() {
            return Self::get_bends_not_pure(dtt, DirectionFlags(ADD_ONE_TURN[ed.0 as usize]), edtt) + 1;
        }
        let b = dtt.intersect(edtt);
        if b.is_none() {
            return Self::get_bends_not_pure(dtt, ed, DirectionFlags(ADD_ONE_TURN[edtt.0 as usize])) + 1;
        }
        if a.union(b) == dtt { 1 } else { 2 }
    }

    /// Get (direction, length, bends) for extending to a neighbor.
    /// `weight` is the stored edge weight (distance * multiplier in the Rust VG).
    fn get_length_and_bends(&self, prev_idx: VertexEntryIndex, vertex: VertexId, weight: f64, graph: &VisibilityGraph) -> (CompassDirection, f64, u32) {
        let prev = self.arena.get(prev_idx);
        let length = prev.length + weight;
        let dir = CompassDirection::from_points(graph.point(prev.vertex), graph.point(vertex))
            .expect("neighbor at same point");
        let bends = if prev.direction.is_some() && Some(dir) != prev.direction { prev.number_of_bends + 1 } else { prev.number_of_bends };
        (dir, length, bends)
    }

    fn init_entry_dirs_at_target(&mut self, vert: VertexId, graph: &VisibilityGraph) -> bool {
        self.entry_directions_to_target = DirectionFlags::NONE;
        for edge in graph.out_edges(vert) {
            if let Some(dir) = CompassDirection::from_points(graph.point(edge.target), graph.point(vert)) {
                self.entry_directions_to_target = self.entry_directions_to_target.union(DirectionFlags::from_direction(dir));
            }
        }
        for &src in graph.in_edges(vert) {
            if let Some(dir) = CompassDirection::from_points(graph.point(src), graph.point(vert)) {
                self.entry_directions_to_target = self.entry_directions_to_target.union(DirectionFlags::from_direction(dir));
            }
        }
        !self.entry_directions_to_target.is_none()
    }

    fn init_path(&mut self, src_entries: Option<&[Option<VertexEntryIndex>; 4]>, source: VertexId, target: VertexId, graph: &mut VisibilityGraph) -> bool {
        if source == target || !self.init_entry_dirs_at_target(target, graph) { return false; }
        self.target = target;
        self.source = source;
        let cost = self.total_cost_from_source(0.0, 0) + self.heuristic_distance(graph.point(source), None, graph);
        if cost >= self.upper_bound_on_cost { return false; }
        self.queue = BinaryHeap::new();
        self.arena = VertexEntryArena::new();
        self.visited_vertices = vec![source];
        match src_entries {
            None => self.enqueue_initial_from_source(cost, graph),
            Some(entries) => {
                for e in entries.iter().flatten() {
                    let c = self.arena.get(*e).cost;
                    self.queue.push(SsstQueueItem { priority: c, entry_index: *e });
                }
            }
        }
        !self.queue.is_empty()
    }

    fn enqueue_initial_from_source(&mut self, cost: f64, graph: &mut VisibilityGraph) {
        let src_idx = self.arena.create_entry(self.source, None, None, 0.0, 0, cost);
        self.arena.get_mut(src_idx).is_closed = true;
        let neighbors = collect_neighbors(graph, self.source);
        for (neig, weight) in neighbors {
            self.extend_to_neighbor(src_idx, neig, weight, graph);
        }
    }

    /// Main entry point. Matches TS `GetPathWithCost(...)`.
    #[allow(clippy::too_many_arguments)]
    pub fn get_path_with_cost(
        &mut self, src_entries: Option<&[Option<VertexEntryIndex>; 4]>, source: VertexId, src_adj: f64,
        mut tgt_entries: Option<&mut [Option<VertexEntryIndex>; 4]>, target: VertexId, tgt_adj: f64,
        prior_best: f64, graph: &mut VisibilityGraph,
    ) -> Option<VertexEntryIndex> {
        self.upper_bound_on_cost = prior_best;
        self.source_cost_adjustment = src_adj;
        self.target_cost_adjustment = tgt_adj;
        if !self.init_path(src_entries, source, target, graph) { return None; }

        while let Some(item) = self.queue.pop() {
            let idx = item.entry_index;
            if self.arena.get(idx).is_closed { continue; }
            let vert = self.arena.get(idx).vertex;

            if vert == self.target {
                if tgt_entries.is_none() { self.cleanup(graph); return Some(idx); }
                if let Some(dir) = self.arena.get(idx).direction {
                    self.entry_directions_to_target = self.entry_directions_to_target.remove(DirectionFlags::from_direction(dir));
                }
                if self.entry_directions_to_target.is_none() {
                    if let Some(ref mut tve) = tgt_entries { **tve = *graph.vertex_entries(self.target); }
                    self.cleanup(graph);
                    return None;
                }
                let ec = self.arena.get(idx).cost;
                self.upper_bound_on_cost = self.multistage_adjusted_cost_bound(ec).min(self.upper_bound_on_cost);
                continue;
            }

            self.arena.get_mut(idx).is_closed = true;
            for nn in &mut self.next_neighbors { nn.clear(); }
            let best_dir = self.arena.get(idx).direction;
            let pref_bend = best_dir.map(|d: CompassDirection| d.right());

            // Collect all neighbors (both directions) and classify into 3 slots
            let neighbors = collect_neighbors(graph, vert);
            let prev_vert = self.arena.previous_vertex(idx);
            for (neig, weight) in &neighbors {
                if Some(*neig) == prev_vert
                    && (graph.degree(vert) > 1 || vert != self.source)
                {
                    continue;
                }
                let neig_dir = match CompassDirection::from_points(graph.point(vert), graph.point(*neig)) {
                    Some(d) => d, None => continue,
                };
                let slot = if Some(neig_dir) == best_dir { 2 }
                    else if Some(neig_dir) == pref_bend { 1 }
                    else { 0 };
                self.next_neighbors[slot].set(*neig, *weight);
            }

            for i in 0..3 {
                if let Some(v) = self.next_neighbors[i].vertex {
                    let w = self.next_neighbors[i].weight;
                    self.extend_to_neighbor(idx, v, w, graph);
                }
            }
        }

        if let Some(ref mut tve) = tgt_entries {
            if graph.has_vertex_entries(self.target) { **tve = *graph.vertex_entries(self.target); }
        }
        self.cleanup(graph);
        None
    }

    fn extend_to_neighbor(&mut self, best_idx: VertexEntryIndex, neig: VertexId, weight: f64, graph: &mut VisibilityGraph) {
        let best_vert = self.arena.get(best_idx).vertex;
        let dir = match CompassDirection::from_points(graph.point(best_vert), graph.point(neig)) {
            Some(d) => d, None => return,
        };
        let neig_entry = graph.vertex_entry(neig, dir);
        match neig_entry {
            None => {
                if !self.try_reversed(best_idx, neig, weight, graph) {
                    self.create_and_enqueue(best_idx, neig, weight, graph);
                }
            }
            Some(ne_idx) => {
                if !self.arena.get(ne_idx).is_closed {
                    self.update_if_needed(best_idx, ne_idx, weight, graph);
                }
            }
        }
    }

    fn create_and_enqueue(&mut self, best_idx: VertexEntryIndex, neig: VertexId, weight: f64, graph: &mut VisibilityGraph) {
        let (dir, len, bends) = self.get_length_and_bends(best_idx, neig, weight, graph);
        let cost = self.total_cost_from_source(len, bends) + self.heuristic_distance(graph.point(neig), Some(dir), graph);
        if cost < self.upper_bound_on_cost {
            if !graph.has_vertex_entries(neig) { self.visited_vertices.push(neig); }
            self.enqueue_entry(best_idx, neig, len, bends, cost, graph);
        }
    }

    fn enqueue_entry(&mut self, best_idx: VertexEntryIndex, neig: VertexId, len: f64, bends: u32, cost: f64, graph: &mut VisibilityGraph) {
        let dir = CompassDirection::from_points(graph.point(self.arena.get(best_idx).vertex), graph.point(neig));
        let idx = self.arena.create_entry(neig, dir, Some(best_idx), len, bends, cost);
        if let Some(d) = dir { graph.set_vertex_entry(neig, d, Some(idx)); }
        self.queue.push(SsstQueueItem { priority: cost, entry_index: idx });
    }

    fn update_if_needed(&mut self, best_idx: VertexEntryIndex, neig_idx: VertexEntryIndex, weight: f64, graph: &VisibilityGraph) {
        let nv = self.arena.get(neig_idx).vertex;
        let (dir, len, bends) = self.get_length_and_bends(best_idx, nv, weight, graph);
        let ne = self.arena.get(neig_idx);
        if self.combined_cost(len, bends) < self.combined_cost(ne.length, ne.number_of_bends) {
            let total = self.total_cost_from_source(len, bends) + self.heuristic_distance(graph.point(nv), Some(dir), graph);
            self.arena.get_mut(neig_idx).reset_entry(Some(best_idx), len, bends, total);
            self.queue.push(SsstQueueItem { priority: total, entry_index: neig_idx });
        }
    }

    fn try_reversed(&mut self, best_idx: VertexEntryIndex, neig: VertexId, weight: f64, graph: &mut VisibilityGraph) -> bool {
        let bv = self.arena.get(best_idx).vertex;
        if !graph.has_vertex_entries(bv) { return false; }
        let dir_from = match CompassDirection::from_points(graph.point(neig), graph.point(bv)) { Some(d) => d, None => return false };
        match graph.vertex_entry(bv, dir_from) {
            Some(efn_idx) => {
                if let Some(nv) = self.arena.previous_vertex(efn_idx) {
                    let (dir, len, bends) = self.get_length_and_bends(best_idx, nv, weight, graph);
                    let efn = self.arena.get(efn_idx);
                    if self.combined_cost(len, bends) < self.combined_cost(efn.length, efn.number_of_bends) || graph.degree(bv) == 1 {
                        let cost = self.total_cost_from_source(len, bends) + self.heuristic_distance(graph.point(nv), Some(dir), graph);
                        self.enqueue_entry(best_idx, nv, len, bends, cost, graph);
                    }
                }
                true
            }
            None => false,
        }
    }

    pub fn restore_path(arena: &VertexEntryArena, entry_idx: Option<VertexEntryIndex>, first_stage: Option<VertexId>, graph: &VisibilityGraph) -> Vec<Point> {
        let mut idx = match entry_idx { Some(i) => i, None => return Vec::new() };
        let mut list = Vec::new();
        #[allow(unused_assignments)]
        let mut skipped = false;
        let mut last_dir: Option<CompassDirection> = None;
        loop {
            let e = arena.get(idx);
            if e.direction == last_dir && last_dir.is_some() { skipped = true; }
            else { skipped = false; list.push(graph.point(e.vertex)); last_dir = e.direction; }
            match e.previous_entry {
                None => break,
                Some(p) => { if first_stage == Some(e.vertex) { break; } idx = p; }
            }
        }
        if skipped { list.push(graph.point(arena.get(idx).vertex)); }
        list.reverse();
        list
    }

    pub fn restore_path_v(arena: &VertexEntryArena, entry_idx: Option<VertexEntryIndex>, graph: &VisibilityGraph) -> Vec<Point> {
        Self::restore_path(arena, entry_idx, None, graph)
    }

    fn cleanup(&mut self, graph: &mut VisibilityGraph) {
        for &v in &self.visited_vertices { graph.remove_vertex_entries(v); }
        self.visited_vertices.clear();
        self.queue = BinaryHeap::new();
    }

    pub fn arena(&self) -> &VertexEntryArena { &self.arena }
}

// ===========================================================================
// PathSearch — proven working search used by the router
// ===========================================================================

/// Local arena entry for the A* search.
#[derive(Clone, Debug)]
struct SearchEntry {
    vertex: VertexId,
    direction: CompassDirection,
    cost: f64,
    length: f64,
    bends: u32,
    prev_entry: Option<usize>,
}

/// Queue item for PathSearch.
#[derive(Clone, Debug)]
struct PsQueueItem {
    f_score: f64,
    arena_index: usize,
}
impl PartialEq for PsQueueItem { fn eq(&self, o: &Self) -> bool { self.f_score == o.f_score } }
impl Eq for PsQueueItem {}
impl PartialOrd for PsQueueItem { fn partial_cmp(&self, o: &Self) -> Option<Ordering> { Some(self.cmp(o)) } }
impl Ord for PsQueueItem {
    fn cmp(&self, o: &Self) -> Ordering { o.f_score.partial_cmp(&self.f_score).unwrap_or(Ordering::Equal) }
}

/// Direction-aware A* on a rectilinear visibility graph.
/// Treats the graph as undirected (both in-edges and out-edges).
pub struct PathSearch {
    pub bend_penalty_as_percentage: f64,
}

impl PathSearch {
    pub fn new(bend_penalty_as_percentage: f64) -> Self {
        Self { bend_penalty_as_percentage }
    }

    pub fn default_penalty() -> Self {
        Self::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE)
    }

    pub fn compute_cost(&self, length: f64, bends: u32, st_dist: f64) -> f64 {
        let bend_cost = st_dist * self.bend_penalty_as_percentage / 100.0;
        length + bend_cost * bends as f64
    }

    pub fn find_path(&self, graph: &mut VisibilityGraph, source: Point, target: Point) -> Option<Vec<Point>> {
        let source_id = graph.find_vertex(source)?;
        let target_id = graph.find_vertex(target)?;
        if source_id == target_id { return Some(vec![source]); }

        let st_dist = manhattan_distance(source, target);
        let vc = graph.vertex_count();
        let mut best_cost = vec![[f64::INFINITY; NUM_DIRECTIONS]; vc];
        let mut arena: Vec<SearchEntry> = Vec::new();
        let mut heap: BinaryHeap<PsQueueItem> = BinaryHeap::new();

        let initial = collect_neighbors(graph, source_id);
        for (nid, ew) in &initial {
            let dir = match CompassDirection::from_points(source, graph.point(*nid)) { Some(d) => d, None => continue };
            let cost = self.compute_cost(*ew, 0, st_dist);
            let h = self.heuristic(graph.point(*nid), target, dir, st_dist);
            if cost < best_cost[nid.0][dir.index()] {
                best_cost[nid.0][dir.index()] = cost;
                let idx = arena.len();
                arena.push(SearchEntry { vertex: *nid, direction: dir, cost, length: *ew, bends: 0, prev_entry: None });
                heap.push(PsQueueItem { f_score: cost + h, arena_index: idx });
            }
        }

        while let Some(item) = heap.pop() {
            let entry = arena[item.arena_index].clone();
            if entry.cost > best_cost[entry.vertex.0][entry.direction.index()] { continue; }
            if entry.vertex == target_id { return Some(self.reconstruct(&arena, item.arena_index, source, graph)); }

            let neighbors = collect_neighbors(graph, entry.vertex);
            for (nid, ew) in &neighbors {
                if *nid == source_id && *nid != target_id { continue; }
                let np = graph.point(*nid);
                let dir = match CompassDirection::from_points(graph.point(entry.vertex), np) { Some(d) => d, None => continue };
                let bends = if dir != entry.direction { entry.bends + 1 } else { entry.bends };
                let len = entry.length + ew;
                let cost = self.compute_cost(len, bends, st_dist);
                if cost < best_cost[nid.0][dir.index()] {
                    best_cost[nid.0][dir.index()] = cost;
                    let h = self.heuristic(np, target, dir, st_dist);
                    let idx = arena.len();
                    arena.push(SearchEntry { vertex: *nid, direction: dir, cost, length: len, bends, prev_entry: Some(item.arena_index) });
                    heap.push(PsQueueItem { f_score: cost + h, arena_index: idx });
                }
            }
        }
        None
    }

    fn heuristic(&self, point: Point, target: Point, dir: CompassDirection, st_dist: f64) -> f64 {
        let manhattan = manhattan_distance(point, target);
        let bends = estimated_bends_to_target(dir, point, target);
        let bc = st_dist * self.bend_penalty_as_percentage / 100.0;
        manhattan + bc * bends as f64
    }

    fn reconstruct(&self, arena: &[SearchEntry], final_idx: usize, source: Point, graph: &VisibilityGraph) -> Vec<Point> {
        let mut pts = Vec::new();
        let mut idx = Some(final_idx);
        while let Some(i) = idx { pts.push(graph.point(arena[i].vertex)); idx = arena[i].prev_entry; }
        pts.push(source);
        pts.reverse();
        if pts.len() <= 2 { return pts; }
        let mut filtered = Vec::with_capacity(pts.len());
        filtered.push(pts[0]);
        for i in 1..pts.len() - 1 {
            let d1 = CompassDirection::from_points(*filtered.last().unwrap(), pts[i]);
            let d2 = CompassDirection::from_points(pts[i], pts[i + 1]);
            if d1 != d2 { filtered.push(pts[i]); }
        }
        filtered.push(*pts.last().unwrap());
        filtered
    }
}

// ===========================================================================
// Helpers
// ===========================================================================

/// Compute the Manhattan distance between two points.
pub fn manhattan_distance(a: Point, b: Point) -> f64 {
    (b.x() - a.x()).abs() + (b.y() - a.y()).abs()
}

/// Collect all neighbors of a vertex (out-edges + in-edges reversed).
fn collect_neighbors(graph: &VisibilityGraph, vertex: VertexId) -> Vec<(VertexId, f64)> {
    let mut neighbors = Vec::new();
    for edge in graph.out_edges(vertex) {
        neighbors.push((edge.target, edge.weight));
    }
    let in_sources: Vec<VertexId> = graph.in_edges(vertex).to_vec();
    for src in in_sources {
        let weight = graph.find_edge(src, vertex).map(|e| e.weight).unwrap_or_else(|| {
            manhattan_distance(graph.point(src), graph.point(vertex))
        });
        neighbors.push((src, weight));
    }
    neighbors
}

/// Estimate bends needed to reach target from direction.
pub fn estimated_bends_to_target(direction: CompassDirection, point: Point, target: Point) -> u32 {
    let dx = target.x() - point.x();
    let dy = target.y() - point.y();
    if dx.abs() < 1e-9 && dy.abs() < 1e-9 { return 0; }
    let going_toward = match direction {
        CompassDirection::East  => dx > 1e-9,
        CompassDirection::West  => dx < -1e-9,
        CompassDirection::North => dy > 1e-9,
        CompassDirection::South => dy < -1e-9,
    };
    let need_h = dx.abs() > 1e-9;
    let need_v = dy.abs() > 1e-9;
    if !need_h && !need_v { 0 }
    else if need_h && need_v { if going_toward { 1 } else { 2 } }
    else if going_toward { 0 }
    else { 2 }
}
