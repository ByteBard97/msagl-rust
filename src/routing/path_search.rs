//! Single-source single-target rectilinear path search.
//!
//! Faithful port of C# `SsstRectilinearPath.cs` (523 lines) and
//! TS `SsstRectilinearPath.ts` (~584 lines).
//!
//! Following "Orthogonal Connector Routing" by Michael Wybrow et al.
//!
//! The `PathSearch` wrapper, `SearchArena`, and utility functions are in
//! `path_search_wrapper.rs` and re-exported here.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use super::compass_direction::Direction;
use crate::geometry::point::Point;
use crate::routing::vertex_entry::VertexEntryIndex;
use crate::visibility::graph::{VertexId, VisibilityGraph};

// Re-export wrapper types and utility functions from path_search_wrapper
pub use super::path_search_wrapper::{
    estimated_bends_to_target, manhattan_distance, PathSearch, SearchArena,
};

/// Default bend penalty as a percentage of source-target Manhattan distance.
/// Matches C# `SsstRectilinearPath.DefaultBendPenaltyAsAPercentageOfDistance = 4`.
pub const DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE: f64 = 4.0;

/// Weight multiplier for overlapped visibility edges.
/// Matches C# `ScanSegment.OverlappedWeight = 100000`.
pub const OVERLAPPED_WEIGHT: f64 = 100000.0;

/// Arena entry for the path search.
///
/// Corresponds to C# `VertexEntry`. Stored in a Vec arena with index-based
/// references (approved Rust adaptation for GC pointers).
#[derive(Clone, Debug)]
pub struct SearchEntry {
    pub vertex: VertexId,
    /// Direction of entry to this vertex (None for source).
    pub direction: Direction,
    pub previous_entry: Option<usize>,
    pub length: f64,
    pub number_of_bends: i32,
    pub cost: f64,
    pub is_closed: bool,
}

/// Priority queue item.
#[derive(Clone, Debug)]
struct QueueItem {
    cost: f64,
    arena_index: usize,
}

impl PartialEq for QueueItem {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl Eq for QueueItem {}

impl PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Min-heap: reverse ordering so smallest cost comes first.
impl Ord for QueueItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

/// Neighbor slot. C# `NextNeighbor` class.
/// Slots: 0=left, 1=preferred bend (right), 2=straight-ahead.
struct NextNeighbor {
    vertex: Option<VertexId>,
    weight: f64,
}

impl NextNeighbor {
    fn new() -> Self {
        Self { vertex: None, weight: f64::NAN }
    }

    fn set(&mut self, v: VertexId, w: f64) {
        self.vertex = Some(v);
        self.weight = w;
    }
}

/// Single-source single-target rectilinear path.
/// Faithful port of C# `SsstRectilinearPath`.
pub struct SsstRectilinearPath {
    pub length_importance: f64,
    pub bends_importance: f64,
    target: Option<VertexId>,
    source: Option<VertexId>,
    entry_directions_to_target: Direction,
    upper_bound_on_cost: f64,
    source_cost_adjustment: f64,
    target_cost_adjustment: f64,
    queue: BinaryHeap<QueueItem>,
    pub(crate) arena: Vec<SearchEntry>,
    visited_vertices: Vec<VertexId>,
    /// Per-vertex per-direction arena index. Replaces C# VertexEntries[4] on vertex.
    vertex_entries: Vec<[Option<usize>; 4]>,
}

impl Default for SsstRectilinearPath {
    fn default() -> Self {
        Self::new()
    }
}

impl SsstRectilinearPath {
    pub fn new() -> Self {
        Self {
            length_importance: 1.0,
            bends_importance: 1.0,
            target: None,
            source: None,
            entry_directions_to_target: Direction::NONE,
            upper_bound_on_cost: f64::INFINITY,
            source_cost_adjustment: 0.0,
            target_cost_adjustment: 0.0,
            queue: BinaryHeap::new(),
            arena: Vec::new(),
            visited_vertices: Vec::new(),
            vertex_entries: Vec::new(),
        }
    }

    /// Access the search arena.
    pub fn arena(&self) -> &SearchArena {
        SearchArena::from_vec(&self.arena)
    }

    fn combined_cost(&self, length: f64, number_of_bends: i32) -> f64 {
        self.length_importance * length + self.bends_importance * number_of_bends as f64
    }

    fn total_cost_from_source(&self, length: f64, bends: i32) -> f64 {
        self.combined_cost(length, bends) + self.source_cost_adjustment
    }

    /// Matches C# `MultistageAdjustedCostBound`.
    pub fn multistage_adjusted_cost_bound(&self, best_cost: f64) -> f64 {
        if best_cost.is_finite() { best_cost + self.bends_importance } else { best_cost }
    }

    fn heuristic_to_target(
        &self, point: Point, entry_dir: Direction, graph: &VisibilityGraph,
    ) -> f64 {
        let tp = graph.point(self.target.unwrap());
        let dx = tp.x() - point.x();
        let dy = tp.y() - point.y();

        if dx.abs() < 1e-6 && dy.abs() < 1e-6 {
            return self.target_cost_adjustment;
        }

        let dir_to_target = Direction::from_vector(dx, dy);
        let edir = if entry_dir.is_none() { Direction::ALL } else { entry_dir };
        let bends = self.get_number_of_bends(edir, dir_to_target);
        self.combined_cost(manhattan_distance(point, tp), bends) + self.target_cost_adjustment
    }

    // -- Bend estimation (compound direction aware, matches C#) --

    fn get_number_of_bends(&self, entry_dir: Direction, dir_to_target: Direction) -> i32 {
        if dir_to_target.is_pure() {
            self.bends_for_pure(entry_dir, dir_to_target)
        } else {
            Self::bends_for_compound(dir_to_target, entry_dir, self.entry_directions_to_target)
        }
    }

    fn bends_for_pure(&self, entry_dir: Direction, dir_to_target: Direction) -> i32 {
        if (dir_to_target & entry_dir) == dir_to_target {
            if self.entry_directions_to_target.contains(dir_to_target) {
                return 0;
            }
            if self.entry_directions_to_target.contains(dir_to_target.left())
                || self.entry_directions_to_target.contains(dir_to_target.right())
            {
                return 2;
            }
            return 4;
        }
        self.bends_for_pure(entry_dir.add_one_turn(), dir_to_target) + 1
    }

    fn bends_for_compound(
        dir_to_target: Direction, entry_dir: Direction, entry_dirs: Direction,
    ) -> i32 {
        let a = dir_to_target & entry_dir;
        if a.is_none() {
            return Self::bends_for_compound(
                dir_to_target, entry_dir.add_one_turn(), entry_dirs,
            ) + 1;
        }
        let b = dir_to_target & entry_dirs;
        if b.is_none() {
            return Self::bends_for_compound(
                dir_to_target, entry_dir, entry_dirs.add_one_turn(),
            ) + 1;
        }
        if (a | b) == dir_to_target { 1 } else { 2 }
    }

    // -- Initialization --

    fn init_entry_dirs_at_target(&mut self, target: VertexId, graph: &VisibilityGraph) -> bool {
        self.entry_directions_to_target = Direction::NONE;
        let tp = graph.point(target);
        for edge in graph.out_edges(target) {
            self.entry_directions_to_target |=
                Direction::from_point_to_point(graph.point(edge.target), tp);
        }
        for &src in &graph.vertex(target).in_edges {
            self.entry_directions_to_target |= Direction::from_point_to_point(graph.point(src), tp);
        }
        !self.entry_directions_to_target.is_none()
    }

    fn ensure_ve(&mut self, count: usize) {
        if self.vertex_entries.len() < count {
            self.vertex_entries.resize(count, [None; 4]);
        }
    }

    fn get_ve(&self, v: VertexId, dir: Direction) -> Option<usize> {
        if v.0 >= self.vertex_entries.len() { return None; }
        self.vertex_entries[v.0][dir.to_index()]
    }

    fn set_ve(&mut self, v: VertexId, dir: Direction, idx: usize) {
        self.ensure_ve(v.0 + 1);
        self.vertex_entries[v.0][dir.to_index()] = Some(idx);
    }

    fn has_any_ve(&self, v: VertexId) -> bool {
        if v.0 >= self.vertex_entries.len() { return false; }
        self.vertex_entries[v.0].iter().any(|e| e.is_some())
    }

    fn init_path(
        &mut self,
        source_ve: Option<&[Option<VertexEntryIndex>; 4]>,
        source: VertexId,
        target: VertexId,
        graph: &VisibilityGraph,
    ) -> bool {
        if source == target || !self.init_entry_dirs_at_target(target, graph) {
            return false;
        }
        self.target = Some(target);
        self.source = Some(source);

        let cost = self.total_cost_from_source(0.0, 0)
            + self.heuristic_to_target(graph.point(source), Direction::NONE, graph);
        if cost >= self.upper_bound_on_cost { return false; }

        self.ensure_ve(graph.vertex_count());
        self.visited_vertices.push(source);

        if source_ve.is_none() {
            self.enqueue_initial(cost, graph);
        }
        !self.queue.is_empty()
    }

    fn enqueue_initial(&mut self, cost: f64, graph: &VisibilityGraph) {
        let src = self.source.unwrap();
        let src_idx = self.arena.len();
        self.arena.push(SearchEntry {
            vertex: src, direction: Direction::NONE, previous_entry: None,
            length: 0.0, number_of_bends: 0, cost, is_closed: true,
        });

        let mut neighbors = Vec::new();
        // C# SsstRectilinearPath.cs line 457: Source.OutEdges.Where(IsPassable)
        for edge in graph.out_edges(src) {
            if edge.check_is_passable() {
                neighbors.push((edge.target, edge.weight));
            }
        }
        // C# line 460: Source.InEdges.Where(IsPassable)
        let in_srcs: Vec<VertexId> = graph.vertex(src).in_edges.clone();
        for s in in_srcs {
            if let Some(edge) = graph.out_edges(s).find(|e| e.target == src && e.check_is_passable()) {
                neighbors.push((s, edge.weight));
            }
        }
        for (n, w) in neighbors {
            self.extend_to_neighbor(src_idx, n, w, graph);
        }
    }

    // -- Main search loop: faithful port of C# GetPathWithCost --

    #[allow(clippy::too_many_arguments)]
    pub fn get_path_with_cost(
        &mut self,
        source_ve: Option<&[Option<VertexEntryIndex>; 4]>,
        source: VertexId,
        source_cost_adj: f64,
        target_ve: Option<&mut [Option<VertexEntryIndex>; 4]>,
        target: VertexId,
        target_cost_adj: f64,
        prior_best: f64,
        graph: &VisibilityGraph,
    ) -> Option<VertexEntryIndex> {
        self.upper_bound_on_cost = prior_best;
        self.source_cost_adjustment = source_cost_adj;
        self.target_cost_adjustment = target_cost_adj;

        if !self.init_path(source_ve, source, target, graph) { return None; }

        let tid = self.target.unwrap();
        let sid = self.source.unwrap();
        let has_tve = target_ve.is_some();

        while let Some(item) = self.queue.pop() {
            let e = &self.arena[item.arena_index];
            if e.is_closed { continue; }
            if (e.cost - item.cost).abs() > 1e-12 { continue; }

            if e.vertex == tid {
                if !has_tve { self.cleanup(); return Some(VertexEntryIndex(item.arena_index)); }
                let ed = e.direction;
                if ed.is_pure() { self.entry_directions_to_target &= !ed; }
                if self.entry_directions_to_target.is_none() { self.cleanup(); return None; }
                let ec = e.cost;
                self.upper_bound_on_cost =
                    self.multistage_adjusted_cost_bound(ec).min(self.upper_bound_on_cost);
                continue;
            }

            self.arena[item.arena_index].is_closed = true;
            let edir = self.arena[item.arena_index].direction;
            let pbd = if edir.is_pure() { edir.right() } else { Direction::NONE };

            let mut nn = [NextNeighbor::new(), NextNeighbor::new(), NextNeighbor::new()];
            let mut edges: Vec<(VertexId, f64)> = Vec::new();

            let bv = self.arena[item.arena_index].vertex;
            // C# SsstRectilinearPath.cs line 424: IsPassable filter on edges
            for &s in &graph.vertex(bv).in_edges {
                if let Some(edge) = graph.out_edges(s).find(|e| e.target == bv && e.check_is_passable()) {
                    edges.push((s, edge.weight));
                }
            }
            for oe in graph.out_edges(bv) {
                if oe.check_is_passable() {
                    edges.push((oe.target, oe.weight));
                }
            }

            let bp = graph.point(bv);
            let pv = self.arena[item.arena_index].previous_entry.map(|i| self.arena[i].vertex);

            for (nv, w) in &edges {
                if Some(*nv) == pv {
                    if graph.degree(bv) > 1 || bv != sid { continue; }
                    nn[2].set(*nv, *w);
                    continue;
                }
                let nd = Direction::pure_from_point_to_point(bp, graph.point(*nv));
                if nd == edir { nn[2].set(*nv, *w); }
                else if nd == pbd { nn[1].set(*nv, *w); }
                else { nn[0].set(*nv, *w); }
            }

            for slot in &nn {
                if let Some(v) = slot.vertex {
                    self.extend_to_neighbor(item.arena_index, v, slot.weight, graph);
                }
            }
        }
        self.cleanup();
        None
    }

    fn extend_to_neighbor(&mut self, bi: usize, nv: VertexId, w: f64, g: &VisibilityGraph) {
        let bp = g.point(self.arena[bi].vertex);
        let dir = Direction::pure_from_point_to_point(bp, g.point(nv));

        if let Some(ei) = self.get_ve(nv, dir) {
            if !self.arena[ei].is_closed { self.update_if_needed(bi, ei, w, g); }
            return;
        }
        if self.try_reversed(bi, nv, w, g) { return; }
        self.create_enqueue(bi, nv, w, g);
    }

    fn update_if_needed(&mut self, bi: usize, ni: usize, w: f64, g: &VisibilityGraph) {
        let nv = self.arena[ni].vertex;
        let (dir, len, bends) = self.len_bends(bi, nv, w, g);
        let old = self.combined_cost(self.arena[ni].length, self.arena[ni].number_of_bends);
        if self.combined_cost(len, bends) < old {
            let total = self.total_cost_from_source(len, bends)
                + self.heuristic_to_target(g.point(nv), dir, g);
            self.arena[ni].previous_entry = Some(bi);
            self.arena[ni].length = len;
            self.arena[ni].number_of_bends = bends;
            self.arena[ni].cost = total;
            self.queue.push(QueueItem { cost: total, arena_index: ni });
        }
    }

    fn try_reversed(&mut self, bi: usize, nv: VertexId, w: f64, g: &VisibilityGraph) -> bool {
        let bv = self.arena[bi].vertex;
        if !self.has_any_ve(bv) { return false; }
        let df = Direction::pure_from_point_to_point(g.point(nv), g.point(bv));
        if let Some(efi) = self.get_ve(bv, df) {
            let pv = self.arena[efi].previous_entry.map(|i| self.arena[i].vertex);
            if pv == Some(nv) {
                self.queue_reversed(bi, efi, w, g);
                return true;
            }
        }
        false
    }

    fn queue_reversed(&mut self, bi: usize, efi: usize, w: f64, g: &VisibilityGraph) {
        let pv = self.arena[efi].previous_entry.map(|i| self.arena[i].vertex).unwrap();
        let (dir, len, bends) = self.len_bends(bi, pv, w, g);
        let old = self.combined_cost(self.arena[efi].length, self.arena[efi].number_of_bends);
        if self.combined_cost(len, bends) < old || g.degree(self.arena[bi].vertex) == 1 {
            let cost = self.total_cost_from_source(len, bends)
                + self.heuristic_to_target(g.point(pv), dir, g);
            self.do_enqueue(bi, pv, dir, len, bends, cost);
        }
    }

    fn create_enqueue(&mut self, bi: usize, nv: VertexId, w: f64, g: &VisibilityGraph) {
        let (dir, len, bends) = self.len_bends(bi, nv, w, g);
        let cost = self.total_cost_from_source(len, bends)
            + self.heuristic_to_target(g.point(nv), dir, g);
        if cost < self.upper_bound_on_cost {
            if !self.has_any_ve(nv) { self.visited_vertices.push(nv); }
            self.do_enqueue(bi, nv, dir, len, bends, cost);
        }
    }

    fn do_enqueue(&mut self, bi: usize, nv: VertexId, d: Direction, l: f64, b: i32, c: f64) {
        let ai = self.arena.len();
        self.arena.push(SearchEntry {
            vertex: nv, direction: d, previous_entry: Some(bi),
            length: l, number_of_bends: b, cost: c, is_closed: false,
        });
        self.set_ve(nv, d, ai);
        self.queue.push(QueueItem { cost: c, arena_index: ai });
    }

    /// Matches C# `GetLengthAndNumberOfBendsToNeighborVertex`.
    fn len_bends(&self, pi: usize, v: VertexId, w: f64, g: &VisibilityGraph) -> (Direction, f64, i32) {
        let p = &self.arena[pi];
        let pp = g.point(p.vertex);
        let vp = g.point(v);
        let len = p.length + manhattan_distance(pp, vp) * w;
        let dir = Direction::pure_from_point_to_point(pp, vp);
        let mut bends = p.number_of_bends;
        if !p.direction.is_none() && dir != p.direction { bends += 1; }
        (dir, len, bends)
    }

    fn cleanup(&mut self) {
        self.visited_vertices.clear();
        self.queue.clear();
    }

    /// Restore path from arena, collapsing collinear points.
    /// Matches C# `RestorePath`.
    pub fn restore_path_v(
        arena: &SearchArena, entry_idx: Option<VertexEntryIndex>, graph: &VisibilityGraph,
    ) -> Vec<Point> {
        let idx = match entry_idx { Some(i) => i.0, None => return Vec::new() };
        let entries = arena.entries();
        let mut list = Vec::new();
        #[allow(unused_assignments)]
        let mut skipped = false;
        let mut last_dir = Direction::NONE;
        let mut ci = idx;
        loop {
            let e = &entries[ci];
            if last_dir == e.direction { skipped = true; }
            else { skipped = false; list.push(graph.point(e.vertex)); last_dir = e.direction; }
            match e.previous_entry { Some(prev) => ci = prev, None => break }
        }
        if skipped { list.push(graph.point(entries[ci].vertex)); }
        list.reverse();
        list
    }
}
