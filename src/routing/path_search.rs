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
use super::vertex_entry::{VertexEntry, VertexEntryIndex};
use crate::geometry::point::Point;
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
///
/// Faithful port of C# `SsstRectilinearPath`.
///
/// **Direction-aware vertex entries:** During search, each visited vertex stores
/// up to four `VertexEntry` records (one per compass direction) in the graph's
/// `vertex_entries[4]` field. These index into this searcher's `arena` Vec.
/// On cleanup, all visited vertices have their entries cleared via
/// `graph.clear_vertex_entries_for()`. This matches C# `Cleanup()` which calls
/// `v.RemoveVertexEntries()` on each visited vertex.
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
    /// Arena of all VertexEntry records created during this search.
    /// Matches C# `VertexEntry` objects kept alive by the priority queue and vertex references.
    pub(crate) arena: Vec<VertexEntry>,
    /// Vertices visited during this search, for cleanup.
    /// Matches C# `visitedVertices: List<VisibilityVertexRectilinear>`.
    visited_vertices: Vec<VertexId>,
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

    /// Get the arena index of the VertexEntry for vertex `v` from direction `dir`.
    ///
    /// Reads from the graph's `vertex_entries` field.
    /// Matches C# `neigVer.VertexEntries[CompassVector.ToIndex(dirToNeighbor)]`.
    fn get_ve(&self, v: VertexId, dir: Direction, graph: &VisibilityGraph) -> Option<usize> {
        if !dir.is_pure() { return None; }
        graph.vertex_entry(v, dir.to_compass()).map(|idx| idx.0)
    }

    /// Store arena index for vertex `v` from direction `dir` in the graph.
    ///
    /// Matches C# `neigVer.SetVertexEntry(entry)`.
    fn set_ve(&mut self, v: VertexId, dir: Direction, arena_idx: usize, graph: &mut VisibilityGraph) {
        if !dir.is_pure() { return; }
        graph.set_vertex_entry(v, dir.to_compass(), Some(VertexEntryIndex(arena_idx)));
    }

    /// Check if vertex `v` has any vertex entries set in the graph.
    ///
    /// Matches C# `neigVer.VertexEntries != null`.
    fn has_any_ve(&self, v: VertexId, graph: &VisibilityGraph) -> bool {
        let vd = graph.vertex(v);
        vd.vertex_entries.iter().any(|e| e.is_some())
    }

    fn init_path(
        &mut self,
        source_ve: Option<&[Option<VertexEntryIndex>; 4]>,
        source: VertexId,
        target: VertexId,
        graph: &mut VisibilityGraph,
    ) -> bool {
        if source == target || !self.init_entry_dirs_at_target(target, graph) {
            return false;
        }
        self.target = Some(target);
        self.source = Some(source);

        let cost = self.total_cost_from_source(0.0, 0)
            + self.heuristic_to_target(graph.point(source), Direction::NONE, graph);
        if cost >= self.upper_bound_on_cost { return false; }

        self.visited_vertices.push(source);

        if source_ve.is_none() {
            self.enqueue_initial(cost, graph);
        } else if let Some(sves) = source_ve {
            self.enqueue_from_source_entries(sves);
        }
        !self.queue.is_empty()
    }

    /// Enqueue initial neighbors from source when no source entries are provided.
    ///
    /// Matches C# `EnqueueInitialVerticesFromSource`.
    fn enqueue_initial(&mut self, cost: f64, graph: &mut VisibilityGraph) {
        let src = self.source.unwrap();
        let src_idx = self.arena.len();
        self.arena.push(VertexEntry::new(
            src, Direction::NONE, None, 0.0, 0, cost,
        ));
        // Mark as closed (source entry is immediately closed).
        self.arena[src_idx].is_closed = true;

        let mut neighbors: Vec<(VertexId, f64)> = Vec::new();
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

    /// Enqueue from existing source vertex entries (multi-stage paths).
    ///
    /// Matches C# `EnqueueInitialVerticesFromSourceEntries`.
    fn enqueue_from_source_entries(&mut self, source_entries: &[Option<VertexEntryIndex>; 4]) {
        // In multi-stage paths, source entries were produced by a prior stage.
        // They are already in this searcher's arena (indices reference self.arena).
        for slot in source_entries.iter().flatten() {
            let ai = slot.0;
            if ai < self.arena.len() {
                let c = self.arena[ai].cost;
                self.queue.push(QueueItem { cost: c, arena_index: ai });
            }
        }
    }

    // -- Main search loop: faithful port of C# GetPathWithCost --

    #[allow(clippy::too_many_arguments)]
    pub fn get_path_with_cost(
        &mut self,
        source_ve: Option<&[Option<VertexEntryIndex>; 4]>,
        source: VertexId,
        source_cost_adj: f64,
        mut target_ve: Option<&mut [Option<VertexEntryIndex>; 4]>,
        target: VertexId,
        target_cost_adj: f64,
        prior_best: f64,
        graph: &mut VisibilityGraph,
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
                if !has_tve {
                    self.cleanup(graph);
                    return Some(VertexEntryIndex(item.arena_index));
                }
                // Multi-stage: remove this direction from valid target directions.
                // Matches C# line 370: EntryDirectionsToTarget &= ~bestEntry.Direction
                let ed = e.direction;
                if ed.is_pure() { self.entry_directions_to_target &= !ed; }
                if self.entry_directions_to_target.is_none() {
                    // C# line 373: copy target vertex entries before cleanup so
                    // the caller (MsmtRectilinearPath) can use them as source
                    // entries for the next stage.
                    if let Some(tve) = target_ve.as_deref_mut() {
                        let vd = graph.vertex(tid);
                        for i in 0..4 {
                            if tve[i].is_none() {
                                tve[i] = vd.vertex_entries[i];
                            } else if let Some(existing) = vd.vertex_entries[i] {
                                let existing_cost = self.arena[existing.0].cost;
                                if let Some(cur) = tve[i] {
                                    if existing_cost < self.arena[cur.0].cost {
                                        tve[i] = Some(existing);
                                    }
                                }
                            }
                        }
                    }
                    self.cleanup(graph);
                    return None;
                }
                let ec = e.cost;
                self.upper_bound_on_cost =
                    self.multistage_adjusted_cost_bound(ec).min(self.upper_bound_on_cost);
                continue;
            }

            // Close this entry — it has the lowest cost so far from this direction.
            // Matches C# `bestEntry.IsClosed = true`.
            self.arena[item.arena_index].is_closed = true;
            let edir = self.arena[item.arena_index].direction;
            // Preferred bend direction: right of the current travel direction.
            // Matches C# `preferredBendDir = Right(bestEntry.Direction)`.
            let pbd = if edir.is_pure() { edir.right() } else { Direction::NONE };

            let mut nn = [NextNeighbor::new(), NextNeighbor::new(), NextNeighbor::new()];
            let mut edges: Vec<(VertexId, f64)> = Vec::new();

            let bv = self.arena[item.arena_index].vertex;
            // Collect passable neighbors (in-edges + out-edges).
            // Matches C# ExtendPathAlongInEdges + ExtendPathAlongOutEdges.
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
            let pv = self.arena[item.arena_index].previous_entry.map(|i| self.arena[i.0].vertex);

            // Sort neighbors into slots: 0=left, 1=preferred (right), 2=straight.
            // Matches C# ExtendPathAlongEdge slot assignment.
            for (nv, w) in &edges {
                if Some(*nv) == pv {
                    // Going back to previous vertex: only allowed for degree-1 source.
                    if graph.degree(bv) > 1 || bv != sid { continue; }
                    nn[2].set(*nv, *w);
                    continue;
                }
                let nd = Direction::pure_from_point_to_point(bp, graph.point(*nv));
                if nd == edir { nn[2].set(*nv, *w); }
                else if nd == pbd { nn[1].set(*nv, *w); }
                else { nn[0].set(*nv, *w); }
            }

            // Process neighbor slots in order 0, 1, 2 (left, right, straight).
            // Matches C# foreach on nextNeighbors after ExtendPathAlongEdge calls.
            for slot in &nn {
                if let Some(v) = slot.vertex {
                    self.extend_to_neighbor(item.arena_index, v, slot.weight, graph);
                }
            }
        }

        // Search exhausted. If multi-stage, copy collected target entries to caller.
        // Matches C# lines 402-404: target.VertexEntries.CopyTo(targetVertexEntries, 0)
        if has_tve {
            if let Some(tve) = target_ve {
                let tid_local = tid;
                let vd = graph.vertex(tid_local);
                for i in 0..4 {
                    if tve[i].is_none() {
                        tve[i] = vd.vertex_entries[i];
                    } else if let Some(existing) = vd.vertex_entries[i] {
                        let existing_cost = self.arena[existing.0].cost;
                        if let Some(cur) = tve[i] {
                            if existing_cost < self.arena[cur.0].cost {
                                tve[i] = Some(existing);
                            }
                        }
                    }
                }
            }
        }

        self.cleanup(graph);
        None
    }

    /// Extend path from arena entry `bi` to neighbor vertex `nv`.
    ///
    /// Faithful port of C# `ExtendPathToNeighborVertex`.
    fn extend_to_neighbor(&mut self, bi: usize, nv: VertexId, w: f64, g: &mut VisibilityGraph) {
        let bp = g.point(self.arena[bi].vertex);
        let dir = Direction::pure_from_point_to_point(bp, g.point(nv));

        // Check if there is already a vertex entry for `nv` from `dir`.
        // Matches C# `neigEntry = neigVer.VertexEntries[CompassVector.ToIndex(dirToNeighbor)]`.
        if let Some(ei) = self.get_ve(nv, dir, g) {
            if !self.arena[ei].is_closed {
                self.update_if_needed(bi, ei, w, g);
            }
            return;
        }
        // No entry yet — check for a reversed entry situation.
        // Matches C# `CreateAndEnqueueReversedEntryToNeighborVertex`.
        if self.try_reversed(bi, nv, w, g) { return; }
        // Create a fresh entry and enqueue.
        // Matches C# `CreateAndEnqueueEntryToNeighborVertex`.
        self.create_enqueue(bi, nv, w, g);
    }

    /// Update an existing open entry if the new path is cheaper.
    ///
    /// Matches C# `UpdateEntryToNeighborVertexIfNeeded`.
    /// C# calls `neigEntry.ResetEntry(bestEntry, length, numberOfBends, newCost)`
    /// then `queue.DecreasePriority(neigEntry, newCost)`. Rust has no decrease-key
    /// on BinaryHeap, so we push a new QueueItem and rely on the `is_closed` check
    /// at dequeue time to discard the stale entry (lazy deletion).
    fn update_if_needed(&mut self, bi: usize, ni: usize, w: f64, g: &mut VisibilityGraph) {
        let nv = self.arena[ni].vertex;
        let (dir, len, bends) = self.len_bends(bi, nv, w, g);
        let old = self.combined_cost(self.arena[ni].length, self.arena[ni].number_of_bends);
        if self.combined_cost(len, bends) < old {
            let total = self.total_cost_from_source(len, bends)
                + self.heuristic_to_target(g.point(nv), dir, g);
            // Mirrors C# ResetEntry: update all four mutable fields in one call.
            self.arena[ni].reset_entry(Some(VertexEntryIndex(bi)), len, bends, total);
            // Push duplicate; stale copy discarded at dequeue via is_closed check.
            self.queue.push(QueueItem { cost: total, arena_index: ni });
        }
    }

    /// Handle the reversed-entry case.
    ///
    /// Matches C# `CreateAndEnqueueReversedEntryToNeighborVertex`. Returns true
    /// if a reversed entry was found and handled; false otherwise.
    fn try_reversed(&mut self, bi: usize, nv: VertexId, w: f64, g: &mut VisibilityGraph) -> bool {
        let bv = self.arena[bi].vertex;
        if !self.has_any_ve(bv, g) { return false; }
        let df = Direction::pure_from_point_to_point(g.point(nv), g.point(bv));
        if let Some(efi) = self.get_ve(bv, df, g) {
            let pv = self.arena[efi].previous_entry.map(|i| self.arena[i.0].vertex);
            if pv == Some(nv) {
                self.queue_reversed(bi, efi, w, g);
                return true;
            }
        }
        false
    }

    /// Enqueue a reversed path entry.
    ///
    /// Matches C# `QueueReversedEntryToNeighborVertexIfNeeded`.
    fn queue_reversed(&mut self, bi: usize, efi: usize, w: f64, g: &mut VisibilityGraph) {
        let pv = self.arena[efi].previous_entry.map(|i| self.arena[i.0].vertex).unwrap();
        let (dir, len, bends) = self.len_bends(bi, pv, w, g);
        let old = self.combined_cost(self.arena[efi].length, self.arena[efi].number_of_bends);
        if self.combined_cost(len, bends) < old || g.degree(self.arena[bi].vertex) == 1 {
            let cost = self.total_cost_from_source(len, bends)
                + self.heuristic_to_target(g.point(pv), dir, g);
            self.do_enqueue(bi, pv, dir, len, bends, cost, g);
        }
    }

    /// Create a new entry and enqueue it.
    ///
    /// Matches C# `CreateAndEnqueueEntryToNeighborVertex`.
    fn create_enqueue(&mut self, bi: usize, nv: VertexId, w: f64, g: &mut VisibilityGraph) {
        let (dir, len, bends) = self.len_bends(bi, nv, w, g);
        let cost = self.total_cost_from_source(len, bends)
            + self.heuristic_to_target(g.point(nv), dir, g);
        if cost < self.upper_bound_on_cost {
            if !self.has_any_ve(nv, g) { self.visited_vertices.push(nv); }
            self.do_enqueue(bi, nv, dir, len, bends, cost, g);
        }
    }

    /// Push a new `VertexEntry` into the arena, record it on the graph vertex,
    /// and add it to the priority queue.
    ///
    /// Matches C# `EnqueueEntry`.
    fn do_enqueue(
        &mut self, bi: usize, nv: VertexId, d: Direction, l: f64, b: i32, c: f64,
        g: &mut VisibilityGraph,
    ) {
        let ai = self.arena.len();
        self.arena.push(VertexEntry::new(nv, d, Some(VertexEntryIndex(bi)), l, b, c));
        // Write the arena index into the graph vertex's directional slot.
        // Matches C# `neigVer.SetVertexEntry(entry)`.
        self.set_ve(nv, d, ai, g);
        self.queue.push(QueueItem { cost: c, arena_index: ai });
    }

    /// Compute direction, cumulative length, and bend count to a neighbor.
    ///
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

    /// Clear vertex entries for all visited vertices and reset search state.
    ///
    /// Matches C# `Cleanup()`: iterates `visitedVertices` and calls
    /// `v.RemoveVertexEntries()` on each, then clears the queue.
    fn cleanup(&mut self, graph: &mut VisibilityGraph) {
        for v in &self.visited_vertices {
            graph.set_vertex_entry(*v, crate::routing::compass_direction::CompassDirection::North, None);
            graph.set_vertex_entry(*v, crate::routing::compass_direction::CompassDirection::East, None);
            graph.set_vertex_entry(*v, crate::routing::compass_direction::CompassDirection::South, None);
            graph.set_vertex_entry(*v, crate::routing::compass_direction::CompassDirection::West, None);
        }
        self.visited_vertices.clear();
        self.queue.clear();
    }

    /// Restore path from arena, collapsing collinear points.
    ///
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
            match e.previous_entry { Some(prev) => ci = prev.0, None => break }
        }
        if skipped { list.push(graph.point(entries[ci].vertex)); }
        list.reverse();
        list
    }
}
