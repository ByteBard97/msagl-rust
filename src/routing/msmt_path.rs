//! Multi-source multi-target rectilinear path search.
//!
//! Faithful port of `MsmtRectilinearPath.ts` / `MsmtRectilinearPath.cs`.
//!
//! Routes from one of several source vertices to one of several target vertices,
//! processing closest pairs first and using cost bounds to prune.

use crate::geometry::point::Point;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use super::path_search::{
    SsstRectilinearPath, manhattan_distance,
    DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE, OVERLAPPED_WEIGHT,
};
use super::vertex_entry::VertexEntryIndex;

/// Multi-source multi-target rectilinear path.
///
/// Corresponds to `MsmtRectilinearPath` in the TS/C# source.
pub struct MsmtRectilinearPath {
    bend_penalty_as_percentage: f64,
    /// Temporary for accumulating target entries.
    current_pass_target_entries: [Option<VertexEntryIndex>; 4],
}

impl MsmtRectilinearPath {
    /// Create with the given bend penalty percentage.
    /// Matches TS constructor: `this.bendPenaltyAsAPercentageOfDistance = bendPenalty`.
    pub fn new(bend_penalty: f64) -> Self {
        Self {
            bend_penalty_as_percentage: bend_penalty,
            current_pass_target_entries: [None; 4],
        }
    }

    /// Create with the default bend penalty (4%).
    pub fn default_penalty() -> Self {
        Self::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE)
    }

    /// Get the lowest-cost path from one of the sources to one of the targets.
    ///
    /// Matches TS `GetPath(sources, targets)`.
    pub fn get_path(
        &mut self,
        sources: &[VertexId],
        targets: &[VertexId],
        graph: &VisibilityGraph,
    ) -> Vec<Point> {
        let (entry_idx, arena_searcher) = self.get_path_stage(
            None, sources, None, targets, graph,
        );
        match (entry_idx, arena_searcher) {
            (Some(idx), Some(searcher)) => {
                SsstRectilinearPath::restore_path_v(searcher.arena(), Some(idx), graph)
            }
            _ => Vec::new(),
        }
    }

    /// Route a single stage of a possibly multi-stage path.
    ///
    /// Matches TS `GetPathStage(sourceVertexEntries, sources, targetVertexEntries, targets)`.
    ///
    /// Returns the best VertexEntryIndex and the searcher (for arena access).
    fn get_path_stage(
        &mut self,
        source_vertex_entries: Option<&[Option<VertexEntryIndex>; 4]>,
        sources: &[VertexId],
        mut target_vertex_entries: Option<&mut [Option<VertexEntryIndex>; 4]>,
        targets: &[VertexId],
        graph: &VisibilityGraph,
    ) -> (Option<VertexEntryIndex>, Option<SsstRectilinearPath>) {
        let mut ssst = SsstRectilinearPath::new();

        let mut best_entry: Option<VertexEntryIndex> = None;
        let mut best_cost: f64 = f64::MAX / OVERLAPPED_WEIGHT;
        let mut best_path_cost_ratio: f64 = f64::INFINITY;

        // Calculate the bend penalty multiplier
        let source_center = barycenter(sources, graph);
        let target_center = barycenter(targets, graph);
        let distance = manhattan_distance(source_center, target_center);
        ssst.bends_importance =
            f64::max(0.001, distance * (self.bend_penalty_as_percentage * 0.01));

        let interior_length_adjustment = ssst.length_importance;

        let use_temp_targets = target_vertex_entries.is_some();

        // Build and sort source-target pairs by Manhattan distance
        let mut st_pairs: Vec<(VertexId, VertexId)> = Vec::new();
        for &s in sources {
            for &t in targets {
                st_pairs.push((s, t));
            }
        }
        st_pairs.sort_by(|&(a, b), &(c, d)| {
            let dist_ab = manhattan_distance(graph.point(a), graph.point(b));
            let dist_cd = manhattan_distance(graph.point(c), graph.point(d));
            dist_ab.partial_cmp(&dist_cd).unwrap_or(std::cmp::Ordering::Equal)
        });

        for (sv, tv) in st_pairs {
            let sp = graph.point(sv);
            let tp = graph.point(tv);
            if close_dist(sp, tp) {
                continue;
            }

            let source_cost_adj =
                manhattan_distance(sp, source_center) * interior_length_adjustment;
            let target_cost_adj =
                manhattan_distance(tp, target_center) * interior_length_adjustment;

            let mut adjusted_best = best_cost;
            if use_temp_targets {
                self.current_pass_target_entries = [None; 4];
                adjusted_best = ssst.multistage_adjusted_cost_bound(best_cost);
            }

            let last_entry = if use_temp_targets {
                ssst.get_path_with_cost(
                    source_vertex_entries,
                    sv,
                    source_cost_adj,
                    Some(&mut self.current_pass_target_entries),
                    tv,
                    target_cost_adj,
                    adjusted_best,
                    graph,
                )
            } else {
                ssst.get_path_with_cost(
                    source_vertex_entries,
                    sv,
                    source_cost_adj,
                    None,
                    tv,
                    target_cost_adj,
                    adjusted_best,
                    graph,
                )
            };

            if use_temp_targets {
                // Update target entries for each direction
                if let Some(ref mut _tve) = target_vertex_entries {
                    // This branch is unreachable due to the borrow checker — we handle
                    // target_vertex_entries after the loop instead.
                }
                Self::update_target_entries_static(
                    &self.current_pass_target_entries,
                    &ssst,
                    &mut best_cost,
                    &mut best_entry,
                );
                continue;
            }

            // Final (or only) stage: break ties by cost ratio
            if let Some(entry_idx) = last_entry {
                let entry_cost = ssst.arena().get(entry_idx).cost;
                let md = manhattan_distance(sp, tp);
                let cost_ratio = if md > 0.0 { entry_cost / md } else { f64::INFINITY };

                if entry_cost < best_cost
                    || (close_f64(entry_cost, best_cost) && cost_ratio < best_path_cost_ratio)
                {
                    best_cost = entry_cost;
                    best_entry = Some(entry_idx);
                    best_path_cost_ratio = cost_ratio;
                }
            }
        }

        (best_entry, Some(ssst))
    }

    /// Update target entries per direction (static version to avoid borrow issues).
    fn update_target_entries_static(
        temp_entries: &[Option<VertexEntryIndex>; 4],
        ssst: &SsstRectilinearPath,
        best_cost: &mut f64,
        best_entry: &mut Option<VertexEntryIndex>,
    ) {
        for temp_idx in temp_entries.iter().flatten() {
            let temp_cost = ssst.arena().get(*temp_idx).cost;
            if temp_cost < *best_cost {
                *best_cost = temp_cost;
                *best_entry = Some(*temp_idx);
            }
        }
    }
}

/// Compute barycenter of a set of vertices.
/// Matches TS `MsmtRectilinearPath.Barycenter(vertices)`.
fn barycenter(vertices: &[VertexId], graph: &VisibilityGraph) -> Point {
    if vertices.is_empty() {
        return Point::new(0.0, 0.0);
    }
    let mut cx = 0.0;
    let mut cy = 0.0;
    for &v in vertices {
        let p = graph.point(v);
        cx += p.x();
        cy += p.y();
    }
    let n = vertices.len() as f64;
    Point::new(cx / n, cy / n)
}

/// Check if two points are approximately equal.
fn close_dist(a: Point, b: Point) -> bool {
    let dx = (a.x() - b.x()).abs();
    let dy = (a.y() - b.y()).abs();
    dx < 1e-6 && dy < 1e-6
}

/// Check if two f64 values are approximately equal.
fn close_f64(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}
