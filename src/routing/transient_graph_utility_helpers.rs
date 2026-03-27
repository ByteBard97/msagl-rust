//! Helper methods for `TransientGraphUtility`: splice source/target traversal,
//! group boundary crossing splicing, and debug verification.
//!
//! Split from `transient_graph_utility_extend.rs` to keep files under 500 lines.
//! All methods are `impl TransientGraphUtility`.

use super::compass_direction::{CompassDirection, Direction};
use super::group_boundary_crossing::{PointAndCrossings, PointAndCrossingsList, BOUNDARY_WIDTH};
use super::obstacle_tree::ObstacleTree;
use super::static_graph_utility::StaticGraphUtility;
use super::transient_graph_utility::TransientGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use std::sync::Arc;

/// Weight for normal scan segments (matches C# ScanSegment.NormalWeight = 1).
const NORMAL_WEIGHT: f64 = 1.0;

/// Weight for overlapped scan segments (matches C# ScanSegment.OverlappedWeight = 100000).
#[allow(dead_code)]
const OVERLAPPED_WEIGHT: f64 = 100_000.0;

/// Weight for reflection scan segments (matches C# ScanSegment.ReflectionWeight = 5).
const REFLECTION_WEIGHT: f64 = 5.0;

/// Width of a group boundary crossing edge.
///
/// Matches C# `GroupBoundaryCrossing.BoundaryWidth = ApproximateComparer.DistanceEpsilon`.
const GROUP_BOUNDARY_WIDTH: f64 = BOUNDARY_WIDTH;

/// Compare two points in the ascending ordering used by PointComparer.Compare.
/// This is X-primary, Y-secondary, matching C# PointComparer.Compare.
fn compare_points(a: Point, b: Point) -> std::cmp::Ordering {
    let x_cmp = GeomConstants::compare(a.x(), b.x());
    if x_cmp != std::cmp::Ordering::Equal {
        return x_cmp;
    }
    GeomConstants::compare(a.y(), b.y())
}

impl TransientGraphUtility {
    // ── C# line 708-729: GetNextSpliceSource ───────────────────────────────────
    pub(super) fn get_next_splice_source(
        graph: &VisibilityGraph,
        splice_source: VertexId,
        splice_target_dir: CompassDirection,
        extend_dir: CompassDirection,
    ) -> Option<VertexId> {
        // VisibilityVertex nextSpliceSource = StaticGraphUtility.FindAdjacentVertex(spliceSource, extendDir);
        let next_splice_source =
            StaticGraphUtility::find_adjacent_vertex(graph, splice_source, extend_dir);

        // if (nextSpliceSource == null)
        if let Some(nss) = next_splice_source {
            // spliceSource = nextSpliceSource;
            return Some(nss);
        }

        // nextSpliceSource = spliceSource;
        let mut next_splice_source = splice_source;

        // for (;;)
        loop {
            // nextSpliceSource = StaticGraphUtility.FindAdjacentVertex(nextSpliceSource, CompassVector.OppositeDir(spliceTargetDir));
            let adj = StaticGraphUtility::find_adjacent_vertex(
                graph,
                next_splice_source,
                splice_target_dir.opposite(),
            );

            // if (nextSpliceSource == null) { return false; }
            match adj {
                None => return None,
                Some(v) => next_splice_source = v,
            }

            // var nextSpliceSourceExtend = StaticGraphUtility.FindAdjacentVertex(nextSpliceSource, extendDir);
            let next_splice_source_extend =
                StaticGraphUtility::find_adjacent_vertex(graph, next_splice_source, extend_dir);

            // if (nextSpliceSourceExtend != null) { nextSpliceSource = nextSpliceSourceExtend; break; }
            if let Some(nsse) = next_splice_source_extend {
                return Some(nsse);
            }
        }
    }

    // ── C# line 731-753: GetSpliceTarget ───────────────────────────────────────
    pub(super) fn get_splice_target(
        graph: &VisibilityGraph,
        splice_source: VertexId,
        splice_target_dir: CompassDirection,
        next_extend_point: Point,
    ) -> (VertexId, Option<VertexId>) {
        let mut splice_source = splice_source;

        // Direction prevDir = PointComparer.GetPureDirection(spliceSource.Point, nextExtendPoint);
        let prev_dir =
            CompassDirection::from_points(graph.point(splice_source), next_extend_point);

        // Direction nextDir = prevDir;
        let mut next_dir = prev_dir;

        // var spliceTarget = spliceSource;
        let mut splice_target = splice_source;

        // while (nextDir == prevDir)
        while next_dir == prev_dir {
            // spliceSource = spliceTarget;
            splice_source = splice_target;

            // spliceTarget = StaticGraphUtility.FindAdjacentVertex(spliceSource, spliceTargetDir);
            let st = StaticGraphUtility::find_adjacent_vertex(
                graph,
                splice_source,
                splice_target_dir,
            );

            // if (spliceTarget == null) { break; }
            match st {
                None => return (splice_source, None),
                Some(t) => splice_target = t,
            }

            // if (PointComparer.Equal(spliceTarget.Point, nextExtendPoint))
            if graph.point(splice_target).close_to(next_extend_point) {
                // spliceTarget = StaticGraphUtility.FindAdjacentVertex(spliceTarget, spliceTargetDir);
                let beyond = StaticGraphUtility::find_adjacent_vertex(
                    graph,
                    splice_target,
                    splice_target_dir,
                );
                return (splice_source, beyond);
            }

            // nextDir = PointComparer.GetPureDirection(spliceTarget.Point, nextExtendPoint);
            next_dir =
                CompassDirection::from_points(graph.point(splice_target), next_extend_point);
        }

        (splice_source, Some(splice_target))
    }

    // ── C# line 495-541: SpliceGroupBoundaryCrossings ──────────────────────────
    pub(super) fn splice_group_boundary_crossings(
        &mut self,
        graph: &mut VisibilityGraph,
        obstacle_tree: &ObstacleTree,
        crossing_list: Option<&mut PointAndCrossingsList>,
        start_vertex: VertexId,
        max_segment_start: Point,
        max_segment_end: Point,
    ) {
        // if ((crossingList == null) || (0 == crossingList.Count)) { return; }
        let crossing_list = match crossing_list {
            None => return,
            Some(cl) => {
                if cl.count() == 0 {
                    return;
                }
                cl
            }
        };

        // crossingList.Reset();
        crossing_list.reset();

        // var start = maxSegment.Start;
        let mut start = max_segment_start;
        // var end = maxSegment.End;
        let mut end = max_segment_end;
        // var dir = PointComparer.GetPureDirection(start, end);
        let mut dir = match CompassDirection::from_points(start, end) {
            Some(d) => d,
            None => return,
        };

        // if (!StaticGraphUtility.IsAscending(dir))
        if !StaticGraphUtility::is_ascending(dir) {
            // start = maxSegment.End;
            // end = maxSegment.Start;
            std::mem::swap(&mut start, &mut end);
            // dir = CompassVector.OppositeDir(dir);
            dir = dir.opposite();
        }

        // startVertex = TraverseToFirstVertexAtOrAbove(startVertex, start, CompassVector.OppositeDir(dir));
        let current_start_vertex =
            Self::traverse_to_first_vertex_at_or_above(graph, start_vertex, start, dir.opposite());

        // for (var currentVertex = startVertex; currentVertex != null; currentVertex = StaticGraphUtility.FindAdjacentVertex(currentVertex, dir))
        let mut current_vertex_opt = Some(current_start_vertex);
        while let Some(current_vertex) = current_vertex_opt {
            let current_point = graph.point(current_vertex);

            // bool isFinalVertex = (PointComparer.Compare(currentVertex.Point, end) >= 0);
            let is_final_vertex = compare_points(current_point, end) != std::cmp::Ordering::Less;

            // while (crossingList.CurrentIsBeforeOrAt(currentVertex.Point))
            while crossing_list.current_is_before_or_at(current_point) {
                // PointAndCrossings pac = crossingList.Pop();
                let pac = crossing_list.pop();

                let start_vertex_point = graph.point(current_start_vertex);

                // if (PointComparer.Compare(pac.Location, startVertex.Point) > 0)
                if compare_points(pac.location, start_vertex_point) == std::cmp::Ordering::Greater
                {
                    // if (PointComparer.Compare(pac.Location, end) <= 0)
                    if compare_points(pac.location, end) != std::cmp::Ordering::Greater {
                        // SpliceGroupBoundaryCrossing(currentVertex, pac, CompassVector.OppositeDir(dir));
                        self.splice_group_boundary_crossing(
                            graph,
                            obstacle_tree,
                            current_vertex,
                            &pac,
                            dir.opposite(),
                        );
                    }
                }

                // if (PointComparer.Compare(pac.Location, startVertex.Point) >= 0)
                if compare_points(pac.location, start_vertex_point) != std::cmp::Ordering::Less {
                    // if (PointComparer.Compare(pac.Location, end) < 0)
                    if compare_points(pac.location, end) == std::cmp::Ordering::Less {
                        // SpliceGroupBoundaryCrossing(currentVertex, pac, dir);
                        self.splice_group_boundary_crossing(
                            graph,
                            obstacle_tree,
                            current_vertex,
                            &pac,
                            dir,
                        );
                    }
                }
            }

            // if (isFinalVertex) { break; }
            if is_final_vertex {
                break;
            }

            // advance: currentVertex = StaticGraphUtility.FindAdjacentVertex(currentVertex, dir)
            current_vertex_opt =
                StaticGraphUtility::find_adjacent_vertex(graph, current_vertex, dir);
        }
    }

    // ── C# line 543-557: TraverseToFirstVertexAtOrAbove ────────────────────────
    fn traverse_to_first_vertex_at_or_above(
        graph: &VisibilityGraph,
        start_vertex: VertexId,
        start: Point,
        dir: CompassDirection,
    ) -> VertexId {
        // var returnVertex = startVertex;
        let mut return_vertex = start_vertex;

        // var oppositeDir = CompassVector.OppositeDir(dir);
        let opposite_dir = dir.opposite();

        // for (;;)
        loop {
            // var nextVertex = StaticGraphUtility.FindAdjacentVertex(returnVertex, dir);
            let next_vertex =
                StaticGraphUtility::find_adjacent_vertex(graph, return_vertex, dir);

            // if ((nextVertex == null) || (PointComparer.GetDirections(nextVertex.Point, start) == oppositeDir))
            match next_vertex {
                None => break,
                Some(nv) => {
                    let dirs = Direction::from_point_to_point(graph.point(nv), start);
                    if dirs == opposite_dir.to_direction() {
                        break;
                    }
                    // returnVertex = nextVertex;
                    return_vertex = nv;
                }
            }
        }

        // return returnVertex;
        return_vertex
    }

    // ── C# line 559-575: SpliceGroupBoundaryCrossing ───────────────────────────
    fn splice_group_boundary_crossing(
        &mut self,
        graph: &mut VisibilityGraph,
        obstacle_tree: &ObstacleTree,
        current_vertex: VertexId,
        pac: &PointAndCrossings,
        dir_to_inside: CompassDirection,
    ) {
        // GroupBoundaryCrossing[] crossings = PointAndCrossingsList.ToCrossingArray(pac.Crossings, dirToInside);
        let crossings =
            PointAndCrossingsList::to_crossing_array(&pac.crossings, dir_to_inside);
        if crossings.is_empty() {
            return;
        }

        // var outerVertex = VisGraph.FindVertex(pac.Location) ?? AddVertex(pac.Location);
        let outer_vertex = match graph.find_vertex(pac.location) {
            Some(v) => v,
            None => self.add_vertex(graph, pac.location),
        };

        // if (currentVertex.Point != outerVertex.Point) { FindOrAddEdge(currentVertex, outerVertex); }
        let current_point = graph.point(current_vertex);
        if !GeomConstants::close(current_point.x(), pac.location.x())
            || !GeomConstants::close(current_point.y(), pac.location.y())
        {
            self.find_or_add_edge(graph, current_vertex, outer_vertex, NORMAL_WEIGHT);
        }

        // var interiorPoint = crossings[0].GetInteriorVertexPoint(pac.Location);
        let interior_point = crossings[0].get_interior_vertex_point(pac.location);

        // var interiorVertex = VisGraph.FindVertex(interiorPoint) ?? AddVertex(interiorPoint);
        let interior_vertex = match graph.find_vertex(interior_point) {
            Some(v) => v,
            None => self.add_vertex(graph, interior_point),
        };

        // FindOrAddEdge(outerVertex, interiorVertex);
        self.find_or_add_edge(graph, outer_vertex, interior_vertex, NORMAL_WEIGHT);

        // var edge = VisGraph.FindEdge(outerVertex.Point, interiorVertex.Point);
        // var crossingsArray = crossings.Select(c => c.Group.InputShape).ToArray();
        // edge.IsPassable = delegate { return crossingsArray.Any(s => s.IsTransparent); }
        //
        // In Rust: capture current is_transparent snapshot for each obstacle.
        // Since routing is a one-shot computation, transparency doesn't change
        // during the session. We capture Arc<[bool]> for the obstacle indices.
        let is_transparent_flags: Vec<bool> = crossings
            .iter()
            .map(|c| obstacle_tree.obstacles[c.obstacle_idx].is_transparent())
            .collect();
        let is_transparent_flags = Arc::new(is_transparent_flags);
        let passable_fn: Arc<dyn Fn() -> bool + Send + Sync> =
            Arc::new(move || is_transparent_flags.iter().any(|&t| t));

        // Set is_passable on the outer→interior edge.
        // Edges are stored source→target in ascending point order (via is_pure_lower).
        let outer_pt = graph.point(outer_vertex);
        let inner_pt = graph.point(interior_vertex);
        let (edge_src, edge_tgt) = if StaticGraphUtility::is_pure_lower(outer_pt, inner_pt) {
            (outer_vertex, interior_vertex)
        } else {
            (interior_vertex, outer_vertex)
        };
        graph.set_edge_passable(edge_src, edge_tgt, passable_fn);
    }

    // ── C# line 755-767: SeeIfSpliceIsStillOverlapped ─────────────────────────
    pub(super) fn see_if_splice_is_still_overlapped(
        &self,
        graph: &VisibilityGraph,
        obstacle_tree: &mut ObstacleTree,
        extend_dir: CompassDirection,
        next_extend_vertex: VertexId,
    ) -> bool {
        // var edge = this.FindNextEdge(nextExtendVertex, CompassVector.RotateLeft(extendDir));
        let edge = TransientGraphUtility::find_next_edge(graph, next_extend_vertex, extend_dir.left());

        // var maybeFreeSpace = (edge == null) ? false : (ScanSegment.NormalWeight == edge.Weight);
        let mut maybe_free_space = match edge {
            None => false,
            Some((_target, weight)) => weight == NORMAL_WEIGHT,
        };

        // if (!maybeFreeSpace)
        if !maybe_free_space {
            // edge = this.FindNextEdge(nextExtendVertex, CompassVector.RotateRight(extendDir));
            let edge2 =
                TransientGraphUtility::find_next_edge(graph, next_extend_vertex, extend_dir.right());
            // maybeFreeSpace = (edge == null) ? false : (ScanSegment.NormalWeight == edge.Weight);
            maybe_free_space = match edge2 {
                None => false,
                Some((_target, weight)) => weight == NORMAL_WEIGHT,
            };
        }

        // return !maybeFreeSpace || this.ObstacleTree.PointIsInsideAnObstacle(nextExtendVertex.Point, extendDir);
        !maybe_free_space
            || obstacle_tree.point_is_inside_an_obstacle_dir(
                graph.point(next_extend_vertex),
                extend_dir.to_direction(),
            )
    }

    // ── C# line 769-779: IsSkippableSpliceSourceWithNullSpliceTarget ───────────
    pub(super) fn is_skippable_splice_source_with_null_splice_target(
        graph: &VisibilityGraph,
        splice_source: VertexId,
        extend_dir: CompassDirection,
    ) -> bool {
        // if (IsSkippableSpliceSourceEdgeWithNullTarget(StaticGraphUtility.FindAdjacentEdge(spliceSource, extendDir)))
        let edge_forward =
            StaticGraphUtility::find_adjacent_edge(graph, splice_source, extend_dir);
        if Self::is_skippable_splice_source_edge_with_null_target(
            graph,
            edge_forward,
            splice_source,
        ) {
            return true;
        }

        // var spliceSourceEdge = StaticGraphUtility.FindAdjacentEdge(spliceSource, CompassVector.OppositeDir(extendDir));
        let splice_source_edge = StaticGraphUtility::find_adjacent_edge(
            graph,
            splice_source,
            extend_dir.opposite(),
        );

        // return (IsSkippableSpliceSourceEdgeWithNullTarget(spliceSourceEdge) || IsReflectionEdge(spliceSourceEdge));
        Self::is_skippable_splice_source_edge_with_null_target(
            graph,
            splice_source_edge,
            splice_source,
        ) || Self::is_reflection_edge(splice_source_edge)
    }

    // ── C# line 781-785: IsSkippableSpliceSourceEdgeWithNullTarget ─────────────
    fn is_skippable_splice_source_edge_with_null_target(
        graph: &VisibilityGraph,
        edge_target: Option<(VertexId, f64)>,
        edge_source_vertex: VertexId,
    ) -> bool {
        // return (spliceSourceEdge != null)
        //     && (spliceSourceEdge.IsPassable != null)
        //     && (PointComparer.Equal(spliceSourceEdge.Length, GroupBoundaryCrossing.BoundaryWidth));
        match edge_target {
            None => false,
            Some((target_id, _weight)) => {
                // Check IsPassable != null: find the edge and check its is_passable field.
                let has_passable = graph
                    .find_edge(edge_source_vertex, target_id)
                    .map_or(false, |e| e.is_passable.is_some());
                if !has_passable {
                    return false;
                }
                // Check length ≈ GROUP_BOUNDARY_WIDTH.
                let source_point = graph.point(edge_source_vertex);
                let target_point = graph.point(target_id);
                let length = (target_point - source_point).length();
                GeomConstants::close(length, GROUP_BOUNDARY_WIDTH)
            }
        }
    }

    // ── C# line 787-789: IsReflectionEdge ──────────────────────────────────────
    fn is_reflection_edge(edge: Option<(VertexId, f64)>) -> bool {
        // return (edge != null) && (edge.Weight == ScanSegment.ReflectionWeight);
        match edge {
            None => false,
            Some((_target, weight)) => weight == REFLECTION_WEIGHT,
        }
    }

    // ── C# line 791-793: IsPointPastSegmentEnd ─────────────────────────────────
    pub(super) fn is_point_past_segment_end(
        max_segment_start: Point,
        max_segment_end: Point,
        point: Point,
    ) -> bool {
        // return PointComparer.GetDirections(maxSegment.Start, maxSegment.End)
        //     == PointComparer.GetDirections(maxSegment.End, point);
        Direction::from_point_to_point(max_segment_start, max_segment_end)
            == Direction::from_point_to_point(max_segment_end, point)
    }

    // ── C# line 672-706: Debug_VerifyNonOverlappedExtension ────────────────────
    #[cfg(debug_assertions)]
    pub(super) fn debug_verify_non_overlapped_extension(
        &self,
        graph: &VisibilityGraph,
        obstacle_tree: &mut ObstacleTree,
        is_overlapped: bool,
        _extend_vertex: VertexId,
        next_extend_vertex: VertexId,
        splice_source: Option<VertexId>,
        splice_target: Option<VertexId>,
    ) {
        // if (isOverlapped) { return; }
        if is_overlapped {
            return;
        }

        // StaticGraphUtility.Assert(!this.ObstacleTree.SegmentCrossesANonGroupObstacle(extendVertex.Point, nextExtendVertex.Point), ...)
        // Note: segment_crosses_a_non_group_obstacle is not yet implemented.
        // Using segment_crosses_an_obstacle as a conservative approximation.
        // TODO: Replace with segment_crosses_a_non_group_obstacle when group support is added.
        // For now, skip this assertion since it would be too strict (would also flag
        // group obstacles which should be allowed).
        // debug_assert!(
        //     !obstacle_tree.segment_crosses_an_obstacle(
        //         graph.point(extend_vertex),
        //         graph.point(next_extend_vertex)
        //     ),
        //     "extendDir edge crosses an obstacle"
        // );

        // if (spliceSource == null) { return; }
        let splice_source = match splice_source {
            None => return,
            Some(s) => s,
        };

        let splice_source_point = graph.point(splice_source);
        let next_extend_point = graph.point(next_extend_vertex);

        // if ((spliceTarget == null)
        //     || (this.VisGraph.FindEdge(spliceSource.Point, spliceTarget.Point) == null
        //         && (this.VisGraph.FindEdge(spliceSource.Point, nextExtendVertex.Point) == null)))
        let should_check = match splice_target {
            None => true,
            Some(st) => {
                graph
                    .find_edge_pp(splice_source_point, graph.point(st))
                    .is_none()
                    && graph
                        .find_edge_pp(splice_source_point, next_extend_point)
                        .is_none()
            }
        };

        if should_check {
            // StaticGraphUtility.Assert(!this.ObstacleTree.SegmentCrossesAnObstacle(spliceSource.Point, nextExtendVertex.Point), ...)
            debug_assert!(
                !obstacle_tree
                    .segment_crosses_an_obstacle(splice_source_point, next_extend_point),
                "spliceSource->extendVertex edge crosses an obstacle"
            );

            // StaticGraphUtility.Assert((spliceTarget == null) || ...)
            if let Some(st) = splice_target {
                let splice_target_point = graph.point(st);
                debug_assert!(
                    graph
                        .find_edge_pp(next_extend_point, splice_target_point)
                        .is_some()
                        || !obstacle_tree
                            .segment_crosses_an_obstacle(next_extend_point, splice_target_point),
                    "extendVertex->spliceTarget edge crosses an obstacle"
                );
            }
        }
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub(super) fn debug_verify_non_overlapped_extension(
        &self,
        _graph: &VisibilityGraph,
        _obstacle_tree: &mut ObstacleTree,
        _is_overlapped: bool,
        _extend_vertex: VertexId,
        _next_extend_vertex: VertexId,
        _splice_source: Option<VertexId>,
        _splice_target: Option<VertexId>,
    ) {
        // No-op in release builds.
    }
}
