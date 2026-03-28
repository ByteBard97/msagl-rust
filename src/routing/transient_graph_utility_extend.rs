//! Extension methods for `TransientGraphUtility`: `ExtendEdgeChain` and helpers.
//!
//! Ported from C# `TransientGraphUtility.cs` lines 418-802.
//! This file is separate from `transient_graph_utility.rs` because the base file
//! is already near the 500-line limit.

use super::compass_direction::{CompassDirection, Direction};
use super::group_boundary_crossing::PointAndCrossingsList;
use super::router_session::RouterSession;
use super::static_graph_utility::StaticGraphUtility;
use super::transient_graph_utility::TransientGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::visibility::graph::VertexId;

/// Weight for normal scan segments (matches C# ScanSegment.NormalWeight = 1).
const NORMAL_WEIGHT: f64 = 1.0;

/// Weight for overlapped scan segments (matches C# ScanSegment.OverlappedWeight = 100000).
const OVERLAPPED_WEIGHT: f64 = 100_000.0;

impl TransientGraphUtility {
    // ── C# line 418-452: ExtendEdgeChain (public entry point) ──────────────────
    pub fn extend_edge_chain_public(
        &mut self,
        session: &mut RouterSession,
        start_vertex: VertexId,
        limit_rect: &Rectangle,
        max_visibility_segment_start: Point,
        max_visibility_segment_end: Point,
        pac_list: Option<&mut PointAndCrossingsList>,
        is_overlapped: bool,
    ) {
        // var dir = PointComparer.GetDirections(maxVisibilitySegment.Start, maxVisibilitySegment.End);
        let dir = Direction::from_point_to_point(
            max_visibility_segment_start,
            max_visibility_segment_end,
        );
        // if (dir == Direction.None) { return; }
        if dir.is_none() {
            return;
        }
        // Debug.Assert(CompassVector.IsPureDirection(dir), ...)
        debug_assert!(dir.is_pure(), "impure max visibility segment");

        // Debug assertion: start vertex is consistent with segment direction
        let start_point = session.vis_graph.point(start_vertex);
        debug_assert!(
            start_point.close_to(max_visibility_segment_start)
                || CompassDirection::from_points(max_visibility_segment_start, start_point)
                    == Some(CompassDirection::from_direction(dir)),
            "Inconsistent direction found"
        );

        // double oppositeFarBound = StaticGraphUtility.GetRectangleBound(limitRect, dir);
        let compass_dir = CompassDirection::from_direction(dir);
        let opposite_far_bound = StaticGraphUtility::get_rectangle_bound(limit_rect, compass_dir);

        // Point maxDesiredSplicePoint = StaticGraphUtility.IsVertical(dir) ? ...
        let max_desired_splice_point = if StaticGraphUtility::is_vertical(compass_dir) {
            Point::round(Point::new(start_point.x(), opposite_far_bound))
        } else {
            Point::round(Point::new(opposite_far_bound, start_point.y()))
        };

        // if (PointComparer.Equal(maxDesiredSplicePoint, startVertex.Point)) { return; }
        if max_desired_splice_point.close_to(start_point) {
            return;
        }

        // if (PointComparer.GetPureDirection(startVertex.Point, maxDesiredSplicePoint) != dir) { return; }
        let dir_to_splice =
            CompassDirection::from_points(start_point, max_desired_splice_point);
        if dir_to_splice != Some(compass_dir) {
            return;
        }

        // var maxDesiredSegment = maxVisibilitySegment;
        let max_desired_segment_start = max_visibility_segment_start;
        let mut max_desired_segment_end = max_visibility_segment_end;

        // if (PointComparer.GetDirections(maxDesiredSplicePoint, maxDesiredSegment.End) == dir)
        //     maxDesiredSegment = new LineSegment(maxDesiredSegment.Start, maxDesiredSplicePoint);
        let dir_splice_to_end = Direction::from_point_to_point(
            max_desired_splice_point,
            max_desired_segment_end,
        );
        if dir_splice_to_end == dir {
            max_desired_segment_end = max_desired_splice_point;
        }

        // ExtendEdgeChain(startVertex, dir, maxDesiredSegment, maxVisibilitySegment, pacList, isOverlapped);
        self.extend_edge_chain(
            session,
            start_vertex,
            compass_dir,
            max_desired_segment_start,
            max_desired_segment_end,
            max_visibility_segment_start,
            max_visibility_segment_end,
            pac_list,
            is_overlapped,
        );
    }

    // ── C# line 454-493: ExtendEdgeChain (private) ─────────────────────────────
    fn extend_edge_chain(
        &mut self,
        session: &mut RouterSession,
        start_vertex: VertexId,
        extend_dir: CompassDirection,
        max_desired_segment_start: Point,
        max_desired_segment_end: Point,
        max_visibility_segment_start: Point,
        max_visibility_segment_end: Point,
        pac_list: Option<&mut PointAndCrossingsList>,
        is_overlapped: bool,
    ) {
        // StaticGraphUtility.Assert(PointComparer.GetPureDirection(maxDesiredSegment.Start, maxDesiredSegment.End) == extendDir, ...)
        debug_assert!(
            CompassDirection::from_points(max_desired_segment_start, max_desired_segment_end)
                == Some(extend_dir),
            "maxDesiredSegment is reversed"
        );

        // Direction segmentDir = PointComparer.GetDirections(startVertex.Point, maxDesiredSegment.End);
        let start_point = session.vis_graph.point(start_vertex);
        let segment_dir = Direction::from_point_to_point(start_point, max_desired_segment_end);

        // if (segmentDir != extendDir)
        if segment_dir != extend_dir.to_direction() {
            // StaticGraphUtility.Assert(isOverlapped || (segmentDir != CompassVector.OppositeDir(extendDir)), ...)
            debug_assert!(
                is_overlapped || segment_dir != extend_dir.opposite().to_direction(),
                "obstacle encountered between prevPoint and startVertex"
            );
            return;
        }

        // Direction spliceSourceDir = CompassVector.RotateLeft(extendDir);
        let splice_source_dir = extend_dir.left();

        // VisibilityVertex spliceSource = StaticGraphUtility.FindAdjacentVertex(startVertex, spliceSourceDir);
        let mut splice_source =
            StaticGraphUtility::find_adjacent_vertex(&session.vis_graph, start_vertex, splice_source_dir);

        let mut actual_splice_source_dir = splice_source_dir;

        // if (spliceSource == null)
        if splice_source.is_none() {
            // spliceSourceDir = CompassVector.OppositeDir(spliceSourceDir);
            actual_splice_source_dir = splice_source_dir.opposite();
            // spliceSource = StaticGraphUtility.FindAdjacentVertex(startVertex, spliceSourceDir);
            splice_source = StaticGraphUtility::find_adjacent_vertex(
                &session.vis_graph,
                start_vertex,
                actual_splice_source_dir,
            );
            // if (spliceSource == null) { return; }
            if splice_source.is_none() {
                return;
            }
        }

        let splice_source_id = splice_source.unwrap();

        // Direction spliceTargetDir = CompassVector.OppositeDir(spliceSourceDir);
        let splice_target_dir = actual_splice_source_dir.opposite();

        // VisibilityVertex spliceTarget;
        // if (ExtendSpliceWorker(spliceSource, extendDir, spliceTargetDir, ...))
        let (should_continue, splice_target) = self.extend_splice_worker(
            session,
            splice_source_id,
            extend_dir,
            splice_target_dir,
            max_desired_segment_start,
            max_desired_segment_end,
            max_visibility_segment_start,
            max_visibility_segment_end,
            is_overlapped,
        );

        if should_continue {
            if let Some(st) = splice_target {
                // ExtendSpliceWorker(spliceTarget, extendDir, spliceSourceDir, ...)
                let _ = self.extend_splice_worker(
                    session,
                    st,
                    extend_dir,
                    actual_splice_source_dir,
                    max_desired_segment_start,
                    max_desired_segment_end,
                    max_visibility_segment_start,
                    max_visibility_segment_end,
                    is_overlapped,
                );
            }
        }

        // SpliceGroupBoundaryCrossings(pacList, startVertex, maxDesiredSegment);
        self.splice_group_boundary_crossings(
            session,
            pac_list,
            start_vertex,
            max_desired_segment_start,
            max_desired_segment_end,
        );
    }

    // ── C# line 579-670: ExtendSpliceWorker ────────────────────────────────────
    fn extend_splice_worker(
        &mut self,
        session: &mut RouterSession,
        splice_source: VertexId,
        extend_dir: CompassDirection,
        splice_target_dir: CompassDirection,
        _max_desired_segment_start: Point,
        max_desired_segment_end: Point,
        max_visibility_segment_start: Point,
        max_visibility_segment_end: Point,
        is_overlapped: bool,
    ) -> (bool, Option<VertexId>) {
        let mut splice_source = splice_source;
        let mut is_overlapped = is_overlapped;

        // VisibilityVertex extendVertex = StaticGraphUtility.FindAdjacentVertex(spliceSource, spliceTargetDir);
        let extend_vertex_opt =
            StaticGraphUtility::find_adjacent_vertex(&session.vis_graph, splice_source, splice_target_dir);
        let mut extend_vertex = match extend_vertex_opt {
            Some(v) => v,
            None => return (false, None),
        };

        // spliceTarget = StaticGraphUtility.FindAdjacentVertex(extendVertex, spliceTargetDir);
        let mut splice_target =
            StaticGraphUtility::find_adjacent_vertex(&session.vis_graph, extend_vertex, splice_target_dir);

        // for (;;)
        loop {
            // if (!GetNextSpliceSource(ref spliceSource, spliceTargetDir, extendDir)) { break; }
            match Self::get_next_splice_source(&session.vis_graph, splice_source, splice_target_dir, extend_dir)
            {
                Some(new_source) => splice_source = new_source,
                None => break,
            }

            // Point nextExtendPoint = StaticGraphUtility.FindBendPointBetween(
            //     extendVertex.Point, spliceSource.Point, CompassVector.OppositeDir(spliceTargetDir));
            let next_extend_point = StaticGraphUtility::find_bend_point_between(
                session.vis_graph.point(extend_vertex),
                session.vis_graph.point(splice_source),
                splice_target_dir.opposite(),
            );

            // if (IsPointPastSegmentEnd(maxVisibilitySegment, nextExtendPoint)) { break; }
            if Self::is_point_past_segment_end(
                max_visibility_segment_start,
                max_visibility_segment_end,
                next_extend_point,
            ) {
                break;
            }

            // spliceTarget = GetSpliceTarget(ref spliceSource, spliceTargetDir, nextExtendPoint);
            let (new_source, new_target) =
                Self::get_splice_target(&session.vis_graph, splice_source, splice_target_dir, next_extend_point);
            splice_source = new_source;
            splice_target = new_target;

            // if (spliceTarget == null)
            if splice_target.is_none() {
                // if (this.IsSkippableSpliceSourceWithNullSpliceTarget(spliceSource, extendDir)) { continue; }
                if Self::is_skippable_splice_source_with_null_splice_target(
                    &session.vis_graph,
                    splice_source,
                    extend_dir,
                ) {
                    continue;
                }

                // if (ObstacleTree.SegmentCrossesAnObstacle(spliceSource.Point, nextExtendPoint)) { return false; }
                if session.obstacle_tree.segment_crosses_an_obstacle(
                    session.vis_graph.point(splice_source),
                    next_extend_point,
                ) {
                    return (false, None);
                }
            }

            // VisibilityVertex nextExtendVertex = VisGraph.FindVertex(nextExtendPoint);
            let next_extend_vertex_existing = session.vis_graph.find_vertex(next_extend_point);

            let next_extend_vertex;

            // if (nextExtendVertex != null)
            if let Some(existing) = next_extend_vertex_existing {
                // if ((spliceTarget == null) || (this.VisGraph.FindEdge(extendVertex.Point, nextExtendPoint) != null))
                if splice_target.is_none()
                    || session.vis_graph
                        .find_edge_pp(session.vis_graph.point(extend_vertex), next_extend_point)
                        .is_some()
                {
                    if splice_target.is_none() {
                        // Debug_VerifyNonOverlappedExtension(isOverlapped, extendVertex, nextExtendVertex, spliceSource:null, spliceTarget:null);
                        self.debug_verify_non_overlapped_extension(
                            &session.vis_graph,
                            &mut session.obstacle_tree,
                            is_overlapped,
                            extend_vertex,
                            existing,
                            None,
                            None,
                        );
                        // FindOrAddEdge(extendVertex, nextExtendVertex, isOverlapped ? OverlappedWeight : NormalWeight);
                        let weight = if is_overlapped {
                            OVERLAPPED_WEIGHT
                        } else {
                            NORMAL_WEIGHT
                        };
                        self.find_or_add_edge(session, extend_vertex, existing, weight);
                    }
                    return (false, splice_target);
                }

                // StaticGraphUtility.Assert(spliceTarget == StaticGraphUtility.FindAdjacentVertex(nextExtendVertex, spliceTargetDir), ...)
                let adj = StaticGraphUtility::find_adjacent_vertex(&session.vis_graph, existing, splice_target_dir);
                debug_assert!(adj == splice_target,
                    "no edge exists between an existing nextExtendVertex and spliceTarget"
                );

                next_extend_vertex = existing;
            } else {
                // StaticGraphUtility.Assert((spliceTarget == null) || spliceTargetDir == PointComparer.GetPureDirection(nextExtendPoint, spliceTarget.Point), ...)
                if let Some(st) = splice_target {
                    debug_assert!(
                        CompassDirection::from_points(next_extend_point, session.vis_graph.point(st))
                            == Some(splice_target_dir),
                        "spliceTarget is not to spliceTargetDir of nextExtendVertex"
                    );
                }

                // nextExtendVertex = this.AddVertex(nextExtendPoint);
                next_extend_vertex = self.add_vertex(session, next_extend_point);
            }

            // FindOrAddEdge(extendVertex, nextExtendVertex, isOverlapped ? OverlappedWeight : NormalWeight);
            let weight = if is_overlapped {
                OVERLAPPED_WEIGHT
            } else {
                NORMAL_WEIGHT
            };
            self.find_or_add_edge(session, extend_vertex, next_extend_vertex, weight);

            // Debug_VerifyNonOverlappedExtension(isOverlapped, extendVertex, nextExtendVertex, spliceSource, spliceTarget);
            self.debug_verify_non_overlapped_extension(
                &session.vis_graph,
                &mut session.obstacle_tree,
                is_overlapped,
                extend_vertex,
                next_extend_vertex,
                Some(splice_source),
                splice_target,
            );

            // FindOrAddEdge(spliceSource, nextExtendVertex, isOverlapped ? OverlappedWeight : NormalWeight);
            self.find_or_add_edge(session, splice_source, next_extend_vertex, weight);

            // if (isOverlapped) { isOverlapped = this.SeeIfSpliceIsStillOverlapped(extendDir, nextExtendVertex); }
            if is_overlapped {
                is_overlapped = self.see_if_splice_is_still_overlapped(
                    session,
                    extend_dir,
                    next_extend_vertex,
                );
            }

            // extendVertex = nextExtendVertex;
            extend_vertex = next_extend_vertex;

            // if (0 == (extendDir & PointComparer.GetDirections(nextExtendPoint, maxDesiredSegment.End)))
            let dirs_to_end =
                Direction::from_point_to_point(next_extend_point, max_desired_segment_end);
            if (extend_dir.to_direction() & dirs_to_end).is_none() {
                splice_target = None;
                break;
            }
        }

        // return spliceTarget != null;
        (splice_target.is_some(), splice_target)
    }

}
