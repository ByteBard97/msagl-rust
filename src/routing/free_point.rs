//! FreePoint — a waypoint or non-obstacle port on a routing path.
//!
//! Split from `port.rs` to keep files under 500 lines.
//! Ported from C# `FreePoint.cs` (132 lines).

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::visibility::graph::VertexId;
use super::compass_direction::CompassDirection;
use super::router_session::RouterSession;

// =========================================================================
// FreePoint — C# FreePoint.cs (132 lines)
//
// A point on a path not associated with an obstacle (waypoint, free port).
// Manages its own visibility vertex and max visibility segments in 4 dirs.
// =========================================================================

/// Segment and crossings data for one direction of max visibility.
///
/// C# FreePoint.cs line 15: SegmentAndCrossings = Tuple<LineSegment, PointAndCrossingsList>
#[derive(Clone, Debug)]
pub struct SegmentAndCrossings {
    /// The max visibility line segment in this direction.
    pub segment_start: Point,
    pub segment_end: Point,
    // Group boundary crossings along this segment (deferred: groups not yet supported).
}

/// A free point (waypoint or non-obstacle port) on a routing path.
///
/// C# file: FreePoint.cs, lines 21-132
/// Manages its visibility vertex, overlap status, out-of-bounds status,
/// and cached max visibility segments in each of 4 compass directions.
#[derive(Clone, Debug)]
pub struct FreePoint {
    /// The visibility vertex for this point.
    /// C# FreePoint.cs line 24: Vertex
    pub vertex: Option<VertexId>,

    /// The point location (derived from vertex).
    /// C# FreePoint.cs line 25: Point
    pub point: Point,

    /// Whether this point is inside an obstacle (overlapped).
    /// C# FreePoint.cs line 26: IsOverlapped
    pub is_overlapped: bool,

    /// Direction from graph boundary if out of bounds.
    /// C# FreePoint.cs line 28: OutOfBoundsDirectionFromGraph
    /// None means in-bounds.
    pub out_of_bounds_direction: Option<CompassDirection>,

    /// Cached max visibility segments and crossings in each of 4 directions.
    /// C# FreePoint.cs line 31: maxVisibilitySegmentsAndCrossings[4]
    pub max_visibility_segments: [Option<SegmentAndCrossings>; 4],
}

impl FreePoint {
    /// Create a new FreePoint, adding a vertex to the graph if needed.
    ///
    /// C# file: FreePoint.cs, lines 34-37
    /// Big-O: O(log V) for vertex find/add
    pub fn new(point: Point, session: &mut RouterSession) -> Self {
        // C#: OutOfBoundsDirectionFromGraph = Direction.None; this.GetVertex(transUtil, point);
        let vertex = session.vis_graph.find_or_add_vertex(point);
        Self {
            vertex: Some(vertex),
            point,
            is_overlapped: false,
            out_of_bounds_direction: None,
            max_visibility_segments: [None, None, None, None],
        }
    }

    /// Get or create the visibility vertex for this point.
    ///
    /// C# file: FreePoint.cs, lines 39-41
    /// Big-O: O(log V) for vertex find/add
    pub fn get_vertex(&mut self, session: &mut RouterSession) {
        // C#: this.Vertex = transUtil.FindOrAddVertex(point);
        let vertex = session.vis_graph.find_or_add_vertex(self.point);
        self.vertex = Some(vertex);
    }

    /// Initial weight for edges from this point (overlapped or normal).
    ///
    /// C# file: FreePoint.cs, line 27: InitialWeight
    pub fn initial_weight(&self) -> f64 {
        if self.is_overlapped {
            100_000.0 // ScanSegment.OverlappedWeight
        } else {
            1.0 // ScanSegment.NormalWeight
        }
    }

    /// Whether this point is out of the graph bounds.
    ///
    /// C# file: FreePoint.cs, line 29: IsOutOfBounds
    pub fn is_out_of_bounds(&self) -> bool {
        self.out_of_bounds_direction.is_some()
    }

    /// Add an edge from this point to an adjacent edge and extend the chain.
    ///
    /// C# file: FreePoint.cs, lines 46-58
    /// Big-O: O(chain) for VG adjacency walk
    /// MUST use StaticGraphUtility.SegmentIntersection for intersection point
    pub fn add_edge_to_adjacent_edge(
        &mut self,
        target_edge: (VertexId, VertexId),
        dir_to_extend: CompassDirection,
        limit_rect: &Rectangle,
        session: &mut RouterSession,
        trans_util: &mut crate::routing::transient_graph_utility::TransientGraphUtility,
    ) -> Option<VertexId> {
        // C#: Point targetIntersect = StaticGraphUtility.SegmentIntersection(targetEdge, this.Point);
        // SegmentIntersection finds the perpendicular projection of this.Point onto the edge.
        // For rectilinear edges, this is the point on the edge sharing the appropriate coordinate.
        let self_vertex = self.vertex?;
        let edge_src_point = session.vis_graph.point(target_edge.0);
        let edge_tgt_point = session.vis_graph.point(target_edge.1);

        // For axis-aligned edges, the intersection is the projection of self.point onto the edge.
        let target_intersect = if GeomConstants::close(edge_src_point.x(), edge_tgt_point.x()) {
            // Vertical edge — intersect at (edge.x, self.y)
            Point::new(edge_src_point.x(), self.point.y())
        } else {
            // Horizontal edge — intersect at (self.x, edge.y)
            Point::new(self.point.x(), edge_src_point.y())
        };

        // C#: VisibilityVertex targetVertex = transUtil.VisGraph.FindVertex(targetIntersect);
        let target_vertex = session.vis_graph.find_vertex(target_intersect);

        if let Some(tv) = target_vertex {
            // Vertex already exists — add edge to it and extend.
            self.add_to_adjacent_vertex(tv, dir_to_extend, limit_rect, session, trans_util);
            return Some(tv);
        }

        // Vertex doesn't exist — AddEdgeToTargetEdge adds it to the graph.
        // C#: targetVertex = transUtil.AddEdgeToTargetEdge(this.Vertex, targetEdge, targetIntersect);
        let new_vertex = trans_util.add_edge_to_target_edge(
            session,
            self_vertex,
            target_edge.0,
            target_edge.1,
            target_intersect,
        );

        // C#: ExtendEdgeChain(transUtil, targetVertex, dirToExtend, limitRect);
        self.extend_edge_chain(new_vertex, dir_to_extend, limit_rect, session, trans_util);
        Some(new_vertex)
    }

    /// Add edge to an adjacent vertex and extend the chain.
    ///
    /// C# file: FreePoint.cs, lines 60-66
    /// Big-O: O(chain) for VG adjacency walk
    pub fn add_to_adjacent_vertex(
        &mut self,
        target_vertex: VertexId,
        dir_to_extend: CompassDirection,
        limit_rect: &Rectangle,
        session: &mut RouterSession,
        trans_util: &mut crate::routing::transient_graph_utility::TransientGraphUtility,
    ) {
        if let Some(self_vertex) = self.vertex {
            let tp = session.vis_graph.point(target_vertex);
            if !(GeomConstants::close(self.point.x(), tp.x())
                && GeomConstants::close(self.point.y(), tp.y()))
            {
                let weight = self.initial_weight();
                trans_util.find_or_add_edge(session, self_vertex, target_vertex, weight);
            }
        }
        self.extend_edge_chain(target_vertex, dir_to_extend, limit_rect, session, trans_util);
    }

    /// Extend the edge chain from a target vertex in the given direction.
    ///
    /// C# file: FreePoint.cs, lines 68-86
    /// Big-O: O(chain) for VG adjacency walk + O(log N) for obstacle tree
    /// MUST use ObstacleTree for CreateMaxVisibilitySegment
    pub fn extend_edge_chain(
        &mut self,
        target_vertex: VertexId,
        dir_to_extend: CompassDirection,
        limit_rect: &Rectangle,
        session: &mut RouterSession,
        trans_util: &mut crate::routing::transient_graph_utility::TransientGraphUtility,
    ) {
        let mut extend_overlapped = self.is_overlapped;
        if extend_overlapped {
            use crate::routing::scan_direction::ScanDirection;
            let scan_dir = if dir_to_extend == CompassDirection::East || dir_to_extend == CompassDirection::West {
                ScanDirection::horizontal()
            } else {
                ScanDirection::vertical()
            };
            extend_overlapped = session.obstacle_tree.point_is_inside_an_obstacle(session.vis_graph.point(target_vertex), scan_dir);
        }

        let start_vertex = if self.is_overlapped { target_vertex } else { self.vertex.unwrap() };
        let (seg_start, seg_end) = self.get_segment_and_crossings(start_vertex, dir_to_extend, session);
        trans_util.extend_edge_chain_public(
            session,
            target_vertex,
            limit_rect,
            seg_start,
            seg_end,
            None,
            extend_overlapped
        );
    }

    fn get_segment_and_crossings(
        &mut self,
        start_vertex: VertexId,
        dir_to_extend: CompassDirection,
        session: &mut RouterSession,
    ) -> (Point, Point) {
        let dir_index = dir_to_extend.index();
        if let Some(ref mut sac) = self.max_visibility_segments[dir_index] {
            let start_point = session.vis_graph.point(start_vertex);
            if CompassDirection::from_points(start_point, sac.segment_start) == Some(dir_to_extend) {
                sac.segment_start = start_point;
            }
            return (sac.segment_start, sac.segment_end);
        }

        let start_point = session.vis_graph.point(start_vertex);
        // Create a very long segment in the direction of extension
        // Then restrict it using the obstacle tree.
        let far_distance = 1e6; // A sufficiently large number representing max visibility
        let far_point = match dir_to_extend {
            CompassDirection::North => Point::new(start_point.x(), start_point.y() + far_distance),
            CompassDirection::South => Point::new(start_point.x(), start_point.y() - far_distance),
            CompassDirection::East => Point::new(start_point.x() + far_distance, start_point.y()),
            CompassDirection::West => Point::new(start_point.x() - far_distance, start_point.y()),
        };

        let (seg_start, seg_end) = session.obstacle_tree.restrict_segment_with_obstacles(start_point, far_point);
        let sac = match &mut self.max_visibility_segments[dir_index] {
            slot => {
                let val = SegmentAndCrossings {
                    segment_start: seg_start,
                    segment_end: seg_end,
                };
                *slot = Some(val.clone());
                val
            }
        };
        (sac.segment_start, sac.segment_end)
    }

    /// Get max visibility endpoint in a direction for non-overlapped free points.
    ///
    /// C# file: FreePoint.cs, lines 106-110
    /// Big-O: O(log N) for obstacle tree if not cached
    pub fn max_visibility_in_direction(
        &mut self,
        dir_to_extend: CompassDirection,
        session: &mut RouterSession,
    ) -> Point {
        let vertex = self.vertex.unwrap();
        let (_start, end) = self.get_segment_and_crossings(vertex, dir_to_extend, session);
        end
    }

    /// Add out-of-bounds edges from a graph corner vertex.
    ///
    /// C# file: FreePoint.cs, lines 112-119
    /// Big-O: O(1) per edge
    pub fn add_oob_edges_from_graph_corner(
        &mut self,
        corner_point: Point,
        graph: &mut crate::visibility::graph::VisibilityGraph,
    ) {
        // C#: Direction dirs = PointComparer.GetDirections(cornerPoint, Vertex.Point);
        //     VisibilityVertex cornerVertex = transUtil.VisGraph.FindVertex(cornerPoint);
        //     transUtil.ConnectVertexToTargetVertex(cornerVertex, this.Vertex,
        //         dirs & (Direction.North | Direction.South), ScanSegment.NormalWeight);
        //     transUtil.ConnectVertexToTargetVertex(cornerVertex, this.Vertex,
        //         dirs & (Direction.East | Direction.West), ScanSegment.NormalWeight);
        use super::compass_direction::Direction;

        let self_vertex = match self.vertex {
            Some(v) => v,
            None => return,
        };

        let corner_vertex = match graph.find_vertex(corner_point) {
            Some(v) => v,
            None => return,
        };

        let vertex_point = graph.point(self_vertex);
        let dirs = Direction::from_point_to_point(corner_point, vertex_point);

        // Extract vertical component (N/S).
        let vertical = dirs & (Direction::NORTH | Direction::SOUTH);
        if vertical.is_pure() {
            // Connect corner to self via vertical direction.
            let cp = graph.point(corner_vertex);
            let vp = graph.point(self_vertex);
            if !(GeomConstants::close(cp.x(), vp.x()) && GeomConstants::close(cp.y(), vp.y())) {
                graph.add_edge(corner_vertex, self_vertex, 1.0);
            }
        }

        // Extract horizontal component (E/W).
        let horizontal = dirs & (Direction::EAST | Direction::WEST);
        if horizontal.is_pure() {
            let cp = graph.point(corner_vertex);
            let vp = graph.point(self_vertex);
            if !(GeomConstants::close(cp.x(), vp.x()) && GeomConstants::close(cp.y(), vp.y())) {
                if !vertical.is_pure() {
                    graph.add_edge(corner_vertex, self_vertex, 1.0);
                } else {
                    // Both components exist — need a bend vertex.
                    let bend_point = Point::new(cp.x(), vp.y());
                    let bend_vertex = graph.find_or_add_vertex(bend_point);
                    let bp = graph.point(bend_vertex);
                    if !(GeomConstants::close(cp.x(), bp.x()) && GeomConstants::close(cp.y(), bp.y())) {
                        graph.add_edge(corner_vertex, bend_vertex, 1.0);
                    }
                    if !(GeomConstants::close(bp.x(), vp.x()) && GeomConstants::close(bp.y(), vp.y())) {
                        graph.add_edge(bend_vertex, self_vertex, 1.0);
                    }
                }
            }
        }
    }

    /// Remove this free point from the visibility graph.
    ///
    /// C# file: FreePoint.cs, lines 121-124
    pub fn remove_from_graph(&mut self) {
        self.vertex = None;
    }
}
