use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::visibility::graph::VertexId;
use super::compass_direction::CompassDirection;

/// A floating port attached to an obstacle.
///
/// Represents a connection point on an obstacle's boundary where an edge
/// can originate or terminate.
#[derive(Clone, Debug)]
pub struct FloatingPort {
    /// Index of the obstacle this port belongs to.
    pub obstacle_index: usize,
    /// Location of the port in 2D space.
    pub location: Point,
}

impl FloatingPort {
    pub fn new(obstacle_index: usize, location: Point) -> Self {
        Self {
            obstacle_index,
            location,
        }
    }
}

// =========================================================================
// ObstaclePort — C# ObstaclePort.cs (83 lines)
//
// A port associated with an obstacle. Tracks the obstacle, location,
// port entrances (border intersections), center vertex, and visibility
// rectangle for the pre-calculated visibility segments.
// =========================================================================

/// An entrance point on the obstacle boundary where visibility edges connect.
///
/// C# file: ObstaclePortEntrance.cs (referenced from ObstaclePort.cs line 21)
/// Each entrance has a direction (outward from the obstacle), the unpadded
/// border intersection point, and a max visibility segment.
#[derive(Clone, Debug)]
pub struct ObstaclePortEntrance {
    /// The unpadded border intersection point.
    /// C# ObstaclePortEntrance: UnpaddedBorderIntersect
    pub unpadded_border_intersect: Point,

    /// The padded border intersection (further out by padding distance).
    /// C# ObstaclePortEntrance: VisibilityBorderIntersect
    pub visibility_border_intersect: Point,

    /// Outward direction from the obstacle at this entrance.
    /// C# ObstaclePortEntrance: OutwardDirection
    pub outward_direction: CompassDirection,

    /// The maximum visibility segment from this entrance.
    /// C# ObstaclePortEntrance: MaxVisibilitySegment (LineSegment)
    pub max_visibility_segment_start: Point,
    pub max_visibility_segment_end: Point,

    /// Whether this entrance is collinear with the port location.
    /// C# ObstaclePortEntrance: IsCollinearWithPort
    pub is_collinear_with_port: bool,

    /// Whether this entrance is on a vertical border.
    /// C# ObstaclePortEntrance: IsVertical
    pub is_vertical: bool,

    /// Whether this entrance is overlapped (inside another obstacle).
    /// C# ObstaclePortEntrance: IsOverlapped
    pub is_overlapped: bool,
}

impl ObstaclePortEntrance {
    /// Create a new port entrance.
    ///
    /// C# file: ObstaclePortEntrance.cs constructor
    /// Big-O: O(log N) for obstacle tree max visibility segment creation
    /// MUST use ObstacleTree R-tree for segment restriction
    pub fn new(
        unpadded_border_intersect: Point,
        outward_direction: CompassDirection,
        _obstacle_index: usize,
    ) -> Self {
        // For rectangular obstacles without groups, we set up the entrance
        // with the unpadded point as both the unpadded and visibility border intersect.
        // The caller (port manager) is responsible for computing the padded border
        // intersect and max visibility segment using the obstacle tree.
        //
        // This is a simplified constructor; the full C# constructor uses
        // ObstacleTree.CreateMaxVisibilitySegment which requires the obstacle tree
        // to be passed in. Since the skeleton doesn't pass the obstacle tree here,
        // we initialize with the unpadded point and expect the caller to set
        // visibility_border_intersect and max_visibility_segment after construction.
        let is_vertical = matches!(outward_direction, CompassDirection::North | CompassDirection::South);

        Self {
            unpadded_border_intersect,
            visibility_border_intersect: unpadded_border_intersect,
            outward_direction,
            max_visibility_segment_start: unpadded_border_intersect,
            max_visibility_segment_end: unpadded_border_intersect,
            is_collinear_with_port: false,
            is_vertical,
            is_overlapped: false,
        }
    }

    /// Whether this entrance wants visibility intersection testing.
    ///
    /// C# ObstaclePortEntrance: WantVisibilityIntersection
    pub fn want_visibility_intersection(&self) -> bool {
        // C#: !this.IsOverlapped && this.CanExtend &&
        //     (!this.ObstaclePort.HasCollinearEntrances || this.IsCollinearWithPort)
        // CanExtend checks that start != end of max visibility segment.
        // We don't have HasCollinearEntrances here (it's on ObstaclePort), so we
        // return the non-collinear-dependent part. The caller should additionally
        // check has_collinear_entrances on the ObstaclePort if needed.
        let can_extend = {
            let dx = self.max_visibility_segment_start.x() - self.max_visibility_segment_end.x();
            let dy = self.max_visibility_segment_start.y() - self.max_visibility_segment_end.y();
            dx.abs() > 1e-6 || dy.abs() > 1e-6
        };
        !self.is_overlapped && can_extend
    }

    /// Whether this entrance has group crossings before the given point.
    ///
    /// C# ObstaclePortEntrance: HasGroupCrossingBeforePoint
    pub fn has_group_crossing_before_point(&self, _point: Point) -> bool {
        // Groups are not supported for rectangular routing — always false.
        false
    }

    /// Whether this entrance has any group crossings.
    ///
    /// C# ObstaclePortEntrance: HasGroupCrossings
    pub fn has_group_crossings(&self) -> bool {
        // Groups are not supported for rectangular routing — always false.
        false
    }

    /// Extend the edge chain from this entrance into the visibility graph.
    ///
    /// C# file: ObstaclePortEntrance.cs ExtendEdgeChain
    /// Big-O: O(chain) for VG adjacency walk
    pub fn extend_edge_chain(
        &self,
        graph: &mut crate::visibility::graph::VisibilityGraph,
        padded_border_vertex: VertexId,
        _target_vertex: VertexId,
        _limit_rect: &Rectangle,
        route_to_center: bool,
    ) {
        // C#: transUtil.ExtendEdgeChain(targetVertex, limitRect, MaxVisibilitySegment,
        //         pointAndCrossingsList, IsOverlapped);
        // TODO: full extend_edge_chain — walk from targetVertex along max visibility
        // segment adding edges to existing vertices. For now, this is a stub that
        // connects the unpadded border vertex to the padded border vertex.

        let unpadded_vertex = graph.find_or_add_vertex(self.unpadded_border_intersect);
        let weight = if self.is_overlapped { 100_000.0 } else { 1.0 };

        // Connect unpadded border to padded border.
        let up = graph.point(unpadded_vertex);
        let pp = graph.point(padded_border_vertex);
        if !(GeomConstants::close(up.x(), pp.x()) && GeomConstants::close(up.y(), pp.y())) {
            graph.add_edge(unpadded_vertex, padded_border_vertex, weight);
        }

        if route_to_center {
            // Connect center vertex to unpadded border vertex.
            // The center vertex should have been set on the ObstaclePort before calling this.
            // We cannot access ObstaclePort.center_vertex from here, so the caller
            // (port_manager) must handle center-to-unpadded connection separately.
            // This matches the C# pattern where ConnectVertexToTargetVertex is called
            // with ObstaclePort.CenterVertex.
        }
    }

    /// Add edge to an adjacent vertex and extend chain.
    ///
    /// C# file: ObstaclePortEntrance.cs AddToAdjacentVertex
    /// Big-O: O(chain) for VG adjacency walk
    pub fn add_to_adjacent_vertex(
        &self,
        graph: &mut crate::visibility::graph::VisibilityGraph,
        target_vertex: VertexId,
        limit_rect: &Rectangle,
        route_to_center: bool,
    ) {
        // C#: First check if a vertex already exists at VisibilityBorderIntersect.
        let border_vertex_existing = graph.find_vertex(self.visibility_border_intersect);

        if let Some(bv) = border_vertex_existing {
            // Border vertex already in graph — just extend.
            self.extend_edge_chain(graph, bv, bv, limit_rect, route_to_center);
            return;
        }

        // Check if target is in the outward direction from visibility border intersect.
        let target_point = graph.point(target_vertex);
        let dir_from_target = CompassDirection::from_points(target_point, self.visibility_border_intersect);

        let border_vertex;
        let mut vis_border = self.visibility_border_intersect;

        if dir_from_target == Some(self.outward_direction) {
            // Target is between obstacle and visibility border — collapse to target.
            vis_border = target_point;
            border_vertex = target_vertex;
        } else {
            // Create a new vertex at the visibility border intersect and connect to target.
            border_vertex = graph.find_or_add_vertex(self.visibility_border_intersect);
            let weight = if self.is_overlapped { 100_000.0 } else { 1.0 };
            // Connect border vertex to target vertex.
            let bp = graph.point(border_vertex);
            let tp = graph.point(target_vertex);
            if !(GeomConstants::close(bp.x(), tp.x()) && GeomConstants::close(bp.y(), tp.y())) {
                graph.add_edge(border_vertex, target_vertex, weight);
            }
        }

        let _ = vis_border; // used conceptually above
        self.extend_edge_chain(graph, border_vertex, target_vertex, limit_rect, route_to_center);
    }
}

/// A port associated with an obstacle, with pre-calculated visibility.
///
/// C# file: ObstaclePort.cs, lines 14-83
/// Holds the obstacle reference, location, center vertex, port entrances,
/// and a visibility rectangle bounding the max visibility segments.
#[derive(Clone, Debug)]
pub struct ObstaclePort {
    /// Index of the obstacle this port belongs to.
    /// C# ObstaclePort.cs line 16: Obstacle
    pub obstacle_index: usize,

    /// Rounded location of the port.
    /// C# ObstaclePort.cs line 26: Location
    pub location: Point,

    /// Visibility vertex at the port center (set when route_to_center is true).
    /// C# ObstaclePort.cs line 18: CenterVertex
    pub center_vertex: Option<VertexId>,

    /// Port entrances on the obstacle boundary.
    /// C# ObstaclePort.cs line 21: PortEntrances
    pub port_entrances: Vec<ObstaclePortEntrance>,

    /// Whether any entrance is collinear with the port location.
    /// C# ObstaclePort.cs line 23: HasCollinearEntrances
    pub has_collinear_entrances: bool,

    /// Rectangle bounding the max visibility segments of all entrances.
    /// C# ObstaclePort.cs line 28: VisibilityRectangle
    pub visibility_rectangle: Rectangle,
}

impl ObstaclePort {
    /// Create a new ObstaclePort for a port on an obstacle.
    ///
    /// C# file: ObstaclePort.cs, lines 30-35
    /// Big-O: O(1)
    pub fn new(obstacle_index: usize, location: Point) -> Self {
        Self {
            obstacle_index,
            location,
            center_vertex: None,
            port_entrances: Vec::new(),
            has_collinear_entrances: false,
            visibility_rectangle: Rectangle::empty(),
        }
    }

    /// Create a port entrance at the given border intersection.
    ///
    /// C# file: ObstaclePort.cs, lines 37-46
    /// Big-O: O(log N) for obstacle tree max visibility segment
    /// MUST use ObstacleTree for visibility segment restriction
    pub fn create_port_entrance(
        &mut self,
        unpadded_border_intersect: Point,
        out_dir: CompassDirection,
        obstacle_index: usize,
    ) {
        // C#: var entrance = new ObstaclePortEntrance(this, unpaddedBorderIntersect, outDir, obstacleTree);
        let mut entrance = ObstaclePortEntrance::new(unpadded_border_intersect, out_dir, obstacle_index);

        // Check if this entrance is collinear with the port location.
        // C#: CompassVector.IsPureDirection(PointComparer.GetDirections(VisibilityBorderIntersect, ObstaclePort.Location))
        let dir = super::compass_direction::Direction::from_point_to_point(
            entrance.visibility_border_intersect,
            self.location,
        );
        entrance.is_collinear_with_port = dir.is_pure();

        // C#: PortEntrances.Add(entrance);
        self.port_entrances.push(entrance.clone());

        // C#: this.VisibilityRectangle.Add(entrance.MaxVisibilitySegment.End);
        self.visibility_rectangle.add_point(entrance.max_visibility_segment_end);

        // C#: this.HasCollinearEntrances |= entrance.IsCollinearWithPort;
        self.has_collinear_entrances |= entrance.is_collinear_with_port;
    }

    /// Clear pre-calculated visibility data (port entrances).
    ///
    /// C# file: ObstaclePort.cs, lines 48-51
    pub fn clear_visibility(&mut self) {
        self.port_entrances.clear();
    }

    /// Add this port to the visibility graph.
    ///
    /// C# file: ObstaclePort.cs, lines 53-58
    /// Big-O: O(log V) for vertex find/add
    pub fn add_to_graph(
        &mut self,
        graph: &mut crate::visibility::graph::VisibilityGraph,
        route_to_center: bool,
    ) {
        // C#: if (routeToCenter) { CenterVertex = transUtil.FindOrAddVertex(this.Location); }
        if route_to_center {
            let vid = graph.find_or_add_vertex(self.location);
            self.center_vertex = Some(vid);
        }
    }

    /// Remove this port from the visibility graph.
    ///
    /// C# file: ObstaclePort.cs, lines 60-62
    pub fn remove_from_graph(&mut self) {
        self.center_vertex = None;
    }

    /// Check if the port location has changed (requiring recreation).
    ///
    /// C# file: ObstaclePort.cs, line 65
    pub fn location_has_changed(&self, current_port_location: Point) -> bool {
        // C#: !PointComparer.Equal(this.Location, ApproximateComparer.Round(this.Port.Location))
        let rounded = Point::new(
            GeomConstants::round(current_port_location.x()),
            GeomConstants::round(current_port_location.y()),
        );
        !(GeomConstants::close(self.location.x(), rounded.x())
            && GeomConstants::close(self.location.y(), rounded.y()))
    }
}

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
    pub fn new(point: Point, graph: &mut crate::visibility::graph::VisibilityGraph) -> Self {
        // C#: OutOfBoundsDirectionFromGraph = Direction.None; this.GetVertex(transUtil, point);
        let vertex = graph.find_or_add_vertex(point);
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
    pub fn get_vertex(&mut self, graph: &mut crate::visibility::graph::VisibilityGraph) {
        // C#: this.Vertex = transUtil.FindOrAddVertex(point);
        let vertex = graph.find_or_add_vertex(self.point);
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
        graph: &mut crate::visibility::graph::VisibilityGraph,
    ) -> Option<VertexId> {
        // C#: Point targetIntersect = StaticGraphUtility.SegmentIntersection(targetEdge, this.Point);
        // SegmentIntersection finds the perpendicular projection of this.Point onto the edge.
        // For rectilinear edges, this is the point on the edge sharing the appropriate coordinate.
        let self_vertex = self.vertex?;
        let edge_src_point = graph.point(target_edge.0);
        let edge_tgt_point = graph.point(target_edge.1);

        // For axis-aligned edges, the intersection is the projection of self.point onto the edge.
        let target_intersect = if GeomConstants::close(edge_src_point.x(), edge_tgt_point.x()) {
            // Vertical edge — intersect at (edge.x, self.y)
            Point::new(edge_src_point.x(), self.point.y())
        } else {
            // Horizontal edge — intersect at (self.x, edge.y)
            Point::new(self.point.x(), edge_src_point.y())
        };

        // C#: VisibilityVertex targetVertex = transUtil.VisGraph.FindVertex(targetIntersect);
        let target_vertex = graph.find_vertex(target_intersect);

        if let Some(tv) = target_vertex {
            // Vertex already exists — add edge to it and extend.
            self.add_to_adjacent_vertex(tv, dir_to_extend, limit_rect, graph);
            self.extend_edge_chain(tv, dir_to_extend, limit_rect, graph);
            return Some(tv);
        }

        // C#: targetVertex = transUtil.AddEdgeToTargetEdge(this.Vertex, targetEdge, targetIntersect);
        // Split the target edge at the intersection point and add an edge from self to it.
        let new_vertex = graph.find_or_add_vertex(target_intersect);
        // We need to split the target edge. Add the new vertex between the edge endpoints.
        // For simplicity, add edges from new_vertex to both edge endpoints, remove original.
        // But since we don't have TransientGraphUtility here, just add edges directly.
        let w = self.initial_weight();
        let sp = graph.point(self_vertex);
        let tp = graph.point(new_vertex);
        if !(GeomConstants::close(sp.x(), tp.x()) && GeomConstants::close(sp.y(), tp.y())) {
            graph.add_edge(self_vertex, new_vertex, w);
        }

        // C#: ExtendEdgeChain(transUtil, targetVertex, dirToExtend, limitRect);
        self.extend_edge_chain(new_vertex, dir_to_extend, limit_rect, graph);
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
        graph: &mut crate::visibility::graph::VisibilityGraph,
    ) {
        // C#: if (!PointComparer.Equal(this.Point, targetVertex.Point)) {
        //         transUtil.FindOrAddEdge(this.Vertex, targetVertex, InitialWeight);
        //     }
        //     ExtendEdgeChain(transUtil, targetVertex, dirToExtend, limitRect);
        if let Some(self_vertex) = self.vertex {
            let tp = graph.point(target_vertex);
            if !(GeomConstants::close(self.point.x(), tp.x())
                && GeomConstants::close(self.point.y(), tp.y()))
            {
                let weight = self.initial_weight();
                graph.add_edge(self_vertex, target_vertex, weight);
            }
        }
        self.extend_edge_chain(target_vertex, dir_to_extend, limit_rect, graph);
    }

    /// Extend the edge chain from a target vertex in the given direction.
    ///
    /// C# file: FreePoint.cs, lines 68-86
    /// Big-O: O(chain) for VG adjacency walk + O(log N) for obstacle tree
    /// MUST use ObstacleTree for CreateMaxVisibilitySegment
    pub fn extend_edge_chain(
        &mut self,
        _target_vertex: VertexId,
        _dir_to_extend: CompassDirection,
        _limit_rect: &Rectangle,
        _graph: &mut crate::visibility::graph::VisibilityGraph,
    ) {
        // C#: var extendOverlapped = IsOverlapped;
        //     if (extendOverlapped) {
        //         extendOverlapped = transUtil.ObstacleTree.PointIsInsideAnObstacle(targetVertex.Point, dirToExtend);
        //     }
        //     SegmentAndCrossings segmentAndCrossings = GetSegmentAndCrossings(
        //         this.IsOverlapped ? targetVertex : this.Vertex, dirToExtend, transUtil);
        //     transUtil.ExtendEdgeChain(targetVertex, limitRect, segmentAndCrossings.Item1,
        //         segmentAndCrossings.Item2, extendOverlapped);

        // TODO: full extend_edge_chain — requires ObstacleTree.CreateMaxVisibilitySegment
        // and TransientGraphUtility.ExtendEdgeChain which are not yet implemented.
        // This is a stub that keeps the code compiling. The full implementation would:
        // 1. Check if the point is inside an obstacle (for overlap extension)
        // 2. Get or create the max visibility segment for the direction
        // 3. Walk the segment adding edges to existing VG vertices along the way
    }

    /// Get max visibility endpoint in a direction for non-overlapped free points.
    ///
    /// C# file: FreePoint.cs, lines 106-110
    /// Big-O: O(log N) for obstacle tree if not cached
    pub fn max_visibility_in_direction(
        &mut self,
        dir_to_extend: CompassDirection,
    ) -> Point {
        // C#: SegmentAndCrossings segmentAndCrossings = GetSegmentAndCrossings(this.Vertex, dirToExtend, transUtil);
        //     return segmentAndCrossings.Item1.End;
        let dir_index = dir_to_extend.index();
        if let Some(ref sac) = self.max_visibility_segments[dir_index] {
            return sac.segment_end;
        }
        // If not cached, return self.point as a fallback.
        // The full implementation would call ObstacleTree.CreateMaxVisibilitySegment here.
        // TODO: integrate with ObstacleTree for proper max visibility segment creation.
        self.point
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
            // For a simple connection: if they share an x coordinate, one edge;
            // otherwise a bend is needed. Use direct edge for now.
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
                // Only add if we didn't already add an edge above
                // (which would be the case if both vertical and horizontal are pure,
                // meaning diagonal — we need a bend vertex in between).
                if !vertical.is_pure() {
                    graph.add_edge(corner_vertex, self_vertex, 1.0);
                } else {
                    // Both components exist — need a bend vertex.
                    // Create bend at (corner.x, self.y) or (self.x, corner.y).
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
