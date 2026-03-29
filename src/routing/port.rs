use crate::geometry::curve::LineSegment;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;
use crate::visibility::graph::VertexId;
use super::compass_direction::CompassDirection;
use super::router_session::RouterSession;

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

    /// Typed `LineSegment` view of the max visibility segment.
    /// C# ObstaclePortEntrance: `public LineSegment MaxVisibilitySegment { get; private set; }`
    /// Set by `create_max_visibility_segment` via `ObstaclePort::create_port_entrance`.
    /// `None` until that method has been called.
    pub max_visibility_segment: Option<LineSegment>,

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
            max_visibility_segment: None,
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
        session: &mut RouterSession,
        trans_util: &mut crate::routing::transient_graph_utility::TransientGraphUtility,
        padded_border_vertex: VertexId,
        target_vertex: VertexId,
        limit_rect: &Rectangle,
        route_to_center: bool,
    ) {
        // C# line 169: transUtil.ExtendEdgeChain(targetVertex, limitRect,
        //     this.MaxVisibilitySegment, this.pointAndCrossingsList, this.IsOverlapped);
        trans_util.extend_edge_chain_public(
            session,
            target_vertex,
            limit_rect,
            self.max_visibility_segment_start,
            self.max_visibility_segment_end,
            None, // pointAndCrossingsList — no group crossings for rectangular obstacles
            self.is_overlapped,
        );

        // C# line 173-174: connect unpadded border to padded border.
        let unpadded_vertex = trans_util.find_or_add_vertex(session, self.unpadded_border_intersect);
        let weight = if self.is_overlapped { 100_000.0 } else { 1.0 };
        let up = session.vis_graph.point(unpadded_vertex);
        let pp = session.vis_graph.point(padded_border_vertex);
        if !(GeomConstants::close(up.x(), pp.x()) && GeomConstants::close(up.y(), pp.y())) {
            trans_util.find_or_add_edge(session, unpadded_vertex, padded_border_vertex, weight);
        }

        if route_to_center {
            // C# line 177: transUtil.ConnectVertexToTargetVertex(
            //     ObstaclePort.CenterVertex, unpaddedBorderVertex, OutwardDirection, InitialWeight);
            // The center vertex is on ObstaclePort, not accessible here.
            // The caller (port_manager) handles center-to-unpadded connection.
        }
    }

    /// Add edge to an adjacent vertex and extend chain.
    ///
    /// C# file: ObstaclePortEntrance.cs AddToAdjacentVertex
    /// Big-O: O(chain) for VG adjacency walk
    pub fn add_to_adjacent_vertex(
        &self,
        session: &mut RouterSession,
        trans_util: &mut crate::routing::transient_graph_utility::TransientGraphUtility,
        target_vertex: VertexId,
        limit_rect: &Rectangle,
        route_to_center: bool,
    ) {
        // C#: First check if a vertex already exists at VisibilityBorderIntersect.
        let border_vertex_existing = session.vis_graph.find_vertex(self.visibility_border_intersect);

        if let Some(bv) = border_vertex_existing {
            // Border vertex already in graph — just extend.
            self.extend_edge_chain(session, trans_util, bv, bv, limit_rect, route_to_center);
            return;
        }

        // Check if target is in the outward direction from visibility border intersect.
        let target_point = session.vis_graph.point(target_vertex);
        let dir_from_target = CompassDirection::from_points(target_point, self.visibility_border_intersect);

        let border_vertex;
        let mut vis_border = self.visibility_border_intersect;

        if dir_from_target == Some(self.outward_direction) {
            // Target is between obstacle and visibility border — collapse to target.
            vis_border = target_point;
            border_vertex = target_vertex;
        } else {
            // Create a new vertex at the visibility border intersect and connect to target.
            border_vertex = trans_util.find_or_add_vertex(session, self.visibility_border_intersect);
            let weight = if self.is_overlapped { 100_000.0 } else { 1.0 };
            // Connect border vertex to target vertex.
            let bp = session.vis_graph.point(border_vertex);
            let tp = session.vis_graph.point(target_vertex);
            if !(GeomConstants::close(bp.x(), tp.x()) && GeomConstants::close(bp.y(), tp.y())) {
                trans_util.find_or_add_edge(session, border_vertex, target_vertex, weight);
            }
        }

        let _ = vis_border; // used conceptually above
        self.extend_edge_chain(session, trans_util, border_vertex, target_vertex, limit_rect, route_to_center);
    }
}

/// A port associated with an obstacle, with pre-calculated visibility.
///
/// C# file: ObstaclePort.cs, lines 14-83
/// Holds the obstacle reference, location, center vertex, port entrances,
/// and a visibility rectangle bounding the max visibility segments.
///
/// # Approved deviations from C#/TS
///
/// - `Port` back-reference (C# line 15) is omitted. In C# `ObstaclePort` holds a
///   reference to the abstract `Port` object (which carries `Location` and `Curve`).
///   In Rust we use index-based arenas, so there is no GC-managed back-reference.
///   Instead we store the derived values directly:
///     - `port_location` — the unrounded location (`Port.Location`, C# line 75)
///     - `port_curve_bbox` — the bounding box of `Port.Curve` (C# line 70)
///   The rounded location is already stored in `location` (C# line 26).
#[derive(Clone, Debug)]
pub struct ObstaclePort {
    /// Index of the obstacle this port belongs to.
    /// C# ObstaclePort.cs line 16: Obstacle
    pub obstacle_index: usize,

    /// Rounded location of the port (used for visibility graph construction).
    /// C# ObstaclePort.cs line 26: Location
    /// `ApproximateComparer.Round(Port.Location)`
    pub location: Point,

    /// Unrounded location of the port.
    /// C# ObstaclePort.cs line 75: PortLocation — `Port.Location`
    /// Stored separately because `location` is rounded (C# line 34) and
    /// `LocationHasChanged` compares the rounded value against this.
    pub port_location: Point,

    /// Bounding box of the obstacle curve the port lives on.
    /// C# ObstaclePort.cs line 70: PortCurve — `Port.Curve` (an ICurve).
    /// In Rust, non-rectangular ICurves are not yet supported; we store the
    /// obstacle's bounding box as the closest equivalent.
    pub port_curve_bbox: Rectangle,

    /// Unpadded boundary polyline for non-rectangular port derivative computation.
    /// C# PortManager.cs GetDerivative: oport.PortCurve used for closest-point / Derivative.
    /// None for rectangular obstacles (derivative is always axis-aligned, never fires).
    pub port_curve_polyline: Option<Polyline>,

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
    ///
    /// `location` is the **rounded** port location (C# `ApproximateComparer.Round`).
    /// `port_location` is the original unrounded location (`Port.Location`).
    /// `port_curve_bbox` is the bounding box of the obstacle curve the port lives on.
    pub fn new(obstacle_index: usize, location: Point) -> Self {
        // When created with just a rounded location, port_location mirrors it.
        // Callers with an unrounded location should use new_with_curve() instead.
        Self {
            obstacle_index,
            location,
            port_location: location,
            port_curve_bbox: Rectangle::empty(),
            port_curve_polyline: None,
            center_vertex: None,
            port_entrances: Vec::new(),
            has_collinear_entrances: false,
            visibility_rectangle: Rectangle::empty(),
        }
    }

    /// Create a new ObstaclePort with an explicit unrounded port location and
    /// the obstacle bounding box as the port curve.
    ///
    /// C# file: ObstaclePort.cs, lines 30-35
    /// - `port_location` — `Port.Location` (unrounded)
    /// - `port_curve_bbox` — bounding box of `Port.Curve` (the obstacle's curve)
    ///
    /// The rounded `location` field is derived from `port_location` here,
    /// matching C# `ApproximateComparer.Round(Port.Location)` (line 34).
    pub fn new_with_curve(
        obstacle_index: usize,
        port_location: Point,
        port_curve_bbox: Rectangle,
    ) -> Self {
        let rounded = Point::new(
            GeomConstants::round(port_location.x()),
            GeomConstants::round(port_location.y()),
        );
        Self {
            obstacle_index,
            location: rounded,
            port_location,
            port_curve_bbox,
            port_curve_polyline: None,
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
        obstacle_tree: &mut super::obstacle_tree::ObstacleTree,
        graph_box: &Rectangle,
    ) {
        // C#: var entrance = new ObstaclePortEntrance(this, unpaddedBorderIntersect, outDir, obstacleTree);
        let mut entrance = ObstaclePortEntrance::new(unpadded_border_intersect, out_dir, obstacle_index);

        // C# constructor calls ObstacleTree.CreateMaxVisibilitySegment to compute
        // the max visibility segment from the unpadded border intersect outward.
        let (seg_start, seg_end, _pac_list) =
            obstacle_tree.create_max_visibility_segment(unpadded_border_intersect, out_dir, graph_box);
        entrance.max_visibility_segment_start = seg_start;
        entrance.max_visibility_segment_end = seg_end;
        entrance.max_visibility_segment = Some(LineSegment::new(seg_start, seg_end));

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

    /// Return a reference to the bounding box of the obstacle curve this port lives on.
    ///
    /// C#/TS API alias for `port_curve_bbox`.
    /// Mirrors C# `ObstaclePort.PortCurve` (line 70) and TS `portCurve`.
    #[inline]
    pub fn port_curve(&self) -> &Rectangle {
        &self.port_curve_bbox
    }

    /// Returns the obstacle index this port is associated with.
    ///
    /// Faithful port of C# ObstaclePort.Port property (ObstaclePort.cs).
    /// In C# this returns the `Port` object; in Rust we represent the port
    /// by its obstacle index since there is no separate Port wrapper class.
    pub fn port(&self) -> usize {
        self.obstacle_index
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

// FreePoint is in free_point.rs (split for file size).
// Re-export for backward compatibility.
pub use super::free_point::{FreePoint, SegmentAndCrossings};

