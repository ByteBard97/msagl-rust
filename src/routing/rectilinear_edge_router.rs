//! Top-level rectilinear edge router.
//!
//! Ties together the visibility graph, A* path search, port splicing,
//! nudging, and curve generation into a single public API.

use crate::geometry::curve::Curve;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::routing::edge_geometry::EdgeGeometry;
use crate::routing::nudging::nudge_paths;
use crate::routing::path_search::PathSearch;
use crate::routing::port_manager::PortManager;
use crate::routing::transient_graph_utility::TransientGraphUtility;
use crate::routing::shape::Shape;
use crate::routing::router_session::RouterSession;
use crate::routing::visibility_graph_generator::generate_visibility_graph;

/// Default padding around obstacles (pixels).
const DEFAULT_PADDING: f64 = 4.0;
/// Default separation between parallel edges.
const DEFAULT_EDGE_SEPARATION: f64 = 8.0;
/// Default bend penalty as a percentage of source-target Manhattan distance.
/// Matches `SsstRectilinearPath.DefaultBendPenaltyAsAPercentageOfDistance = 4` from the TS source.
const DEFAULT_BEND_PENALTY_PCT: f64 = 4.0;
/// Default corner arc radius (0 = sharp corners).
const DEFAULT_CORNER_FIT_RADIUS: f64 = 0.0;

/// Result of running the router.
pub struct RoutingResult {
    pub edges: Vec<RoutedEdge>,
}

/// A single routed edge: the bend points and a curve representation.
pub struct RoutedEdge {
    /// The rectilinear path waypoints (source, bends, target).
    pub points: Vec<Point>,
    /// Compound curve built from the waypoints.
    pub curve: Curve,
}

/// Builder + executor for rectilinear edge routing.
///
/// # Example
/// ```
/// use msagl_rust::{RectilinearEdgeRouter, Shape, FloatingPort, EdgeGeometry, Point};
///
/// let shapes = vec![
///     Shape::rectangle(0.0, 0.0, 50.0, 30.0),
///     Shape::rectangle(200.0, 0.0, 50.0, 30.0),
/// ];
/// let mut router = RectilinearEdgeRouter::new(&shapes).padding(4.0);
/// router.add_edge(EdgeGeometry::new(
///     FloatingPort::new(0, Point::new(54.0, 15.0)),
///     FloatingPort::new(1, Point::new(196.0, 15.0)),
/// ));
/// let result = router.run();
/// assert_eq!(result.edges.len(), 1);
/// ```
pub struct RectilinearEdgeRouter {
    shapes: Vec<Shape>,
    edges: Vec<EdgeGeometry>,
    padding: f64,
    edge_separation: f64,
    corner_fit_radius: f64,
    bend_penalty_as_percentage: f64,
}

impl RectilinearEdgeRouter {
    /// Create a router for the given obstacle shapes.
    pub fn new(shapes: &[Shape]) -> Self {
        Self {
            shapes: shapes.to_vec(),
            edges: Vec::new(),
            padding: DEFAULT_PADDING,
            edge_separation: DEFAULT_EDGE_SEPARATION,
            corner_fit_radius: DEFAULT_CORNER_FIT_RADIUS,
            bend_penalty_as_percentage: DEFAULT_BEND_PENALTY_PCT,
        }
    }

    /// Set obstacle padding (space between shape boundary and routes).
    pub fn padding(mut self, p: f64) -> Self {
        self.padding = p;
        self
    }

    /// Set minimum separation between parallel edge segments.
    pub fn edge_separation(mut self, s: f64) -> Self {
        self.edge_separation = s;
        self
    }

    /// Set corner arc radius. 0 = sharp 90-degree corners.
    pub fn corner_fit_radius(mut self, r: f64) -> Self {
        self.corner_fit_radius = r;
        self
    }

    /// Set bend penalty as a percentage of average obstacle size.
    pub fn bend_penalty_as_percentage(mut self, p: f64) -> Self {
        self.bend_penalty_as_percentage = p;
        self
    }

    /// Add an edge to be routed.
    pub fn add_edge(&mut self, edge: EdgeGeometry) {
        self.edges.push(edge);
    }

    /// Run the full routing pipeline: visibility graph, path search,
    /// nudging, and curve generation.
    pub fn run(&mut self) -> RoutingResult {
        if self.edges.is_empty() {
            return RoutingResult { edges: Vec::new() };
        }

        // 1. Build visibility graph from shapes (H/V scan segment trees kept on session)
        let mut session = generate_visibility_graph(&self.shapes, self.padding);

        // 2. Route each edge: splice ports -> A* -> unsplice
        let search = PathSearch::new(self.bend_penalty_as_percentage);
        let mut paths: Vec<Vec<Point>> = Vec::new();

        let graph_box = session.obstacle_tree.graph_box();

        // Phase 2: Route each edge with a single TGU shared by entrance creation and port
        // splicing. This matches the C# design where one TGU per edge-routing call owns all
        // transient VG modifications; remove_from_graph at the end restores the graph cleanly.
        for edge in &self.edges {
            let mut tgu = TransientGraphUtility::new();

            // Phase 2a: Create port entrances at obstacle boundaries into the shared TGU.
            Self::create_port_entrances_for_location(
                edge.source.location, &self.shapes, self.padding,
                &mut session, &mut tgu, &graph_box,
            );
            Self::create_port_entrances_for_location(
                edge.target.location, &self.shapes, self.padding,
                &mut session, &mut tgu, &graph_box,
            );

            // Phase 2b: Splice ports using the SAME TGU so all modifications are consistent.
            PortManager::splice_port_into(&mut session, edge.source.location, &mut tgu);
            PortManager::splice_port_into(&mut session, edge.target.location, &mut tgu);

            let path = search.find_path(&session.vis_graph, edge.source.location, edge.target.location);
            paths.push(path.unwrap_or_else(|| {
                debug_assert!(false, "path search failed for {:?} -> {:?}",
                    edge.source.location, edge.target.location);
                vec![edge.source.location, edge.target.location]
            }));

            // Phase 2c: Remove ALL modifications (entrances + splices) atomically.
            tgu.remove_from_graph(&mut session);
        }

        // 3. Nudge paths for edge separation
        let obstacles = self.padded_obstacles();
        nudge_paths(&mut paths, &obstacles, self.edge_separation);

        // 4. Convert to curves
        let edges = paths
            .into_iter()
            .map(|pts| {
                let curve = self.points_to_curve(&pts);
                RoutedEdge { points: pts, curve }
            })
            .collect();

        RoutingResult { edges }
    }

    /// Create port entrances at obstacle boundaries for a port location.
    ///
    /// If the location is inside an obstacle (center port), creates entrances
    /// at the boundary intersections and extends edge chains outward.
    /// This matches C#: PortManager.CreateObstaclePortEntrancesFromPoints +
    /// AddObstaclePortEntranceToGraph.
    fn create_port_entrances_for_location(
        location: Point,
        shapes: &[Shape],
        padding: f64,
        session: &mut crate::routing::router_session::RouterSession,
        tgu: &mut TransientGraphUtility,
        graph_box: &Rectangle,
    ) {
        use crate::routing::compass_direction::CompassDirection;
        use crate::geometry::point_comparer::GeomConstants;

        // Find which obstacle contains this port location.
        let containing_obs = session.obstacle_tree.obstacles.iter().enumerate().find(|(_, obs)| {
            let bb = obs.padded_bounding_box();
            location.x() > bb.left() + GeomConstants::DISTANCE_EPSILON
                && location.x() < bb.right() - GeomConstants::DISTANCE_EPSILON
                && location.y() > bb.bottom() + GeomConstants::DISTANCE_EPSILON
                && location.y() < bb.top() - GeomConstants::DISTANCE_EPSILON
        });

        let (_obs_idx, obs) = match containing_obs {
            Some((i, o)) => (i, o),
            None => return, // Port is not inside any obstacle — splice_port handles it
        };

        let padded_box = obs.padded_bounding_box();
        let rounded = Point::round(location);

        // Create entrances at boundary intersections (matches C# CreateObstaclePortEntrancesFromPoints).
        // For rectangular obstacles: project port location to each boundary edge.
        let mut entrances = Vec::new();

        // Horizontal pass: if port is not on top/bottom boundary, create E and W entrances
        if !GeomConstants::close(rounded.y(), padded_box.top())
            && !GeomConstants::close(rounded.y(), padded_box.bottom())
        {
            let w_border = Point::new(padded_box.left(), rounded.y());
            let e_border = Point::new(padded_box.right(), rounded.y());

            if !w_border.close_to(rounded) {
                entrances.push((e_border, CompassDirection::East));
            }
            if !e_border.close_to(rounded) {
                entrances.push((w_border, CompassDirection::West));
            }
        }

        // Vertical pass: if port is not on left/right boundary, create N and S entrances
        if !GeomConstants::close(rounded.x(), padded_box.left())
            && !GeomConstants::close(rounded.x(), padded_box.right())
        {
            let s_border = Point::new(rounded.x(), padded_box.bottom());
            let n_border = Point::new(rounded.x(), padded_box.top());

            if !s_border.close_to(rounded) {
                entrances.push((n_border, CompassDirection::North));
            }
            if !n_border.close_to(rounded) {
                entrances.push((s_border, CompassDirection::South));
            }
        }

        // For each entrance: add vertex at boundary, connect to perpendicular VG neighbors,
        // then extend outward via max visibility segment.
        //
        // Perpendicular connections are needed because the VG generator only creates scan
        // segments at obstacle boundary Y-coordinates; it does NOT create edges along obstacle
        // left/right sides. A boundary vertex like (61,10) is therefore isolated until we
        // connect it to the nearest live VG vertex in each perpendicular direction on the
        // same axis. This is safe here because we are inside a single per-edge TGU — any
        // edge splits done here and by splice_port_into below all belong to the same TGU and
        // are removed atomically by tgu.remove_from_graph at the end of each edge.
        for (border_point, out_dir) in entrances {
            let border_vertex = tgu.find_or_add_vertex(session, border_point);

            // Connect the boundary vertex to nearest live VG vertices in each perpendicular
            // direction so that splice_port_into can find a crossing edge from the port center.
            let (perp_a, perp_b) = match out_dir {
                CompassDirection::East | CompassDirection::West =>
                    (CompassDirection::North, CompassDirection::South),
                CompassDirection::North | CompassDirection::South =>
                    (CompassDirection::East, CompassDirection::West),
            };
            for &pdir in &[perp_a, perp_b] {
                if let Some(nbr) = session.vis_graph.find_nearest_vertex_in_direction(border_point, pdir) {
                    if nbr == border_vertex {
                        continue;
                    }
                    let nbr_pt = session.vis_graph.point(nbr);
                    let on_axis = match pdir {
                        CompassDirection::North | CompassDirection::South =>
                            GeomConstants::close(nbr_pt.x(), border_point.x()),
                        CompassDirection::East | CompassDirection::West =>
                            GeomConstants::close(nbr_pt.y(), border_point.y()),
                    };
                    if on_axis && session.vis_graph.degree(nbr) > 0 {
                        let dist = ((nbr_pt.x() - border_point.x()).powi(2)
                            + (nbr_pt.y() - border_point.y()).powi(2))
                        .sqrt();
                        tgu.find_or_add_edge(session, border_vertex, nbr, dist);
                    }
                }
            }

            // Compute max visibility segment and extend outward from boundary vertex.
            let (seg_start, seg_end, _pac) =
                session.obstacle_tree.create_max_visibility_segment(border_point, out_dir, graph_box);
            if !seg_start.close_to(seg_end) {
                tgu.extend_edge_chain_public(
                    session, border_vertex, graph_box,
                    seg_start, seg_end, None, false,
                );
            }
        }
    }

    /// Build padded obstacle rectangles for nudging.
    fn padded_obstacles(&self) -> Vec<Rectangle> {
        self.shapes
            .iter()
            .map(|s| {
                let mut r = *s.bounding_box();
                r.pad(self.padding);
                r
            })
            .collect()
    }

    /// Convert a path (list of waypoints) into a Curve.
    ///
    /// If `corner_fit_radius > 0`, inserts arcs at bends. Otherwise
    /// produces straight line segments.
    fn points_to_curve(&self, points: &[Point]) -> Curve {
        let mut curve = Curve::new();
        if points.len() < 2 {
            return curve;
        }

        if self.corner_fit_radius <= 0.0 || points.len() < 3 {
            // Simple polyline
            for pair in points.windows(2) {
                curve.add_line(pair[0], pair[1]);
            }
            return curve;
        }

        // With corner rounding: shorten segments and insert arcs at bends
        let r = self.corner_fit_radius;

        // First segment start
        let mut prev = points[0];

        for i in 1..points.len() - 1 {
            let curr = points[i];
            let next = points[i + 1];

            // Distance from curr back to prev and forward to next
            let dist_back = Self::axis_distance(prev, curr);
            let dist_fwd = Self::axis_distance(curr, next);
            let fit = r.min(dist_back / 2.0).min(dist_fwd / 2.0);

            if fit < 1e-6 {
                // Too tight for an arc, just use lines
                curve.add_line(prev, curr);
                prev = curr;
                continue;
            }

            // Points where the arc starts and ends
            let arc_start = Self::lerp_toward(curr, prev, fit);
            let arc_end = Self::lerp_toward(curr, next, fit);

            // Line from previous to arc start
            curve.add_line(prev, arc_start);

            // Arc at the bend
            let ccw = Self::is_ccw_turn(prev, curr, next);
            curve.add_arc(arc_start, arc_end, curr, ccw);

            prev = arc_end;
        }

        // Final segment
        curve.add_line(prev, *points.last().unwrap());
        curve
    }

    /// Manhattan (axis-aligned) distance between two points.
    fn axis_distance(a: Point, b: Point) -> f64 {
        (b.x() - a.x()).abs() + (b.y() - a.y()).abs()
    }

    /// Move from `from` toward `toward` by `dist` along the axis.
    fn lerp_toward(from: Point, toward: Point, dist: f64) -> Point {
        let dx = toward.x() - from.x();
        let dy = toward.y() - from.y();
        let len = dx.abs() + dy.abs();
        if len < 1e-12 {
            return from;
        }
        let t = dist / len;
        Point::new(from.x() + dx * t, from.y() + dy * t)
    }

    /// Determine if the turn from a->b->c is counter-clockwise.
    fn is_ccw_turn(a: Point, b: Point, c: Point) -> bool {
        let cross = (b.x() - a.x()) * (c.y() - b.y()) - (b.y() - a.y()) * (c.x() - b.x());
        cross > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::curve::CurveSegment;

    #[test]
    fn points_to_curve_straight_line() {
        let router = RectilinearEdgeRouter::new(&[]);
        let pts = vec![Point::new(0.0, 0.0), Point::new(100.0, 0.0)];
        let curve = router.points_to_curve(&pts);
        assert_eq!(curve.segment_count(), 1);
    }

    #[test]
    fn points_to_curve_with_bend_no_radius() {
        let router = RectilinearEdgeRouter::new(&[]);
        let pts = vec![
            Point::new(0.0, 0.0),
            Point::new(50.0, 0.0),
            Point::new(50.0, 50.0),
        ];
        let curve = router.points_to_curve(&pts);
        assert_eq!(curve.segment_count(), 2);
    }

    #[test]
    fn points_to_curve_with_corner_radius() {
        let router = RectilinearEdgeRouter::new(&[]).corner_fit_radius(5.0);
        let pts = vec![
            Point::new(0.0, 0.0),
            Point::new(50.0, 0.0),
            Point::new(50.0, 50.0),
        ];
        let curve = router.points_to_curve(&pts);
        // Should be: line + arc + line = 3 segments
        assert_eq!(curve.segment_count(), 3);
        // Check the arc is present
        let segs: Vec<_> = curve.segments().collect();
        assert!(matches!(segs[1], CurveSegment::Arc { .. }));
    }

    #[test]
    fn empty_router_returns_empty_result() {
        let mut router = RectilinearEdgeRouter::new(&[Shape::rectangle(0.0, 0.0, 10.0, 10.0)]);
        let result = router.run();
        assert!(result.edges.is_empty());
    }

    #[test]
    fn bend_penalty_as_percentage_can_be_configured() {
        let router = RectilinearEdgeRouter::new(&[Shape::rectangle(0.0, 0.0, 50.0, 50.0)])
            .bend_penalty_as_percentage(8.0);
        assert!((router.bend_penalty_as_percentage - 8.0).abs() < 1e-10);
    }
}
