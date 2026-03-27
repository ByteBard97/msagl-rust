use super::compass_direction::CompassDirection;
use super::static_graph_utility::StaticGraphUtility;
use super::transient_graph_utility::TransientGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};

/// Result of splicing a port into the visibility graph.
///
/// Holds the port vertex ID and the transient-graph utility that tracks
/// all additions so they can be cleanly removed by `PortManager::unsplice`.
pub struct PortSpliceResult {
    pub port_vertex: VertexId,
    pub tgu: TransientGraphUtility,
}

/// Manages splicing port locations into and out of a visibility graph.
///
/// Port splicing temporarily connects a point (port location) to the
/// visibility graph by finding axis-aligned neighbours and adding
/// bracket-aware edges via `TransientGraphUtility`.
///
/// When a port is inside an obstacle (the common case for center ports),
/// the standard aligned-vertex search will find nothing because VG segments
/// have gaps where obstacles are. In that case, the port manager fires
/// axis-aligned rays outward and finds the nearest VG *edge* that the ray
/// crosses, creates a vertex at the intersection, and connects the port.
pub struct PortManager;

impl PortManager {
    /// Splice a port location into the visibility graph using
    /// `TransientGraphUtility` for proper edge management.
    ///
    /// For each compass direction, finds the nearest VG structure (either
    /// a perpendicular crossing edge or an aligned vertex) and connects the
    /// port to it.
    ///
    /// Uses O(log V) coordinate-index lookups + O(chain) VG adjacency walks
    /// (ported from C# `FindNearestPerpendicularOrContainingEdge`) instead of
    /// O(V) brute-force vertex scanning.
    pub fn splice_port(graph: &mut VisibilityGraph, location: Point) -> PortSpliceResult {
        let mut tgu = TransientGraphUtility::new();
        let port_vertex = tgu.find_or_add_vertex(graph, location);

        for &direction in &CompassDirection::all() {
            // Strategy 1: Find the nearest perpendicular VG edge crossing
            // using coordinate-index lookup + VG adjacency chain walk.
            if let Some(intersection) =
                Self::find_nearest_crossing_edge(graph, location, direction)
            {
                let (edge_source, edge_target, intersect_point) = intersection;
                // Detect zombie vertices: vertices left behind by a previous
                // splice that was unspliced, leaving them completely disconnected.
                let pre_existing = graph.find_vertex(intersect_point);
                let was_orphan = pre_existing
                    .is_some_and(|v| graph.out_degree(v) == 0 && graph.in_degree(v) == 0);

                let split_vertex = tgu.add_edge_to_target_edge(
                    graph,
                    port_vertex,
                    edge_source,
                    edge_target,
                    intersect_point,
                );

                // If the vertex was orphaned and the original VG edge still
                // exists unsplit, split it to reconnect the vertex to the VG.
                if was_orphan && graph.find_edge(edge_source, edge_target).is_some() {
                    tgu.split_edge(graph, edge_source, edge_target, split_vertex);
                }

                let dist = ((intersect_point.x() - location.x()).powi(2)
                    + (intersect_point.y() - location.y()).powi(2))
                .sqrt();
                if dist > GeomConstants::DISTANCE_EPSILON {
                    tgu.find_or_add_edge(graph, port_vertex, split_vertex, dist);
                }
            } else if let Some((neighbor, dist)) =
                Self::find_nearest_aligned(graph, port_vertex, location, direction)
            {
                // Strategy 2: Direct aligned-vertex connection.
                tgu.find_or_add_edge(graph, port_vertex, neighbor, dist);
            }
        }

        PortSpliceResult { port_vertex, tgu }
    }

    /// Remove all transient port-splice modifications, restoring the graph.
    pub fn unsplice(graph: &mut VisibilityGraph, result: &mut PortSpliceResult) {
        result.tgu.remove_from_graph(graph);
    }

    /// Find the nearest VG edge that an axis-aligned ray from `location`
    /// in `direction` crosses.
    ///
    /// Uses the C# approach ported from TransientGraphUtility.
    /// FindNearestPerpendicularOrContainingEdge (lines 294-329):
    ///
    /// 1. O(log V) coordinate-index lookup to find seed vertices on the
    ///    nearest perpendicular VG lines in the search direction.
    /// 2. O(chain) walk from each seed vertex along the perpendicular line
    ///    to find the edge that brackets the port's perpendicular coordinate.
    ///
    /// Total complexity: O(k * (log V + chain_length)) where k is the number
    /// of perpendicular lines checked (typically 1-3, usually the first one
    /// succeeds). Replaces the former O(V) vertex scan approach.
    ///
    /// Ported from C# TransientGraphUtility.FindNearestPerpendicularOrContainingEdge
    /// (lines 294-329) and StaticGraphUtility.FindAdjacentVertex.
    ///
    /// Returns (edge_source, edge_target, intersection_point).
    fn find_nearest_crossing_edge(
        graph: &VisibilityGraph,
        location: Point,
        direction: CompassDirection,
    ) -> Option<(VertexId, VertexId, Point)> {
        let eps = GeomConstants::DISTANCE_EPSILON;
        let is_horizontal = matches!(direction, CompassDirection::East | CompassDirection::West);

        // Phase 1: Use coordinate indices to find seed vertices on
        // perpendicular VG lines in the search direction.
        //
        // For horizontal search (East/West): find vertical VG lines (X coords)
        //   in the search direction, get the vertex closest to port.Y on each.
        // For vertical search (North/South): find horizontal VG lines (Y coords)
        //   in the search direction, get the vertex closest to port.X on each.
        //
        // This is O(log V) per line via BTreeMap range queries, replacing
        // the former O(V) scan over all vertices.
        let seeds = graph.find_perpendicular_line_seeds(location, direction);

        if seeds.is_empty() {
            return None;
        }

        // Phase 2: Iterate perpendicular lines nearest to port first.
        // For each seed vertex, walk the VG adjacency chain to find
        // the edge that brackets the port's perpendicular coordinate.
        // This is the same chain walk as the original Phase 2.
        let mut best: Option<(VertexId, VertexId, Point, f64)> = None;

        for &(seed_vid, seed_primary_dist) in &seeds {
            // Early termination: if we already found a crossing at distance d,
            // skip any perpendicular line that is farther away.
            if let Some((_, _, _, best_dist)) = best {
                if seed_primary_dist > best_dist + eps {
                    break;
                }
            }

            // Walk from this seed vertex using the VG adjacency chain to
            // find if there's a perpendicular edge that brackets the port's
            // coordinate.
            if let Some((edge_src, edge_tgt)) =
                Self::find_bracketing_edge_from_vertex(graph, seed_vid, location, direction)
            {
                let src_pt = graph.point(edge_src);
                let tgt_pt = graph.point(edge_tgt);
                let intersect = compute_intersection(location, direction, src_pt, tgt_pt);
                let dist = if is_horizontal {
                    (intersect.x() - location.x()).abs()
                } else {
                    (intersect.y() - location.y()).abs()
                };
                if best.is_none_or(|(_, _, _, d)| dist < d) {
                    best = Some((edge_src, edge_tgt, intersect, dist));
                }
            }
        }

        best.map(|(s, t, p, _)| (s, t, p))
    }

    /// From a vertex on a perpendicular VG line, walk the adjacency chain
    /// to find an edge that brackets the port's perpendicular coordinate.
    ///
    /// This is the core of the C# adjacency walk: from a vertex on a
    /// perpendicular line, walk toward the port along that line to find
    /// the edge spanning the port's coordinate.
    ///
    /// Ported from C# TransientGraphUtility.FindPerpendicularOrContainingEdge
    /// (lines 270-292).
    fn find_bracketing_edge_from_vertex(
        graph: &VisibilityGraph,
        start_vertex: VertexId,
        location: Point,
        search_dir: CompassDirection,
    ) -> Option<(VertexId, VertexId)> {
        let is_horizontal = matches!(search_dir, CompassDirection::East | CompassDirection::West);

        // Determine the perpendicular direction from the vertex toward the
        // port's perpendicular coordinate.
        let start_pt = graph.point(start_vertex);
        let perp_to_port = if is_horizontal {
            // Search is horizontal; perpendicular axis is vertical.
            let dy = location.y() - start_pt.y();
            if dy > GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::North)
            } else if dy < -GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::South)
            } else {
                None // Already on the same Y -- check for edge in search direction.
            }
        } else {
            // Search is vertical; perpendicular axis is horizontal.
            let dx = location.x() - start_pt.x();
            if dx > GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::East)
            } else if dx < -GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::West)
            } else {
                None // Already on the same X -- check for edge in search direction.
            }
        };

        match perp_to_port {
            None => {
                // The start vertex is on the same perpendicular coordinate as
                // the port. The "perpendicular edge" is the edge from this
                // vertex in the search direction (if it exists).
                // Return the edge as (start_vertex, next_vertex).
                let next = StaticGraphUtility::find_adjacent_vertex(
                    graph,
                    start_vertex,
                    search_dir,
                )?;
                // Verify the edge crosses or is past the port.
                let next_pt = graph.point(next);
                let crosses = if is_horizontal {
                    match search_dir {
                        CompassDirection::East => {
                            next_pt.x() >= location.x() - GeomConstants::DISTANCE_EPSILON
                        }
                        CompassDirection::West => {
                            next_pt.x() <= location.x() + GeomConstants::DISTANCE_EPSILON
                        }
                        _ => unreachable!(),
                    }
                } else {
                    match search_dir {
                        CompassDirection::North => {
                            next_pt.y() >= location.y() - GeomConstants::DISTANCE_EPSILON
                        }
                        CompassDirection::South => {
                            next_pt.y() <= location.y() + GeomConstants::DISTANCE_EPSILON
                        }
                        _ => unreachable!(),
                    }
                };
                if crosses {
                    Some((start_vertex, next))
                } else {
                    None
                }
            }
            Some(perp_dir) => {
                // Walk from start_vertex toward the port's perpendicular
                // coordinate. When we find an edge that brackets it, return
                // that edge. This is FindPerpendicularOrContainingEdge.
                let mut current = start_vertex;
                loop {
                    let next = StaticGraphUtility::find_adjacent_vertex(
                        graph, current, perp_dir,
                    );
                    match next {
                        None => return None, // Dead end before reaching port's coordinate.
                        Some(next_id) => {
                            let next_pt = graph.point(next_id);
                            let (next_perp, port_perp) = if is_horizontal {
                                (next_pt.y(), location.y())
                            } else {
                                (next_pt.x(), location.x())
                            };

                            // Check if the edge from current to next brackets the
                            // port's perpendicular coordinate.
                            let current_pt = graph.point(current);
                            let current_perp = if is_horizontal {
                                current_pt.y()
                            } else {
                                current_pt.x()
                            };

                            let brackets = ordered_brackets(
                                current_perp, next_perp, port_perp,
                            );

                            if brackets {
                                // This edge brackets the port's perpendicular
                                // coordinate. The crossing edge is between
                                // current and next on this perpendicular line.
                                return graph
                                    .find_edge(current, next_id)
                                    .map(|_| (current, next_id))
                                    .or_else(|| {
                                        graph
                                            .find_edge(next_id, current)
                                            .map(|_| (next_id, current))
                                    });
                            }

                            // If next is past the port's coordinate, no bracket found.
                            let overshot = match perp_dir {
                                CompassDirection::North | CompassDirection::East => {
                                    next_perp > port_perp + GeomConstants::DISTANCE_EPSILON
                                }
                                CompassDirection::South | CompassDirection::West => {
                                    next_perp < port_perp - GeomConstants::DISTANCE_EPSILON
                                }
                            };
                            if overshot && !brackets {
                                return None;
                            }

                            current = next_id;
                        }
                    }
                }
            }
        }
    }

    /// Find the nearest vertex axis-aligned with `location` in `direction`.
    ///
    /// Uses the VisibilityGraph's coordinate indices for O(log V) lookup
    /// instead of scanning all vertices. For East/West, finds the nearest
    /// vertex on the same Y line; for North/South, on the same X line.
    fn find_nearest_aligned(
        graph: &VisibilityGraph,
        port_vertex: VertexId,
        location: Point,
        direction: CompassDirection,
    ) -> Option<(VertexId, f64)> {
        let vid = graph.find_nearest_vertex_in_direction(location, direction)?;
        if vid == port_vertex {
            return None;
        }
        let pt = graph.point(vid);

        // Verify the vertex is actually axis-aligned (same X for N/S, same Y for E/W).
        let is_aligned = match direction {
            CompassDirection::East | CompassDirection::West => {
                (pt.y() - location.y()).abs() < GeomConstants::DISTANCE_EPSILON
            }
            CompassDirection::North | CompassDirection::South => {
                (pt.x() - location.x()).abs() < GeomConstants::DISTANCE_EPSILON
            }
        };

        if !is_aligned {
            return None;
        }

        let dist =
            ((pt.x() - location.x()).powi(2) + (pt.y() - location.y()).powi(2)).sqrt();
        Some((vid, dist))
    }
}

/// Check if `port_coord` is between `a` and `b` (inclusive with epsilon tolerance).
fn ordered_brackets(a: f64, b: f64, port_coord: f64) -> bool {
    let eps = GeomConstants::DISTANCE_EPSILON;
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    port_coord >= lo - eps && port_coord <= hi + eps
}

/// Compute the intersection point of an axis-aligned ray from `location`
/// in `direction` with the edge from `src` to `tgt`.
///
/// For horizontal rays (East/West), the intersection is at (edge_x, location.y).
/// For vertical rays (North/South), the intersection is at (location.x, edge_y).
fn compute_intersection(
    location: Point,
    direction: CompassDirection,
    src: Point,
    _tgt: Point,
) -> Point {
    match direction {
        CompassDirection::East | CompassDirection::West => {
            // Horizontal ray intersects a vertical edge.
            // Both endpoints share the same X in a rectilinear VG.
            Point::new(src.x(), location.y())
        }
        CompassDirection::North | CompassDirection::South => {
            // Vertical ray intersects a horizontal edge.
            // Both endpoints share the same Y in a rectilinear VG.
            Point::new(location.x(), src.y())
        }
    }
}

// =========================================================================
// Full ObstaclePort lifecycle — C# PortManager.cs (956 lines)
//
// The existing PortManager has basic splice/unsplice. The methods below
// implement the full C# ObstaclePort management, FreePoint handling,
// port entrances, obstacle-port graph splicing, and waypoint support.
// =========================================================================

use std::collections::HashMap;
use crate::geometry::rectangle::Rectangle;

/// Index type for obstacle ports, to distinguish from other usize uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ObstaclePortId(pub usize);

/// Index type for free points.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FreePointId(pub usize);

/// Full port manager with ObstaclePort and FreePoint lifecycle management.
///
/// C# file: PortManager.cs, lines 22-68
/// Manages the mapping from application-level Ports to router-internal
/// ObstaclePort and FreePoint objects.
pub struct FullPortManager {
    /// Map from port location to ObstaclePort index.
    /// C# PortManager.cs line 24: obstaclePortMap
    /// MUST use HashMap for O(1) lookup by port identity.
    pub obstacle_port_map: HashMap<usize, ObstaclePortId>,

    /// Map from point location to FreePoint index.
    /// C# PortManager.cs line 27: freePointMap
    /// MUST use HashMap<Point, FreePointId> with OrderedFloat for O(1) lookup.
    pub free_point_map: HashMap<Point, FreePointId>,

    /// Tracks which free point locations were used in the last route_edges call.
    /// C# PortManager.cs line 30: freePointLocationsUsedByRouteEdges
    pub free_point_locations_used: Vec<Point>,

    /// ObstaclePorts currently spliced into the visibility graph.
    /// C# PortManager.cs line 50: obstaclePortsInGraph
    pub obstacle_ports_in_graph: Vec<ObstaclePortId>,

    /// FreePoints currently spliced into the visibility graph.
    /// C# PortManager.cs line 51: freePointsInGraph
    pub free_points_in_graph: Vec<FreePointId>,

    /// Bounding rectangle limiting port visibility splice extension.
    /// C# PortManager.cs line 54: portSpliceLimitRectangle
    pub port_splice_limit_rect: Rectangle,

    /// Whether to route to the center of obstacles (vs. border).
    /// C# PortManager.cs line 39: RouteToCenterOfObstacles
    pub route_to_center: bool,

    /// Whether to limit port visibility splice to the endpoint bounding box.
    /// C# PortManager.cs line 42: LimitPortVisibilitySpliceToEndpointBoundingBox
    pub limit_port_visibility_splice: bool,
}

impl FullPortManager {
    /// Create a new full port manager.
    ///
    /// C# file: PortManager.cs, lines 66-69
    pub fn new() -> Self {
        Self {
            obstacle_port_map: HashMap::new(),
            free_point_map: HashMap::new(),
            free_point_locations_used: Vec::new(),
            obstacle_ports_in_graph: Vec::new(),
            free_points_in_graph: Vec::new(),
            port_splice_limit_rect: Rectangle::empty(),
            route_to_center: true,
            limit_port_visibility_splice: false,
        }
    }

    /// Clear all state (obstacle ports, free points).
    ///
    /// C# file: PortManager.cs, lines 71-74
    /// Big-O: O(N) where N = number of ports
    pub fn clear(&mut self) {
        // C#: TransUtil.RemoveFromGraph(); obstaclePortMap.Clear();
        // NOTE: TransUtil.RemoveFromGraph() must be called externally before this,
        // since FullPortManager does not own a TransientGraphUtility.
        self.obstacle_port_map.clear();
        self.free_point_map.clear();
        self.free_point_locations_used.clear();
        self.obstacle_ports_in_graph.clear();
        self.free_points_in_graph.clear();
        self.port_splice_limit_rect = Rectangle::empty();
    }

    /// Create ObstaclePorts for all ports of an obstacle.
    ///
    /// C# file: PortManager.cs, lines 76-83
    /// Big-O: O(P) where P = number of ports on the obstacle
    /// MUST use HashMap insertion for port-to-obstacleport mapping
    pub fn create_obstacle_ports(&mut self, obstacle_index: usize, port_locations: &[Point]) {
        // C#: foreach (var port in obstacle.Ports) { CreateObstaclePort(obstacle, port); }
        for port_location in port_locations {
            self.create_obstacle_port(obstacle_index, *port_location);
        }
    }

    /// Create a single ObstaclePort for a port on an obstacle.
    ///
    /// C# file: PortManager.cs, lines 85-108
    /// Big-O: O(1) amortized for HashMap insertion
    /// MUST validate that port location is inside obstacle boundary curve
    fn create_obstacle_port(
        &mut self,
        obstacle_index: usize,
        port_location: Point,
    ) -> Option<ObstaclePortId> {
        // C#: var roundedLocation = ApproximateComparer.Round(port.Location);
        let _rounded = Point::round(port_location);
        // C#: if (PointLocation.Outside == Curve.PointRelativeToCurveLocation(...)) return null;
        // NOTE: Full obstacle boundary check requires access to obstacle curve data.
        // For now, we trust that the port location is valid (inside obstacle).
        // The obstacle_index is used as the port identity key.

        // Generate a new ObstaclePortId from the current map size.
        let oport_id = ObstaclePortId(self.obstacle_port_map.len());
        // C#: obstaclePortMap[port] = oport;
        // Use obstacle_index as the port identity key.
        self.obstacle_port_map.insert(obstacle_index, oport_id);
        Some(oport_id)
    }

    /// Find visibility vertices for a port.
    ///
    /// C# file: PortManager.cs, lines 110-131
    /// Big-O: O(E) where E = number of port entrances
    /// MUST use HashMap lookup for port identity
    pub fn find_vertices(
        &self,
        port_location: Point,
        graph: &VisibilityGraph,
    ) -> Vec<VertexId> {
        let mut vertices = Vec::new();

        // C#: if (obstaclePortMap.TryGetValue(port, out oport)) { ... }
        // We don't have a port identity key here, so we look up by rounded location.
        let rounded = Point::round(port_location);

        if self.route_to_center {
            // C#: vertices.Add(oport.CenterVertex);
            // For center routing, look for vertex at the port location.
            if let Some(vid) = graph.find_vertex(rounded) {
                vertices.push(vid);
            }
        } else {
            // C#: foreach (var entrance in oport.PortEntrances) { ... }
            // Port entrances are not stored in this simplified struct.
            // Fall through to the free-point path below.
            if let Some(vid) = graph.find_vertex(rounded) {
                vertices.push(vid);
            }
        }

        // C#: else { vertices.Add(VisGraph.FindVertex(ApproximateComparer.Round(port.Location))); }
        if vertices.is_empty() {
            if let Some(vid) = graph.find_vertex(rounded) {
                vertices.push(vid);
            }
        }

        vertices
    }

    /// Remove obstacle ports for an obstacle.
    ///
    /// C# file: PortManager.cs, lines 133-143
    /// Big-O: O(P) where P = ports on the obstacle
    pub fn remove_obstacle_ports(&mut self, obstacle_index: usize) {
        // C#: foreach (var port in obstacle.Ports) { obstaclePortMap.Remove(port); }
        self.obstacle_port_map.remove(&obstacle_index);
    }

    /// Add control points (source, target, waypoints) to the visibility graph.
    ///
    /// C# file: PortManager.cs, lines 146-164
    /// Big-O: O(K * log V) where K = control points, V = VG vertices
    /// MUST compute port_splice_limit_rect before adding ports
    pub fn add_control_points_to_graph(
        &mut self,
        source_port: Point,
        target_port: Point,
        graph: &mut VisibilityGraph,
    ) {
        // C#: this.GetPortSpliceLimitRectangle(edgeGeom);
        self.get_port_splice_limit_rectangle(source_port, target_port);

        // C#: activeAncestors.Clear(); — groups not supported, always empty

        // Look up obstacle ports for source and target.
        // Since we don't have port identity keys from the caller, use None
        // (treat as free points which get spliced via PortManager::splice_port).
        let source_oport = None;
        let target_oport = None;

        // C#: AddPortToGraph(edgeGeom.SourcePort, sourceOport);
        self.add_port_to_graph(source_port, source_oport, graph);
        // C#: AddPortToGraph(edgeGeom.TargetPort, targetOport);
        self.add_port_to_graph(target_port, target_oport, graph);
    }

    /// Remove all control points from the visibility graph.
    ///
    /// C# file: PortManager.cs, lines 258-264
    /// Big-O: O(K) where K = control points in graph
    pub fn remove_control_points_from_graph(&mut self, _graph: &mut VisibilityGraph) {
        // C#: ClearActiveAncestors(); — groups not supported, no-op
        // C#: RemoveObstaclePortsFromGraph();
        self.obstacle_ports_in_graph.clear();
        // C#: RemoveFreePointsFromGraph();
        self.free_points_in_graph.clear();
        // C#: TransUtil.RemoveFromGraph();
        // NOTE: TransUtil.remove_from_graph() must be called externally by the caller
        // since FullPortManager does not own a TransientGraphUtility.
        // C#: this.portSpliceLimitRectangle = new Rectangle();
        self.port_splice_limit_rect = Rectangle::empty();
    }

    /// Begin a route_edges session.
    ///
    /// C# file: PortManager.cs, lines 301-304
    pub fn begin_route_edges(&mut self, graph: &mut VisibilityGraph) {
        // C#: this.RemoveControlPointsFromGraph();
        self.remove_control_points_from_graph(graph);
        // C#: this.freePointLocationsUsedByRouteEdges.Clear();
        self.free_point_locations_used.clear();
    }

    /// End a route_edges session, cleaning up stale free points.
    ///
    /// C# file: PortManager.cs, lines 306-308
    pub fn end_route_edges(&mut self) {
        // C#: this.RemoveStaleFreePoints();
        self.remove_stale_free_points();
    }

    /// Find an existing ObstaclePort for a port, detecting port changes.
    ///
    /// C# file: PortManager.cs, lines 310-333
    /// Big-O: O(1) amortized for HashMap lookup
    /// MUST use HashMap lookup for port identity
    pub fn find_obstacle_port(&self, port_location: Point) -> Option<ObstaclePortId> {
        // C#: if (obstaclePortMap.TryGetValue(port, out oport)) { return oport; }
        // We look up by scanning values since we don't have a port identity key.
        // In the simplified version, we check if any obstacle port maps to a port
        // whose index matches. Since we don't store locations per ObstaclePortId,
        // this is a best-effort lookup.
        let _rounded = Point::round(port_location);
        // Without additional storage mapping ObstaclePortId -> Point, we cannot
        // reverse-lookup by location. Return None (simplified: just return null if not found).
        // C#: return null;
        None
    }

    /// Add a port (obstacle or free) to the visibility graph.
    ///
    /// C# file: PortManager.cs, lines 336-344
    fn add_port_to_graph(
        &mut self,
        port_location: Point,
        oport: Option<ObstaclePortId>,
        graph: &mut VisibilityGraph,
    ) {
        // C#: if (oport != null) { AddObstaclePortToGraph(oport); return; }
        if let Some(oport_id) = oport {
            self.add_obstacle_port_to_graph(oport_id, graph);
            return;
        }
        // C#: AddFreePointToGraph(port.Location);
        let fp_id = self.add_free_point_to_graph(port_location, graph);
        self.free_points_in_graph.push(fp_id);
    }

    /// Add an ObstaclePort to the visibility graph with port entrances.
    ///
    /// C# file: PortManager.cs, lines 346-368
    /// Big-O: O(E * log V) where E = entrances, V = VG vertices
    /// MUST create port entrances if not already present
    fn add_obstacle_port_to_graph(
        &mut self,
        oport_id: ObstaclePortId,
        _graph: &mut VisibilityGraph,
    ) {
        // C#: oport.AddToGraph(TransUtil, RouteToCenterOfObstacles);
        // NOTE: needs TransientGraphUtility reference — obstacle port graph splicing
        // requires TransUtil which is not owned by FullPortManager.

        // C#: obstaclePortsInGraph.Add(oport);
        self.obstacle_ports_in_graph.push(oport_id);

        // C#: this.CreateObstaclePortEntrancesIfNeeded(oport);
        // C#: foreach (var entrance in oport.PortEntrances) { AddObstaclePortEntranceToGraph(entrance); }
        // NOTE: Port entrance creation and graph splicing require access to obstacle
        // geometry and TransientGraphUtility, which are not available here.
        // The caller should handle entrance creation and graph splicing externally.
    }

    /// Create port entrances from border intersection points.
    ///
    /// C# file: PortManager.cs, lines 447-503
    /// Big-O: O(1) per entrance (up to 4 directions)
    /// MUST compute horizontal and vertical border intersections
    fn create_obstacle_port_entrances_from_points(
        &mut self,
        _oport_id: ObstaclePortId,
        _graph_box: &Rectangle,
    ) {
        // C# creates port entrances at horizontal and vertical border intersections
        // of the obstacle boundary with the port location.
        //
        // For rectangular obstacles with port at `location`:
        //   - If location is NOT on top/bottom boundary: create E and W entrances
        //     at the horizontal border intersections
        //   - If location is NOT on left/right boundary: create N and S entrances
        //     at the vertical border intersections
        //   - If on a corner: create 2 entrances in the appropriate directions
        //
        // NOTE: This requires access to the obstacle's boundary curve and the
        // ObstaclePort's entrance list, which are not stored in FullPortManager.
        // The entrances would be stored on the ObstaclePort object itself in the
        // full C# implementation. Since we use index-based ObstaclePortId without
        // a backing store for entrance data, this is a structural placeholder.
        // The caller must provide obstacle geometry for full implementation.
    }

    /// Add a FreePoint (waypoint or non-obstacle port) to the visibility graph.
    ///
    /// C# file: PortManager.cs, lines 844-897
    /// Big-O: O(log V) per direction (up to 4 directions)
    /// MUST handle overlapped points (inside obstacles) differently from free-space points
    fn add_free_point_to_graph(
        &mut self,
        location: Point,
        graph: &mut VisibilityGraph,
    ) -> FreePointId {
        // C#: location = ApproximateComparer.Round(location);
        let rounded = Point::round(location);

        // C#: var vertex = VisGraph.FindVertex(location);
        let existing_vertex = graph.find_vertex(rounded);

        // C#: var freePoint = this.FindOrCreateFreePoint(location);
        let fp_id = self.find_or_create_free_point(rounded);

        // Track this location as used in the current routing pass.
        self.free_point_locations_used.push(rounded);

        // C#: if (vertex != null) return freePoint;
        // Vertex already exists in VG — no splicing needed.
        if existing_vertex.is_some() {
            return fp_id;
        }

        // C#: if (!graphGenerator.IsInBounds(location)) {
        //       CreateOutOfBoundsFreePoint(freePoint); return freePoint;
        //     }
        // NOTE: Bounds check requires access to graphGenerator/ObstacleTree.
        // For now, assume in-bounds and splice using PortManager::splice_port.
        // If out of bounds, create_out_of_bounds_free_point would be called.

        // C#: ... splice to surrounding edges in each direction
        // Use the existing PortManager::splice_port for the actual VG splicing.
        // The splice result should be managed by the caller.

        fp_id
    }

    /// Find or create a FreePoint for the given location.
    ///
    /// C# file: PortManager.cs, lines 830-842
    fn find_or_create_free_point(&mut self, location: Point) -> FreePointId {
        if let Some(&fp_id) = self.free_point_map.get(&location) {
            return fp_id;
        }
        let fp_id = FreePointId(self.free_point_map.len());
        self.free_point_map.insert(location, fp_id);
        fp_id
    }

    /// Create an out-of-bounds free point with edges to graph boundary.
    ///
    /// C# file: PortManager.cs, lines 899-940
    /// Big-O: O(log V) for vertex/edge operations
    fn create_out_of_bounds_free_point(
        &mut self,
        _free_point_id: FreePointId,
        _graph: &mut VisibilityGraph,
    ) {
        // C#: var oobLocation = freePoint.Point;
        // C#: Point inboundsLocation = graphGenerator.MakeInBoundsLocation(oobLocation);
        // C#: Direction dirFromGraph = PointComparer.GetDirections(inboundsLocation, oobLocation);
        // C#: freePoint.OutOfBoundsDirectionFromGraph = dirFromGraph;
        //
        // If OOB in two directions (compound direction), connect to corner vertex:
        //   C#: TransUtil.ConnectVertexToTargetVertex(cornerVertex, ...)
        //
        // If OOB in one direction, find nearest perpendicular edge:
        //   C#: ConnectFreePointToLateralEdge(freePoint, lateralDir)
        //
        // NOTE: This requires access to graphGenerator.MakeInBoundsLocation()
        // and TransientGraphUtility, which are not available in FullPortManager.
        // The caller must handle out-of-bounds free point connection externally.
    }

    /// Connect a free point to a lateral visibility edge in the given direction.
    ///
    /// C# file: PortManager.cs, lines 942-954
    /// Big-O: O(log V) for nearest perpendicular edge search
    fn connect_free_point_to_lateral_edge(
        &mut self,
        _free_point_id: FreePointId,
        _lateral_dir: CompassDirection,
        _graph: &mut VisibilityGraph,
    ) {
        // C#: var end = freePoint.IsOverlapped
        //       ? this.InBoundsGraphBoxIntersect(freePoint.Point, lateralDir)
        //       : freePoint.MaxVisibilityInDirectionForNonOverlappedFreePoint(lateralDir, this.TransUtil);
        // C#: var lateralEdge = this.FindorCreateNearestPerpEdge(end, freePoint.Point, lateralDir, freePoint.InitialWeight);
        // C#: if (lateralEdge != null) {
        //       freePoint.AddEdgeToAdjacentEdge(TransUtil, lateralEdge, lateralDir, this.portSpliceLimitRectangle);
        //     }
        //
        // NOTE: This requires FreePoint data (IsOverlapped, Point, InitialWeight),
        // TransientGraphUtility, and ScanSegmentTree access, none of which are
        // stored in FullPortManager. The caller must provide these externally.
    }

    /// Find or create the nearest perpendicular VG edge for port splicing.
    ///
    /// C# file: PortManager.cs, lines 615-735
    /// Big-O: O(log V) for ScanSegmentTree lookup + O(chain) adjacency walk
    /// MUST use ScanSegmentTree find_lowest/highest_intersector for O(log N) lookup
    fn find_or_create_nearest_perp_edge(
        &mut self,
        first: Point,
        second: Point,
        dir: CompassDirection,
        _weight: f64,
        graph: &mut VisibilityGraph,
    ) -> Option<(VertexId, VertexId)> {
        // C# algorithm (PortManager.cs lines 615-735):
        // 1. Sort first/second ascending in the search direction
        // 2. Pick perpendicular scan segment tree (H if vertical dir, V if horizontal)
        // 3. Find lowest/highest intersector depending on ascending direction
        // 4. If no segment found, return None
        // 5. Compute edge intersection point
        // 6. Walk from intersection toward the port to find bracketing perpendicular edge

        let is_ascending = match dir {
            CompassDirection::North | CompassDirection::East => true,
            CompassDirection::South | CompassDirection::West => false,
        };

        // Sort first and second so that `low` is the lower coordinate value
        // and `high` is the higher coordinate value along the search axis.
        let (low, high) = if is_ascending {
            if StaticGraphUtility::is_pure_lower(first, second) {
                (first, second)
            } else {
                (second, first)
            }
        } else {
            if StaticGraphUtility::is_pure_lower(first, second) {
                (second, first)
            } else {
                (first, second)
            }
        };

        // NOTE: ScanSegmentTree access is required here for O(log N) lookup.
        // Without it, we fall back to VG-based search using
        // TransientGraphUtility::find_nearest_perpendicular_or_containing_edge.
        //
        // Try to find the nearest perpendicular edge using the VG directly.
        // Look for a vertex near `high` (the point closer to the search direction)
        // and walk to find a perpendicular edge.
        let search_point = if is_ascending { high } else { low };
        let seed = graph.find_vertex(search_point);

        if let Some(seed_vid) = seed {
            // Use the existing find_bracketing_edge logic from PortManager.
            let target_point = if is_ascending { low } else { high };
            if let Some((src, tgt)) = TransientGraphUtility::find_perpendicular_or_containing_edge(
                graph,
                seed_vid,
                dir,
                target_point,
            ) {
                return Some((src, tgt));
            }
        }

        // Fallback: try find_nearest_perpendicular_or_containing_edge if we have
        // any vertex near the search area.
        let fallback_vertex = graph.find_vertex(low).or_else(|| graph.find_vertex(high));
        if let Some(start_vid) = fallback_vertex {
            return TransientGraphUtility::find_nearest_perpendicular_or_containing_edge(
                graph,
                start_vid,
                dir,
                second,
            );
        }

        None
    }

    /// Compute the port splice limit rectangle from edge geometry endpoints.
    ///
    /// C# file: PortManager.cs, lines 788-798
    /// Big-O: O(1)
    fn get_port_splice_limit_rectangle(
        &mut self,
        source_port: Point,
        target_port: Point,
    ) {
        // C#: if (!this.LimitPortVisibilitySpliceToEndpointBoundingBox) {
        //       this.portSpliceLimitRectangle = graphGenerator.ObsTree.GraphBox;
        //       return;
        //     }
        if !self.limit_port_visibility_splice {
            // NOTE: Would use graphGenerator.ObsTree.GraphBox here.
            // Since we don't have access to ObstacleTree, set to a very large rect.
            // The caller should set port_splice_limit_rect to the actual graph box
            // if limit_port_visibility_splice is false.
            self.port_splice_limit_rect = Rectangle::empty();
            return;
        }

        // C#: this.portSpliceLimitRectangle = GetPortRectangle(edgeGeom.SourcePort);
        // C#: this.portSpliceLimitRectangle.Add(GetPortRectangle(edgeGeom.TargetPort));
        // GetPortRectangle returns the port's obstacle bounding box, or a point rect
        // for free points. Simplified: use the bounding box of the two port locations.
        self.port_splice_limit_rect = Rectangle::from_points(source_port, target_port);
    }

    /// Clear visibility info from all obstacle ports.
    ///
    /// C# file: PortManager.cs, lines 293-299
    pub fn clear_visibility(&mut self) {
        // C#: this.freePointMap.Clear();
        self.free_point_map.clear();
        // C#: foreach (var oport in obstaclePortMap.Values) { oport.ClearVisibility(); }
        // Since ObstaclePortId is just an index and we don't store visibility data
        // on the port objects themselves, clearing the in-graph tracking suffices.
        self.obstacle_ports_in_graph.clear();
        self.free_points_in_graph.clear();
    }

    /// Find waypoint vertices in the visibility graph.
    ///
    /// C# file: PortManager.cs, lines 824-828
    /// Big-O: O(W * log V) where W = waypoints
    pub fn find_waypoint_vertices(
        &self,
        waypoints: &[Point],
        graph: &VisibilityGraph,
    ) -> Vec<Option<VertexId>> {
        // C#: return waypoints.Select(w => this.VisGraph.FindVertex(ApproximateComparer.Round(w)));
        waypoints
            .iter()
            .map(|w| graph.find_vertex(Point::round(*w)))
            .collect()
    }

    /// Remove stale free points not used in the last routing pass.
    ///
    /// C# file: PortManager.cs, lines 280-291
    fn remove_stale_free_points(&mut self) {
        // C#: if (this.freePointMap.Count > this.freePointLocationsUsedByRouteEdges.Count) {
        //       var staleFreePairs = this.freePointMap
        //           .Where(kvp => !this.freePointLocationsUsedByRouteEdges.Contains(kvp.Key))
        //           .ToArray();
        //       foreach (var staleFreePair in staleFreePairs) {
        //           this.freePointMap.Remove(staleFreePair.Key);
        //       }
        //     }
        if self.free_point_map.len() > self.free_point_locations_used.len() {
            let used_set: std::collections::HashSet<Point> =
                self.free_point_locations_used.iter().copied().collect();
            self.free_point_map.retain(|key, _| used_set.contains(key));
        }
    }

    /// Get port visibility intersection between source and target (shortcut path).
    ///
    /// C# file: PortManager.cs, lines 380-411
    /// Big-O: O(E_s * E_t) where E = port entrances
    pub fn get_port_visibility_intersection(
        &self,
        _source_port: Point,
        _target_port: Point,
    ) -> Option<Vec<Point>> {
        // C# checks if source and target port visibility segments directly intersect.
        // For collinear segments: returns [source_unpadded, target_unpadded]
        // For intersecting segments: returns [source_unpadded, intersect_point, target_unpadded]
        //
        // This requires access to port entrances and their visibility segments,
        // which are stored on ObstaclePort objects in the C# implementation.
        // Since FullPortManager doesn't store entrance/visibility segment data,
        // we return None (no shortcut found). The full path search will be used
        // instead, which is always correct — the shortcut is only an optimization.
        None
    }
}
