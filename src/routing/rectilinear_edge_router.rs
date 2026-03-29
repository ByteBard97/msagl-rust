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
use crate::routing::visibility_graph_generator::{
    generate_visibility_graph, VisibilityGraphGenerator,
};

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
    /// True when the path search failed and this edge uses a straight-line fallback.
    pub is_fallback: bool,
}

/// Stateful rectilinear edge router.
///
/// Owns the `RouterSession` (visibility graph + obstacle tree) and rebuilds it
/// incrementally when shapes change. Call `add_shape` / `remove_shape` to mutate
/// the obstacle set, then `run()` to produce routes.
///
/// # Refactor 5
/// Previously a builder that rebuilt the VG from scratch on every `run()` call.
/// Now the session is owned and the VG is regenerated only when the obstacle set
/// or padding changes, matching C# `RectilinearEdgeRouter`'s incremental model.
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
    /// Owns the visibility graph, obstacle tree, and scan segment trees.
    session: RouterSession,
    /// Stateless VG generator; held as a field so `add_shape`/`remove_shape`
    /// can call `vg_generator.generate(&mut self.session)` on demand.
    vg_generator: VisibilityGraphGenerator,
    edges: Vec<EdgeGeometry>,
    padding: f64,
    edge_separation: f64,
    corner_fit_radius: f64,
    bend_penalty_as_percentage: f64,
    /// When true the session is stale and `run()` must call `rebuild()` first.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RebuildTreeAndGraph()` which is called after
    /// every obstacle mutation.  We defer the rebuild so that batched mutations
    /// (e.g. `add_obstacles` / `remove_obstacles`) only rebuild once.
    dirty: bool,
    /// When true, ports are placed at obstacle centers rather than the nearest
    /// padded boundary.  Mirrors C# `RouteToCenterOfObstacles`.
    route_to_center: bool,
    /// When true, the port-visibility splice is restricted to the bounding box of
    /// the two endpoints.  Mirrors C# `LimitPortVisibilitySpliceToEndpointBoundingBox`.
    limit_splice_to_bbox: bool,
    /// When true (default), the nudger removes staircase patterns from routed paths.
    /// Mirrors C# `RemoveStaircases`.
    remove_staircases_flag: bool,
}

impl RectilinearEdgeRouter {
    /// Create a router for the given obstacle shapes.
    ///
    /// Builds the visibility graph immediately from `shapes` using
    /// `DEFAULT_PADDING`. Call `.padding(p)` in the builder chain to
    /// override before adding edges.
    pub fn new(shapes: &[Shape]) -> Self {
        let session = generate_visibility_graph(shapes, DEFAULT_PADDING);
        Self {
            session,
            vg_generator: VisibilityGraphGenerator,
            edges: Vec::new(),
            padding: DEFAULT_PADDING,
            edge_separation: DEFAULT_EDGE_SEPARATION,
            corner_fit_radius: DEFAULT_CORNER_FIT_RADIUS,
            bend_penalty_as_percentage: DEFAULT_BEND_PENALTY_PCT,
            dirty: false,
            route_to_center: false,
            limit_splice_to_bbox: false,
            remove_staircases_flag: true,
        }
    }

    /// Set obstacle padding (space between shape boundary and routes).
    ///
    /// If the new padding differs from the current value, rebuilds the
    /// obstacle tree and regenerates the visibility graph.
    pub fn padding(mut self, p: f64) -> Self {
        if (self.padding - p).abs() > 1e-12 {
            self.padding = p;
            let shapes = Self::extract_shapes(&self.session);
            self.session = generate_visibility_graph(&shapes, p);
        }
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

    /// If true, route to obstacle centers rather than the nearest padded boundary.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RouteToCenterOfObstacles`.
    /// Default: false.
    pub fn route_to_center_of_obstacles(mut self, val: bool) -> Self {
        self.route_to_center = val;
        self
    }

    /// If true, limit port-visibility splicing to the bounding box of the two endpoints.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.LimitPortVisibilitySpliceToEndpointBoundingBox`.
    /// Default: false.
    pub fn limit_port_visibility_splice_to_endpoint_bounding_box(mut self, val: bool) -> Self {
        self.limit_splice_to_bbox = val;
        self
    }

    /// If false, skip the staircase-removal pass in the nudger.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RemoveStaircases`.
    /// Default: true.
    pub fn remove_staircases(mut self, val: bool) -> Self {
        self.remove_staircases_flag = val;
        self
    }

    // ── Obstacle mutation API ────────────────────────────────────────────────

    /// Add an obstacle shape.
    ///
    /// The VG is NOT rebuilt immediately; it will be rebuilt lazily on the next
    /// `run()` call (or you can call `rebuild()` explicitly).  This matches the
    /// C# `AddObstacleWithoutRebuild` + `RebuildTreeAndGraph` split but defers
    /// the rebuild to the batch boundary.
    pub fn add_shape_without_rebuild(&mut self, shape: Shape) {
        let idx = self.session.obstacle_tree.obstacles.len();
        self.session.obstacle_tree.obstacles.push(
            crate::routing::obstacle::Obstacle::from_shape(&shape, self.padding, idx),
        );
        self.dirty = true;
    }

    /// Add an obstacle shape and immediately rebuild the VG.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.AddObstacle(Shape)`.
    pub fn add_shape(&mut self, shape: Shape) {
        self.add_shape_without_rebuild(shape);
        self.rebuild();
    }

    /// Remove the obstacle shape at `index` (by insertion order).
    ///
    /// The VG is NOT rebuilt immediately.  Mirrors C# `RemoveObstacleWithoutRebuild`.
    pub fn remove_shape_without_rebuild(&mut self, index: usize) {
        if index < self.session.obstacle_tree.obstacles.len() {
            self.session.obstacle_tree.obstacles.remove(index);
        }
        self.dirty = true;
    }

    /// Remove the obstacle shape at `index` and immediately rebuild the VG.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RemoveObstacle(Shape)`.
    pub fn remove_shape(&mut self, index: usize) {
        self.remove_shape_without_rebuild(index);
        self.rebuild();
    }

    /// Alias for `remove_shape` using the C# method name.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RemoveObstacle(Shape)`.
    pub fn remove_obstacle(&mut self, index: usize) {
        self.remove_shape(index);
    }

    /// Batch-add obstacles without rebuilding after each one.
    ///
    /// The VG is rebuilt once at the end.
    /// Mirrors C# `AddObstacles(IEnumerable<Shape>)`.
    pub fn add_obstacles(&mut self, shapes: &[Shape]) {
        for shape in shapes {
            self.add_shape_without_rebuild(shape.clone());
        }
        self.rebuild();
    }

    /// Batch-remove obstacles by index (descending order to preserve indices).
    ///
    /// The VG is rebuilt once at the end.
    /// Mirrors C# `RemoveObstacles(IEnumerable<Shape>)`.
    pub fn remove_obstacles(&mut self, mut indices: Vec<usize>) {
        // Remove in descending order so earlier indices remain valid.
        indices.sort_unstable_by(|a, b| b.cmp(a));
        indices.dedup();
        for idx in indices {
            self.remove_shape_without_rebuild(idx);
        }
        self.rebuild();
    }

    /// Replace the shape at `index` with `new_shape` and rebuild the VG.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.UpdateObstacle(Shape)`, which calls
    /// `UpdateObstacleWithoutRebuild` then `RebuildTreeAndGraph`.
    /// (C# line 158–161.)
    pub fn update_obstacle(&mut self, index: usize, new_shape: Shape) {
        if index < self.session.obstacle_tree.obstacles.len() {
            self.session.obstacle_tree.obstacles[index] =
                crate::routing::obstacle::Obstacle::from_shape(&new_shape, self.padding, index);
            self.dirty = true;
        }
        self.rebuild();
    }

    /// Add a single obstacle and immediately rebuild the visibility graph.
    ///
    /// Alias for `add_shape` — mirrors C# `RectilinearEdgeRouter.AddObstacle(Shape)`.
    /// (C# line 137–140.)
    #[inline]
    pub fn add_obstacle(&mut self, shape: Shape) {
        self.add_shape(shape);
    }

    /// Replace multiple obstacles in one batch, rebuilding the VG once.
    ///
    /// Each entry is `(index, new_shape)`. Indices that are out of range are
    /// silently skipped.  A single `rebuild()` is performed at the end.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.UpdateObstacles(IEnumerable<Shape>)`.
    /// (C# line 146–152.)
    pub fn update_obstacles(&mut self, updates: &[(usize, Shape)]) {
        for (index, new_shape) in updates {
            if *index < self.session.obstacle_tree.obstacles.len() {
                self.session.obstacle_tree.obstacles[*index] =
                    crate::routing::obstacle::Obstacle::from_shape(new_shape, self.padding, *index);
                self.dirty = true;
            }
        }
        self.rebuild();
    }

    /// Return a slice of all edge geometries currently registered for routing.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.EdgeGeometriesToRoute`.
    /// (C# line 94–96.)
    pub fn edge_geometries_to_route(&self) -> &[EdgeGeometry] {
        &self.edges
    }

    /// Remove an edge geometry by index from the active routing set.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RemoveEdgeGeometryToRoute(EdgeGeometry)`.
    /// (C# line 85–87.)
    pub fn remove_edge_geometry(&mut self, edge_idx: usize) {
        if edge_idx < self.edges.len() {
            self.edges.remove(edge_idx);
        }
    }

    /// Remove all obstacles and edges, resetting the router to a clean state.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.Clear()` → `InternalClear(retainObstacles: false)`.
    /// (C# line 260–262.)
    pub fn clear(&mut self) {
        self.edges.clear();
        self.session = generate_visibility_graph(&[], self.padding);
        self.dirty = false;
    }

    /// Explicitly rebuild the visibility graph from the current obstacle set.
    ///
    /// Called automatically by `run()` when `dirty == true`.  Can also be
    /// called manually after batched obstacle mutations.
    ///
    /// Mirrors C# `RebuildTreeAndGraph()` (C# line 233–245).
    pub fn rebuild(&mut self) {
        let shapes = Self::extract_shapes_from_obstacles(&self.session);
        self.session = generate_visibility_graph(&shapes, self.padding);
        self.dirty = false;
    }

    /// Add an edge to be routed.
    pub fn add_edge(&mut self, edge: EdgeGeometry) {
        self.edges.push(edge);
    }

    /// Add an edge geometry to the route — C#/TS API alias for `add_edge`.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.AddEdgeGeometryToRoute(EdgeGeometry)` and
    /// TS `addEdgeGeometryToRoute`.
    #[inline]
    pub fn add_edge_geometry_to_route(&mut self, edge: EdgeGeometry) {
        self.add_edge(edge);
    }

    /// Return the bend penalty as a percentage of source-target distance.
    ///
    /// C#/TS API alias for the `bend_penalty_as_percentage` field accessor.
    /// Mirrors C# `RectilinearEdgeRouter.BendPenaltyAsAPercentageOfDistance` and
    /// TS `bendPenaltyAsAPercentageOfDistance`.
    #[inline]
    pub fn bend_penalty_as_a_percentage_of_distance(&self) -> f64 {
        self.bend_penalty_as_percentage
    }

    /// Remove all obstacles and edges, resetting the router to a clean state.
    ///
    /// C#/TS API alias for `clear`.
    /// Mirrors C# `RectilinearEdgeRouter.RemoveAllObstacles` and TS `removeAllObstacles`.
    #[inline]
    pub fn remove_all_obstacles(&mut self) {
        self.clear();
    }

    /// Return the number of edge geometries currently registered for routing.
    ///
    /// Mirrors C# `router.EdgeGeometriesToRoute.Count()`.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Clear accumulated per-edge visibility vertex entries.
    ///
    /// Faithful port of C# `RectilinearEdgeRouter.RemoveVertexEntries` (RectilinearEdgeRouter.cs).
    /// Called between routing runs to reset per-edge visibility state.
    /// No-op in the current implementation: vertex entry caching is handled
    /// inside `RouterSession` rather than accumulated on the router itself.
    pub fn clear_vertex_entries(&mut self) {
        // C# RectilinearEdgeRouter: RemoveVertexEntries — no vertex entry cache in current implementation
    }

    /// Alias for `clear_vertex_entries`.
    ///
    /// Mirrors C# `RectilinearEdgeRouter.RemoveVertexEntries`.
    #[inline]
    pub fn remove_vertex_entries(&mut self) {
        self.clear_vertex_entries();
    }

    /// Run the full routing pipeline: visibility graph, path search,
    /// nudging, and curve generation.
    pub fn run(&mut self) -> RoutingResult {
        // Rebuild the VG lazily if any obstacle mutation happened since last run.
        // Mirrors C# RunInternal() which calls GenerateVisibilityGraph() first.
        if self.dirty {
            self.rebuild();
        }

        if self.edges.is_empty() {
            return RoutingResult { edges: Vec::new() };
        }

        // Extract source/target locations and obstacle indices upfront (Points are Copy)
        // so that the loop body can take &mut self.session without conflicting with the
        // &self.edges borrow.
        let edge_locs: Vec<(Point, Point, usize, usize)> = self
            .edges
            .iter()
            .map(|e| (e.source.location, e.target.location,
                      e.source.obstacle_index, e.target.obstacle_index))
            .collect();

        let search = PathSearch::new(self.bend_penalty_as_percentage);
        let mut paths: Vec<(Vec<Point>, bool)> = Vec::new(); // (waypoints, is_fallback)

        let graph_box = self.session.obstacle_tree.graph_box();

        // Overlapped-weight multiplier used by SegmentWeight::Overlapped.
        // Matches scan_segment.rs SegmentWeight::Overlapped.value() = 500.
        const OVERLAPPED_FACTOR: f64 = 500.0;

        // Route each edge with a single TGU shared by entrance creation and port
        // splicing. This matches the C# design where one TGU per edge-routing call
        // owns all transient VG modifications; remove_from_graph at the end restores
        // the graph cleanly.
        for (src_loc, tgt_loc, src_obs_idx, tgt_obs_idx) in edge_locs {
            let mut tgu = TransientGraphUtility::new();

            // C# PortManager.cs lines 788-798: GetPortSpliceLimitRectangle.
            // When limit_splice_to_bbox is true, restrict visibility splicing to the
            // union of the two endpoint port rectangles; otherwise use the full graph box.
            //
            // C# GetPortRectangle: for ObstaclePort returns oport.Obstacle.VisibilityBoundingBox
            // (the padded bounding box of the containing obstacle); for FreePoint returns
            // new Rectangle(port.Location) — a degenerate zero-size rect at the point.
            // Using point-to-point bounding box was wrong for obstacle ports because it
            // ignores the obstacle extent and under-clips the splice.
            let limit_rect = if self.limit_splice_to_bbox {
                let src_rect = Self::get_port_rectangle(src_loc, &self.session.obstacle_tree.obstacles);
                let mut tgt_rect = Self::get_port_rectangle(tgt_loc, &self.session.obstacle_tree.obstacles);
                tgt_rect.add_rect(&src_rect);
                tgt_rect
            } else {
                graph_box
            };

            // Phase 2a: Create port entrances at obstacle boundaries.
            Self::create_port_entrances_for_location(
                src_loc,
                &mut self.session,
                &mut tgu,
                &limit_rect,
                self.route_to_center,
            );
            Self::create_port_entrances_for_location(
                tgt_loc,
                &mut self.session,
                &mut tgu,
                &limit_rect,
                self.route_to_center,
            );

            // Phase 2b: Splice ports using the SAME TGU so all modifications are consistent.
            PortManager::splice_port_into(&mut self.session, src_loc, &mut tgu);
            PortManager::splice_port_into(&mut self.session, tgt_loc, &mut tgu);

            // C# FloatingPort transparency: when a port is associated with an obstacle
            // (FloatingPort.obstacle_index), that obstacle is transparent for this routing.
            // C# RectilinearEdgeRouter sets Obstacle.IsTransparent = true before routing the
            // edge, then restores it after. A transparent obstacle is excluded from the sweep
            // as a blocker, so the VG is regenerated without it and vertical/horizontal scan
            // segments extend freely through its interior.
            //
            // We implement this by marking the source and target obstacles transparent,
            // rebuilding the VG, running path search, then restoring and rebuilding again.
            // C# FloatingPort transparency only applies when the port obstacle is
            // non-rectangular. Rectangular obstacles never need the VG rebuild path because
            // their padded bbox is already the sweep boundary; making them transparent would
            // be incorrect and rebuilding for every edge is catastrophically expensive
            // (2× VG rebuilds × N edges on the large benchmark).
            let n_obstacles = self.session.obstacle_tree.obstacles.len();
            let src_needs_transparent = src_obs_idx < n_obstacles
                && !self.session.obstacle_tree.obstacles[src_obs_idx].is_rectangle();
            let tgt_needs_transparent = tgt_obs_idx < n_obstacles
                && tgt_obs_idx != src_obs_idx
                && !self.session.obstacle_tree.obstacles[tgt_obs_idx].is_rectangle();

            let src_was_transparent = if src_needs_transparent {
                let was = self.session.obstacle_tree.obstacles[src_obs_idx].is_transparent();
                if !was {
                    self.session.obstacle_tree.obstacles[src_obs_idx].set_transparent(true);
                }
                was
            } else {
                true // skip restore
            };
            let tgt_was_transparent = if tgt_needs_transparent {
                let was = self.session.obstacle_tree.obstacles[tgt_obs_idx].is_transparent();
                if !was {
                    self.session.obstacle_tree.obstacles[tgt_obs_idx].set_transparent(true);
                }
                was
            } else {
                true // skip restore
            };

            // If we changed any transparency flags, rebuild the VG.
            let rebuilt = !src_was_transparent || !tgt_was_transparent;
            if rebuilt {
                VisibilityGraphGenerator.generate(&mut self.session);
                // Re-create port entrances and splice with the new VG.
                tgu = TransientGraphUtility::new();
                Self::create_port_entrances_for_location(
                    src_loc,
                    &mut self.session,
                    &mut tgu,
                    &limit_rect,
                    self.route_to_center,
                );
                Self::create_port_entrances_for_location(
                    tgt_loc,
                    &mut self.session,
                    &mut tgu,
                    &limit_rect,
                    self.route_to_center,
                );
                PortManager::splice_port_into(&mut self.session, src_loc, &mut tgu);
                PortManager::splice_port_into(&mut self.session, tgt_loc, &mut tgu);
            }

            let path = search.find_path(&mut self.session.vis_graph, src_loc, tgt_loc);

            // Restore transparency flags and rebuild VG to original state.
            if rebuilt {
                tgu.remove_from_graph(&mut self.session);
                tgu = TransientGraphUtility::new();
                if !src_was_transparent {
                    self.session.obstacle_tree.obstacles[src_obs_idx].set_transparent(false);
                }
                if !tgt_was_transparent && tgt_obs_idx != src_obs_idx {
                    self.session.obstacle_tree.obstacles[tgt_obs_idx].set_transparent(false);
                }
                VisibilityGraphGenerator.generate(&mut self.session);
            }

            let is_fallback = path.is_none();
            let pts = path.unwrap_or_else(|| {
                debug_assert!(
                    false,
                    "path search failed for {:?} -> {:?}",
                    src_loc, tgt_loc
                );
                vec![src_loc, tgt_loc]
            });
            paths.push((pts, is_fallback));

            // Phase 2c: Remove ALL modifications (entrances + splices) atomically.
            tgu.remove_from_graph(&mut self.session);
        }

        // 3. Nudge paths for edge separation
        let obstacles = self.padded_obstacles();
        let (mut raw_paths, fallback_flags): (Vec<Vec<Point>>, Vec<bool>) =
            paths.into_iter().unzip();
        nudge_paths(&mut raw_paths, &obstacles, self.edge_separation, self.remove_staircases_flag);

        // 4. Convert to curves
        let edges = raw_paths
            .into_iter()
            .zip(fallback_flags)
            .map(|(pts, is_fallback)| {
                let curve = self.points_to_curve(&pts);
                RoutedEdge { points: pts, curve, is_fallback }
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
        session: &mut RouterSession,
        tgu: &mut TransientGraphUtility,
        limit_rect: &Rectangle,
        route_to_center: bool,
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
                CompassDirection::East | CompassDirection::West => {
                    (CompassDirection::North, CompassDirection::South)
                }
                CompassDirection::North | CompassDirection::South => {
                    (CompassDirection::East, CompassDirection::West)
                }
            };
            for &pdir in &[perp_a, perp_b] {
                if let Some(nbr) =
                    session.vis_graph.find_nearest_vertex_in_direction(border_point, pdir)
                {
                    if nbr == border_vertex {
                        continue;
                    }
                    let nbr_pt = session.vis_graph.point(nbr);
                    let on_axis = match pdir {
                        CompassDirection::North | CompassDirection::South => {
                            GeomConstants::close(nbr_pt.x(), border_point.x())
                        }
                        CompassDirection::East | CompassDirection::West => {
                            GeomConstants::close(nbr_pt.y(), border_point.y())
                        }
                    };
                    if on_axis && session.vis_graph.degree(nbr) > 0 {
                        let dist = ((nbr_pt.x() - border_point.x()).powi(2)
                            + (nbr_pt.y() - border_point.y()).powi(2))
                        .sqrt();
                        tgu.find_or_add_edge(session, border_vertex, nbr, dist);
                    }
                }
            }

            // Compute max visibility segment using the full graph box as the outer
            // boundary for ray casting.  `limit_rect` only restricts how far the splice
            // extends along existing VG edges (P2b), so we must pass the real graph box
            // here to get the true max extent.
            let full_graph_box = session.obstacle_tree.graph_box();
            let (seg_start, seg_end, _pac) =
                session.obstacle_tree.create_max_visibility_segment(border_point, out_dir, &full_graph_box);
            if !seg_start.close_to(seg_end) {
                // P2b: pass limit_rect (endpoint bounding box or full graph box depending
                // on limit_splice_to_bbox) so that the chain is clamped accordingly.
                // C# PortManager.cs line 591: entrance.ExtendEdgeChain(TransUtil, bv, bv,
                //     this.portSpliceLimitRectangle, RouteToCenterOfObstacles)
                tgu.extend_edge_chain_public(
                    session,
                    border_vertex,
                    limit_rect,
                    seg_start,
                    seg_end,
                    None,
                    false,
                );
            }

            // P2a: when route_to_center is true, connect the obstacle center location to
            // the padded border vertex so that paths can originate from the center.
            // C# ObstaclePortEntrance.cs line 177: transUtil.ConnectVertexToTargetVertex(
            //     ObstaclePort.CenterVertex, unpaddedBorderVertex, OutwardDirection, InitialWeight)
            if route_to_center {
                let center_vertex = tgu.find_or_add_vertex(session, location);
                let bp = session.vis_graph.point(border_vertex);
                let cp = session.vis_graph.point(center_vertex);
                if !bp.close_to(cp) {
                    let dist = ((bp.x() - cp.x()).powi(2) + (bp.y() - cp.y()).powi(2)).sqrt();
                    tgu.find_or_add_edge(session, center_vertex, border_vertex, dist);
                }
            }
        }
    }

    /// Compute the port rectangle for a single endpoint location.
    ///
    /// Faithful port of C# `PortManager.GetPortRectangle` (PortManager.cs line 800-814).
    ///
    /// For an `ObstaclePort` (location inside an obstacle) returns the obstacle's
    /// `VisibilityBoundingBox` — the padded bounding box (or convex-hull bbox when
    /// the obstacle is in a hull).  For a free point returns a degenerate rectangle
    /// at the point itself, matching C# `new Rectangle(ApproximateComparer.Round(port.Location))`.
    fn get_port_rectangle(location: Point, obstacles: &[crate::routing::obstacle::Obstacle]) -> Rectangle {
        use crate::geometry::point_comparer::GeomConstants;

        // Check whether `location` falls strictly inside any obstacle's padded bbox.
        // This mirrors the C# `obstaclePortMap.TryGetValue(port, out oport)` lookup;
        // in the Rust port we identify obstacle membership by containment test.
        for obs in obstacles {
            if obs.is_sentinel() {
                continue;
            }
            let bb = obs.padded_bounding_box();
            if location.x() > bb.left() + GeomConstants::DISTANCE_EPSILON
                && location.x() < bb.right() - GeomConstants::DISTANCE_EPSILON
                && location.y() > bb.bottom() + GeomConstants::DISTANCE_EPSILON
                && location.y() < bb.top() - GeomConstants::DISTANCE_EPSILON
            {
                // Obstacle port: return the obstacle's visibility bounding box.
                // C#: return oport.Obstacle.VisibilityBoundingBox;
                return obs.visibility_bounding_box();
            }
        }

        // Free point: return a degenerate rectangle at the rounded location.
        // C#: return new Rectangle(ApproximateComparer.Round(port.Location));
        Rectangle::from_point(Point::round(location))
    }

    /// Extract the original (unpadded) shapes from the session's obstacle tree.
    ///
    /// Each `Obstacle` stores the `Shape` it was built from, so shapes can be
    /// recovered when the padding changes or a shape is added/removed.
    fn extract_shapes(session: &RouterSession) -> Vec<Shape> {
        session
            .obstacle_tree
            .obstacles
            .iter()
            .map(|obs| obs.shape.clone())
            .collect()
    }

    /// Same as `extract_shapes` but named to match the rebuild call site clearly.
    fn extract_shapes_from_obstacles(session: &RouterSession) -> Vec<Shape> {
        Self::extract_shapes(session)
    }

    /// Build padded obstacle rectangles for nudging.
    fn padded_obstacles(&self) -> Vec<Rectangle> {
        self.session
            .obstacle_tree
            .obstacles
            .iter()
            .map(|obs| {
                let mut r = *obs.shape.bounding_box();
                r.pad(self.session.padding);
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
