use super::compass_direction::CompassDirection;
use super::static_graph_utility::StaticGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};

/// Manages transient (temporary) modifications to a visibility graph.
///
/// During port splicing, vertices and edges are temporarily added to the VG.
/// After path search completes, all additions are removed via `remove_from_graph`.
///
/// Ported from `TransientGraphUtility.ts`. Group boundary crossing logic and
/// `ExtendEdgeChain` / `ExtendSpliceWorker` are deferred.
pub struct TransientGraphUtility {
    /// Vertices added during this transient session.
    added_vertices: Vec<VertexId>,
    /// Toll-free edges added (source, target) — ascending order.
    added_edges: Vec<(VertexId, VertexId)>,
    /// Original edges removed by splitting — to be restored on cleanup.
    /// Stored as (source, target, weight, is_toll_free).
    edges_to_restore: Vec<(VertexId, VertexId, f64, bool)>,
}

impl TransientGraphUtility {
    pub fn new() -> Self {
        Self {
            added_vertices: Vec::new(),
            added_edges: Vec::new(),
            edges_to_restore: Vec::new(),
        }
    }

    /// Add a transient vertex at the given location.
    ///
    /// Always creates a new vertex and tracks it for removal.
    pub fn add_vertex(&mut self, graph: &mut VisibilityGraph, location: Point) -> VertexId {
        let id = graph.add_vertex(location);
        self.added_vertices.push(id);
        id
    }

    /// Find an existing vertex at `location`, or add a transient one.
    pub fn find_or_add_vertex(&mut self, graph: &mut VisibilityGraph, location: Point) -> VertexId {
        if let Some(existing) = graph.find_vertex(location) {
            return existing;
        }
        self.add_vertex(graph, location)
    }

    /// Find or add a vertex at `location`, splitting any existing VG edge
    /// that passes through that point.
    ///
    /// This is needed when creating vertices on obstacle boundaries where
    /// VG scan segment edges pass through. Without splitting, the vertex
    /// is isolated (0 edges) and unreachable by the path search.
    ///
    /// Matches the C# pattern where AddVertex + FindOrAddEdge together
    /// handle edge splitting via bracket detection.
    pub fn find_or_add_vertex_splitting(
        &mut self,
        graph: &mut VisibilityGraph,
        location: Point,
    ) -> VertexId {
        if let Some(existing) = graph.find_vertex(location) {
            return existing;
        }
        let vertex = self.add_vertex(graph, location);

        // Check all 4 directions for an existing VG edge that contains this point.
        // If found, split that edge at this vertex.
        use super::compass_direction::CompassDirection;
        for &dir in &CompassDirection::all() {
            if let Some(neighbor) = super::static_graph_utility::StaticGraphUtility::find_adjacent_vertex(graph, vertex, dir) {
                // The vertex already has a neighbor — it was connected by add_vertex
                // (this shouldn't happen for a new vertex, but check anyway).
                let _ = neighbor;
            }
        }

        // The vertex is new and isolated. Look for edges passing through its location
        // by checking if there are VG vertices on opposite sides of this point
        // along the same axis.
        let dirs = [
            (CompassDirection::North, CompassDirection::South),
            (CompassDirection::East, CompassDirection::West),
        ];
        for (dir_a, dir_b) in dirs {
            // Find nearest VG vertex in each direction along this axis
            let va = graph.find_nearest_vertex_in_direction(location, dir_a);
            let vb = graph.find_nearest_vertex_in_direction(location, dir_b);

            if let (Some(va_id), Some(vb_id)) = (va, vb) {
                let pa = graph.point(va_id);
                let pb = graph.point(vb_id);

                // Check they're on the same axis as our point
                let same_axis = match dir_a {
                    CompassDirection::North | CompassDirection::South => {
                        crate::geometry::point_comparer::GeomConstants::close(pa.x(), location.x())
                            && crate::geometry::point_comparer::GeomConstants::close(pb.x(), location.x())
                    }
                    CompassDirection::East | CompassDirection::West => {
                        crate::geometry::point_comparer::GeomConstants::close(pa.y(), location.y())
                            && crate::geometry::point_comparer::GeomConstants::close(pb.y(), location.y())
                    }
                };

                if !same_axis {
                    continue;
                }

                // Check if there's an edge between va and vb (or any edge chain
                // that passes through our point). If so, split it.
                if graph.find_edge(va_id, vb_id).is_some() {
                    self.split_edge(graph, va_id, vb_id, vertex);
                    break;
                }
                if graph.find_edge(vb_id, va_id).is_some() {
                    self.split_edge(graph, vb_id, va_id, vertex);
                    break;
                }
            }
        }

        vertex
    }

    /// Add a transient edge with bracket detection, using default weight.
    pub fn find_or_add_edge_vv(
        &mut self,
        graph: &mut VisibilityGraph,
        source: VertexId,
        target: VertexId,
    ) {
        self.find_or_add_edge(graph, source, target, NORMAL_WEIGHT);
    }

    /// Add a transient edge with bracket detection.
    ///
    /// If there is an existing edge chain from `source` toward `target` that
    /// brackets `target`, the bracketing edge is split. Otherwise a new
    /// toll-free edge is created.
    ///
    /// Faithfully ports `FindOrAddEdge` from the TS source.
    pub fn find_or_add_edge(
        &mut self,
        graph: &mut VisibilityGraph,
        source: VertexId,
        target: VertexId,
        weight: f64,
    ) {
        let source_point = graph.point(source);
        let target_point = graph.point(target);

        let dir_to_target = match CompassDirection::from_points(source_point, target_point) {
            Some(d) => d,
            None => return, // same point — nothing to do
        };

        // GetBrackets: find bracketing vertices in both directions.
        let mut bracket_source = source;
        let mut bracket_target = target;
        let mut split_vertex = target;

        let found_forward = Self::find_bracketing_vertices(
            graph,
            source,
            target_point,
            dir_to_target,
            &mut bracket_source,
            &mut bracket_target,
        );

        if !found_forward {
            // No bracketing from source side. Try from target side in reverse.
            let reverse_dir = dir_to_target.opposite();
            let mut rev_bracket_source = target;
            let mut rev_bracket_target = target; // placeholder
            let found_reverse = Self::find_bracketing_vertices(
                graph,
                target,
                source_point,
                reverse_dir,
                &mut rev_bracket_source,
                &mut rev_bracket_target,
            );

            if found_reverse {
                // Target side brackets source. Update bracket_source and split_vertex.
                bracket_source = rev_bracket_target;
                split_vertex = source;
            }
            bracket_target = rev_bracket_source;
        }

        // If there's an existing edge between bracket_source and bracket_target, split it.
        let edge_info =
            graph.find_edge_pp(graph.point(bracket_source), graph.point(bracket_target));

        if let Some((edge_src, edge_tgt, _edge_weight)) = edge_info {
            self.split_edge(graph, edge_src, edge_tgt, split_vertex);
        } else {
            self.create_edge(graph, bracket_source, bracket_target, weight);
        }
    }

    /// Walk the edge chain from `source` in `dir_to_target` looking for a pair
    /// of vertices that bracket `target_point`.
    ///
    /// Returns true if a bracket is found. On return:
    /// - `bracket_source` is the vertex just before or at `target_point`
    /// - `bracket_target` is the vertex just past `target_point`
    ///
    /// If `bracket_target.point == target_point` (close enough), the edge
    /// already exists and this is considered a bracket found.
    ///
    /// Faithfully ports `FindBracketingVertices` from the TS source.
    fn find_bracketing_vertices(
        graph: &VisibilityGraph,
        source: VertexId,
        target_point: Point,
        dir_to_target: CompassDirection,
        bracket_source: &mut VertexId,
        bracket_target: &mut VertexId,
    ) -> bool {
        *bracket_source = source;
        loop {
            let next =
                StaticGraphUtility::find_adjacent_vertex(graph, *bracket_source, dir_to_target);
            match next {
                None => {
                    *bracket_target = *bracket_source;
                    break;
                }
                Some(next_id) => {
                    *bracket_target = next_id;
                    let next_point = graph.point(next_id);

                    // If next vertex is at the target point, edge already exists.
                    if GeomConstants::close(next_point.x(), target_point.x())
                        && GeomConstants::close(next_point.y(), target_point.y())
                    {
                        return true;
                    }

                    // Check if next vertex is past target in the traversal direction.
                    let dir_from_next = CompassDirection::from_points(next_point, target_point);
                    if dir_from_next != Some(dir_to_target) {
                        // bracket_target is past target — we found a bracket.
                        break;
                    }

                    // Keep walking.
                    *bracket_source = next_id;
                }
            }
        }

        // Return true if bracket_target != bracket_source (i.e., we found a far vertex).
        *bracket_target != *bracket_source
    }

    /// Create a toll-free edge between two vertices.
    ///
    /// Edges are stored in ascending order (lower → higher) matching the TS
    /// convention where `IsPureLower(source, target)` must hold.
    fn create_edge(
        &mut self,
        graph: &mut VisibilityGraph,
        first: VertexId,
        second: VertexId,
        weight: f64,
    ) {
        // All edges in the graph are stored in ascending order.
        let (source, target) =
            if StaticGraphUtility::is_pure_lower(graph.point(first), graph.point(second)) {
                (first, second)
            } else {
                (second, first)
            };

        graph.add_toll_free_edge(source, target, weight);
        self.added_edges.push((source, target));
    }

    /// Split edge `source→far` by inserting `split_vertex` in between.
    ///
    /// Produces `source→split_vertex` and `split_vertex→far`.
    /// If `split_vertex` is at either endpoint, no split is needed.
    ///
    /// Faithfully ports `SplitEdge` from the TS source.
    pub fn split_edge(
        &mut self,
        graph: &mut VisibilityGraph,
        source: VertexId,
        far: VertexId,
        split_vertex: VertexId,
    ) {
        let source_point = graph.point(source);
        let far_point = graph.point(far);
        let split_point = graph.point(split_vertex);

        // No split needed if split_vertex is at either endpoint.
        if (GeomConstants::close(source_point.x(), split_point.x())
            && GeomConstants::close(source_point.y(), split_point.y()))
            || (GeomConstants::close(far_point.x(), split_point.x())
                && GeomConstants::close(far_point.y(), split_point.y()))
        {
            return;
        }

        // Find the original edge weight.
        let original_edge = graph.find_edge(source, far);
        let (orig_weight, orig_toll_free) = match original_edge {
            Some(e) => (e.weight, e.is_toll_free),
            None => return, // edge doesn't exist
        };

        // Store original for restoration if it was not toll-free.
        if !orig_toll_free {
            self.edges_to_restore
                .push((source, far, orig_weight, false));
        }

        // Remove the original edge.
        graph.remove_edge(source, far);

        // Create the two replacement edges with the original weight.
        self.create_edge(graph, source, split_vertex, orig_weight);
        self.create_edge(graph, split_vertex, far, orig_weight);
    }

    /// Connect `source_vertex` to `target_vertex`, adding a bend vertex if needed.
    ///
    /// `final_edge_dir` is the required direction of the final segment to
    /// `target_vertex`. If they are collinear, one edge suffices; otherwise
    /// a bend vertex is created.
    ///
    /// Faithfully ports `ConnectVertexToTargetVertex` from the TS source.
    pub fn connect_vertex_to_target(
        &mut self,
        graph: &mut VisibilityGraph,
        source: VertexId,
        target: VertexId,
        final_edge_dir: CompassDirection,
        weight: f64,
    ) {
        let source_point = graph.point(source);
        let target_point = graph.point(target);

        if GeomConstants::close(source_point.x(), target_point.x())
            && GeomConstants::close(source_point.y(), target_point.y())
        {
            return;
        }

        // If collinear, just one edge.
        if let Some(_dir) = CompassDirection::from_points(source_point, target_point) {
            if StaticGraphUtility::is_collinear(source_point, target_point) {
                self.find_or_add_edge_vv(graph, source, target);
                return;
            }
        }

        // Need a bend vertex.
        let bend_point =
            StaticGraphUtility::find_bend_point_between(source_point, target_point, final_edge_dir);
        let bend_vertex = self.find_or_add_vertex(graph, bend_point);
        self.find_or_add_edge(graph, source, bend_vertex, weight);
        self.find_or_add_edge(graph, bend_vertex, target, weight);
    }

    /// Add an edge from `source_vertex` to a point on `target_edge`, splitting
    /// the target edge if needed.
    ///
    /// Returns the vertex at `target_intersect`.
    ///
    /// Faithfully ports `AddEdgeToTargetEdge` from the TS source.
    pub fn add_edge_to_target_edge(
        &mut self,
        graph: &mut VisibilityGraph,
        source: VertexId,
        target_edge_source: VertexId,
        target_edge_target: VertexId,
        target_intersect: Point,
    ) -> VertexId {
        let target_vertex = match graph.find_vertex(target_intersect) {
            Some(v) => v,
            None => {
                let v = self.add_vertex(graph, target_intersect);
                self.split_edge(graph, target_edge_source, target_edge_target, v);
                v
            }
        };
        self.find_or_add_edge_vv(graph, source, target_vertex);
        target_vertex
    }

    /// Find the next edge from `vertex` in the given direction.
    pub fn find_next_edge(
        graph: &VisibilityGraph,
        vertex: VertexId,
        dir: CompassDirection,
    ) -> Option<(VertexId, f64)> {
        StaticGraphUtility::find_adjacent_edge(graph, vertex, dir)
    }

    /// Find the edge from `start_vertex` in `dir` that is perpendicular to
    /// `point_location`, or that contains it.
    ///
    /// Walks the edge chain in `dir` until we find an edge that brackets or
    /// reaches `point_location` on the perpendicular axis.
    ///
    /// Faithfully ports `FindPerpendicularOrContainingEdge` from the TS source.
    pub fn find_perpendicular_or_containing_edge(
        graph: &VisibilityGraph,
        start_vertex: VertexId,
        dir: CompassDirection,
        point_location: Point,
    ) -> Option<(VertexId, VertexId)> {
        let mut current = start_vertex;
        loop {
            let next_id = StaticGraphUtility::find_adjacent_vertex(graph, current, dir)?;
            let next_point = graph.point(next_id);

            // If next vertex is past point_location (opposite direction), we bracket it.
            let dir_check = CompassDirection::from_points(next_point, point_location);
            if dir_check == Some(dir.opposite()) {
                return Some((current, next_id));
            }
            // If on the same point, we found it.
            if GeomConstants::close(next_point.x(), point_location.x())
                && GeomConstants::close(next_point.y(), point_location.y())
            {
                return Some((current, next_id));
            }

            current = next_id;
        }
    }

    /// Find the nearest perpendicular or containing edge by first walking
    /// toward `point_location`, then searching for perpendicular edges.
    ///
    /// Similar to `find_perpendicular_or_containing_edge`, but first tries to
    /// move closer to `point_location` along the perpendicular axis before
    /// searching in `dir`.
    ///
    /// Faithfully ports `FindNearestPerpendicularOrContainingEdge` from C#
    /// TransientGraphUtility.cs lines 294-329.
    ///
    /// Algorithm:
    /// 1. Compute `dir_toward_location` = the perpendicular component of the
    ///    direction from `start_vertex` to `point_location` (excluding `dir`).
    /// 2. Walk toward `point_location` along `dir_toward_location` as far as
    ///    possible without overshooting.
    /// 3. From the closest vertex found, try `find_perpendicular_or_containing_edge`.
    /// 4. If that fails, walk back toward `start_vertex` trying each vertex
    ///    until we find a perpendicular edge or return to `start_vertex`.
    pub fn find_nearest_perpendicular_or_containing_edge(
        graph: &VisibilityGraph,
        start_vertex: VertexId,
        dir: CompassDirection,
        point_location: Point,
    ) -> Option<(VertexId, VertexId)> {
        let start_point = graph.point(start_vertex);

        // Compute the perpendicular direction toward point_location.
        // C#: Direction dirTowardLocation = ~dir & GetDirections(startVertex.Point, pointLocation)
        let dir_toward_location =
            Self::perpendicular_component(start_point, point_location, dir);

        let mut current_vertex = start_vertex;

        // Phase 1: Move toward pointLocation as far as we can.
        if let Some(toward_dir) = dir_toward_location {
            let mut current_dir_toward = dir_toward_location;
            while current_dir_toward.is_some() {
                let next = StaticGraphUtility::find_adjacent_vertex(
                    graph,
                    current_vertex,
                    toward_dir,
                );
                let next_id = match next {
                    Some(id) => id,
                    None => break,
                };

                let next_point = graph.point(next_id);
                // Check if next vertex has overshot point_location in the toward direction.
                if CompassDirection::from_points(next_point, point_location)
                    == Some(toward_dir.opposite())
                {
                    break;
                }

                current_vertex = next_id;
                // Recompute the perpendicular component from the new position.
                current_dir_toward = Self::perpendicular_component(
                    graph.point(current_vertex),
                    point_location,
                    dir,
                );
            }
        }

        // Phase 2: Find a perpendicular edge, walking back toward start if needed.
        loop {
            let perp_edge = Self::find_perpendicular_or_containing_edge(
                graph,
                current_vertex,
                dir,
                point_location,
            );
            if perp_edge.is_some() || current_vertex == start_vertex {
                return perp_edge;
            }

            // Walk back toward start_vertex (opposite of dir_toward_location).
            match dir_toward_location {
                Some(toward_dir) => {
                    match StaticGraphUtility::find_adjacent_vertex(
                        graph,
                        current_vertex,
                        toward_dir.opposite(),
                    ) {
                        Some(prev) => current_vertex = prev,
                        None => return None,
                    }
                }
                None => return None,
            }
        }
    }

    /// Compute the perpendicular component of the direction from `from` to `to`,
    /// excluding the given `main_dir` and its opposite.
    ///
    /// For example, if `main_dir` is East and the direction from `from` to `to`
    /// is NorthEast, this returns Some(North).
    /// If collinear (same axis as main_dir), returns None.
    fn perpendicular_component(
        from: Point,
        to: Point,
        main_dir: CompassDirection,
    ) -> Option<CompassDirection> {
        let dx = to.x() - from.x();
        let dy = to.y() - from.y();
        let eps = GeomConstants::DISTANCE_EPSILON;

        match main_dir {
            // main_dir is horizontal -> perpendicular component is vertical
            CompassDirection::East | CompassDirection::West => {
                if dy > eps {
                    Some(CompassDirection::North)
                } else if dy < -eps {
                    Some(CompassDirection::South)
                } else {
                    None // collinear
                }
            }
            // main_dir is vertical -> perpendicular component is horizontal
            CompassDirection::North | CompassDirection::South => {
                if dx > eps {
                    Some(CompassDirection::East)
                } else if dx < -eps {
                    Some(CompassDirection::West)
                } else {
                    None // collinear
                }
            }
        }
    }

    /// Remove all transient additions, restoring the graph to its pre-splicing state.
    ///
    /// Faithfully ports `RemoveFromGraph` from TransientGraphUtility.ts lines 178-182.
    /// Order matters: vertices first (removes all incident edges), then remaining
    /// edges, then restore split edges.
    pub fn remove_from_graph(&mut self, graph: &mut VisibilityGraph) {
        self.remove_added_vertices(graph);
        self.remove_added_edges(graph);
        self.restore_removed_edges(graph);
    }

    /// Remove all transient vertices (and their incident edges) from the graph.
    ///
    /// Faithfully ports `RemoveAddedVertices` from TransientGraphUtility.ts lines 184-193.
    fn remove_added_vertices(&mut self, graph: &mut VisibilityGraph) {
        for &vertex in &self.added_vertices {
            // Removing all transient vertices will remove all associated transient
            // edges as well.
            if graph.find_vertex(graph.point(vertex)).is_some() {
                graph.remove_vertex(vertex);
            }
        }
        self.added_vertices.clear();
    }

    /// Remove any remaining transient edges that weren't already removed by vertex removal.
    ///
    /// Faithfully ports `RemoveAddedEdges` from TransientGraphUtility.ts lines 195-203.
    fn remove_added_edges(&mut self, graph: &mut VisibilityGraph) {
        for &(s, t) in &self.added_edges {
            // If either vertex was removed, so was the edge, so just check source.
            if graph.find_vertex(graph.point(s)).is_some() {
                graph.remove_edge(s, t);
            }
        }
        self.added_edges.clear();
    }

    /// Restore original edges that were split during transient splicing.
    ///
    /// Faithfully ports `RestoreRemovedEdges` from TransientGraphUtility.ts lines 206-218.
    fn restore_removed_edges(&mut self, graph: &mut VisibilityGraph) {
        for &(s, t, w, _toll_free) in &self.edges_to_restore {
            graph.add_edge(s, t, w);
        }
        self.edges_to_restore.clear();
    }

    /// Access the list of added vertices (for external inspection/testing).
    pub fn added_vertices(&self) -> &[VertexId] {
        &self.added_vertices
    }

    /// Access the list of added edges (for external inspection/testing).
    pub fn added_edges(&self) -> &[(VertexId, VertexId)] {
        &self.added_edges
    }
}

impl Default for TransientGraphUtility {
    fn default() -> Self {
        Self::new()
    }
}

/// Default weight for normal scan segments (matches ScanSegment.NormalWeight = 1).
const NORMAL_WEIGHT: f64 = 1.0;
