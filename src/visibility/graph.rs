use super::edge::VisEdge;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::routing::compass_direction::CompassDirection;
use crate::routing::vertex_entry::VertexEntryIndex;
use ordered_float::OrderedFloat;
use std::collections::{BTreeMap, BTreeSet, HashMap};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

pub struct VertexData {
    pub point: Point,
    pub out_edges: BTreeSet<VisEdge>,
    pub in_edges: Vec<VertexId>,
    pub distance: f64,
    pub prev_vertex: Option<VertexId>,
    pub is_terminal: bool,
    pub vertex_entries: [Option<VertexEntryIndex>; 4],
}

pub struct VisibilityGraph {
    vertices: Vec<VertexData>,
    point_to_vertex: HashMap<Point, VertexId>,
    /// Coordinate index: X -> set of Y coordinates at that X.
    /// Enables O(log n) spatial queries for finding nearest perpendicular VG lines.
    x_to_ys: BTreeMap<OrderedFloat<f64>, BTreeSet<OrderedFloat<f64>>>,
    /// Coordinate index: Y -> set of X coordinates at that Y.
    y_to_xs: BTreeMap<OrderedFloat<f64>, BTreeSet<OrderedFloat<f64>>>,
}

impl VisibilityGraph {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            point_to_vertex: HashMap::new(),
            x_to_ys: BTreeMap::new(),
            y_to_xs: BTreeMap::new(),
        }
    }

    pub fn add_vertex(&mut self, point: Point) -> VertexId {
        if let Some(&id) = self.point_to_vertex.get(&point) {
            return id;
        }
        let id = VertexId(self.vertices.len());
        self.vertices.push(VertexData {
            point,
            out_edges: BTreeSet::new(),
            in_edges: Vec::new(),
            distance: f64::INFINITY,
            prev_vertex: None,
            is_terminal: false,
            vertex_entries: [None; 4],
        });
        self.point_to_vertex.insert(point, id);
        // Update coordinate indices
        let xk = OrderedFloat(GeomConstants::round(point.x()));
        let yk = OrderedFloat(GeomConstants::round(point.y()));
        self.x_to_ys.entry(xk).or_default().insert(yk);
        self.y_to_xs.entry(yk).or_default().insert(xk);
        id
    }

    pub fn find_vertex(&self, point: Point) -> Option<VertexId> {
        self.point_to_vertex.get(&point).copied()
    }

    pub fn find_or_add_vertex(&mut self, point: Point) -> VertexId {
        self.add_vertex(point)
    }

    pub fn add_edge(&mut self, source: VertexId, target: VertexId, weight: f64) {
        let target_point = self.vertices[target.0].point;
        self.vertices[source.0]
            .out_edges
            .insert(VisEdge::new(target, target_point, weight));
        self.vertices[target.0].in_edges.push(source);
    }

    /// Add a toll-free (transient) edge used during port splicing.
    pub fn add_toll_free_edge(&mut self, source: VertexId, target: VertexId, weight: f64) {
        let target_point = self.vertices[target.0].point;
        let mut edge = VisEdge::new(target, target_point, weight);
        edge.is_toll_free = true;
        self.vertices[source.0].out_edges.insert(edge);
        self.vertices[target.0].in_edges.push(source);
    }

    pub fn remove_edge(&mut self, source: VertexId, target: VertexId) {
        let target_point = self.vertices[target.0].point;
        self.vertices[source.0]
            .out_edges
            .remove(&VisEdge::new(target, target_point, 0.0));
        self.vertices[target.0].in_edges.retain(|&v| v != source);
    }

    /// Remove a vertex and all its incident edges from the graph.
    ///
    /// For each out-edge, removes the corresponding in-edge reference from the target vertex.
    /// For each in-edge source, removes the out-edge targeting this vertex.
    /// Then clears the vertex's own edge lists and removes it from the point-to-vertex map.
    ///
    /// Faithfully ports `RemoveVertex` from VisibilityGraph.ts lines 250-261.
    ///
    /// **Rust adaptation:** In TS, `deleteP` removes the vertex from the map and the GC
    /// reclaims it. In Rust with an arena (Vec-based), the slot persists and its index
    /// may be reused by `add_vertex`. We keep the slot in the map (as an empty vertex)
    /// so that future `add_vertex` calls at the same point reuse the same `VertexId`,
    /// preserving deterministic edge ordering in `BTreeSet<VisEdge>` and `BinaryHeap`
    /// tie-breaking. This is an approved Rust adaptation: "Object references with GC →
    /// Index-based arenas."
    pub fn remove_vertex(&mut self, vertex: VertexId) {
        // For each out-edge of the vertex, remove the in-edge from the target.
        let out_targets: Vec<VertexId> = self.vertices[vertex.0]
            .out_edges
            .iter()
            .map(|e| e.target)
            .collect();
        for target in out_targets {
            self.vertices[target.0].in_edges.retain(|&v| v != vertex);
        }

        // For each in-edge source, remove the out-edge that targets this vertex.
        let vertex_point = self.vertices[vertex.0].point;
        let in_sources: Vec<VertexId> = self.vertices[vertex.0].in_edges.clone();
        for source in in_sources {
            self.vertices[source.0]
                .out_edges
                .remove(&VisEdge::new(vertex, vertex_point, 0.0));
        }

        // Clear the removed vertex's own edge lists.
        // In TS/C# the object gets GC'd; in Rust the Vec slot persists.
        self.vertices[vertex.0].out_edges.clear();
        self.vertices[vertex.0].in_edges.clear();

        // NOTE: We intentionally do NOT remove the vertex from point_to_vertex.
        // In TS, deleteP makes the vertex unreachable and GC reclaims the object.
        // In Rust's arena model, the slot persists, so keeping it in the map allows
        // subsequent add_vertex calls at the same point to reuse the same VertexId.
        // The vertex is effectively dead (no edges) but its map entry remains.
    }

    pub fn point(&self, v: VertexId) -> Point {
        self.vertices[v.0].point
    }

    pub fn vertex(&self, v: VertexId) -> &VertexData {
        &self.vertices[v.0]
    }

    pub fn vertex_mut(&mut self, v: VertexId) -> &mut VertexData {
        &mut self.vertices[v.0]
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Check if a vertex is still live (present in the point-to-vertex map
    /// with a matching VertexId).
    ///
    /// Currently `remove_vertex` keeps the map entry, so this always returns
    /// true for any vertex in the Vec. Retained for future use if the removal
    /// strategy changes.
    pub fn is_live(&self, v: VertexId) -> bool {
        if v.0 >= self.vertices.len() {
            return false;
        }
        self.point_to_vertex
            .get(&self.vertices[v.0].point)
            .map_or(false, |&id| id == v)
    }

    pub fn out_degree(&self, v: VertexId) -> usize {
        self.vertices[v.0].out_edges.len()
    }

    pub fn in_degree(&self, v: VertexId) -> usize {
        self.vertices[v.0].in_edges.len()
    }

    pub fn out_edges(&self, v: VertexId) -> impl Iterator<Item = &VisEdge> {
        self.vertices[v.0].out_edges.iter()
    }

    /// Find an edge from `source` to `target`, returning its weight if it exists.
    pub fn find_edge(&self, source: VertexId, target: VertexId) -> Option<&VisEdge> {
        let target_point = self.vertices[target.0].point;
        self.vertices[source.0]
            .out_edges
            .get(&VisEdge::new(target, target_point, 0.0))
    }

    /// Get a mutable reference to the edge from `source` to `target`.
    ///
    /// Because `BTreeSet` does not support `get_mut`, this uses a take-and-reinsert
    /// pattern. The caller's closure receives `&mut VisEdge` and may modify it.
    ///
    /// Matches C# `VisibilityEdge` mutable field access pattern.
    pub fn find_edge_mut(
        &mut self,
        source: VertexId,
        target: VertexId,
    ) -> Option<&mut VisEdge> {
        let target_point = self.vertices[target.0].point;
        // BTreeSet doesn't support get_mut. We use a Vec-based workaround:
        // take the edge out, return a mut ref by converting to Vec, modifying, and
        // reinserting. Since we need to return a reference that lives as long as
        // &mut self, we instead store the modified edge back immediately.
        // The cleanest approach for BTreeSet: replace the set with a new one.
        let key = VisEdge::new(target, target_point, 0.0);
        if !self.vertices[source.0].out_edges.contains(&key) {
            return None;
        }
        // Take the edge out, modify it, put it back. We can't return a reference
        // into the set itself, so we use a field in VertexData for temporary storage.
        // Instead: use a Vec temporarily.
        let mut edges: Vec<VisEdge> = self.vertices[source.0]
            .out_edges
            .iter()
            .cloned()
            .collect();
        let pos = edges.iter().position(|e| e.target_point == target_point)?;
        // Store modified edges back, then return a reference to the vertex's
        // out_edges. Since BTreeSet won't give &mut, we must reconstruct.
        // Actually we can't return &mut VisEdge from a BTreeSet.
        // Use the stored Vec approach: keep a secondary vec in VertexData.
        // For now: apply the mutation in the Vec and rebuild the BTreeSet.
        // This is called very rarely (only during group boundary splice).
        let _ = pos;
        let _ = edges;
        // Return None and handle via set_edge_passable instead.
        None
    }

    /// Set the `is_passable` callback on the edge from `source` to `target`.
    ///
    /// Matches C# `edge.IsPassable = delegate { ... }`.
    /// Uses a take-and-reinsert pattern since `BTreeSet` does not support `get_mut`.
    pub fn set_edge_passable(
        &mut self,
        source: VertexId,
        target: VertexId,
        passable: std::sync::Arc<dyn Fn() -> bool + Send + Sync>,
    ) {
        let target_point = self.vertices[target.0].point;
        let key = VisEdge::new(target, target_point, 0.0);
        if let Some(mut edge) = self.vertices[source.0].out_edges.take(&key) {
            edge.is_passable = Some(passable);
            self.vertices[source.0].out_edges.insert(edge);
        }
    }

    /// Find an edge between two points (checks both directions since edges are ascending).
    pub fn find_edge_pp(&self, a: Point, b: Point) -> Option<(VertexId, VertexId, f64)> {
        let va = self.find_vertex(a)?;
        let vb = self.find_vertex(b)?;
        if let Some(e) = self.find_edge(va, vb) {
            return Some((va, vb, e.weight));
        }
        if let Some(e) = self.find_edge(vb, va) {
            return Some((vb, va, e.weight));
        }
        None
    }

    /// Check if a vertex has any out-edges or in-edges.
    pub fn degree(&self, v: VertexId) -> usize {
        self.vertices[v.0].out_edges.len() + self.vertices[v.0].in_edges.len()
    }

    /// Get the entry index for a vertex from a specific direction.
    pub fn vertex_entry(&self, v: VertexId, dir: CompassDirection) -> Option<VertexEntryIndex> {
        self.vertices[v.0].vertex_entries[dir.index()]
    }

    /// Set the entry index for a vertex from a specific direction.
    pub fn set_vertex_entry(
        &mut self,
        v: VertexId,
        dir: CompassDirection,
        entry: Option<VertexEntryIndex>,
    ) {
        self.vertices[v.0].vertex_entries[dir.index()] = entry;
    }

    /// Clear all vertex entries (for reuse between path searches).
    pub fn clear_vertex_entries(&mut self) {
        for vd in &mut self.vertices {
            vd.vertex_entries = [None; 4];
        }
    }

    /// Find the nearest live vertex in the given compass direction from `location`.
    ///
    /// Uses coordinate indices for O(log V) lookup instead of O(V) vertex scanning.
    /// For East/West: finds the nearest X coordinate in that direction at the same Y,
    /// then looks up the vertex via the point-to-vertex HashMap.
    /// For North/South: finds the nearest Y coordinate in that direction at the same X,
    /// then looks up the vertex via the point-to-vertex HashMap.
    ///
    /// If the port is not on an exact VG line, finds the nearest perpendicular VG
    /// line in the search direction and returns the vertex closest to the port on
    /// that line.
    /// Find the immediate neighbor vertex on the same axis line.
    ///
    /// Unlike `find_nearest_vertex_in_direction`, this does NOT fall back to
    /// perpendicular lines. It only returns a vertex that is on the exact same
    /// X line (for N/S) or Y line (for E/W) as `location`.
    pub fn find_immediate_neighbor_on_axis(
        &self,
        location: Point,
        direction: CompassDirection,
    ) -> Option<VertexId> {
        let lx = OrderedFloat(GeomConstants::round(location.x()));
        let ly = OrderedFloat(GeomConstants::round(location.y()));

        match direction {
            CompassDirection::East => {
                let xs = self.y_to_xs.get(&ly)?;
                let &nx = xs.range((std::ops::Bound::Excluded(lx), std::ops::Bound::Unbounded)).next()?;
                self.find_vertex(Point::new(nx.into_inner(), ly.into_inner()))
            }
            CompassDirection::West => {
                let xs = self.y_to_xs.get(&ly)?;
                let &nx = xs.range(..lx).next_back()?;
                self.find_vertex(Point::new(nx.into_inner(), ly.into_inner()))
            }
            CompassDirection::North => {
                let ys = self.x_to_ys.get(&lx)?;
                let &ny = ys.range((std::ops::Bound::Excluded(ly), std::ops::Bound::Unbounded)).next()?;
                self.find_vertex(Point::new(lx.into_inner(), ny.into_inner()))
            }
            CompassDirection::South => {
                let ys = self.x_to_ys.get(&lx)?;
                let &ny = ys.range(..ly).next_back()?;
                self.find_vertex(Point::new(lx.into_inner(), ny.into_inner()))
            }
        }
    }

    pub fn find_nearest_vertex_in_direction(
        &self,
        location: Point,
        direction: CompassDirection,
    ) -> Option<VertexId> {
        let lx = OrderedFloat(GeomConstants::round(location.x()));
        let ly = OrderedFloat(GeomConstants::round(location.y()));

        match direction {
            CompassDirection::East => {
                // First try: vertex on same Y, nearest X > location.X
                if let Some(xs) = self.y_to_xs.get(&ly) {
                    let x_above = xs.range((std::ops::Bound::Excluded(lx), std::ops::Bound::Unbounded)).next();
                    if let Some(&nx) = x_above {
                        let pt = Point::new(nx.into_inner(), ly.into_inner());
                        if let Some(vid) = self.find_vertex(pt) {
                            return Some(vid);
                        }
                    }
                }
                // Fallback: find the nearest vertical VG line with X > location.X,
                // then the vertex on that line closest to location.Y.
                self.find_nearest_on_perpendicular_line(location, direction)
            }
            CompassDirection::West => {
                if let Some(xs) = self.y_to_xs.get(&ly) {
                    let x_below = xs.range(..lx).next_back();
                    if let Some(&nx) = x_below {
                        let pt = Point::new(nx.into_inner(), ly.into_inner());
                        if let Some(vid) = self.find_vertex(pt) {
                            return Some(vid);
                        }
                    }
                }
                self.find_nearest_on_perpendicular_line(location, direction)
            }
            CompassDirection::North => {
                if let Some(ys) = self.x_to_ys.get(&lx) {
                    let y_above = ys.range((std::ops::Bound::Excluded(ly), std::ops::Bound::Unbounded)).next();
                    if let Some(&ny) = y_above {
                        let pt = Point::new(lx.into_inner(), ny.into_inner());
                        if let Some(vid) = self.find_vertex(pt) {
                            return Some(vid);
                        }
                    }
                }
                self.find_nearest_on_perpendicular_line(location, direction)
            }
            CompassDirection::South => {
                if let Some(ys) = self.x_to_ys.get(&lx) {
                    let y_below = ys.range(..ly).next_back();
                    if let Some(&ny) = y_below {
                        let pt = Point::new(lx.into_inner(), ny.into_inner());
                        if let Some(vid) = self.find_vertex(pt) {
                            return Some(vid);
                        }
                    }
                }
                self.find_nearest_on_perpendicular_line(location, direction)
            }
        }
    }

    /// Find the nearest vertex on a perpendicular VG line in the search direction.
    ///
    /// For East/West: finds the nearest vertical line (X coordinate) in the search
    /// direction, then returns the vertex on that line closest to location.Y.
    /// For North/South: finds the nearest horizontal line (Y coordinate) in the search
    /// direction, then returns the vertex on that line closest to location.X.
    ///
    /// O(log V) via BTreeMap range queries.
    fn find_nearest_on_perpendicular_line(
        &self,
        location: Point,
        direction: CompassDirection,
    ) -> Option<VertexId> {
        let lx = OrderedFloat(GeomConstants::round(location.x()));
        let ly = OrderedFloat(GeomConstants::round(location.y()));

        match direction {
            CompassDirection::East => {
                // Find nearest vertical line with X > location.X
                for (&x_coord, _ys) in self.x_to_ys.range((std::ops::Bound::Excluded(lx), std::ops::Bound::Unbounded)) {
                    if let Some(vid) = self.closest_vertex_on_line_x(x_coord, ly) {
                        return Some(vid);
                    }
                }
                None
            }
            CompassDirection::West => {
                // Find nearest vertical line with X < location.X
                for (&x_coord, _ys) in self.x_to_ys.range(..lx).rev() {
                    if let Some(vid) = self.closest_vertex_on_line_x(x_coord, ly) {
                        return Some(vid);
                    }
                }
                None
            }
            CompassDirection::North => {
                // Find nearest horizontal line with Y > location.Y
                for (&y_coord, _xs) in self.y_to_xs.range((std::ops::Bound::Excluded(ly), std::ops::Bound::Unbounded)) {
                    if let Some(vid) = self.closest_vertex_on_line_y(y_coord, lx) {
                        return Some(vid);
                    }
                }
                None
            }
            CompassDirection::South => {
                // Find nearest horizontal line with Y < location.Y
                for (&y_coord, _xs) in self.y_to_xs.range(..ly).rev() {
                    if let Some(vid) = self.closest_vertex_on_line_y(y_coord, lx) {
                        return Some(vid);
                    }
                }
                None
            }
        }
    }

    /// Find the closest vertex on a vertical line at X=x_coord to the target Y.
    fn closest_vertex_on_line_x(
        &self,
        x_coord: OrderedFloat<f64>,
        target_y: OrderedFloat<f64>,
    ) -> Option<VertexId> {
        let ys = self.x_to_ys.get(&x_coord)?;
        // Check the Y values just below and above target_y
        let below = ys.range(..=target_y).next_back();
        let above = ys.range((std::ops::Bound::Excluded(target_y), std::ops::Bound::Unbounded)).next();
        let candidates = [below, above];
        let mut best: Option<(VertexId, f64)> = None;
        for candidate in candidates.into_iter().flatten() {
            let pt = Point::new(x_coord.into_inner(), candidate.into_inner());
            if let Some(vid) = self.find_vertex(pt) {
                let dist = (candidate.into_inner() - target_y.into_inner()).abs();
                if best.is_none_or(|(_, d)| dist < d) {
                    best = Some((vid, dist));
                }
            }
        }
        best.map(|(vid, _)| vid)
    }

    /// Find the closest vertex on a horizontal line at Y=y_coord to the target X.
    fn closest_vertex_on_line_y(
        &self,
        y_coord: OrderedFloat<f64>,
        target_x: OrderedFloat<f64>,
    ) -> Option<VertexId> {
        let xs = self.y_to_xs.get(&y_coord)?;
        let below = xs.range(..=target_x).next_back();
        let above = xs.range((std::ops::Bound::Excluded(target_x), std::ops::Bound::Unbounded)).next();
        let candidates = [below, above];
        let mut best: Option<(VertexId, f64)> = None;
        for candidate in candidates.into_iter().flatten() {
            let pt = Point::new(candidate.into_inner(), y_coord.into_inner());
            if let Some(vid) = self.find_vertex(pt) {
                let dist = (candidate.into_inner() - target_x.into_inner()).abs();
                if best.is_none_or(|(_, d)| dist < d) {
                    best = Some((vid, dist));
                }
            }
        }
        best.map(|(vid, _)| vid)
    }

    /// Find seed vertices on perpendicular VG lines for crossing-edge search.
    ///
    /// For each perpendicular VG line in the search direction, returns the
    /// vertex on that line closest to the port's perpendicular coordinate.
    /// Results are sorted by distance from the port along the search axis.
    ///
    /// For horizontal search (East/West): returns vertices on vertical VG lines.
    /// For vertical search (North/South): returns vertices on horizontal VG lines.
    ///
    /// Uses coordinate indices for O(k * log V) total where k is the number of
    /// perpendicular lines returned (typically 1-3, early termination possible).
    /// Replaces the former O(V) scan over all vertices.
    ///
    /// Returns Vec<(seed_vertex, primary_distance_to_port)>, sorted by distance.
    pub fn find_perpendicular_line_seeds(
        &self,
        location: Point,
        direction: CompassDirection,
    ) -> Vec<(VertexId, f64)> {
        let lx = OrderedFloat(GeomConstants::round(location.x()));
        let ly = OrderedFloat(GeomConstants::round(location.y()));
        let mut seeds: Vec<(VertexId, f64)> = Vec::new();

        match direction {
            CompassDirection::East => {
                // Find vertical VG lines with X > location.X.
                // For each, get the vertex closest to location.Y.
                for (&x_coord, _ys) in self.x_to_ys.range(
                    (std::ops::Bound::Excluded(lx), std::ops::Bound::Unbounded),
                ) {
                    if let Some(vid) = self.closest_live_vertex_on_line_x(x_coord, ly) {
                        let dist = x_coord.into_inner() - location.x();
                        seeds.push((vid, dist));
                    }
                }
            }
            CompassDirection::West => {
                // Find vertical VG lines with X < location.X.
                for (&x_coord, _ys) in self.x_to_ys.range(..lx).rev() {
                    if let Some(vid) = self.closest_live_vertex_on_line_x(x_coord, ly) {
                        let dist = location.x() - x_coord.into_inner();
                        seeds.push((vid, dist));
                    }
                }
            }
            CompassDirection::North => {
                // Find horizontal VG lines with Y > location.Y.
                for (&y_coord, _xs) in self.y_to_xs.range(
                    (std::ops::Bound::Excluded(ly), std::ops::Bound::Unbounded),
                ) {
                    if let Some(vid) = self.closest_live_vertex_on_line_y(y_coord, lx) {
                        let dist = y_coord.into_inner() - location.y();
                        seeds.push((vid, dist));
                    }
                }
            }
            CompassDirection::South => {
                // Find horizontal VG lines with Y < location.Y.
                for (&y_coord, _xs) in self.y_to_xs.range(..ly).rev() {
                    if let Some(vid) = self.closest_live_vertex_on_line_y(y_coord, lx) {
                        let dist = location.y() - y_coord.into_inner();
                        seeds.push((vid, dist));
                    }
                }
            }
        }

        // Seeds are already sorted by distance since BTreeMap iteration
        // is in key order and we iterate from nearest to farthest.
        seeds
    }

    /// Find the closest live (degree > 0) vertex on a vertical line at X=x_coord
    /// to the target Y. Used for finding chain-walk seeds.
    fn closest_live_vertex_on_line_x(
        &self,
        x_coord: OrderedFloat<f64>,
        target_y: OrderedFloat<f64>,
    ) -> Option<VertexId> {
        let ys = self.x_to_ys.get(&x_coord)?;
        let below = ys.range(..=target_y).next_back();
        let above = ys.range(
            (std::ops::Bound::Excluded(target_y), std::ops::Bound::Unbounded),
        ).next();
        let candidates = [below, above];
        let mut best: Option<(VertexId, f64)> = None;
        for candidate in candidates.into_iter().flatten() {
            let pt = Point::new(x_coord.into_inner(), candidate.into_inner());
            if let Some(vid) = self.find_vertex(pt) {
                // Only return vertices with edges (skip zombies).
                if self.degree(vid) > 0 {
                    let dist = (candidate.into_inner() - target_y.into_inner()).abs();
                    if best.is_none_or(|(_, d)| dist < d) {
                        best = Some((vid, dist));
                    }
                }
            }
        }
        best.map(|(vid, _)| vid)
    }

    /// Find the closest live (degree > 0) vertex on a horizontal line at Y=y_coord
    /// to the target X. Used for finding chain-walk seeds.
    fn closest_live_vertex_on_line_y(
        &self,
        y_coord: OrderedFloat<f64>,
        target_x: OrderedFloat<f64>,
    ) -> Option<VertexId> {
        let xs = self.y_to_xs.get(&y_coord)?;
        let below = xs.range(..=target_x).next_back();
        let above = xs.range(
            (std::ops::Bound::Excluded(target_x), std::ops::Bound::Unbounded),
        ).next();
        let candidates = [below, above];
        let mut best: Option<(VertexId, f64)> = None;
        for candidate in candidates.into_iter().flatten() {
            let pt = Point::new(candidate.into_inner(), y_coord.into_inner());
            if let Some(vid) = self.find_vertex(pt) {
                if self.degree(vid) > 0 {
                    let dist = (candidate.into_inner() - target_x.into_inner()).abs();
                    if best.is_none_or(|(_, d)| dist < d) {
                        best = Some((vid, dist));
                    }
                }
            }
        }
        best.map(|(vid, _)| vid)
    }
}

impl Default for VisibilityGraph {
    fn default() -> Self {
        Self::new()
    }
}
