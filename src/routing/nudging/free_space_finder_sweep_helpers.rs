//! Helper methods for FreeSpaceFinderSweep.
//!
//! Constraint application, geometry predicates, side tree operations,
//! edge restriction helpers, and result application methods.

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::routing::nudging::axis_edge::{AxisEdge, AxisEdgeId};

use super::free_space_finder_sweep::{ActiveSide, AxisEdgesContainer, FreeSpaceFinderSweep, SweepEvent, SweepEventEntry};

impl FreeSpaceFinderSweep {
    // -- Constraint application --

    /// C# ConstraintEdgeWithObstaclesAtZ (lines 167-171)
    pub(super) fn constraint_edge_with_obstacles_at_z(
        &mut self,
        edge_id: AxisEdgeId,
        point: Point,
        axis_edges: &[AxisEdge],
    ) {
        self.constraint_from_left(edge_id, point, axis_edges);
        self.constraint_from_right(edge_id, point, axis_edges);
    }

    /// C# ConstraintEdgeWithObstaclesAtZFromLeft (lines 186-193)
    fn constraint_from_left(
        &mut self,
        edge_id: AxisEdgeId,
        point: Point,
        _axis_edges: &[AxisEdge],
    ) {
        let point_perp = self.x_projection(point);
        // MUST use BTreeMap range() -- NOT linear scan.
        let side_opt = self
            .right_side_tree
            .range(..=OrderedFloat(point_perp + GeomConstants::DISTANCE_EPSILON))
            .next_back()
            .and_then(|(_, sides)| {
                sides.iter().find(|s| {
                    Self::point_to_right_of_line_or_on_line(point, s.start, s.end)
                }).cloned()
            });

        if let Some(side) = side_opt {
            if self.not_restricting(edge_id, side.obstacle_idx) {
                return;
            }
            let x = self.intersection_of_side_and_sweep_line(&side);
            self.left_bounds.push((edge_id, x));
        }
    }

    /// C# ConstraintEdgeWithObstaclesAtZFromRight (lines 173-179)
    fn constraint_from_right(
        &mut self,
        edge_id: AxisEdgeId,
        point: Point,
        _axis_edges: &[AxisEdge],
    ) {
        let point_perp = self.x_projection(point);
        // MUST use BTreeMap range() -- NOT linear scan.
        let side_opt = self
            .left_side_tree
            .range(OrderedFloat(point_perp - GeomConstants::DISTANCE_EPSILON)..)
            .next()
            .and_then(|(_, sides)| {
                sides.iter().find(|s| {
                    Self::point_to_left_of_line_or_on_line(point, s.start, s.end)
                }).cloned()
            });

        if let Some(side) = side_opt {
            if self.not_restricting(edge_id, side.obstacle_idx) {
                return;
            }
            let x = self.intersection_of_side_and_sweep_line(&side);
            self.right_bounds.push((edge_id, x));
        }
    }

    // -- NotRestricting --

    /// C# NotRestricting (line 400-403): edge originated from this obstacle
    pub(super) fn not_restricting(&self, edge_id: AxisEdgeId, obstacle_idx: usize) -> bool {
        if edge_id < self.axis_edge_to_obstacle.len() {
            self.axis_edge_to_obstacle[edge_id] == Some(obstacle_idx)
        } else {
            false
        }
    }

    // -- Neighbor helpers --

    pub(super) fn try_to_add_right_neighbor(
        &mut self,
        left_edge: AxisEdgeId,
        right_edge: AxisEdgeId,
        axis_edges: &[AxisEdge],
    ) {
        if self.projections_overlap(&axis_edges[left_edge], &axis_edges[right_edge]) {
            self.neighbor_pairs.push((left_edge, right_edge));
        }
    }

    /// C# ProjectionsOfEdgesOverlap (lines 125-131)
    pub(super) fn projections_overlap(&self, left: &AxisEdge, right: &AxisEdge) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        match self.direction {
            crate::routing::scan_direction::Direction::North => {
                !(left.target.y() < right.source.y() - eps
                    || right.target.y() < left.source.y() - eps)
            }
            crate::routing::scan_direction::Direction::East => {
                !(left.target.x() < right.source.x() - eps
                    || right.target.x() < left.source.x() - eps)
            }
        }
    }

    // -- Container helpers --

    /// C# GetOrCreateAxisEdgesContainer (lines 320-329)
    /// MUST use BTreeMap range query -- NOT linear scan.
    pub(super) fn get_or_create_container_key(&mut self, source: Point) -> OrderedFloat<f64> {
        let prj = self.x_projection(source);
        let half_eps = GeomConstants::DISTANCE_EPSILON / 2.0;

        let existing_key = self
            .edge_containers
            .range(OrderedFloat(prj - half_eps)..)
            .next()
            .and_then(|(k, _)| {
                if k.into_inner() <= prj + half_eps {
                    Some(*k)
                } else {
                    None
                }
            });

        if let Some(key) = existing_key {
            key
        } else {
            let key = OrderedFloat(prj);
            self.edge_containers.insert(
                key,
                AxisEdgesContainer {
                    source,
                    edges: Vec::new(),
                },
            );
            key
        }
    }

    /// C# RemoveEdge (lines 415-420)
    pub(super) fn remove_edge(&mut self, edge_id: AxisEdgeId, source: Point) {
        let prj = self.x_projection(source);
        let half_eps = GeomConstants::DISTANCE_EPSILON / 2.0;

        let key_opt = self
            .edge_containers
            .range(OrderedFloat(prj - half_eps)..)
            .next()
            .and_then(|(k, _)| {
                if k.into_inner() <= prj + half_eps {
                    Some(*k)
                } else {
                    None
                }
            });

        if let Some(key) = key_opt {
            let should_remove = {
                let container = self.edge_containers.get_mut(&key).unwrap();
                container.edges.retain(|&e| e != edge_id);
                container.edges.is_empty()
            };
            if should_remove {
                self.edge_containers.remove(&key);
            }
        }
    }

    // -- Result application --

    pub fn apply_results(&self, axis_edges: &mut [AxisEdge]) {
        self.apply_bounds_only(axis_edges);
        for &(left_id, right_id) in &self.neighbor_pairs {
            if !axis_edges[left_id].right_neighbors.contains(&right_id) {
                axis_edges[left_id].right_neighbors.push(right_id);
            }
        }
    }

    /// Apply only the bound constraints (not neighbor relationships).
    pub fn apply_bounds_only(&self, axis_edges: &mut [AxisEdge]) {
        for &(edge_id, bound) in &self.left_bounds {
            axis_edges[edge_id].bound_from_left(bound);
        }
        for &(edge_id, bound) in &self.right_bounds {
            axis_edges[edge_id].bound_from_right(bound);
        }
    }

    // -- Geometry helpers --

    #[inline]
    pub(super) fn x_projection(&self, p: Point) -> f64 {
        match self.direction {
            crate::routing::scan_direction::Direction::North => p.x(),
            crate::routing::scan_direction::Direction::East => -p.y(),
        }
    }

    #[inline]
    pub(super) fn sweep_dir_vec(&self) -> Point {
        match self.direction {
            crate::routing::scan_direction::Direction::North => Point::new(0.0, 1.0),
            crate::routing::scan_direction::Direction::East => Point::new(1.0, 0.0),
        }
    }

    #[inline]
    pub(super) fn dir_perp_vec(&self) -> Point {
        match self.direction {
            crate::routing::scan_direction::Direction::North => Point::new(1.0, 0.0),
            crate::routing::scan_direction::Direction::East => Point::new(0.0, -1.0),
        }
    }

    #[inline]
    pub(super) fn dot_sweep(&self, p: Point) -> f64 {
        p.dot(self.sweep_dir_vec())
    }

    #[inline]
    pub(super) fn dot_perp(&self, p: Point) -> f64 {
        p.dot(self.dir_perp_vec())
    }

    #[inline]
    pub(super) fn get_z(&self, p: Point) -> f64 {
        self.dot_sweep(p)
    }

    pub(super) fn edge_is_parallel_to_sweep_dir(&self, edge: &AxisEdge) -> bool {
        edge.direction == self.direction
    }

    pub(super) fn ordered_endpoints(&self, edge: &AxisEdge) -> (Point, Point) {
        let sz = self.dot_sweep(edge.source);
        let tz = self.dot_sweep(edge.target);
        if sz <= tz {
            (edge.source, edge.target)
        } else {
            (edge.target, edge.source)
        }
    }

    pub(super) fn get_lowest_vertex_idx(&self, vertices: &[Point; 4]) -> usize {
        let mut best = 0;
        for i in 1..4 {
            if self.sweep_less(vertices[i], vertices[best]) {
                best = i;
            }
        }
        best
    }

    fn sweep_less(&self, a: Point, b: Point) -> bool {
        let az = self.dot_sweep(a);
        let bz = self.dot_sweep(b);
        if az < bz {
            return true;
        }
        if az > bz {
            return false;
        }
        self.dot_perp(a) < self.dot_perp(b)
    }

    pub(super) fn enqueue_event(&mut self, event: SweepEvent) {
        let site = event.site();
        let z = OrderedFloat(self.dot_sweep(site));
        let perp = OrderedFloat(self.dot_perp(site));
        self.event_queue.push(SweepEventEntry { z, perp, event });
    }

    // -- Side tree operations --

    pub(super) fn intersection_of_side_and_sweep_line(&self, side: &ActiveSide) -> f64 {
        let start_z = self.dot_sweep(side.start);
        let end_z = self.dot_sweep(side.end);
        let dz = end_z - start_z;

        if dz.abs() < GeomConstants::DISTANCE_EPSILON {
            self.dot_perp(side.start)
        } else {
            let start_perp = self.dot_perp(side.start);
            let end_perp = self.dot_perp(side.end);
            let t = (self.z - start_z) / dz;
            start_perp + t * (end_perp - start_perp)
        }
    }

    fn side_key(&self, side: &ActiveSide) -> OrderedFloat<f64> {
        OrderedFloat(self.intersection_of_side_and_sweep_line(side))
    }

    pub(super) fn insert_left_side(&mut self, start: Point, end: Point, obstacle_idx: usize) {
        let side = ActiveSide { start, end, obstacle_idx };
        let key = self.side_key(&side);
        self.left_side_tree.entry(key).or_insert_with(Vec::new).push(side);
    }

    pub(super) fn insert_right_side(&mut self, start: Point, end: Point, obstacle_idx: usize) {
        let side = ActiveSide { start, end, obstacle_idx };
        let key = self.side_key(&side);
        self.right_side_tree.entry(key).or_insert_with(Vec::new).push(side);
    }

    pub(super) fn remove_left_side(&mut self, start: Point, end: Point) {
        let side = ActiveSide { start, end, obstacle_idx: 0 };
        let key = self.side_key(&side);
        let half_eps = GeomConstants::DISTANCE_EPSILON;

        let mut key_to_clean = None;
        for (&k, sides) in self
            .left_side_tree
            .range(OrderedFloat(key.into_inner() - half_eps)..=OrderedFloat(key.into_inner() + half_eps))
        {
            if sides.iter().any(|s| s.start.close_to(start) && s.end.close_to(end)) {
                key_to_clean = Some(k);
                break;
            }
        }

        if let Some(k) = key_to_clean {
            let sides = self.left_side_tree.get_mut(&k).unwrap();
            sides.retain(|s| !(s.start.close_to(start) && s.end.close_to(end)));
            if sides.is_empty() {
                self.left_side_tree.remove(&k);
            }
        }
    }

    pub(super) fn remove_right_side(&mut self, start: Point, end: Point) {
        let side = ActiveSide { start, end, obstacle_idx: 0 };
        let key = self.side_key(&side);
        let half_eps = GeomConstants::DISTANCE_EPSILON;

        let mut key_to_clean = None;
        for (&k, sides) in self
            .right_side_tree
            .range(OrderedFloat(key.into_inner() - half_eps)..=OrderedFloat(key.into_inner() + half_eps))
        {
            if sides.iter().any(|s| s.start.close_to(start) && s.end.close_to(end)) {
                key_to_clean = Some(k);
                break;
            }
        }

        if let Some(k) = key_to_clean {
            let sides = self.right_side_tree.get_mut(&k).unwrap();
            sides.retain(|s| !(s.start.close_to(start) && s.end.close_to(end)));
            if sides.is_empty() {
                self.right_side_tree.remove(&k);
            }
        }
    }

    // -- Geometry predicates --

    pub(super) fn point_to_left_of_line_or_on_line(a: Point, line0: Point, line1: Point) -> bool {
        Point::signed_doubled_triangle_area(a, line0, line1)
            > -GeomConstants::INTERSECTION_EPSILON
    }

    pub(super) fn point_to_right_of_line_or_on_line(a: Point, line0: Point, line1: Point) -> bool {
        Point::signed_doubled_triangle_area(line0, line1, a)
            < GeomConstants::INTERSECTION_EPSILON
    }

    // -- Edge restriction helpers --

    /// C# RestrictEdgeFromTheLeftOfEvent (lines 445-454)
    pub(super) fn restrict_edge_from_left_of_event(
        &mut self,
        site: Point,
        obstacle_idx: usize,
        _axis_edges: &[AxisEdge],
    ) {
        let site_x = self.x_projection(site);
        let site_perp = self.dot_perp(site);

        let container_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range(..=OrderedFloat(site_x + GeomConstants::DISTANCE_EPSILON))
            .next_back()
            .map(|(_, c)| c.edges.clone());

        if let Some(edges) = container_edges {
            for &edge_id in &edges {
                if !self.not_restricting(edge_id, obstacle_idx) {
                    self.right_bounds.push((edge_id, site_perp));
                }
            }
        }
    }

    /// C# RestrictEdgeContainerToTheRightOfEvent (lines 387-398)
    pub(super) fn restrict_edge_container_to_right_of_event(
        &mut self,
        site: Point,
        obstacle_idx: usize,
        _axis_edges: &[AxisEdge],
    ) {
        let site_x = self.x_projection(site);
        let site_perp = self.dot_perp(site);

        let container_edges: Option<Vec<AxisEdgeId>> = self
            .edge_containers
            .range(OrderedFloat(site_x - GeomConstants::DISTANCE_EPSILON)..)
            .next()
            .map(|(_, c)| c.edges.clone());

        if let Some(edges) = container_edges {
            for &edge_id in &edges {
                if !self.not_restricting(edge_id, obstacle_idx) {
                    self.left_bounds.push((edge_id, site_perp));
                }
            }
        }
    }
}
